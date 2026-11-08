/***************
Rack UI controller — BLE-MIDI peripheral implementation (see the header for
the role rationale + UUIDs). GATT server built at runtime from static
storage (C++-safe — no nested compound literals), advertised on sync.
The Apple BLE-MIDI decoder tracks the same byte positions as the seed's
central parser (leading packet timestamp, per-message timestamps, running
status) but REASSEMBLES complete messages into 4-byte USB-MIDI event
packets instead of passing a raw byte stream.
***************/

#include "BtMidiReceiver.hpp"
#include "RackLink.hpp"

#include "esp_log.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "esp_heap_caps.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

using namespace CTAG::DRIVERS;

static const char *TAG = "BLE_MIDI";

// Standard BLE-MIDI UUIDs (kept verbatim from the seed — the spec's
// 03B80E5A-EDE8-4B33-A751-6CE34EC4C700 / ...C702 pair).
static const ble_uuid128_t MIDI_SVC_UUID = BLE_UUID128_INIT(
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03);
static const ble_uuid128_t MIDI_CHR_UUID = BLE_UUID128_INIT(
    0x02, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03);
static const ble_uuid16_t CCCD_UUID = BLE_UUID16_INIT(0x2902);

static volatile bool s_connected = false;
static uint16_t s_midi_val_handle = 0;
static volatile uint32_t s_msg_count = 0;
static volatile uint32_t s_drop_count = 0;

static int gap_event_cb(struct ble_gap_event *event, void *arg);  // fwd

// ---- GATT table (built at runtime, static storage) ----
static struct ble_gatt_dsc_def s_midi_dsc[2];
static struct ble_gatt_chr_def s_midi_chrs[2];
static struct ble_gatt_svc_def s_gatt_svcs[2];

// ---- Apple BLE-MIDI -> 4-byte USB-MIDI event packets ----
namespace {

struct MsgAccum {
    uint8_t status;    // 0 = none
    int expected;      // data bytes still wanted
    int nd;
    uint8_t d[2];
};
MsgAccum s_msg;            // current message under assembly
uint8_t s_running = 0;     // running status across packets (BLE-MIDI allows)
bool s_done = true;        // current message complete (running-status armed)

void emitVoice(uint8_t status, uint8_t d0, uint8_t d1) {
    const uint8_t pkt[4] = { (uint8_t)(status >> 4), status, d0, d1 };
    CTAG::CTRL::RackLink::SendMidi(pkt);
    s_msg_count = s_msg_count + 1;
}

void emitRealtime(uint8_t st) {
    const uint8_t pkt[4] = { 0x0F, st, 0, 0 };   // CIN 0xF single byte
    CTAG::CTRL::RackLink::SendMidi(pkt);
    s_msg_count = s_msg_count + 1;
}

void beginMessage(uint8_t status) {
    s_msg.status = status;
    s_msg.expected = (status == 0xC0 || status == 0xD0) ? 1 : 2;
    s_msg.nd = 0;
    s_running = status;
    s_done = false;
}

void feedData(uint8_t b) {
    if (!s_done && s_msg.status && s_msg.nd < s_msg.expected) {
        s_msg.d[s_msg.nd++] = b;
        if (s_msg.nd >= s_msg.expected) {
            emitVoice(s_msg.status, s_msg.d[0],
                      s_msg.expected > 1 ? s_msg.d[1] : 0);
            s_done = true;    // running status armed for the next data byte
        }
        return;
    }
    // Running status: a data byte at a message boundary reuses the last
    // channel status (BLE-MIDI streams allow it).
    if (s_running >= 0x80 && s_running < 0xF0) {
        beginMessage(s_running);
        s_msg.d[s_msg.nd++] = b;
        if (s_msg.nd >= s_msg.expected) {
            emitVoice(s_msg.status, s_msg.d[0],
                      s_msg.expected > 1 ? s_msg.d[1] : 0);
            s_done = true;
        }
        return;
    }
    s_drop_count = s_drop_count + 1;   // data byte with no status context
}

// Packet layout (seed parser's position logic, verbatim): optional 2-byte
// leading timestamp, then messages each optionally preceded by a timestamp;
// bit7-set bytes are statuses at message boundaries / mid-message overrides,
// timestamps between completed messages.
void parseBleMidi(const uint8_t *data, int len) {
    int i = 0;
    if (len >= 2 && (data[0] & 0x80)) i = 2;
    bool expectStatus = true;
    int expected = 0;
    while (i < len) {
        const uint8_t b = data[i];
        if ((b & 0x80) == 0) {            // data byte
            feedData(b);
            if (expected > 0) expected--;
            i++;
            continue;
        }
        if (b >= 0xF8) {                  // real-time: 1 byte anywhere
            emitRealtime(b);
            i++;
            continue;
        }
        if (expectStatus) {               // boundary bit7 byte = status
            if (b >= 0xF0) { s_drop_count = s_drop_count + 1; i++; continue; }  // sysex/common
            beginMessage(b);
            expected = s_msg.expected;
            expectStatus = false;
            i++;
            continue;
        }
        if (expected > 0) {               // mid-message: status override
            if (b >= 0xF0) { s_drop_count = s_drop_count + 1; i++; continue; }
            beginMessage(b);
            expected = s_msg.expected;
            i++;
            continue;
        }
        // Message complete: this bit7 byte is a TIMESTAMP (+ its low byte).
        i += 2;
        expectStatus = true;
    }
}

} // namespace

// ---- GATT callbacks ----
static int midiChrAccess(uint16_t conn, uint16_t attr,
                         struct ble_gatt_access_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_WRITE_CHR: {
            struct os_mbuf *om = ctxt->om;
            uint8_t buf[128];
            uint16_t n = OS_MBUF_PKTLEN(om);
            if (n > sizeof(buf)) n = sizeof(buf);
            os_mbuf_copydata(om, 0, n, buf);
            parseBleMidi(buf, n);
            return 0;
        }
        case BLE_GATT_ACCESS_OP_READ_CHR:
            // Empty read (some centrals probe the characteristic).
            return 0;
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

static int cccdAccess(uint16_t conn, uint16_t attr,
                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    // The characteristic advertises NOTIFY so centrals that require it can
    // subscribe; the rack panel does not stream MIDI back (stage 6 is
    // panel-as-input). Accept and record.
    return 0;
}

static void startAdvertising() {
    ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = const_cast<ble_uuid128_t *>(&MIDI_SVC_UUID);
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv set fields: %d", rc);
        return;
    }
    struct ble_gap_adv_params advp;
    memset(&advp, 0, sizeof(advp));
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, nullptr, BLE_HS_FOREVER,
                           &advp, gap_event_cb, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv start: %d", rc);
    } else {
        ESP_LOGI(TAG, "advertising BLE-MIDI (Rack UI)");
    }
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_connected = true;
                ESP_LOGI(TAG, "central connected (handle %u)",
                         event->connect.conn_handle);
            } else {
                // Connection attempt failed — advertise again.
                startAdvertising();
            }
            return 0;
        case BLE_GAP_EVENT_DISCONNECT:
            s_connected = false;
            s_msg = {};
            s_done = true;
            s_running = 0;
            ESP_LOGI(TAG, "central disconnected — advertising again");
            startAdvertising();
            return 0;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            startAdvertising();
            return 0;
        case BLE_GAP_EVENT_PASSKEY_ACTION: {
            // Auto-accept Just Works (same as the seed's pairing handling —
            // ignoring the action stalls some centrals' stacks).
            struct ble_sm_io pkey;
            memset(&pkey, 0, sizeof(pkey));
            pkey.action = event->passkey.params.action;
            if (pkey.action == BLE_SM_IOACT_NUMCMP) pkey.numcmp_accept = 1;
            ble_sm_inject_io(event->connect.conn_handle, &pkey);
            return 0;
        }
        default:
            return 0;
    }
}

static void buildGatt() {
    memset(s_midi_dsc, 0, sizeof(s_midi_dsc));
    s_midi_dsc[0].uuid = &CCCD_UUID.u;
    s_midi_dsc[0].access_cb = cccdAccess;
    s_midi_dsc[0].att_flags = BLE_ATT_F_READ | BLE_ATT_F_WRITE;

    memset(s_midi_chrs, 0, sizeof(s_midi_chrs));
    s_midi_chrs[0].uuid = &MIDI_CHR_UUID.u;
    s_midi_chrs[0].access_cb = midiChrAccess;
    s_midi_chrs[0].flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                           BLE_GATT_CHR_F_WRITE_NO_RSP |
                           BLE_GATT_CHR_F_NOTIFY;
    s_midi_chrs[0].val_handle = &s_midi_val_handle;
    s_midi_chrs[0].descriptors = s_midi_dsc;

    memset(s_gatt_svcs, 0, sizeof(s_gatt_svcs));
    s_gatt_svcs[0].type = BLE_GATT_SVC_TYPE_PRIMARY;
    s_gatt_svcs[0].uuid = &MIDI_SVC_UUID.u;
    s_gatt_svcs[0].characteristics = s_midi_chrs;
}

static void on_sync(void) {
    ESP_LOGI(TAG, "NimBLE host synced");
    buildGatt();
    int rc = ble_gatts_count_cfg(s_gatt_svcs);
    if (rc == 0) rc = ble_gatts_add_svcs(s_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT registration failed: %d", rc);
        return;
    }
    ble_gatts_start();
    rc = ble_svc_gap_device_name_set("Rack UI");
    if (rc != 0) ESP_LOGW(TAG, "gap name set: %d", rc);
    startAdvertising();
}

static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

namespace CTAG::DRIVERS {

bool BtMidiReceiver::IsConnected() { return s_connected; }
uint32_t BtMidiReceiver::MsgCount() { return s_msg_count; }
uint32_t BtMidiReceiver::DropCount() { return s_drop_count; }

void BtMidiReceiver::Init() {
    // ECO3 ROM BT helpers required (chip revision >= 3) — the seed's guard.
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    if (ci.revision < 3) {
        ESP_LOGW(TAG, "BLE-MIDI unavailable: chip rev %u < 3 (ECO3 ROM)",
                 ci.revision);
        return;
    }
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
        nvs_flash_init();
    }

    ESP_LOGI(TAG, "heap before NimBLE: free=%u largest=%u",
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    nimble_port_init();
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "BLE-MIDI peripheral initializing (advertising on sync)");
}

}
