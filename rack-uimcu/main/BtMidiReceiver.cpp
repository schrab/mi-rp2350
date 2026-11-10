/***************
Rack UI controller — BLE-MIDI dual-role implementation (see the header).

PERIPHERAL path (this file's original, benched 2026-11-11): GATT server
with the Apple BLE-MIDI service; centrals WRITE BLE-MIDI packets to the
C702 characteristic -> midiChrAccess -> parseBleMidi -> SendMidi.

CENTRAL path (lifted from the seed's central, git d901e76 — it was
written against this desk's SMC-PAD, so its quirks ride along): scan ->
auto-connect to the first advertiser carrying the BLE-MIDI service ->
service/characteristic/descriptor discovery -> reliable CCCD write ->
keepalive poke (the pad only starts notifying once the client writes to
it) -> NOTIFY_RX -> the SAME parseBleMidi -> SendMidi. Passkey actions
are auto-accepted (the pad initiates pairing right after discovery and
stalls if ignored); the CCCD is re-written after encryption settles
(pairing resets CCCD state).

Role state machine: SetEnabled/SetMode (SYS cells / console) call
applyRole() — stop advertise / cancel scan / disconnect the peer as
needed, then start the wanted role. NimBLE supports calling gap APIs
from any task (the host locks); the SYS pot handler and the console run
in their own tasks, exactly like the seed's TickReconnect did.
***************/

#include "BtMidiReceiver.hpp"

#include <cstring>
#include <cstdio>

#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_heap_caps.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"

#include "RackLink.hpp"

#include <strings.h>

static const char *TAG = "BLE_MIDI";

// File-scope alias: the mode constants live on the class (CTAG::DRIVERS),
// and the gap/discovery code below sits outside that namespace.
using CTAG::DRIVERS::BtMidiReceiver;

// Unified GAP callback (both roles) — defined below, used by adv/scan.
static int gap_event_cb(struct ble_gap_event *event, void *arg);

// Standard BLE-MIDI UUIDs (kept verbatim from the seed — the spec's
// 03B80E5A-EDE8-4B33-A751-6CE34EC4C700 / ...C702 pair).
static const ble_uuid128_t MIDI_SVC_UUID = BLE_UUID128_INIT(
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03);
static const ble_uuid128_t MIDI_CHR_UUID = BLE_UUID128_INIT(
    0x02, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03);
// This pad's actual MIDI I/O characteristic UUID (the M-VAVE/SINCO
// variant — the Apple C702 UUID is NOT in its GATT).
static const ble_uuid128_t MIDI_CHR_SINCO_UUID = BLE_UUID128_INIT(
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77);
static const ble_uuid16_t CCCD_UUID = BLE_UUID16_INIT(0x2902);

// ---- shared state ----
static volatile bool s_enabled = true;      // NVS "bleui"/en
static volatile uint8_t s_mode = 0;         // NVS "bleui"/mode (0 PER, 1 CEN)
static volatile bool s_connected = false;   // either role
static volatile uint32_t s_msg_count = 0;
static volatile uint32_t s_drop_count = 0;
// The LOCAL GATT value handle (peripheral side; registered at sync).
static uint16_t s_midi_val_handle = 0;

// ---- central-side state (the ctag-tbd central's shape, verbatim where it
// matters — subscribe = write_flat(val_handle+1) inside chr discovery, the
// MIDI service only; its flow is the one proven against this pad) ----
static bool s_scanning = false;
static bool s_connecting = false;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static bool s_conn_is_central = false;   // our role for the CURRENT connection
static uint16_t s_peer_val_handle = 0;   // peer's MIDI characteristic
static uint16_t s_peer_cccd_handle = 0;  // peer's 0x2902 once discovered
static int64_t s_next_scan_earliest_us = 0;  // connect-failure backoff

// NimBLE store config (bonds/CCC persistence bookkeeping) — the ctag
// uimcu Init calls it; declared extern "C" in NimBLE's store module.
extern "C" void ble_store_config_init(void);

// Scan device list (the console `ble scan` view; the AUTO-connect path
// keys on the advertised BLE-MIDI service, not this list).
static constexpr int kMaxScanDevices = 8;
struct ScanDevice {
    uint8_t bda[6];
    uint8_t addr_type;
    char name[24];
    bool midi_svc;      // advertises the standard BLE-MIDI service
};
static ScanDevice s_scan[kMaxScanDevices];
static int s_scan_count = 0;

// ---- Apple BLE-MIDI -> 4-byte USB-MIDI event packets (ONE parser for
// both RX paths — a GATT write and a GATT notification carry the same
// packet format) ----
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

// ===================== peripheral (GATT server) =========================
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

// ===================== central (scanner + subscriber) ===================
// Discovery/subscribe flow mirrors the ctag uimcu C3 panel's
// BtMidiReceiver (the implementation that streams from this pad on the
// same chip) VERBATIM: explicit MTU exchange -> service discovery ->
// characteristic discovery -> 0x2902 CCCD via RELIABLE write -> Apple-
// spec READ of the MIDI I/O characteristic -> write_no_rsp keepalive.
// One GATT procedure in flight at a time (callbacks chain the next).
static int on_svc_discovered(uint16_t conn,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_svc *svc, void *arg);
static int on_chr_discovered(uint16_t conn,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *chr, void *arg);
static int on_dsc_discovered(uint16_t conn,
                             const struct ble_gatt_error *error,
                             uint16_t chr_val_handle,
                             const struct ble_gatt_dsc *dsc, void *arg);
static int cccd_write_cb(uint16_t conn, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg);
static int midi_read_cb(uint16_t conn, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg);

static int on_mtu_exchanged(uint16_t conn,
                            const struct ble_gatt_error *error,
                            uint16_t mtu, void *arg) {
    ESP_LOGI(TAG, "MTU exchange complete: status=%d mtu=%u, discovering "
                  "services", error ? error->status : -1, (unsigned)mtu);
    int rc = ble_gattc_disc_all_svcs(conn, on_svc_discovered, nullptr);
    if (rc != 0) ESP_LOGE(TAG, "Failed to start svc discovery: %d", rc);
    return 0;
}

static void startScan();

static int midi_read_cb(uint16_t conn, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg) {
    if (error && error->status != 0) {
        ESP_LOGW(TAG, "midi chr read FAILED: %d", error->status);
    }
    if (s_peer_val_handle != 0) {
        const uint8_t ka[3] = {0x80, 0x80, 0xF8};
        ble_gattc_write_no_rsp_flat(conn, s_peer_val_handle, ka,
                                    sizeof(ka));
    }
    return 0;
}

static int cccd_write_cb(uint16_t conn, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg) {
    if (error && error->status != 0) {
        ESP_LOGE(TAG, "CCCD write REJECTED at ATT level: %d", error->status);
    } else {
        ESP_LOGI(TAG, "CCCD write accepted — notifications enabled");
        // Apple BLE-MIDI spec: the Central READS the MIDI I/O
        // characteristic upon establishing a connection. Chained after
        // the CCCD write so only one GATT procedure is in flight.
        if (s_peer_val_handle != 0) {
            ble_gattc_read(conn, s_peer_val_handle, midi_read_cb, nullptr);
        }
    }
    return 0;
}

static int on_svc_discovered(uint16_t conn,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_svc *svc, void *arg) {
    if (error->status == 0 && svc != nullptr) {
        if (ble_uuid_cmp(&svc->uuid.u, &MIDI_SVC_UUID.u) == 0) {
            ESP_LOGI(TAG, "Found MIDI service, discovering characteristics");
            int rc = ble_gattc_disc_all_chrs(conn, svc->start_handle,
                                             svc->end_handle,
                                             on_chr_discovered, nullptr);
            if (rc != 0) ESP_LOGE(TAG, "Failed to start chr discovery: %d", rc);
        }
    } else if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Service discovery complete");
    } else {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
    }
    return 0;
}

static int on_chr_discovered(uint16_t conn,
                             const struct ble_gatt_error *error,
                             const struct ble_gatt_chr *chr, void *arg) {
    if (error->status == 0 && chr != nullptr) {
        // The Apple UUID first, then this pad's actual device-specific
        // UUID (7772E5DB-...), then any chr in the MIDI service.
        if (ble_uuid_cmp(&chr->uuid.u, &MIDI_CHR_UUID.u) == 0 ||
            ble_uuid_cmp(&chr->uuid.u, &MIDI_CHR_SINCO_UUID.u) == 0) {
            ESP_LOGI(TAG, "Found MIDI I/O characteristic");
        } else if (s_peer_val_handle == 0) {
            ESP_LOGI(TAG, "Using device-specific MIDI characteristic");
        } else {
            return 0; // already found
        }
        s_peer_val_handle = chr->val_handle;
    } else if (error->status == BLE_HS_EDONE) {
        if (s_peer_val_handle != 0) {
            int rc = ble_gattc_disc_all_dscs(conn, s_peer_val_handle,
                                             s_peer_val_handle + 4,
                                             on_dsc_discovered, nullptr);
            if (rc != 0) ESP_LOGE(TAG, "dsc discovery failed: %d", rc);
        }
    } else {
        ESP_LOGE(TAG, "Characteristic discovery error: %d", error->status);
    }
    return 0;
}

static int on_dsc_discovered(uint16_t conn,
                             const struct ble_gatt_error *error,
                             uint16_t chr_val_handle,
                             const struct ble_gatt_dsc *dsc, void *arg) {
    if (error->status == 0 && dsc != nullptr) {
        if (dsc->uuid.u.type == BLE_UUID_TYPE_16 &&
            dsc->uuid.u16.value == 0x2902 && s_peer_cccd_handle == 0) {
            s_peer_cccd_handle = dsc->handle;
            uint8_t val[2] = {0x01, 0x00};   // notify enable, LE
            // Reliable write WITH response — write-without-response CCCD
            // pokes are silently ignored by some peripherals (M-VAVE).
            struct os_mbuf *om = ble_hs_mbuf_from_flat(val, sizeof(val));
            int rc = om ? ble_gattc_write(conn, dsc->handle, om,
                                          cccd_write_cb, nullptr)
                        : BLE_HS_ENOMEM;
            ESP_LOGI(TAG, "CCCD 0x%04x write started rc=%d", dsc->handle, rc);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (s_peer_cccd_handle == 0 && s_peer_val_handle != 0) {
            // No 0x2902 in range — legacy fallback: CCCD at val+1.
            s_peer_cccd_handle = s_peer_val_handle + 1;
            uint8_t val[2] = {0x01, 0x00};
            struct os_mbuf *om = ble_hs_mbuf_from_flat(val, sizeof(val));
            int rc = om ? ble_gattc_write(conn, s_peer_val_handle + 1, om,
                                          cccd_write_cb, nullptr)
                        : BLE_HS_ENOMEM;
            ESP_LOGW(TAG, "no 0x2902 dsc; fallback CCCD val+1 write "
                          "started rc=%d", rc);
        }
        ESP_LOGI(TAG, "Descriptor discovery complete");
    } else {
        ESP_LOGE(TAG, "Descriptor discovery error: %d", error->status);
    }
    return 0;
}

static void startScan() {
    if (s_scanning || !s_enabled || s_mode != BtMidiReceiver::kModeCentral ||
        s_connected || s_connecting)
        return;
    if (esp_timer_get_time() < s_next_scan_earliest_us) return;
    ble_gap_disc_params p;
    memset(&p, 0, sizeof(p));
    p.itvl = 0x0010;                     // the seed's brisk window
    p.window = 0x0010;
    p.filter_policy = 0;
    p.limited = 0;
    p.passive = 0;                       // active: scan responses carry names
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &p,
                          gap_event_cb, nullptr);
    if (rc == 0) {
        s_scanning = true;
        s_scan_count = 0;
        ESP_LOGI(TAG, "central: scanning for BLE-MIDI peripherals");
    } else {
        ESP_LOGW(TAG, "disc start: %d", rc);
    }
}

// AD-field scan: does this advertisement carry the BLE-MIDI service UUID
// (types 0x06 incomplete / 0x07 complete 128-bit list)?
static bool advHasMidiSvc(const uint8_t *d, int dlen) {
    int off = 0;
    while (off + 1 < dlen) {
        const int fieldLen = d[off];
        if (fieldLen == 0 || off + 1 + fieldLen > dlen) break;
        const int type = d[off + 1];
        if (type == 0x06 || type == 0x07) {
            const uint8_t *u = &d[off + 2];
            const int n = fieldLen - 1;
            for (int i = 0; i + 16 <= n; i += 16) {
                // AD 128-bit UUIDs are little-endian, the same byte order
                // BLE_UUID128_INIT stores in .value.
                if (memcmp(u + i, MIDI_SVC_UUID.value, 16) == 0) return true;
            }
        }
        off += 1 + fieldLen;
    }
    return false;
}

// AD-field scan: device name (types 0x08 short / 0x09 complete).
static const char *advName(const uint8_t *d, int dlen, char *buf, int bufsz) {
    int off = 0;
    while (off + 1 < dlen) {
        const int fieldLen = d[off];
        if (fieldLen == 0 || off + 1 + fieldLen > dlen) break;
        const int type = d[off + 1];
        if (type == 0x08 || type == 0x09) {
            int n = fieldLen - 1;
            if (n > bufsz - 1) n = bufsz - 1;
            memcpy(buf, &d[off + 2], n);
            buf[n] = '\0';
            return buf;
        }
        off += 1 + fieldLen;
    }
    return nullptr;
}

static void tryAutoConnect(const ble_addr_t *addr, bool midiSvc) {
    if (!midiSvc || !s_enabled || s_mode != BtMidiReceiver::kModeCentral ||
        s_connected || s_connecting)
        return;
    ble_gap_disc_cancel();
    s_scanning = false;
    struct ble_gap_conn_params p;
    memset(&p, 0, sizeof(p));
    p.scan_itvl = 0x0010;
    p.scan_window = 0x0010;
    // BLE-MIDI spec intervals (7.5-15 ms) — the pad itself requests
    // itvl 6-9 after subscribing; our old 30-50 ms may leave its MIDI
    // TX idle.
    p.itvl_min = 6;
    p.itvl_max = 12;
    p.latency = 0;
    p.supervision_timeout = 0x00C8;
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, addr, 30000, &p,
                             gap_event_cb, nullptr);
    if (rc == 0) {
        s_connecting = true;
        ESP_LOGI(TAG, "central: connecting to BLE-MIDI peer "
                      "%02x:%02x:%02x:%02x:%02x:%02x",
                 addr->val[5], addr->val[4], addr->val[3], addr->val[2],
                 addr->val[1], addr->val[0]);
    } else {
        ESP_LOGW(TAG, "central: connect rejected: %d", rc);
        s_next_scan_earliest_us = esp_timer_get_time() + 3000000;  // 3 s
        startScan();
    }
}

// ===================== unified GAP events (both roles) ==================
static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                s_conn_handle = event->connect.conn_handle;
                s_connected = true;
                s_connecting = false;
                s_scanning = false;
                // Remember our role for THIS connection (applyRole's
                // role-switch terminator needs it).
                struct ble_gap_conn_desc desc;
                if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0)
                    s_conn_is_central = (desc.role == BLE_GAP_ROLE_MASTER);
                ESP_LOGI(TAG, "connected (handle %u, %s role)",
                         event->connect.conn_handle,
                         s_conn_is_central ? "central" : "peripheral");
                if (s_conn_is_central) {
                    s_peer_val_handle = 0;
                    s_peer_cccd_handle = 0;
                    // Explicit MTU exchange first (the uimcu central's
                    // flow) — service discovery chains off its callback.
                    int rc = ble_gattc_exchange_mtu(s_conn_handle,
                                                    on_mtu_exchanged, nullptr);
                    if (rc != 0) {
                        ESP_LOGW(TAG, "MTU exchange start rc=%d, discovering "
                                      "services", rc);
                        ble_gattc_disc_all_svcs(s_conn_handle,
                                                on_svc_discovered, nullptr);
                    }
                }
            } else {
                // Central connection attempt failed — rescan (with backoff).
                s_connecting = false;
                s_next_scan_earliest_us = esp_timer_get_time() + 3000000;
                startScan();
            }
            return 0;
        case BLE_GAP_EVENT_DISCONNECT:
            s_connected = false;
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            s_connecting = false;
            s_msg = {};
            s_done = true;
            s_running = 0;
            s_peer_val_handle = 0;
            s_peer_cccd_handle = 0;
            ESP_LOGI(TAG, "disconnected (mode %s)",
                     s_mode == BtMidiReceiver::kModeCentral ? "cen" : "per");
            // Restart the CURRENT role.
            if (!s_enabled) return 0;
            if (s_mode == BtMidiReceiver::kModeCentral) {
                startScan();
            } else {
                startAdvertising();
            }
            return 0;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            if (s_enabled && s_mode == BtMidiReceiver::kModePeripheral)
                startAdvertising();
            return 0;
        case BLE_GAP_EVENT_DISC_COMPLETE:
            s_scanning = false;
            ESP_LOGI(TAG, "scan complete, %d devices", s_scan_count);
            startScan();                    // keep hunting
            return 0;
        case BLE_GAP_EVENT_DISC: {
            if (!s_enabled || s_mode != BtMidiReceiver::kModeCentral)
                return 0;
            const struct ble_gap_disc_desc &disc = event->disc;
            const bool midiSvc = advHasMidiSvc(disc.data, disc.length_data);
            char nameBuf[24];
            const char *nm = advName(disc.data, disc.length_data, nameBuf,
                                     sizeof(nameBuf));
            // Record (dedup by address) for the console `ble scan` view.
            int slot = -1;
            for (int i = 0; i < s_scan_count; ++i) {
                if (memcmp(s_scan[i].bda, disc.addr.val, 6) == 0) {
                    slot = i;
                    break;
                }
            }
            if (slot < 0 && s_scan_count < kMaxScanDevices) {
                slot = s_scan_count++;
                memcpy(s_scan[slot].bda, disc.addr.val, 6);
                s_scan[slot].addr_type = disc.addr.type;
                s_scan[slot].midi_svc = false;
                snprintf(s_scan[slot].name, sizeof(s_scan[slot].name), "-");
            }
            if (slot >= 0) {
                if (nm) snprintf(s_scan[slot].name, sizeof(s_scan[slot].name),
                                 "%s", nm);
                if (midiSvc) s_scan[slot].midi_svc = true;
            }
            if (midiSvc) {
                ESP_LOGI(TAG, "central: BLE-MIDI advertiser %s"
                              " %02x:%02x:%02x:%02x:%02x:%02x",
                         nm ? nm : "(unnamed)", disc.addr.val[5],
                         disc.addr.val[4], disc.addr.val[3], disc.addr.val[2],
                         disc.addr.val[1], disc.addr.val[0]);
                tryAutoConnect(&disc.addr, midiSvc);
            }
            return 0;
        }
        case BLE_GAP_EVENT_NOTIFY_RX: {
            const struct os_mbuf *om = event->notify_rx.om;
            if (om && om->om_len > 0) {
                // Probe: the first notifications log handle + bytes so a
                // silent stream can be told apart from a misparsed one.
                static int s_notify_probe = 0;
                if (s_notify_probe < 5) {
                    ++s_notify_probe;
                    char hx[64] = {0};
                    int n = om->om_len < 20 ? om->om_len : 20;
                    for (int i = 0; i < n; ++i)
                        snprintf(hx + i * 3, 4, "%02x ", om->om_data[i]);
                    ESP_LOGI(TAG, "notify #%d (attr %u, %d B): %s",
                             s_notify_probe, event->notify_rx.attr_handle,
                             om->om_len, hx);
                }
                parseBleMidi(om->om_data, om->om_len);
            }
            return 0;
        }
        case BLE_GAP_EVENT_PASSKEY_ACTION: {
            // Auto-accept Just Works / numeric comparison. The SMC-PAD
            // initiates pairing right after discovery — ignoring the
            // action stalls its stack and notifications never start.
            struct ble_sm_io pkey;
            memset(&pkey, 0, sizeof(pkey));
            pkey.action = event->passkey.params.action;
            if (pkey.action == BLE_SM_IOACT_NUMCMP) pkey.numcmp_accept = 1;
            ble_sm_inject_io(event->connect.conn_handle, &pkey);
            return 0;
        }
        case BLE_GAP_EVENT_ENC_CHANGE:
            // Pairing settled; pairing resets CCCD state — re-subscribe the
            // MIDI characteristic once encrypted (ctag-tbd-era finding).
            if (event->enc_change.status == 0 && s_peer_val_handle != 0) {
                uint8_t val[2] = {0x01, 0x00};
                ble_gattc_write_flat(s_conn_handle, s_peer_val_handle + 1,
                                     val, 2, nullptr, nullptr);
                ESP_LOGI(TAG, "CCCD re-subscribe after pairing (chr %u)",
                         s_peer_val_handle);
            }
            return 0;
        default:
            return 0;
    }
}

// ===================== GATT table (peripheral side) =====================
static struct ble_gatt_dsc_def s_midi_dsc[2];
static struct ble_gatt_chr_def s_midi_chrs[2];
static struct ble_gatt_svc_def s_gatt_svcs[2];

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

// ===================== role state machine ===============================
static void persistRole() {
    nvs_handle_t h;
    if (nvs_open("bleui", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "en", s_enabled ? 1 : 0);
    nvs_set_u8(h, "mode", s_mode);
    nvs_commit(h);
    nvs_close(h);
}

// Start the CURRENT role (host-context; called from on_sync + role changes).
// A live connection of the OTHER role is terminated; the DISCONNECT handler
// restarts whatever the then-current role wants.
static void applyRole() {
    if (!s_enabled) {
        ble_gap_adv_stop();              // fine if not advertising
        if (s_scanning) {
            ble_gap_disc_cancel();
            s_scanning = false;
        }
        if (s_connected && s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        ESP_LOGI(TAG, "BLE disabled (no advertise, no scan)");
        return;
    }
    if (s_mode == BtMidiReceiver::kModeCentral) {
        // Leave the peripheral role: stop advertising, drop a peripheral-
        // side client, then scan for peers.
        ble_gap_adv_stop();
        if (s_connected && !s_conn_is_central &&
            s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return;                      // DISCONNECT restarts the new role
        }
        startScan();
    } else {
        // Peripheral mode: stop any scan, drop a central-role peer, then
        // advertise.
        if (s_scanning) {
            ble_gap_disc_cancel();
            s_scanning = false;
        }
        if (s_connected && s_conn_is_central &&
            s_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return;                      // DISCONNECT restarts the new role
        }
        startAdvertising();
    }
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
    ble_svc_gap_device_name_set("Rack UI");
    // Restore the persisted role and start it.
    nvs_handle_t h;
    if (nvs_open("bleui", NVS_READONLY, &h) == ESP_OK) {
        uint8_t v = 1;
        nvs_get_u8(h, "en", &v);
        s_enabled = v != 0;
        v = 0;
        if (nvs_get_u8(h, "mode", &v) == ESP_OK && v <= 1) s_mode = v;
        nvs_close(h);
    }
    applyRole();
}

static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

namespace CTAG::DRIVERS {

bool BtMidiReceiver::IsConnected() { return s_connected; }
uint32_t BtMidiReceiver::MsgCount() { return s_msg_count; }
uint32_t BtMidiReceiver::DropCount() { return s_drop_count; }
bool BtMidiReceiver::Enabled() { return s_enabled; }
uint8_t BtMidiReceiver::Mode() { return s_mode; }

void BtMidiReceiver::SetEnabled(bool on) {
    if (s_enabled == on) return;
    s_enabled = on;
    persistRole();
    ESP_LOGI(TAG, "BLE %s (user)", on ? "ON" : "OFF");
    applyRole();
}

void BtMidiReceiver::SetMode(uint8_t mode) {
    if (mode > kModeCentral || mode == s_mode) return;
    s_mode = mode;
    persistRole();
    ESP_LOGI(TAG, "BLE mode -> %s (user)",
             mode == kModeCentral ? "CENTRAL" : "PERIPHERAL");
    applyRole();   // terminates a live old-role peer; DISCONNECT restarts
}

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
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
        nvs_flash_init();
    }

    ESP_LOGI(TAG, "heap before NimBLE: free=%u largest=%u",
             (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL |
                                                MALLOC_CAP_8BIT),
             (unsigned) heap_caps_get_largest_free_block(
                 MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    nimble_port_init();
    ble_store_config_init();         // NimBLE store (bonds/CCC) — the ctag
                                     // uimcu Init calls it explicitly
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "BLE-MIDI dual-role initializing (persisted mode applies "
                  "on sync)");
}

void BtMidiReceiver::ConsoleCmd(const char *args) {
    while (args != nullptr && *args == ' ') ++args;
    if (args == nullptr || *args == '\0' || strcasecmp(args, "status") == 0) {
        printf("ble: %s, mode %s, %s | msgs=%lu drops=%lu | scan: %d devs\r\n",
               s_enabled ? "ON" : "OFF",
               s_mode == kModeCentral ? "CENTRAL" : "PERIPHERAL",
               s_connected ? "connected" : (s_connecting ? "connecting"
                                         : (s_scanning ? "scanning" : "idle")),
               (unsigned long)MsgCount(), (unsigned long)DropCount(),
               s_scan_count);
        for (int i = 0; i < s_scan_count; ++i) {
            printf("  [%d] %s %02x:%02x:%02x:%02x:%02x:%02x %s\r\n", i,
                   s_scan[i].name,
                   s_scan[i].bda[5], s_scan[i].bda[4], s_scan[i].bda[3],
                   s_scan[i].bda[2], s_scan[i].bda[1], s_scan[i].bda[0],
                   s_scan[i].midi_svc ? "(BLE-MIDI)" : "");
        }
        return;
    }
    if (strcasecmp(args, "on") == 0) { SetEnabled(true); return; }
    if (strcasecmp(args, "off") == 0) { SetEnabled(false); return; }
    if (strcasecmp(args, "per") == 0 || strcasecmp(args, "peripheral") == 0) {
        SetMode(kModePeripheral);
        return;
    }
    if (strcasecmp(args, "cen") == 0 || strcasecmp(args, "central") == 0) {
        SetMode(kModeCentral);
        return;
    }
    if (strcasecmp(args, "scan") == 0) {
        s_scan_count = 0;
        startScan();
        printf("ble: scan restarted (auto-connects the first BLE-MIDI "
               "advertiser; 'ble status' lists)\r\n");
        return;
    }
    printf("ble: usage [status|on|off|per|cen|scan]\r\n");
}

} // namespace CTAG::DRIVERS
