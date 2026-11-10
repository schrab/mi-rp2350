/***************
CTAG TBD >>to be determined<< is an open source eurorack synthesizer module.
(c) 2020 by Robert Manzke. All rights reserved.
Licensed under GPL 3.0.
***************/

#include "BtMidiReceiver.hpp"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

extern "C" void ble_store_config_init(void);

using namespace CTAG::DRIVERS;

static const char *TAG = "BLE_MIDI";

// BLE MIDI service UUID: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
static const ble_uuid128_t MIDI_SVC_UUID = BLE_UUID128_INIT(
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03);
// Apple BLE-MIDI I/O characteristic UUID: 7772E5DB-3868-4112-A1A9-F2669D106BF3
static const ble_uuid128_t MIDI_CHR_UUID = BLE_UUID128_INIT(
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77);

// Ring buffer
#define BT_MIDI_RING_SZ 2048
static uint8_t ring[BT_MIDI_RING_SZ];
static volatile int ring_wr = 0;
static volatile int ring_rd = 0;

// Device list (discovered during scan)
#define MAX_DEVICES 16
static BtDeviceInfo devices[MAX_DEVICES];
static int deviceCount = 0;

// Connection state
static bool bt_connected = false;
static bool bt_scanning = false;
static bool bt_peer_probe = false;   // bounded reconnect probe scan in flight
static bool bt_probe_connect = false; // probe hit seen; connect after teardown
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;

// Characteristic value handle for MIDI data (for notification subscription)
static uint16_t midi_val_handle = 0;
static uint16_t midiSvcStart = 0;      // start handle of the MIDI service
static uint16_t midiSvcEnd = 0;        // end handle of the MIDI service (dsc scan)
static uint16_t midi_cccd_handle = 0;  // 0x2902 CCCD handle once discovered
static bool midi_cccd_written = false; // 0x2902 CCCD write landed

// Persistent peer (last successfully connected device): stored in NVS on
// connect, restored at host sync, retried from TickReconnect() until the
// peer answers. Explicit Disconnect() forgets it.
static int gap_event_cb(struct ble_gap_event *event, void *arg);  // fwd
static bool storedValid = false;
static uint8_t storedType = 0;
static uint8_t storedBda[6];
static bool bt_connecting = false;

static void storePeer(uint8_t type, const uint8_t *bda) {
    nvs_handle_t h;
    if (nvs_open("btmidi", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "atype", type);
    nvs_set_blob(h, "bda", bda, 6);
    nvs_commit(h);
    nvs_close(h);
    storedType = type;
    memcpy(storedBda, bda, 6);
    storedValid = true;
}

static void loadPeer() {
    nvs_handle_t h;
    if (nvs_open("btmidi", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t type = 0;
    size_t sz = sizeof(storedBda);
    if (nvs_get_u8(h, "atype", &type) == ESP_OK &&
        nvs_get_blob(h, "bda", storedBda, &sz) == ESP_OK) {
        storedType = type;
        storedValid = true;
    }
    nvs_close(h);
}

static void forgetPeer() {
    storedValid = false;
    nvs_handle_t h;
    if (nvs_open("btmidi", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "atype");
        nvs_erase_key(h, "bda");
        nvs_commit(h);
        nvs_close(h);
    }
}

static bool connectStored() {
    if (!storedValid || bt_connected || bt_connecting) return false;
    ble_addr_t addr;
    addr.type = storedType;
    memcpy(addr.val, storedBda, 6);
    // Clear any cached LTK for this peer before connecting so NimBLE's
    // ble_sm_sec_req_rx initiates a fresh Just-Works pairing
    // (ble_sm_pair_initiate -> first_pair_flag=1 on the SMC-PAD) instead of
    // an LTK encryption restore (LINK_ENCRYPTION_RECONNECT), after which the
    // stock SMC-PAD firmware stops sending MIDI notifications on power-cycle.
    ble_store_util_delete_peer(&addr);
    struct ble_gap_conn_params p;
    memset(&p, 0, sizeof(p));
    p.scan_itvl = 0x0010;
    p.scan_window = 0x0010;
    p.itvl_min = 0x0018;
    p.itvl_max = 0x0028;
    p.latency = 0;
    p.supervision_timeout = 0x00C8;
    // Short reconnect attempts with a half-duty connect scan: with the pad
    // off this loop runs forever, and a 100%-duty 30 s procedure starves
    // the link task past the DSP's panel-quiet timeout (DSP OFF, loads
    // refused). The pad either answers in ~1 s or it is not there.
    p.scan_itvl = 0x0020;
    p.scan_window = 0x0010;
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &addr, 3000, &p,
                             gap_event_cb, nullptr);
    if (rc == 0) {
        bt_connecting = true;
        ESP_LOGI(TAG, "auto-reconnect to stored BLE peer...");
    } else {
        ESP_LOGW(TAG, "auto-reconnect rejected: %d", rc);
    }
    return rc == 0;
}

static void ring_write(const uint8_t *data, int len) {
    for (int i = 0; i < len; i++) {
        int next = (ring_wr + 1) & (BT_MIDI_RING_SZ - 1);
        if (next == ring_rd) break;
        ring[ring_wr] = data[i];
        ring_wr = next;
    }
}

static int ring_read(uint8_t *buf, int max) {
    int n = 0;
    while (ring_rd != ring_wr && n < max) {
        buf[n++] = ring[ring_rd];
        ring_rd = (ring_rd + 1) & (BT_MIDI_RING_SZ - 1);
    }
    return n;
}

// Returns the expected number of data bytes for a given MIDI status byte.
// Returns -1 for SysEx start (0xF0), 0 for statuses with no data (0xF6, 0xF7, 0xF8..0xFF).
static inline int midi_status_data_len(uint8_t status) {
    if (status < 0x80) return 0;
    switch (status & 0xF0) {
        case 0x80: // Note Off
        case 0x90: // Note On
        case 0xA0: // Polyphonic Key Pressure
        case 0xB0: // Control Change
        case 0xE0: // Pitch Bend Change
            return 2;
        case 0xC0: // Program Change
        case 0xD0: // Channel Pressure
            return 1;
        case 0xF0:
            switch (status) {
                case 0xF1: // MTC Quarter Frame
                case 0xF3: // Song Select
                    return 1;
                case 0xF2: // Song Position Pointer
                    return 2;
                case 0xF0: // SysEx Start
                    return -1;
                default:   // 0xF4..0xF7, 0xF8..0xFF
                    return 0;
            }
        default:
            return 0;
    }
}

// Parse Apple / MMA BLE-MIDI notification data.
// Packet layout:
//   [Header: 0b1xxxxxxx]
// Followed by one or more events, each preceded by:
//   [Timestamp Low: 0b1xxxxxxx] (1 byte!)
// followed by either:
//   - Explicit status [Status: 0b1xxxxxxx] + [Data...]
//   - Running status [Data 1] + [Data 2] ...
// System Real-Time bytes (0xF8..0xFF) can be interleaved anywhere.
static void parse_ble_midi_notify(const uint8_t *data, int len) {
    if (len < 3) return;
    if ((data[0] & 0x80) == 0) return; // Header must have bit 7 set

    int i = 1; // data[0] is Header; data[1] is the first Timestamp Low
    uint8_t running_status = 0;

    while (i < len) {
        uint8_t b = data[i];

        // System Real-Time bytes (0xF8..0xFF) can appear anywhere (1 byte)
        if (b >= 0xF8) {
            ring_write(&b, 1);
            i++;
            continue;
        }

        // At a message boundary, data[i] is a Timestamp Low byte (bit 7 set)
        if (b & 0x80) {
            // Skip the 1-byte Timestamp Low
            i++;
            if (i >= len) break;

            // Handle any Real-Time bytes immediately following timestamp
            while (i < len && data[i] >= 0xF8) {
                uint8_t rt = data[i++];
                ring_write(&rt, 1);
            }
            if (i >= len) break;

            b = data[i];
            if (b & 0x80) {
                // Explicit MIDI Status byte
                running_status = (b < 0xF0) ? b : 0;
                ring_write(&b, 1);
                i++;

                // SysEx
                if (b == 0xF0) {
                    while (i < len) {
                        uint8_t sb = data[i++];
                        if (sb >= 0xF8) {
                            ring_write(&sb, 1);
                            continue;
                        }
                        if (sb & 0x80) {
                            if (sb == 0xF7) ring_write(&sb, 1);
                            else i--; // Next timestamp; put back
                            break;
                        }
                        ring_write(&sb, 1);
                    }
                    continue;
                }

                int expected = midi_status_data_len(b);
                while (expected > 0 && i < len) {
                    uint8_t db = data[i++];
                    if (db >= 0xF8) {
                        ring_write(&db, 1);
                        continue;
                    }
                    if (db & 0x80) {
                        // Unexpected status or timestamp; put back to recover
                        i--;
                        break;
                    }
                    ring_write(&db, 1);
                    expected--;
                }
            } else {
                // Running status applies to data byte following timestamp
                if (running_status != 0) {
                    ring_write(&running_status, 1);
                    int expected = midi_status_data_len(running_status);
                    while (expected > 0 && i < len) {
                        uint8_t db = data[i++];
                        if (db >= 0xF8) {
                            ring_write(&db, 1);
                            continue;
                        }
                        if (db & 0x80) {
                            i--;
                            break;
                        }
                        ring_write(&db, 1);
                        expected--;
                    }
                } else {
                    i++; // Unknown status: skip orphan data byte
                }
            }
        } else {
            // Unexpected data byte outside message boundary
            i++;
        }
    }
}

static int on_svc_discovered(uint16_t conn,
                              const struct ble_gatt_error *error,
                              const struct ble_gatt_svc *svc,
                              void *arg);

static int on_chr_discovered(uint16_t conn,
                              const struct ble_gatt_error *error,
                              const struct ble_gatt_chr *chr,
                              void *arg);

static int on_dsc_discovered(uint16_t conn,
                             const struct ble_gatt_error *error,
                             uint16_t chr_val_handle,
                             const struct ble_gatt_dsc *dsc,
                             void *arg);

static int midi_read_cb(uint16_t conn, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg) {
    if (error && error->status != 0) {
        ESP_LOGW(TAG, "midi chr read FAILED: %d", error->status);
    }
    if (midi_val_handle != 0) {
        const uint8_t ka[3] = {0x80, 0x80, 0xF8};
        ble_gattc_write_no_rsp_flat(conn, midi_val_handle, ka, sizeof(ka));
    }
    return 0;
}

static int cccd_write_cb(uint16_t conn, const struct ble_gatt_error *error,
                         struct ble_gatt_attr *attr, void *arg) {
    if (error && error->status != 0) {
        ESP_LOGE(TAG, "CCCD write REJECTED at ATT level: %d", error->status);
    } else {
        midi_cccd_written = true;
        ESP_LOGI(TAG, "CCCD write accepted — notifications enabled");
        // Apple BLE-MIDI spec: Central shall read the MIDI I/O characteristic
        // upon establishing a connection. Chained after CCCD write so only
        // one GATT procedure is in flight at a time.
        if (midi_val_handle != 0) {
            ble_gattc_read(conn, midi_val_handle, midi_read_cb, nullptr);
        }
    }
    return 0;
}

static int on_mtu_exchanged(uint16_t conn,
                            const struct ble_gatt_error *error,
                            uint16_t mtu,
                            void *arg) {
    ESP_LOGI(TAG, "MTU exchange complete: status=%d mtu=%u, discovering services",
             error ? error->status : -1, (unsigned) mtu);
    int rc = ble_gattc_disc_all_svcs(conn, on_svc_discovered, nullptr);
    if (rc != 0) ESP_LOGE(TAG, "Failed to start svc discovery: %d", rc);
    return 0;
}

static int on_svc_discovered(uint16_t conn,
                              const struct ble_gatt_error *error,
                              const struct ble_gatt_svc *svc,
                              void *arg) {
    if (error->status == 0 && svc != nullptr) {
        char uuid_str[37];
        ble_uuid_to_str(&svc->uuid.u, uuid_str);
        ESP_LOGI(TAG, "svc %s (%u-%u)", uuid_str, svc->start_handle, svc->end_handle);
        if (ble_uuid_cmp(&svc->uuid.u, &MIDI_SVC_UUID.u) == 0) {
            ESP_LOGI(TAG, "Found MIDI service (%u-%u)",
                     svc->start_handle, svc->end_handle);
            midiSvcStart = svc->start_handle;
            midiSvcEnd = svc->end_handle;
        }
    } else if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Service discovery complete");
        if (midiSvcStart != 0 && midiSvcEnd != 0) {
            int rc = ble_gattc_disc_all_chrs(conn, midiSvcStart, midiSvcEnd,
                                             on_chr_discovered, nullptr);
            if (rc != 0) ESP_LOGE(TAG, "Failed to start chr discovery: %d", rc);
        }
    } else {
        ESP_LOGE(TAG, "Service discovery error: %d", error->status);
    }
    return 0;
}

static int on_chr_discovered(uint16_t conn,
                              const struct ble_gatt_error *error,
                              const struct ble_gatt_chr *chr,
                              void *arg) {
    if (error->status == 0 && chr != nullptr) {
        char uuid_str[37];
        ble_uuid_to_str(&chr->uuid.u, uuid_str);
        // Keep this visible: the CCCD subscribe depends on finding this
        // characteristic and its 0x2902 descriptor.
        ESP_LOGI(TAG, "chr %s def=%u val=%u props=0x%02x",
                 uuid_str, chr->def_handle, chr->val_handle, chr->properties);
        // Match the Apple BLE-MIDI characteristic UUID; fall back to the
        // first (notify-capable) characteristic in the MIDI service.
        if (ble_uuid_cmp(&chr->uuid.u, &MIDI_CHR_UUID.u) == 0) {
            ESP_LOGI(TAG, "Found Apple BLE-MIDI characteristic");
        } else if (midi_val_handle == 0 && (chr->properties & BLE_GATT_CHR_F_NOTIFY)) {
            ESP_LOGI(TAG, "Using device-specific MIDI characteristic");
        } else {
            return 0; // already subscribed
        }
        midi_val_handle = chr->val_handle;
    } else if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Characteristic discovery complete");
        if (midi_val_handle != 0) {
            // Discover the 0x2902 CCCD descriptor and write it with a
            // RELIABLE write — write-without-response CCCD pokes are
            // silently ignored by some peripherals (observed on M-VAVE).
            int rc = ble_gattc_disc_all_dscs(conn, midi_val_handle, midiSvcEnd,
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
                             const struct ble_gatt_dsc *dsc,
                             void *arg) {
    if (error->status == 0 && dsc != nullptr) {
        char uuid_str[37];
        ble_uuid_to_str(&dsc->uuid.u, uuid_str);
        ESP_LOGI(TAG, "dsc 0x%04x %s", dsc->handle, uuid_str);
        if (dsc->uuid.u.type == BLE_UUID_TYPE_16 &&
            dsc->uuid.u16.value == 0x2902 && midi_cccd_handle == 0) {
            midi_cccd_handle = dsc->handle;
            uint8_t val[2] = {0x01, 0x00};   // notify enable, little-endian
            // Reliable write WITH response — write-without-response CCCD
            // pokes are silently ignored by some peripherals (M-VAVE).
            // The callback surfaces the ATT-level status: rc=0 only means
            // the procedure started.
            struct os_mbuf *om = ble_hs_mbuf_from_flat(val, sizeof(val));
            int rc = om ? ble_gattc_write(conn, dsc->handle, om,
                                          cccd_write_cb, nullptr) : BLE_HS_ENOMEM;
            ESP_LOGI(TAG, "CCCD 0x%04x write started rc=%d", dsc->handle, rc);
        }
    } else if (error->status == BLE_HS_EDONE) {
        if (midi_cccd_handle == 0 && midi_val_handle != 0) {
            // No 0x2902 found in range — legacy fallback: CCCD at val+1.
            midi_cccd_handle = midi_val_handle + 1;
            uint8_t val[2] = {0x01, 0x00};
            struct os_mbuf *om = ble_hs_mbuf_from_flat(val, sizeof(val));
            int rc = om ? ble_gattc_write(conn, midi_val_handle + 1, om,
                                          cccd_write_cb, nullptr) : BLE_HS_ENOMEM;
            ESP_LOGW(TAG, "no 0x2902 dsc; fallback CCCD val+1 write started rc=%d", rc);
        }
        ESP_LOGI(TAG, "Descriptor discovery complete");
    } else {
        ESP_LOGE(TAG, "Descriptor discovery error: %d", error->status);
    }
    return 0;
}

static int gap_event_cb(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            bt_connected = true;
            bt_connecting = false;
            bt_scanning = false;
            ESP_LOGI(TAG, "Connected, exchanging MTU");
            // Remember the peer for persistent reconnection.
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->connect.conn_handle, &desc) == 0)
                storePeer(desc.peer_id_addr.type, desc.peer_id_addr.val);
            int rc = ble_gattc_exchange_mtu(conn_handle, on_mtu_exchanged, nullptr);
            if (rc != 0) {
                ESP_LOGW(TAG, "MTU exchange start rc=%d, discovering services", rc);
                ble_gattc_disc_all_svcs(conn_handle, on_svc_discovered, nullptr);
            }
        } else {
            ESP_LOGE(TAG, "Connection failed: %d", event->connect.status);
            bt_connected = false;
            bt_connecting = false;   // TickReconnect() will retry
        }
        return 0;
    }
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        bt_connected = false;
        bt_connecting = false;       // stored peer (if any) auto-reconnects
        midi_val_handle = 0;
        midiSvcStart = 0;
        midiSvcEnd = 0;
        midi_cccd_handle = 0;
        midi_cccd_written = false;
        return 0;
    case BLE_GAP_EVENT_DISC: {
        const struct ble_gap_disc_desc &disc = event->disc;
        // Peer probe hit: the stored pad is advertising again — stop the
        // probe and connect; the procedure completes immediately because
        // the peer is really there. The pad must advertise its identity
        // address (it does — the old blind reconnect by stored address
        // only worked under that same assumption).
        if (bt_peer_probe && storedValid && !bt_connected && !bt_connecting &&
            disc.addr.type == storedType &&
            memcmp(disc.addr.val, storedBda, 6) == 0) {
            bt_peer_probe = false;
            // The disc teardown completes asynchronously (DISC_COMPLETE fires
            // after this callback) — connecting here races it and the first
            // attempt is rejected with BLE_HS_EBUSY. Defer one event — unless
            // the scan already ended on its own (cancel fails: no event will
            // come), in which case connect right away.
            if (ble_gap_disc_cancel() == 0) {
                bt_probe_connect = true;
            } else {
                connectStored();
            }
            return 0;
        }
        if (bt_peer_probe) return 0;   // probe results stay out of the UI list
        // Manually parse AD data for device name (type 0x08=short, 0x09=complete)
        char nameBuf[32];
        const char *advName = nullptr;
        {
            int off = 0;
            const uint8_t *d = disc.data;
            int dlen = disc.length_data;
            while (off + 1 < dlen) {
                int fieldLen = d[off];
                if (fieldLen == 0 || off + 1 + fieldLen > dlen) break;
                int type = d[off + 1];
                if (type == 0x08 || type == 0x09) {
                    int nameLen = fieldLen - 1;
                    if (nameLen > 31) nameLen = 31;
                    memcpy(nameBuf, &d[off + 2], nameLen);
                    nameBuf[nameLen] = '\0';
                    advName = nameBuf;
                    break;
                }
                off += 1 + fieldLen;
            }
        }
        // Deduplicate: check if address already in list
        for (int i = 0; i < deviceCount; i++) {
            if (memcmp(devices[i].bda, disc.addr.val, 6) == 0) {
                // Update name from scan response (may carry name not in adv)
                if (advName) {
                    int copyLen = strlen(advName) < 31 ? strlen(advName) : 31;
                    memcpy(devices[i].name, advName, copyLen);
                    devices[i].name[copyLen] = '\0';
                }
                return 0;
            }
        }
        // New device
        if (deviceCount >= MAX_DEVICES) return 0;
        BtDeviceInfo &d = devices[deviceCount];
        if (advName) {
            int copyLen = strlen(advName) < 31 ? strlen(advName) : 31;
            memcpy(d.name, advName, copyLen);
            d.name[copyLen] = '\0';
        } else {
            snprintf(d.name, sizeof(d.name), "BLE-%02X%02X%02X%02X%02X%02X",
                     disc.addr.val[5], disc.addr.val[4], disc.addr.val[3],
                     disc.addr.val[2], disc.addr.val[1], disc.addr.val[0]);
        }
        memcpy(d.bda, disc.addr.val, 6);
        deviceCount++;
        return 0;
    }
    case BLE_GAP_EVENT_DISC_COMPLETE:
        bt_peer_probe = false;
        if (bt_probe_connect) {
            // Probe hit + scan teardown finished — now the connect lands.
            bt_probe_connect = false;
            connectStored();
        }
        if (bt_scanning) {
            ESP_LOGI(TAG, "Scan complete, %d devices found", deviceCount);
            bt_scanning = false;
        }
        return 0;
    case BLE_GAP_EVENT_NOTIFY_RX: {
        const struct os_mbuf *om = event->notify_rx.om;
        if (om && om->om_len > 0) {
            parse_ble_midi_notify(om->om_data, om->om_len);
        }
        return 0;
    }
    case BLE_GAP_EVENT_PASSKEY_ACTION: {
        // The SMC-PAD initiates pairing right after discovery; ignoring it
        // stalls its stack and notifications never start. Auto-accept
        // numeric comparison (Just Works on IO-cap-none devices) and pass
        // the rest through so the procedure completes.
        struct ble_sm_io pkey;
        memset(&pkey, 0, sizeof(pkey));
        pkey.action = event->passkey.params.action;
        if (pkey.action == BLE_SM_IOACT_NUMCMP) pkey.numcmp_accept = 1;
        int rc = ble_sm_inject_io(event->passkey.conn_handle, &pkey);
        ESP_LOGI(TAG, "passkey action %d injected rc=%d", pkey.action, rc);
        return 0;
    }
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        struct ble_gap_conn_desc desc;
        if (ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc) == 0) {
            ble_store_util_delete_peer(&desc.peer_id_addr);
        }
        ESP_LOGI(TAG, "repeat pairing — deleted old bond, retrying");
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    case BLE_GAP_EVENT_ENC_CHANGE:
        // Pairing/encryption settled: pairing resets CCCD state, so
        // re-subscribe once the connection is encrypted.
        ESP_LOGI(TAG, "enc change: status=%d", event->enc_change.status);
        if (event->enc_change.status == 0 && midi_val_handle != 0 &&
            midi_cccd_handle != 0) {
            uint8_t val[2] = {0x01, 0x00};
            struct os_mbuf *om = ble_hs_mbuf_from_flat(val, sizeof(val));
            int rc = om ? ble_gattc_write(conn_handle, midi_cccd_handle, om,
                                          cccd_write_cb, nullptr) : BLE_HS_ENOMEM;
            ESP_LOGI(TAG, "CCCD re-subscribe after pairing rc=%d", rc);
        }
        return 0;
    case BLE_GAP_EVENT_L2CAP_UPDATE_REQ:
        if (event->conn_update_req.peer_params != nullptr) {
            const auto *pp = event->conn_update_req.peer_params;
            ESP_LOGI(TAG, "L2CAP conn update req: itvl=%u..%u latency=%u timeout=%u",
                     (unsigned) pp->itvl_min, (unsigned) pp->itvl_max,
                     (unsigned) pp->latency, (unsigned) pp->supervision_timeout);
        }
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

static void on_sync(void) {
    ESP_LOGI(TAG, "NimBLE host synced with controller");
    // Restore the persistent peer as soon as the host is ready, but do NOT
    // blind-connect here: with the pad off that is one more connect into
    // the void (TickReconnect's probe handles the reconnection).
    loadPeer();
}

static void host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void BtMidiReceiver::Init() {
    // The precompiled C3 BT controller needs the ECO3 ROM BT helpers
    // (chip revision >= 3, i.e. v0.3+ — this bench's v0.4 qualifies).
    // IDF 5.5.4 links esp32c3.rom.eco3*_bt_funcs.ld only when REV_MIN >= 3,
    // so boards older than v0.3 cannot run BLE here; skip them cleanly.
    esp_chip_info_t ci;
    esp_chip_info(&ci);
    if (ci.revision < 3) {
        ESP_LOGW(TAG, "BLE-MIDI unavailable: chip rev %u < 3 (ECO3 ROM required)",
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
    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("CTAG-TBD");
    ble_store_config_init();
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    nimble_port_freertos_init(host_task);

    ESP_LOGI(TAG, "BLE MIDI initialized (central mode)");
}

void BtMidiReceiver::Read(uint8_t *buf, uint32_t *len) {
    *len = (uint32_t)ring_read(buf, BT_MIDI_RING_SZ);
}

// Called from the midi relay task (~10 ms cadence): every 3 s, run a
// bounded 2 s advertisement probe for the stored peer. The pad only
// advertises while disconnected, so the probe is the presence detector:
// with the pad off nothing is heard and no procedure is ever pending
// (blind connect attempts into the void were starving the single core
// and taking the board down). Seeing the pad triggers one connect that
// completes immediately.
void BtMidiReceiver::TickReconnect() {
    static uint32_t lastTry = 0;
    // A probe hit deferred its connect but the DISC_COMPLETE event was lost
    // (scan expired in the same instant) — recover here.
    if (bt_probe_connect && !bt_connected && !bt_connecting) {
        bt_probe_connect = false;
        connectStored();
        return;
    }
    if (!storedValid || bt_connected || bt_connecting || bt_scanning) return;
    uint32_t now = xTaskGetTickCount();
    if (now - lastTry < pdMS_TO_TICKS(3000)) return;
    lastTry = now;
    struct ble_gap_disc_params params = {};
    params.itvl = 0x0020;          // 20 ms interval / 10 ms window: half-duty,
    params.window = 0x0010;        // the probe must not hog the single core
    params.filter_policy = BLE_HCI_CONN_FILT_NO_WL;
    bt_peer_probe = true;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, 2000, &params, gap_event_cb, nullptr);
    if (rc != 0) {
        bt_peer_probe = false;
        if (rc != BLE_HS_EBUSY) ESP_LOGW(TAG, "peer probe scan failed: %d", rc);
    }
}

// Client → pad write-without-response on the MIDI characteristic
// (props=0x16 supports WRITE_NO_RSP, not WRITE).
void BtMidiReceiver::SendKeepalive(const uint8_t *data, uint32_t len) {
    if (!bt_connected || !midi_cccd_written || midi_val_handle == 0) return;
    ble_gattc_write_no_rsp_flat(conn_handle, midi_val_handle, data,
                                (uint16_t) len);
}

bool BtMidiReceiver::IsConnected() { return bt_connected; }
bool BtMidiReceiver::IsScanning() { return bt_scanning; }

int BtMidiReceiver::GetDeviceCount() { return deviceCount; }

const BtDeviceInfo* BtMidiReceiver::GetDevice(int idx) {
    if (idx < 0 || idx >= deviceCount) return nullptr;
    return &devices[idx];
}

void BtMidiReceiver::StartScan() {
    // Scanning while connected (or mid-connect) is allowed: drop the
    // current connection first. The stored peer is kept — a new successful
    // connection replaces it.
    if (bt_connecting) ble_gap_conn_cancel();
    if (bt_connected)
        ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    bt_connected = false;
    bt_connecting = false;
    bt_peer_probe = false;
    bt_scanning = true;
    deviceCount = 0;
    struct ble_gap_disc_params params = {};
    params.filter_policy = BLE_HCI_CONN_FILT_NO_WL;
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &params, gap_event_cb, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "Scan start failed: %d", rc);
        bt_scanning = false;
    } else {
        ESP_LOGI(TAG, "Scanning for BLE MIDI devices...");
    }
}

void BtMidiReceiver::StopScan() {
    if (!bt_scanning) return;
    ble_gap_disc_cancel();
    bt_scanning = false;
}

void BtMidiReceiver::Connect(int idx) {
    if (bt_connected || idx < 0 || idx >= deviceCount) return;
    StopScan();
    ble_addr_t addr;
    addr.type = BLE_ADDR_PUBLIC;
    memcpy(addr.val, devices[idx].bda, 6);
    ble_store_util_delete_peer(&addr);
    struct ble_gap_conn_params params;
    memset(&params, 0, sizeof(params));
    params.scan_itvl = 0x0010;
    params.scan_window = 0x0010;
    params.itvl_min = 0x0018;
    params.itvl_max = 0x0028;
    params.latency = 0;
    params.supervision_timeout = 0x00C8;
    params.min_ce_len = 0;
    params.max_ce_len = 0;
    int rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &addr, 30000, &params, gap_event_cb, nullptr);
    if (rc != 0) {
        ESP_LOGE(TAG, "Connect failed: %d", rc);
    } else {
        ESP_LOGI(TAG, "Connecting to %s...", devices[idx].name);
    }
}

void BtMidiReceiver::Disconnect() {
    if (!bt_connected) return;
    forgetPeer();   // explicit disconnect = forget the peer
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
}
