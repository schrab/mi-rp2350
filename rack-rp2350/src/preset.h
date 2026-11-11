// Rack RP2350 — T4a preset serialization + apply + autosave + console bench.
//
// OWNERSHIP (the plan's Presets section, decisions final): the RP2350 OWNS
// serialization + apply (it holds the state); the C3 owns the flash bank
// (its own 4 MB flash — RP2350 flash erase/program would stall XIP, i.e.
// audio, so saves never touch this chip). One preset = full rack state:
//
//   RP2350 part (this blob, 871 B): active module, chain pair/engage,
//     clock bus (global source + BPM), the mod engine's 10 persistent
//     source records (mod_engine_preset_write), and ALL FOUR modules'
//     observed CC values + a "seen" bitmap (which CCs a controller ever
//     sent — cache zeros are ambiguous). Binary, versioned, crc16'd.
//   C3 part (PresetBank, opaque to us): UI-local state (last page, slot).
//
// WIRE (docs/ui-protocol.md §2, PRESET_* — T4a semantics):
//   PRESET_REQ  C3->RP2350 [slot u8]      "give me your current part"
//               (slot 0xFF = the debounced autosave push; otherwise the
//               slot the user is saving into — echoed in PART for context)
//   PRESET_PART RP2350->C3 [slot][seq][total][data <=252]  chunked burst
//               (unsolicited with slot=0xFF from the autosave debounce)
//   PRESET_APPLY C3->RP2350 [seq][total][data <=252]       chunked blob
//               (C3 read a bank slot; the RP2350 validates + applies and
//               acks with a full TELEM snapshot)
//
// APPLY ORDER (core 0; module/chain land via the normal pending machinery):
//   clock bus + mod sources -> CC cache + seen -> module swap if the
//   preset's module differs (core 1's swap replays the cache after Init)
//   -> chain engage/disengage if the preset's chain state differs.
// A same-module load touches no audio path; a module/chain change mutes for
// the usual 20-44 ms swap window (the accepted artifact).
//
// REPLAY: preset_replay_module(mod) re-sends every SEEN CC of a module
// through its control_change hook — called after EVERY factory re-Init
// (swap, chain engage/leave, A re-init), which also retires the old
// "factory defaults on swap desync the panel" debt. Skips: CC 64 (momentary
// PLAY/TRIG/STRUM pulses must not fire from a stored latch value) and the
// chained-rings poly clamp (budget safety). Runs on CORE 1 inside the swap
// sequences — the adapters' control_change hooks are print-free stores.
//
// CONSOLE (bench phase per the plan; the C3 bank wraps this later):
//   preset            status (version, size, dirty, module/chain/bpm summary)
//   preset dump       the blob as "PRxx <48 hex>" rows (24 B per row)
//   preset load       collect rows, validate, apply
//   preset apply      re-apply the last collected blob
//   preset reload     apply the last collected blob WITHOUT re-collecting
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "mod_engine.h"   // g_mod_engine + preset write/read + record size

// ---- forward decls (definitions later in main.cpp's TU, after this
// header — cmd_chain is defined below; the swap/chain init sites above it
// call replay through these same decls) ----
static void cmd_chain(const char* args);

// ---- format ----
static const uint8_t kPresetMagic[4] = {'R', 'K', 'P', 'R'};
static const uint8_t kPresetVersion = 1;
//  [0..3]   magic "RKPR"
//  [4]      version
//  [5]      active module id
//  [6]      chain active (0/1)
//  [7]      chain pair index (1 elements->clouds / 2 plaits / 3 rings)
//  [8..292]  mod-engine record (285 B: global_clock + bpm f32 + 10x28)
//  [293..356] cc_seen[4][16] (64 B)
//  [357..868] cc_cache[4][128] (512 B)
//  [869..870] crc16-CCITT LE over [0..868]
// Derived from the real record sizes — a hardcoded 301 here once assumed a
// 293-byte mod record that mod_engine_preset_write never produced (285 B:
// 1 + 4 + 10x28), shifting the cache/crc down and leaving stack garbage in
// bytes 871-878 of every dump (bench-caught 2026-11-11).
static const uint16_t kPresetOffMod = 8;
static const uint16_t kPresetModLen =
    5 + MOD_SOURCE_COUNT * kModPresetSrcRecord;                     // 285
static const uint16_t kPresetOffSeen =
    kPresetOffMod + kPresetModLen;                                  // 293
static const uint16_t kPresetOffCache =
    kPresetOffSeen + RACK_MODULE_COUNT * 16;                        // 357
static const uint16_t kPresetOffCrc =
    kPresetOffCache + RACK_MODULE_COUNT * 128;                      // 869
static const uint16_t kPresetSize = kPresetOffCrc + 2;              // 871
static const uint16_t kPresetBlobMax = 1024;
static const uint8_t kPresetChunkData = 252;   // PART payload data cap

// CRC-16/CCITT (poly 0x1021, init 0xFFFF, MSB-first, no xorout) — only
// self-consistency matters (both ends are this same header).
static uint16_t preset_crc16(const uint8_t* d, uint16_t n) {
  uint16_t c = 0xFFFF;
  for (uint16_t i = 0; i < n; ++i) {
    c ^= (uint16_t)(d[i] << 8);
    for (uint8_t b = 0; b < 8; ++b)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

// Chain pair index <-> A-slot module id (b is clouds in every pair).
static const uint8_t kPresetPairA[4] = {0, 0, 2, 3};  // [1]=elements [2]=plaits [3]=rings

// ===================== serialize =====================
static uint16_t preset_serialize(uint8_t* buf) {
  uint8_t* w = buf;
  memcpy(w, kPresetMagic, 4);
  w[4] = kPresetVersion;
  w[5] = g_active_module < RACK_MODULE_COUNT ? g_active_module : 0;
  w[6] = g_chain.active ? 1 : 0;
  uint8_t pair = 0;
  if (g_chain.active) {
    for (uint8_t p = 1; p <= 3; ++p)
      if (kPresetPairA[p] == g_chain.a) pair = p;
  }
  w[7] = pair;
  w += kPresetOffMod;
  w += mod_engine_preset_write(w);            // 293 B (clock + sources)
  memcpy(w, g_ui_cc_seen, RACK_MODULE_COUNT * 16);
  w += kPresetOffCache - kPresetOffSeen;
  memcpy(w, g_ui_cc_cache, RACK_MODULE_COUNT * 128);
  w += kPresetOffCrc - kPresetOffCache;
  const uint16_t crc = preset_crc16(buf, kPresetOffCrc);
  memcpy(w, &crc, 2);                          // u16 LE
  return kPresetSize;
}

// ===================== validate =====================
static const char* preset_validate(const uint8_t* buf, uint16_t len,
                                   char* why, size_t whyn) {
  if (len != kPresetSize) {
    snprintf(why, whyn, "len %u != %u", len, kPresetSize);
    return nullptr;
  }
  if (memcmp(buf, kPresetMagic, 4) != 0) {
    snprintf(why, whyn, "bad magic");
    return nullptr;
  }
  if (buf[4] != kPresetVersion) {
    snprintf(why, whyn, "version %u != %u (blob too old/new)", buf[4],
             kPresetVersion);
    return nullptr;
  }
  uint16_t crc;
  memcpy(&crc, buf + kPresetOffCrc, 2);
  const uint16_t calc = preset_crc16(buf, kPresetOffCrc);
  if (crc != calc) {
    snprintf(why, whyn, "crc %04x != %04x (corrupt)", crc, calc);
    return nullptr;
  }
  return "OK";
}

// ===================== replay (core 1, after every factory re-Init) ======
// Declared before the swap/chain sequences (main.cpp calls them there); the
// definition lives below the cache externs' visibility point in this TU.
void preset_replay_module(uint8_t mod);

// ===================== apply =====================
// Applies a VALIDATED blob. Called from the PRESET_APPLY RX path and the
// console loader. Core 0 context (same discipline as the CC dispatch).
static void preset_apply_blob(const uint8_t* buf) {
  const uint8_t mod = buf[5] < RACK_MODULE_COUNT ? buf[5] : 0;
  const bool chain_active = buf[6] != 0;
  uint8_t pair = buf[7];
  if (pair < 1 || pair > 3) pair = 1;

  // (1) CC cache + seen + mod engine / clock bus.
  memcpy(g_ui_cc_seen, buf + kPresetOffSeen, RACK_MODULE_COUNT * 16);
  memcpy(g_ui_cc_cache, buf + kPresetOffCache, RACK_MODULE_COUNT * 128);
  mod_engine_preset_read(buf + kPresetOffMod);
  Serial.printf("preset: applied clock (%s, %lu.%lu bpm) + 10 mod sources + "
                "4-module CC cache\r\n",
                g_mod_engine.global_clock ? "midi" : "internal",
                (unsigned long)(int)g_mod_engine.bpm,
                (unsigned long)(((int)(g_mod_engine.bpm * 10.0f)) % 10));

  // (2) Chain state first (the engage path re-Inits BOTH slots — their
  // replay below restores the params; a module swap here would race it).
  const bool want_chain = chain_active;
  if (want_chain) {
    const uint8_t a = kPresetPairA[pair];
    if (!g_chain.active || g_chain.a != a) {
      g_pending_chain_a = static_cast<int8_t>(a);
      g_pending_chain_b = static_cast<int8_t>(RACK_MODULE_CLOUDS);
      g_pending_chain = 1;
      Serial.printf("preset: chain %s->clouds queued (pair %u)\r\n",
                    kRackModules[a].name, pair);
    }
  } else if (g_chain.active) {
    g_pending_chain = 0;
    Serial.println("preset: chain off queued");
  }

  // (3) Module swap (single-module only; while chained the A slot IS the
  // active module and step (2) re-Inits it).
  if (!want_chain && !g_chain.active && mod != g_active_module) {
    g_pending_module = static_cast<int8_t>(mod);   // core 1 swaps + replays
    Serial.printf("preset: module swap to %s queued\r\n",
                  kRackModules[mod].name);
  } else if (!want_chain && !g_chain.active) {
    // Same module, no chain change: no swap will run — replay directly.
    preset_replay_module(g_active_module);
    Serial.println("preset: CC cache replayed (no swap needed)");
  }
}

// ---- definition (the cache statics above are visible here) ----
void preset_replay_module(uint8_t mod) {
  if (mod >= RACK_MODULE_COUNT) return;
  for (uint16_t cc = 0; cc < 128; ++cc) {
    const uint8_t val = g_ui_cc_cache[mod][cc];
    if (!(g_ui_cc_seen[mod][cc >> 4] & (1u << (cc & 15)))) continue;
    if (cc == 64) continue;              // momentary pulse, never replayed
    if (mod == RACK_MODULE_RINGS &&
        rack_modules::mod_rings::chain_rejects_cc((uint8_t)cc, val)) {
      continue;                          // chained-rings budget clamp
    }
    if (mod == RACK_MODULE_CLOUDS && cc >= 40 && cc <= 50) {
      if (cc <= 48) {
        kRackModules[mod].control_change(static_cast<uint8_t>(cc - 40 + 14), val);
      } else if (cc == 49) {
        kRackModules[mod].control_change(23, val);
      } else if (cc == 50) {
        kRackModules[mod].control_change(24, val);
      }
      continue;
    }
    kRackModules[mod].control_change((uint8_t)cc, val);
  }
}

// ===================== PRESET_PART burst (RP2350 -> C3) ==================
// Serializes the CURRENT state and sends it as [slot][seq][total][data]
// chunks. slot: the bank slot the C3 is saving into, or 0xFF = autosave.
void ui_preset_req_reply(uint8_t slot) {
  static uint8_t blob[kPresetBlobMax];
  const uint16_t n = preset_serialize(blob);
  const uint8_t total = (uint8_t)((n + kPresetChunkData - 1) / kPresetChunkData);
  for (uint8_t seq = 0; seq < total; ++seq) {
    uint8_t pl[3 + kPresetChunkData];
    pl[0] = slot;
    pl[1] = seq;
    pl[2] = total;
    const uint16_t off = (uint16_t)(seq * kPresetChunkData);
    uint16_t take = (uint16_t)(n - off);
    if (take > kPresetChunkData) take = kPresetChunkData;
    memcpy(pl + 3, blob + off, take);
    ui_link_send_frame(kUiTypePresetPart, pl, (uint8_t)(3 + take));
  }
  Serial.printf("preset: PART burst sent (slot 0x%02x, %u B, %u chunks)\r\n",
                slot, n, total);
}

// ===================== PRESET_APPLY reassembly (C3 -> RP2350) ============
static uint8_t g_preset_rx[kPresetBlobMax];
static uint8_t g_preset_rx_total = 0;
static uint8_t g_preset_rx_have = 0;

void ui_preset_apply_chunk(const uint8_t* pl, uint8_t len) {
  if (len < 2) {
    Serial.println("preset: APPLY chunk too short");
    return;
  }
  const uint8_t seq = pl[0], total = pl[1];
  if (total == 0 || total > 8 || seq >= total) {
    Serial.printf("preset: APPLY chunk bad seq/total %u/%u - dropped\r\n",
                  seq, total);
    g_preset_rx_total = 0;
    g_preset_rx_have = 0;
    return;
  }
  if (total != g_preset_rx_total) {          // new transfer
    g_preset_rx_total = total;
    g_preset_rx_have = 0;
  }
  if (seq != g_preset_rx_have) {             // hole: restart (UART is in-order;
    Serial.printf("preset: APPLY hole (want %u got %u) - restarting\r\n",
                  g_preset_rx_have, seq);    // a restart is the only failure
    g_preset_rx_have = 0;
    if (seq != 0) return;
  }
  const uint8_t take = (uint8_t)(len - 2);
  if ((uint32_t)seq * kPresetChunkData + take > kPresetBlobMax) {
    Serial.println("preset: APPLY overflow - dropped");
    g_preset_rx_total = 0;
    g_preset_rx_have = 0;
    return;
  }
  memcpy(g_preset_rx + (uint32_t)seq * kPresetChunkData, pl + 2, take);
  ++g_preset_rx_have;
  if (g_preset_rx_have != g_preset_rx_total) return;

  // Complete: validate + apply + ack with a full TELEM snapshot.
  g_preset_rx_total = 0;
  g_preset_rx_have = 0;
  char why[64];
  if (!preset_validate(g_preset_rx, kPresetSize, why, sizeof(why))) {
    Serial.printf("preset: APPLY REJECTED (%s)\r\n", why);
    return;
  }
  preset_apply_blob(g_preset_rx);
  Serial.println("preset: APPLY done (bank load) - full TELEM acked");
  ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
}

// ===================== autosave (core 0 loop, debounced) =================
void preset_autosave_poll() {
  if (!g_preset_dirty) return;
  if (millis() - g_preset_dirty_ms < 3000) return;
  g_preset_dirty = false;                    // settled — push once
  ui_preset_req_reply(0xFF);
}

// ===================== console (bench phase) =============================
static uint8_t g_preset_collect[kPresetBlobMax];
static uint16_t g_preset_collect_have = 0;
static bool g_preset_collecting = false;

// Row form: "PRxx:<hex>" — xx = row index, 24 bytes (48 hex) per row except
// the last. Consumed BEFORE normal command dispatch while collecting.
bool preset_collecting() { return g_preset_collecting; }

static int hexval(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool preset_collect_line(const char* line) {
  if (strncmp(line, "PR", 2) != 0) return false;
  const int row = (hexval(line[2]) << 4) | hexval(line[3]);
  if (row < 0 || line[4] != ':') return false;
  const uint16_t off = (uint16_t)(row * 24);
  if (off >= kPresetSize) return true;       // spurious trailing row: swallow
  uint16_t take = (uint16_t)(kPresetSize - off);
  if (take > 24) take = 24;
  const char* h = line + 5;
  for (uint16_t i = 0; i < take; ++i) {
    const int hi = hexval(*h++);
    const int lo = hexval(*h++);
    if (hi < 0 || lo < 0) {
      Serial.printf("preset: row %u bad hex - aborting collect\r\n", row);
      g_preset_collecting = false;
      g_preset_collect_have = 0;
      return true;
    }
    g_preset_collect[off + i] = (uint8_t)((hi << 4) | lo);
  }
  if (off + take > g_preset_collect_have) g_preset_collect_have = off + take;
  if (g_preset_collect_have >= kPresetSize) {
    g_preset_collecting = false;
    Serial.printf("preset: blob collected (%u B, crc %04x) - 'preset load' "
                  "to apply\r\n", g_preset_collect_have,
                  preset_crc16(g_preset_collect, kPresetOffCrc));
  }
  return true;
}

static void preset_cmd(const char* args) {
  while (*args == ' ') ++args;
  if (*args == '\0' || strcasecmp(args, "status") == 0) {
    uint8_t pair = 0;
    if (g_chain.active)
      for (uint8_t p = 1; p <= 3; ++p)
        if (kPresetPairA[p] == g_chain.a) pair = p;
    Serial.printf("preset: v%u, %u B, module=%s chain=%s pair=%u "
                  "bpm=%lu.%lu clk=%s dirty=%u seen=",
                  kPresetVersion, kPresetSize, kRackModules[g_active_module].name,
                  g_chain.active ? "on" : "off", pair,
                  (unsigned long)(int)g_mod_engine.bpm,
                  (unsigned long)(((int)(g_mod_engine.bpm * 10.0f)) % 10),
                  g_mod_engine.global_clock ? "midi" : "internal",
                  g_preset_dirty ? 1 : 0);
    uint32_t seen_total = 0;
    for (uint8_t m = 0; m < RACK_MODULE_COUNT; ++m)
      for (uint8_t w = 0; w < 8; ++w) {
        uint16_t v = g_ui_cc_seen[m][w];
        while (v) { seen_total += (v & 1); v >>= 1; }
      }
    Serial.printf("%lu CCs\r\n", (unsigned long)seen_total);
    Serial.println("preset: dump | load | apply | reload (bench: hex rows)");
    return;
  }
  if (strcasecmp(args, "dump") == 0) {
    uint8_t blob[kPresetBlobMax];
    const uint16_t n = preset_serialize(blob);
    const uint16_t rows = (uint16_t)((n + 23) / 24);
    Serial.printf("preset: dump %u B in %u rows (paste into 'preset load')\r\n",
                  n, rows);
    for (uint16_t r = 0; r < rows; ++r) {
      const uint16_t off = (uint16_t)(r * 24);
      uint16_t take = (uint16_t)(n - off);
      if (take > 24) take = 24;
      char line[64];
      int p = snprintf(line, sizeof(line), "PR%02x:", (unsigned)r);
      for (uint16_t i = 0; i < take; ++i)
        p += snprintf(line + p, sizeof(line) - p, "%02x", blob[off + i]);
      Serial.println(line);
    }
    return;
  }
  if (strcasecmp(args, "collect") == 0) {
    g_preset_collecting = true;
    g_preset_collect_have = 0;
    Serial.printf("preset: collecting - send %u PRxx rows "
                  "(24 B each, 'preset dump' prints them)\r\n",
                  (kPresetSize + 23) / 24);
    return;
  }
  if (strcasecmp(args, "load") == 0) {
    if (g_preset_collecting) {
      Serial.println("preset: already collecting - send PRxx rows");
      return;
    }
    if (g_preset_collect_have < kPresetSize) {
      g_preset_collecting = true;
      g_preset_collect_have = 0;
      Serial.printf("preset: collecting - send %u PRxx rows "
                    "(24 B each, 'preset dump' prints them)\r\n",
                    (kPresetSize + 23) / 24);
      return;
    }
    // Buffer already full from a previous collect: fall through to apply.
    args = "apply";
  }
  if (strcasecmp(args, "apply") == 0 || strcasecmp(args, "reload") == 0) {
    if (g_preset_collect_have < kPresetSize) {
      Serial.println("preset: no collected blob (run 'preset load' + rows)");
      return;
    }
    char why[64];
    if (!preset_validate(g_preset_collect, kPresetSize, why, sizeof(why))) {
      Serial.printf("preset: REJECTED (%s) - 'preset collect' + fresh rows "
                  "to retry\r\n", why);
      return;
    }
    Serial.printf("preset: blob valid (%s) - applying\r\n", why);
    preset_apply_blob(g_preset_collect);
    ui_link_send_telem(g_ui_active_page, kUiTelemFull, 0);
    return;
  }
  Serial.println("preset: usage [status|dump|collect|load|apply|reload]");
}
