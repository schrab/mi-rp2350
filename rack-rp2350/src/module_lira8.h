// Rack RP2350 — LIRA-8 adapter (rack-lira8, 2026-11-15).
//
// The ONLY rack module that is a hand port rather than a vendored upstream
// C++ lib: the engine (src/lira8/*.h) is a faithful re-implementation of
// MikeMorenoAudio's LIRA-8 Pure Data patch (BSD), developed TDD against
// headless-pd golden renders (rack-rp2350/test/lira8_ref/). Pure C++, no
// MCU headers in the engine — host compilation doubles as the ctag-tbd
// portability proof.
//
// Follows the established adapter shape (module_elements.h / module_plaits.h):
// arena-placed State (engine by value + the delay buffers carved from the
// T8 unified SRAM pool), pending slots for CCs/sensors, the mod-engine hooks,
// the T1 button twin. TWO rate contexts: standalone 44.1 kHz (176,416 B
// delay carve), chain A 32 kHz (128,016 B) — the chain carve is what lets
// lira8->clouds share the pool with clouds' 128 KiB chained buffers.
//
// AUDIO CONTRACT: the engine renders pd-style 64-sample blocks internally.
// Standalone the rack grid is 16 samples — Process() accumulates 4 host
// blocks per internal block, so every 4th call bursts (up to ~4x its share of
// the block budget). Chain A calls Process(24) on the 32 kHz grid (bursts
// every ~2.67 blocks). The I2S DMA bank (8 x 16 frames = 2.9 ms standalone;
// 32 x 16 chain) absorbs the burst (PORTING-HIGHLIGHTS §5 rule: bank >
// spike, average < budget).
//
// SENSOR SEMANTICS (the Lyra-8's 8 touch pads): MIDI notes 24-31 (C1-G1, one
// octave below the Pd patch's notemap) gate sensors 1-8 (note-on = press with
// velocity -> envelope peak, note-off = release); mod triggers CC 55-62
// (LATCH: the sensor follows the source level); the PLAY button pulses
// sensor 1 for 400 ms (the "press a pad" feel).
//
// ARP PITCH (T8b, 2026-11-16): CC 54 (P3 "ARP HARM") retunes all eight voices
// from the routed MOD-ARP's harmony (scale/root/chord: low group = perfect
// bass stack, high group = scale chord); routing an ARP to PITCH 1234/5678
// (CC 25/26) instead makes that group play the ARP's exact step notes as its
// root. Both paths live in lira8/lira_harmony.h (pure, host-tested).
#pragma once

#include <new>                   // placement-new into the arenas
#include <cstring>

#include "lira8/lira_engine.h"
#include "lira8/lira8_tc.h"
#include "lira8/lira_harmony.h"  // ARP-driven voice pitch (T8b)
#include "lira8_engine_o3.h"     // -O3 Process() wrapper (lib/lira8_engine)
#include "pico/time.h"           // time_us_32 (static inline in pico-sdk 2.x)

#include "rack_arena.h"
#include "module_registry.h"     // RackModDestinations (T3 mod routing table)

// Forward declaration of mod_engine destination query (defined in mod_engine.h).
bool mod_is_dest_routed(uint8_t cc);

namespace rack_modules {
namespace mod_lira8 {

static const char* const kName = "lira8";
// Two rate contexts (plaits precedent: 48k switched / 32k chained):
// standalone runs at 44.1 kHz (the Pd abstraction's rlpf constants are
// 44.1k-authored); a chain A slot renders at the 32 kHz chain grid.
static const uint32_t kSampleRate = 44100;
static constexpr uint32_t kChainSampleRate = 32000;
static const size_t kBlockSize = 16;     // host grid; the engine bursts 64
// Report budget = the INTERNAL 64-sample block (1.451 ms at 44.1 kHz), NOT
// the 16-sample host grid: Process() renders a whole internal block on one
// call (every 4th), so proc_us_max is a single-block number — compare it
// against the block period. The 4x burst is absorbed by the 2.9 ms I2S bank;
// the per-call stall threshold is kStallUs below.
static const uint32_t kBudgetUs =
    (uint32_t)((1000000ULL * l8pd::Lira8Engine::kBlockSize +
                kSampleRate / 2) / kSampleRate);  // 1451
static const uint32_t kI2sBuffers = 8;   // 8 x 16 words = 128 frames (~2.9 ms)
static const uint32_t kI2sWords = 16;
static const uint32_t kStallUs = 900;    // >2x the 363 us write period

// Delay storage: the ORIGINAL HARDWARE's delay tops out at 1 SECOND (user,
// 2026-11-15) — the Pd patch's 5.944 s range is a patch-ism. int16, 2 lines.
// Standalone 44.1 kHz: (44100+4) * 2 * 2 = 176,416 B. Chain A runs at the
// 32 kHz chain grid: (32000+4) * 2 * 2 = 128,016 B — that smaller carve is
// what lets lira8->clouds fit the pool. The pd TIME law is kept
// (2^(12k/127)*1.45125 ms at 44.1k; the port derives it from sr), so the top
// ~22% of the TIME knob saturates at the cap, like a pot hitting its stop.
static constexpr float kCapMs = 1000.0f;
static constexpr size_t kDelayBytes =
    l8pd::Lira8Engine::delay_bytes_for(kCapMs, kSampleRate);       // 176,416
static constexpr size_t kChainDelayBytes =
    l8pd::Lira8Engine::delay_bytes_for(kCapMs, kChainSampleRate);  // 128,016
static_assert(kDelayBytes <= kRackPoolBytes,
              "lira8 delay carve exceeds the rack SRAM pool");
static_assert(kChainDelayBytes <= kRackPoolBytes,
              "lira8 chain delay carve exceeds the rack SRAM pool");

// CC map (the spec §12 order, remapped OFF the rack-reserved CCs:
// 32 = module switch, 33/34/35 = mod mux, 40-50 + 65/66 = the clouds B-slot
// intercept while chained, 64 = PLAY pseudo-CC). The delay-line section
// lives at 80-88 (not 41-49): the clouds-B alias bank owns 40-50 while
// chained, and lira8->clouds would otherwise lose its delay controls.
// Sensors also speak MIDI notes 24-31 (C1-G1).
static const char* const kCcMap =
    "1-8 tune, 9-12 fast, 13-16 sharp, 17-20 mod, 21-24 source, "
    "25-26 pitch, 27-28 hold, 29 switch, 30 total-fb, 31 vibrato, "
    "36 drv, 37 dst-mix, 38 vol, 39 f-a, 80 f-b, 81 andor, 82 link, "
    "83 time-1, 84 time-2, 85 feedback, 86 del-mix, 87-88 mod-1/2, "
    "51 delay-mode (0 self / 1 lfo-tri / 2 lfo-sqr), 52 lfo-wave direct, "
    "53 quantize, 54 arp-harm (voices follow the routed ARP's scale/root), "
    "55-62 sensors 1-8 (latch)";

// T3 mod destinations: all 47 continuous params (the spec §12 order mapped
// to the CC numbers above); triggers = the 8 sensors (LATCH — a sensor gate
// follows the source level, exactly like the patch's tgl semantics).
static const uint8_t kModContinuousCcs[] = {
    1, 2, 3, 4, 5, 6, 7, 8,                 // tune 1-8
    13, 14, 15, 16,                         // sharp
    17, 18, 19, 20,                         // mod
    25, 26,                                 // pitch
    27, 28,                                 // hold
    36, 37, 38,                             // drv, dst-mix, vol
    39, 80,                                 // f-a, f-b
    83, 84, 85, 86, 87, 88,                 // time-1/2, feedback, del-mix, mod-1/2
};
static const uint8_t kModTriggerCcs[] = {55, 56, 57, 58, 59, 60, 61, 62};
static const uint8_t kModTriggerKinds[] = {
    RACK_TRIG_LATCH, RACK_TRIG_LATCH, RACK_TRIG_LATCH, RACK_TRIG_LATCH,
    RACK_TRIG_LATCH, RACK_TRIG_LATCH, RACK_TRIG_LATCH, RACK_TRIG_LATCH,
};
static const RackModDestinations kModDestinations = {
    "lira8: continuous tune 1-8, sharp 13-16, mod 17-20, pitch 25-26, "
    "hold 27-28, drv/vol 36/38, times 83-84, fb 85 | triggers: 55-62 "
    "sensors 1-8 (latch); notes 24-31 = sensors; ARP -> 25/26 = group "
    "pitch (exact ARP notes)",
    sizeof(kModContinuousCcs), kModContinuousCcs,
    8, kModTriggerCcs, kModTriggerKinds,
};

// --- engine state: arena-placed (the engine BY VALUE; the delay buffers
// carve from the caller's pool region at init) ---
struct State {
  l8pd::Lira8Engine eng;

  // T1 hardware PLAY button: pulse sensor 1 for kPluckWindowMs.
  volatile bool     pluck_active;
  volatile uint32_t pluck_until_ms;

  // 2 s report mirrors (core 0 publishes the requested state, core 1 the
  // output RMS).
  volatile float    out_rms_report;
  volatile uint8_t  sensor_mask;   // bit N = sensor N+1 requested ON

  // T8b ARP-driven pitch. arp_pitch_mode = CC 54 (core 0 write); the group
  // mask/seq/override are audio-core state (apply_arp_pitch); the override
  // flag is the core-0-visible "pads must reach the adapter" signal used by
  // the MIDI ARP-hijack bypass.
  volatile bool     arp_pitch_mode;
  uint8_t           arp_group_mask;    // bit 0 = voices 1-4, bit 1 = 5-8
  uint32_t          arp_seq_seen;
  volatile bool     arp_pitch_override;

  explicit State(int16_t* delay_mem, float sr)
      : eng(sr, kCapMs, delay_mem),
        pluck_active(false),
        pluck_until_ms(0),
        out_rms_report(0.0f),
        sensor_mask(0),
        arp_pitch_mode(false),
        arp_group_mask(0),
        arp_seq_seen(0),
        arp_pitch_override(false) {}
};

static State* s = nullptr;   // placement-new'd by init(); arena offset 0

static size_t state_size() { return sizeof(State); }
// Buffer demand: the int16 delay carve at the module's CURRENT rate context.
static constexpr size_t buf_bytes() { return kDelayBytes; }
static constexpr size_t chain_buf_bytes() { return kChainDelayBytes; }

void init_at(uint8_t* obj_base, uint8_t* buf_base) {
  // Delay carve from the caller's pool region (44.1 kHz standalone context);
  // the pool union discipline means only the active resident set is live.
  memset(buf_base, 0, kDelayBytes);
  s = new (obj_base) State(reinterpret_cast<int16_t*>(buf_base),
                           static_cast<float>(kSampleRate));
}

void init() {
  rack_mem_reset();
  init_at(static_cast<uint8_t*>(rack_mem_alloc(state_size(), 8)),
          static_cast<uint8_t*>(rack_mem_alloc(buf_bytes(), 8)));
}

// T6: chain A-slot init at the 32 kHz chain grid (see module_plaits's
// init_chain_a — same rate-context contract). The engine captures sr at
// construction (all rlpf/phasor/line~ constants derive from it), and the
// smaller 1 s delay carve (128,016 B) is what makes lira8->clouds fit the
// unified pool alongside clouds' 128 KiB chained buffers.
void init_chain_a(uint8_t* obj_base, uint8_t* buf_base) {
  memset(buf_base, 0, kChainDelayBytes);
  s = new (obj_base) State(reinterpret_cast<int16_t*>(buf_base),
                           static_cast<float>(kChainSampleRate));
}

// Sensor note range: C1-G1 (24-31) — the Pd patch's notemap (36-43) shifted
// one octave down by request (2026-11-16). Velocity scales the press peak.
static const uint8_t kSensorBaseNote = 24;
static inline bool is_sensor_note(uint8_t note) {
  return note >= kSensorBaseNote && note < kSensorBaseNote + 8;
}

// Every sensor write goes through here so the report mask stays honest.
static void set_sensor(int8_t idx, bool on, float level = 1.0f) {
  s->eng.set_sensor(idx, on, level);
  if (idx >= 0 && idx < 8) {
    if (on) s->sensor_mask |= (1u << idx);
    else    s->sensor_mask &= ~(1u << idx);
  }
}

// T1 hardware PLAY button twin (see module_plaits.h): press = sensor 1
// pressed for 400 ms (a musical "hit the pad"), release ends it early.
static const uint32_t kPluckWindowMs = 400;

void button_trig(bool down) {
  if (down) {
    set_sensor(0, true);
    s->pluck_active = true;
    s->pluck_until_ms = millis() + kPluckWindowMs;
  } else if (s->pluck_active) {
    set_sensor(0, false);
    s->pluck_active = false;
  }
}

void pluck_poll() {
  if (s->pluck_active && millis() >= s->pluck_until_ms) {
    set_sensor(0, false);
    s->pluck_active = false;
  }
}

// ===================== T8b: ARP-driven voice pitch =====================
// Called between engine blocks on the audio core. The mod engine published
// the snapshot earlier in the same block (mod_engine_process_block runs
// before the module render). Group 0 = voices 1-4 (pairs 0/1, P1), group 1 =
// voices 5-8 (pairs 2/3, P2):
//   - mapped ARP (CC 25 low / CC 26 high) -> that group's root is the ARP's
//     exact current step note, always (the switch is irrelevant);
//   - otherwise the ARP HARM switch (CC 54) makes both groups follow the
//     ARP's harmonic state (root folded to C3..B3).
// Voices leave the override (and return to their TUNE knobs) when the source
// goes away. Recomputes only when the snapshot's seq or the group mask moves.
static void apply_arp_pitch() {
  const l8harmony::Snapshot& h = l8harmony::g_lira_harmony;
  const bool mode = s->arp_pitch_mode;
  uint8_t want = 0;
  if (h.lo_mapped || (mode && h.harmony_valid)) want |= 1u;
  if (h.hi_mapped || (mode && h.harmony_valid)) want |= 2u;
  if (h.seq == s->arp_seq_seen && want == s->arp_group_mask) {
    s->arp_pitch_override = (want != 0);
    return;
  }
  s->arp_seq_seen = h.seq;
  for (int grp = 0; grp < 2; ++grp) {
    const bool on = ((want >> grp) & 1u) != 0;
    const bool was = ((s->arp_group_mask >> grp) & 1u) != 0;
    if (!on) {
      if (was) {
        for (int k = 0; k < 4; ++k) s->eng.clear_voice_note_hz(grp * 4 + k);
      }
      continue;
    }
    const l8harmony::GroupCtx* ctx =
        (grp == 0) ? (h.lo_mapped ? &h.lo_ctx : &h.harmony)
                   : (h.hi_mapped ? &h.hi_ctx : &h.harmony);
    int8_t off[l8harmony::kGroupVoices];
    l8harmony::group_offsets(*ctx, grp, off);
    for (int k = 0; k < l8harmony::kGroupVoices; ++k) {
      const int note = static_cast<int>(ctx->root) + off[k];
      s->eng.set_voice_note_hz(grp * 4 + k, l8harmony::note_hz(note));
    }
  }
  s->arp_group_mask = want;
  s->arp_pitch_override = (want != 0);
}

// Core-0 hook (midi.h ARP-hijack bypass): true while any group is being
// driven by the ARP/harmony and sensor notes must reach this adapter.
bool arp_pitch_active() { return s != nullptr && s->arp_pitch_override; }

// T6 chain A-slot render — exactly n samples (n <= 24 = the chain grid) into
// caller buffers, NO I2S write, NO proc_us_max update (the chain loop times
// A + route + B together; clouds-B also owns the pair meters). The engine's
// Process() accepts 1..64 and buffers its internal 64-sample block, so the
// 24-sample grid call pattern just works (a burst lands when the internal
// block completes).
void chain_render_a(float* out_l, float* out_r, size_t n) {
  pluck_poll();
  apply_arp_pitch();
  l8pd::process_block_o3(s->eng, out_l, out_r, static_cast<int>(n));
}

void render() {
  pluck_poll();
  apply_arp_pitch();

  uint32_t t0 = time_us_32();
  float l[kBlockSize], r[kBlockSize];
  l8pd::process_block_o3(s->eng, l, r, kBlockSize);
  uint32_t dt = time_us_32() - t0;
  if (dt > proc_us_max) proc_us_max = dt;

  // Meters: output block RMS (mono — both channels carry the same signal),
  // max since the last 2 s report.
  float acc = 0.0f;
  for (size_t i = 0; i < kBlockSize; ++i) acc += l[i] * l[i];
  const float rms = sqrtf(acc / kBlockSize);
  if (rms > rack_meter_a) rack_meter_a = rms;
  s->out_rms_report = rms;

  // The writes pace the render loop: one iteration = one 16-frame DMA block,
  // blocks free every 500 us; > 900 us = stalled. g_render_gain = the
  // swap-mute ramp.
  const float gain = g_render_gain;
  uint32_t w0 = time_us_32();
  for (size_t i = 0; i < kBlockSize; ++i) {
    const int16_t out = static_cast<int16_t>(l[i] * gain * 32768.0f);
    i2s.write16(out, out);
  }
  if (time_us_32() - w0 > kStallUs) {
    ++i2s_stall_count;
  }
}

void note_on(uint8_t note, uint8_t vel) {
  // The pad notemap (shifted one octave below the Pd patch's 36-43):
  // note-on = press, velocity = envelope peak (gate law e^2 keeps the
  // hardware's touch curve).
  if (is_sensor_note(note)) {
    set_sensor(note - kSensorBaseNote, true, static_cast<float>(vel) / 127.0f);
  }
}

void note_off(uint8_t note) {
  if (is_sensor_note(note)) set_sensor(note - kSensorBaseNote, false);
}

// CC -> engine param index (the spec §12 order; the CC numbers avoid the
// rack-reserved ranges and the clouds-B alias band 40-50). Returns -1 for
// CCs that are not params.
static int8_t cc_to_param(uint8_t cc) {
  if (cc >= 1 && cc <= 31) return static_cast<int8_t>(cc - 1);       // 0-30
  if (cc >= 36 && cc <= 39) return static_cast<int8_t>(cc - 5);      // 31-34
  if (cc >= 80 && cc <= 88) return static_cast<int8_t>(cc - 45);     // 35-43
  if (cc >= 51 && cc <= 53) return static_cast<int8_t>(cc - 7);      // 44-46
  return -1;
}

// T8b ARP HARM switch (P3 p15): ON = the eight voices follow the routed
// ARP's scale/root/chord (lira8/lira_harmony.h). The audio core applies /
// restores the pitch overrides at the next block.
static const uint8_t kArpPitchCc = 54;

void control_change(uint8_t cc, uint8_t val) {
  // PLAY pseudo-CC (64, the registry button path): pulse sensor 1.
  if (cc == 64) {
    button_trig(val >= 64);
    return;
  }
  // ARP HARM switch (54): ON/OFF.
  if (cc == kArpPitchCc) {
    s->arp_pitch_mode = (val >= 64);
    return;
  }
  // Sensor latch CCs 55-62 (also reachable as mod triggers).
  if (cc >= 55 && cc <= 62) {
    set_sensor(cc - 55, val >= 64);
    return;
  }
  // CC 51 = the combined DELAY MODE switch. The hardware has a two-way
  // SELF/LFO switch plus a separate LFO-waveform switch; since the LFO does
  // not shape the delay in SELF mode, the UI merges them into one 3-state
  // selector: 0 = SELF, 1 = LFO triangle, 2 = LFO square. Small values are
  // the C3's discrete states; larger MIDI values map in thirds.
  if (cc == 51) {
    int mode;
    if (val <= 2) mode = val;
    else mode = (val < 43) ? 0 : (val < 85 ? 1 : 2);
    s->eng.set_param(44, (mode == 0) ? 0.0f : 2.0f);        // del-mod
    if (mode > 0) s->eng.set_param(45, (mode == 2) ? 1.0f : 0.0f);
    return;
  }
  const int8_t idx = cc_to_param(cc);
  if (idx >= 0) s->eng.set_param(idx, static_cast<float>(val));
}

// ===================== T3 mod-engine hooks (core 1) =====================
// Same between-blocks discipline as every adapter: the mod engine only runs
// while not swapping, so `s` is always the live state. The engine applies
// pending params between its internal blocks (the pending_params_ slots).

bool mod_set_cc_float(uint8_t cc, float v) {
  if (v < 0.0f) v = 0.0f;
  if (v > 1.0f) v = 1.0f;
  if (cc == 51) {   // combined delay-mode selector (see control_change)
    const int mode = (v < 0.34f) ? 0 : (v < 0.67f ? 1 : 2);
    s->eng.set_param(44, (mode == 0) ? 0.0f : 2.0f);
    if (mode > 0) s->eng.set_param(45, (mode == 2) ? 1.0f : 0.0f);
    return true;
  }
  const int8_t idx = cc_to_param(cc);
  if (idx < 0) return false;
  s->eng.set_param(idx, v * 127.0f);
  return true;
}

// Trigger destinations: CCs 55-62 = sensors 1-8, LATCH semantics (the gate
// follows the source level — the patch's tgl behavior).
bool mod_trigger_cc(uint8_t cc, bool high) {
  if (cc >= 55 && cc <= 62) {
    set_sensor(cc - 55, high);
    return true;
  }
  return false;
}

}  // namespace mod_lira8
}  // namespace rack_modules
