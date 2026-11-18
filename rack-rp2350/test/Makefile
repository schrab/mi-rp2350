# rack-lira8 host test harness (TDD) — plain g++, no MCU headers.
# Engine sources land in ../src/lira8/ as components arrive (L2/L3); every
# test here must compile them WITHOUT any Arduino/pico header (that property
# is itself the ctag-tbd portability proof).
CXX      ?= g++
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -I../src/lira8 -I.
BUILD    = build/

ENGINE_SRCS =   # e.g. lira8_engine.cpp

# T9 warps/reverb/stage host tests: compile against the vendored lib copies
# (the same include roots the firmware uses). WARPSFLAGS add the elements
# library roots; -DRACK_RP2350_TC selects the rack-guarded arms of the lib
# patches (the RAM_CODE markers still no-op: no ARDUINO_ARCH_RP2040).
WARPSFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -I../lib/elements -I../lib/stmlib -I.
RACKFLAGS  ?= $(WARPSFLAGS) -DRACK_RP2350_TC

TEST_BINS = test_wav_io test_pd_primitives test_voice test_engine test_harmony test_warps test_reverb test_warps_stage test_fx_chain test_swede_eq test_speaker_ir test_varg test_rings_chord
GOLDEN_BIN = render_golden

all: $(TEST_BINS) $(GOLDEN_BIN)

$(BUILD):
	mkdir -p $(BUILD)

test_wav_io: test_wav_io.cc wav_io.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $(BUILD)$@

test_pd_primitives: test_pd_primitives.cc ../src/lira8/pd_primitives.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $(BUILD)$@

test_voice: test_voice.cc ../src/lira8/lira_osc.h ../src/lira8/lira_voice.h ../src/lira8/pd_primitives.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $(BUILD)$@

test_engine: test_engine.cc ../src/lira8/lira_engine.h ../src/lira8/lira_delay.h ../src/lira8/lira_lfo.h ../src/lira8/lira_master.h ../src/lira8/lira_voice.h ../src/lira8/lira_osc.h ../src/lira8/pd_primitives.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $(BUILD)$@

test_harmony: test_harmony.cc ../src/lira8/lira_harmony.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $(BUILD)$@

test_varg: test_varg.cc ../src/varg.h ../src/gen_tables.h ../src/lira8/lira_harmony.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $(BUILD)$@

test_rings_chord: test_rings_chord.cc ../src/rings_chord.h ../src/rack_dist.h ../src/varg.h ../src/lira8/lira_harmony.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $(BUILD)$@

# T9: warps vendored copy (rack deltas under test), reverb engine size, and
# the Part warps stage. The .inc/.cc TUs are compiled in (mirroring
# elements_all.cpp) so no firmware build is needed to run these.
test_warps: test_warps.cc ../lib/elements/warps/dsp/modulator.h ../lib/elements/warps/dsp/parameters.h | $(BUILD)
	$(CXX) $(WARPSFLAGS) $< ../lib/elements/warps/dsp/modulator.cc ../lib/elements/warps/dsp/oscillator.cc ../lib/elements/warps/resources.cc -o $(BUILD)$@

test_reverb: test_reverb.cc ../lib/elements/elements/dsp/fx/reverb.h ../lib/elements/elements/dsp/fx/fx_engine.h | $(BUILD)
	$(CXX) $(RACKFLAGS) $< -o $(BUILD)$@

test_warps_stage: test_warps_stage.cc ../lib/elements/elements/dsp/part.h ../lib/elements/warps/dsp/modulator.h | $(BUILD)
	$(CXX) $(RACKFLAGS) $< ../lib/elements/warps/dsp/modulator.cc ../lib/elements/warps/dsp/oscillator.cc ../lib/elements/warps/resources.cc -o $(BUILD)$@

render_golden: render_golden.cc wav_io.h ../src/lira8/lira_engine.h ../src/lira8/lira_delay.h ../src/lira8/lira_lfo.h ../src/lira8/lira_master.h ../src/lira8/lira_voice.h ../src/lira8/lira_osc.h ../src/lira8/pd_primitives.h | $(BUILD)
	$(CXX) $(CXXFLAGS) $< -o $(BUILD)$@

engine: $(patsubst %.cpp,%.o,$(ENGINE_SRCS))

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do ./$(BUILD)$$t || exit 1; done

# golden compare CLI (python side lives in lira8_ref/)
goldens:
	@.venv_missing_hint() { echo "run from test/lira8_ref/: ../lira8_ref/compare_wav.py ref.wav got.wav"; }

clean:
	rm -rf $(BUILD) $(TEST_BINS)

.PHONY: all test engine goldens clean

# T9 reverb A/B: rack arm (FxEngine<21632>) vs upstream arm (FxEngine<32768>).
# Law: a correct wrap makes engine size sound-neutral -> dumps MUST be
# byte-identical. Any diff = the fx_engine.h `& (size-1)` wrap bug is live.
reverb_ab: build/test_reverb_rack build/test_reverb_upstream
	./build/test_reverb_rack build/reverb_rack.raw
	./build/test_reverb_upstream build/reverb_upstream.raw
	@if cmp -s build/reverb_rack.raw build/reverb_upstream.raw; then \
	  echo "reverb A/B: BIT-IDENTICAL (wrap correct)"; \
	else \
	  echo "reverb A/B: DUMPS DIFFER — FxEngine<21632> masking bug LIVE"; \
	  exit 1; \
	fi

build/test_reverb_rack: reverb_ab.cc ../lib/elements/elements/dsp/fx/reverb.h ../lib/elements/elements/dsp/fx/fx_engine.h | build
	$(CXX) $(RACKFLAGS) reverb_ab.cc -o $(BUILD)test_reverb_rack

build/test_reverb_upstream: reverb_ab.cc ../lib/elements/elements/dsp/fx/reverb.h ../lib/elements/elements/dsp/fx/fx_engine.h | build
	$(CXX) $(WARPSFLAGS) reverb_ab.cc -o $(BUILD)test_reverb_upstream

.PHONY: reverb_ab

# T9 chain law: reverb is the LAST fx (distortion+warps BEFORE). Bypass is
# bit-exact; the tail stays diffuse with the shaper active pre-reverb.
test_fx_chain: test_fx_chain.cc ../lib/elements/elements/dsp/part.h ../src/rack_dist.h | build
	$(CXX) $(RACKFLAGS) -I../src test_fx_chain.cc ../lib/elements/warps/dsp/modulator.cc ../lib/elements/warps/dsp/oscillator.cc ../lib/elements/warps/resources.cc -o $(BUILD)$@

# Swedish Chainsaw upgrade: 32 kHz HM-2 band redesign + FMV B/M/T knobs.
test_swede_eq: test_swede_eq.cc ../src/rack_dist.h | build
	$(CXX) $(CXXFLAGS) -I../src test_swede_eq.cc -o $(BUILD)$@

# Speaker cab convolution (swede): impulse law + naive-reference law.
test_speaker_ir: test_speaker_ir.cc ../src/rack_dist.h ../src/speaker_ir.h | build
	$(CXX) $(CXXFLAGS) -I../src test_speaker_ir.cc -o $(BUILD)$@
