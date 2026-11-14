# rack-lira8 host test harness (TDD) — plain g++, no MCU headers.
# Engine sources land in ../src/lira8/ as components arrive (L2/L3); every
# test here must compile them WITHOUT any Arduino/pico header (that property
# is itself the ctag-tbd portability proof).
CXX      ?= g++
CXXFLAGS ?= -O2 -g -std=c++17 -Wall -Wextra -I../src/lira8 -I.
BUILD    = build/

ENGINE_SRCS =   # e.g. lira8_engine.cpp

TEST_BINS = test_wav_io test_pd_primitives test_voice test_engine

all: $(TEST_BINS)

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

engine: $(patsubst %.cpp,%.o,$(ENGINE_SRCS))

test: $(TEST_BINS)
	@for t in $(TEST_BINS); do ./$(BUILD)$$t || exit 1; done

# golden compare CLI (python side lives in lira8_ref/)
goldens:
	@.venv_missing_hint() { echo "run from test/lira8_ref/: ../lira8_ref/compare_wav.py ref.wav got.wav"; }

clean:
	rm -rf $(BUILD) $(TEST_BINS)

.PHONY: all test engine goldens clean
