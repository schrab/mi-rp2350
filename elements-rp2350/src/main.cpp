// Task 2 — I2S + PCM5102A bring-up: 220 Hz sine at 32 kHz.
// Wiring: BCK -> GP6 (LRCK/WS follows on GP7 automatically), DIN -> GP8,
// SCK/MCLK -> GND, XSMT -> 3V3 (unmute), FLT/DEMP/FMT -> GND.
#include <Arduino.h>
#include <I2S.h>
#include <Adafruit_TinyUSB.h>

#include <STMLIB.h>
#include <ELEMENTS.h>

// Firmware-side definitions of the elements::Dsp statics. They are declared as
// class statics in ELEMENTS/src/elements/dsp/dsp.h but defined nowhere in the
// library (same definitions are used in plan T3).
float elements::Dsp::kSampleRate = 32000.0f;
float elements::Dsp::kSrFactor = 1.0f;
float elements::Dsp::kIntervalCorrection = 0.0f;

static const uint32_t kSampleRate = 32000;
static const uint32_t kPinBCLK = 6;   // LRCK follows on GP7
static const uint32_t kPinDOUT = 8;

I2S i2s(OUTPUT, kPinBCLK, kPinDOUT);

void setup() {
  Serial.begin(115200);
  i2s.setBitsPerSample(16);
  i2s.setBuffers(8, 16);          // 8 buffers x 16 words = 128 stereo frames (~4 ms)
  i2s.setFrequency(kSampleRate);
  if (!i2s.begin()) {
    Serial.println("I2S begin FAILED");
  } else {
    Serial.println("I2S up");
  }
}

void loop() {
  // 220 Hz sine, 16-bit, stereo via the 2-arg form confirmed in T1.
  static float phase = 0.0f;
  static const float dt = 2.0f * PI * 220.0f / (float)kSampleRate;
  for (int i = 0; i < 32; ++i) {
    int16_t s = (int16_t)(sinf(phase) * 8000.0f);
    phase += dt;
    if (phase > 2.0f * PI) phase -= 2.0f * PI;
    i2s.write16(s, s);   // blocking write, one L/R frame; paces loop at 32 kHz
  }
}
