#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#include <STMLIB.h>
#include <ELEMENTS.h>

// Firmware-side definitions of the elements::Dsp statics. They are declared as
// class statics in ELEMENTS/src/elements/dsp/dsp.h but defined nowhere in the
// library (same definitions are used in plan T3).
float elements::Dsp::kSampleRate = 32000.0f;
float elements::Dsp::kSrFactor = 1.0f;
float elements::Dsp::kIntervalCorrection = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("elements-rp2350 scaffold OK");
  Serial.printf("CPU %u Hz, SR %f\n", F_CPU, elements::Dsp::getSr());
}

void loop() {}
