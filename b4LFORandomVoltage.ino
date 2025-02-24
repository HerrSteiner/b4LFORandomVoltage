#include <Arduino.h>

#define SAMPLE_RATE 48000
#define attackPot A1
#define releasePot A2
#define peakTrigger D7
#define peakLED D8
#define endLED D10

const uint32_t LEDSHINETIME = 1024;
const uint32_t maxAnalogIn = 4095;
const uint32_t maxAnalogOut = 1023;
const uint32_t shiftfactor = 1024;
const uint32_t peakValue = maxAnalogIn * maxAnalogIn;
const uint32_t freqScale = shiftfactor * 32;

typedef enum {
  ATTACK,
  SUSTAIN,
  RELEASE
} ARstate;

typedef enum {
  LFO,
  AR,
  RANDOM
} Mode;

struct ARData {
  uint32_t inc = 0;
  uint32_t dec = 0;
  uint32_t phase_accumulator = maxAnalogIn;
  //uint32_t tilt = 0;
  ARstate state = ATTACK;
  uint32_t peakLEDCounter = 0; // determines how long the LED is led
  uint32_t endLEDCounter = 0;
  Mode modus = LFO;
};

volatile struct ARData ar;

void tcConfigure(int sampleRate) {
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN |  // Connect GCLK0 at 48MHz as a clock source for TC4 and TC5
                      GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC4_TC5;

  NVIC_SetPriority(TC4_IRQn, 0);  // Set the Nested Vector Interrupt Controller (NVIC) priority for TC4 to 0 (highest)
  NVIC_EnableIRQ(TC4_IRQn);       // Connect the TC4 timer to the Nested Vector Interrupt Controller (NVIC)

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_OVF;     // Enable compare overflow (OVF) interrupt
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_WAVEGEN_MFRQ;  // Set-up TC4 timer for Match Frequency mode (MFRQ)
  TC4->COUNT16.CC[0].reg = (uint16_t)(SystemCoreClock / sampleRate - 1);
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization

  TC4->COUNT16.CTRLA.bit.ENABLE = 1;  // Enable timer TC4
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;  // Wait for synchronization
}

void TC4_Handler() {
  uint32_t sample = 0;
  uint32_t accumulator = ar.phase_accumulator;
    switch (ar.state) {
      case ATTACK:
        accumulator+= ar.inc;
        if (accumulator >= peakValue) {
          accumulator = peakValue;
          ar.state = RELEASE;
          digitalWrite(peakTrigger, HIGH);
          digitalWrite(peakLED, HIGH);
          ar.peakLEDCounter = LEDSHINETIME;
        }
        break;
      case SUSTAIN:
        break;
      case RELEASE:
         int32_t signedAccumulator = accumulator - ar.dec;
            if (signedAccumulator <= 0) {
                accumulator = 0;
                ar.state = ATTACK;
                digitalWrite(endLED, HIGH);
                ar.endLEDCounter = LEDSHINETIME;
            }
            else {
              accumulator = (uint32_t) signedAccumulator;
            }
        break;
    }
    ar.phase_accumulator = accumulator;
    sample = accumulator >> 14;

    
    DAC->DATA.reg = sample;
  TC4->COUNT16.INTFLAG.bit.OVF = 1;  // Clear the interrupt flag
}

void setup() {
  analogWriteResolution(10);
  analogWrite(A0, 0);
  analogReadResolution(12);
  pinMode(attackPot, INPUT);
  pinMode(releasePot, INPUT);
  pinMode(peakTrigger, OUTPUT);
  pinMode(peakLED, OUTPUT);
  pinMode(endLED, OUTPUT);
  tcConfigure(SAMPLE_RATE);
}

uint16_t counter = 1000;
void loop() {
  counter++;
  if (counter > 100) {
    uint32_t attack = analogRead(attackPot) + 10;
    uint32_t release = analogRead(releasePot) + 10;
    
    ar.inc = (attack * freqScale) / SAMPLE_RATE;
    ar.dec = (release * freqScale) / SAMPLE_RATE;

    if (ar.peakLEDCounter > 0) {
      ar.peakLEDCounter -= 1;
      if (ar.peakLEDCounter == 0){
        digitalWrite(peakTrigger,LOW);
        digitalWrite(peakLED,LOW);
      }
    }
    if (ar.endLEDCounter > 0) {
      ar.endLEDCounter -= 1;
      if (ar.endLEDCounter == 0){
        digitalWrite(endLED,LOW);
      }
    }
    counter = 0;
  }
}
