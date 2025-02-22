#include <Arduino.h>

#define SAMPLE_RATE 48000
#define frequencyPot A1
#define tiltPot A2

const uint32_t maxAnalogIn = 4095;
const uint32_t maxAnalogOut = 1023;
const uint32_t shiftfactor = 1024;

typedef enum {
  ATTACK,
  SUSTAIN,
  RELEASE
} ARstate;

struct ARData {
  uint32_t inc = 0;
  uint32_t phase_accumulator = maxAnalogIn;
  uint32_t frequency = 0;
  uint32_t tilt = 0;
  uint32_t frequencyCounter = 0;
  ARstate state = ATTACK;
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

  ar.frequencyCounter+= ar.inc;
  if (ar.frequencyCounter > (maxAnalogIn << 4)) {
    ar.frequencyCounter = 0;
    switch (ar.state) {
      case ATTACK:
        accumulator+= (1 + ar.tilt);
        if (accumulator >= maxAnalogIn * maxAnalogIn) {
          accumulator = maxAnalogIn * maxAnalogIn;
          ar.state = RELEASE;
        }
        break;
      case SUSTAIN:
        break;
      case RELEASE:
         int32_t signedAccumulator = accumulator - (1 + (maxAnalogIn - ar.tilt) );
            if (signedAccumulator <= 0) {
                accumulator = 0;
                ar.state = ATTACK;
            }
            else {
              accumulator = (uint32_t) signedAccumulator;
            }
        break;
    }
    ar.phase_accumulator = accumulator;
    sample = accumulator >> 14;
    DAC->DATA.reg = sample;
  }
  TC4->COUNT16.INTFLAG.bit.OVF = 1;  // Clear the interrupt flag
}

void setup() {
  analogWriteResolution(10);
  analogWrite(A0, 0);
  analogReadResolution(12);
  pinMode(frequencyPot, INPUT);
  pinMode(tiltPot, INPUT);

  tcConfigure(SAMPLE_RATE);
}

uint16_t counter = 1000;
void loop() {
  counter++;
  if (counter > 100) {
    ar.frequency = analogRead(frequencyPot);
    if (ar.frequency == 0) {
      ar.frequency = 1;
    }
    uint32_t tilt = analogRead(tiltPot);
    
    ar.tilt = tilt;//map(tilt,0,maxAnalogIn,25,maxAnalogIn - 25); //(tilt * maxAnalogIn) / (tilt + (maxAnalogIn - tilt));
    //uint32_t period = (2 * maxAnalogIn * maxAnalogIn) / ar.tilt + 
    //                (2 * maxAnalogIn * maxAnalogIn) / (maxAnalogIn - ar.tilt);
    uint32_t compensation = tilt > (maxAnalogIn>>1) ? map(tilt,(maxAnalogIn>>1), maxAnalogIn,  1, maxAnalogIn) : map(tilt,0,(maxAnalogIn>>1),maxAnalogIn,1);
    ar.inc = (ar.frequency * (compensation));// - (compensation >> 2);
    counter = 0;
  }
}