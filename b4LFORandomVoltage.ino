/*
b4LFORandomVoltage module firmware
Copyright (C) 2025 Malte Steiner

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
firmware for XIAO SAMD21 based modulator module for my modular synthesizer
it consists of 3 modes:
AR: it is a attack release envelope, needs a trigger. Pots control attack and release time, there is a gate output for peak sustain and a trigger output when release ended
LFO: the same but looping, doesn´t react on trigger input
Random Voltage: trigger input causes the DAC to output a random voltage. Another trigger output triggers randomly and one trigger output triggers regular like a clock,
the frequencies are controlled with the two pots independently. 
*/

#include <Arduino.h>

#define SAMPLE_RATE 48000
#define attackPot A1
#define releasePot A2
#define LFOModeSwitch D3
#define RandomModeSwitch D4
#define triggerInput D6
#define peakTrigger D7
#define peakLED D8
#define endTrigger D9
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
  RELEASE,
  END
} ARstate;

typedef enum {
  START,
  LFO,
  AR,
  RANDOM
} Mode;

struct ARData {
  uint32_t inc = 0;
  uint32_t dec = 0;
  uint32_t phase_accumulator = maxAnalogIn;
  uint32_t random_accumulator = 0;
  uint32_t trigger = 0;         // the value of the attack potentiometer
  uint32_t peakLEDCounter = 0;  // determines how long the LED is led
  uint32_t endLEDCounter = 0;
  ARstate state = ATTACK;
  Mode modus = START;
  bool generated = false;  // used in the random generator mode
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
  switch (ar.modus) {
    case LFO:
      switch (ar.state) {
        case ATTACK:
          accumulator += ar.inc;
          if (accumulator >= peakValue) {
            accumulator = peakValue;
            ar.state = RELEASE;
            digitalWrite(peakTrigger, HIGH);
            digitalWrite(peakLED, HIGH);
            ar.peakLEDCounter = LEDSHINETIME;
          }
          break;
        case SUSTAIN:
        case END:
        case RELEASE:
          int32_t signedAccumulator = accumulator - ar.dec;
          if (signedAccumulator <= 0) {
            accumulator = 0;
            ar.state = ATTACK;
            digitalWrite(endTrigger, HIGH);
            digitalWrite(endLED, HIGH);
            ar.endLEDCounter = LEDSHINETIME;
          } else {
            accumulator = (uint32_t)signedAccumulator;
          }
          break;
      }
      ar.phase_accumulator = accumulator;
      sample = accumulator >> 14;
      DAC->DATA.reg = sample;
      break;

    case AR:
      if (ar.trigger == HIGH) {
        if (ar.state == SUSTAIN) {
          ar.peakLEDCounter = LEDSHINETIME;  // keep the flame burning
        } else {
          ar.state = ATTACK;
          accumulator += ar.inc;
          if (accumulator >= peakValue) {
            accumulator = peakValue;
            ar.state = SUSTAIN;
            digitalWrite(peakTrigger, HIGH);
            digitalWrite(peakLED, HIGH);
            ar.peakLEDCounter = LEDSHINETIME;
          }
          ar.phase_accumulator = accumulator;
          sample = accumulator >> 14;
          DAC->DATA.reg = sample;
        }
      } else {
        if (ar.state != END) {
          ar.state = RELEASE;
          int32_t signedAccumulator = accumulator - ar.dec;
          if (signedAccumulator <= 0) {
            accumulator = 0;
            ar.state = END;
            digitalWrite(endTrigger, HIGH);
            digitalWrite(endLED, HIGH);
            ar.endLEDCounter = LEDSHINETIME;
          } else {
            accumulator = (uint32_t)signedAccumulator;
          }
          ar.phase_accumulator = accumulator;
          sample = accumulator >> 14;
          DAC->DATA.reg = sample;
        }
      }

      break;

    case RANDOM:  // ------------- random voltage generator / clock mode ----------------------------

      // random generator part
      if (ar.trigger == HIGH && ar.generated == false) {
        sample = random(maxAnalogOut);
        ar.generated = true;
        DAC->DATA.reg = sample;
      }

      // clock part
      accumulator += ar.dec;  // the release decrement is used as increment in this mode
      if (accumulator >= peakValue) {
        accumulator = 0;
        digitalWrite(endTrigger, HIGH);
        digitalWrite(endLED, HIGH);
        ar.endLEDCounter = 100;
      }
      ar.phase_accumulator = accumulator;

      // random trigger part
      uint32_t random_accumulator = ar.random_accumulator;
      random_accumulator += ar.inc;
      if (random_accumulator > peakValue) {
        random_accumulator = 0;
        uint32_t randomValue = random(100);
        if (randomValue > 70) {
          digitalWrite(peakTrigger, HIGH);
          digitalWrite(peakLED, HIGH);
          ar.peakLEDCounter = 100;
        }
      }
      ar.random_accumulator = random_accumulator;
      break;
  }

  TC4->COUNT16.INTFLAG.bit.OVF = 1;  // Clear the interrupt flag
}

void setup() {
  analogWriteResolution(10);
  analogWrite(A0, 0);
  analogReadResolution(12);

  pinMode(attackPot, INPUT);
  pinMode(releasePot, INPUT);
  pinMode(LFOModeSwitch, INPUT);
  pinMode(RandomModeSwitch, INPUT);
  pinMode(triggerInput, INPUT);

  pinMode(peakTrigger, OUTPUT);
  pinMode(peakLED, OUTPUT);
  pinMode(endLED, OUTPUT);

  tcConfigure(SAMPLE_RATE);
}

uint16_t counter = 1000;
void loop() {
  counter++;
  if (counter > 100) {
    uint32_t trigger = digitalRead(triggerInput) == LOW ? HIGH : LOW; // trigger is inverted
    ar.trigger = trigger;
    if (trigger == LOW) {
      ar.generated = false;
    }

    uint32_t release = analogRead(releasePot) + 10;
    uint32_t attack = analogRead(attackPot) + 10;
    ar.inc = (attack * freqScale) / SAMPLE_RATE;
    ar.dec = (release * freqScale) / SAMPLE_RATE;

    if (ar.peakLEDCounter > 0) {
      ar.peakLEDCounter -= 1;
      if (ar.peakLEDCounter == 0) {
        digitalWrite(peakTrigger, LOW);
        digitalWrite(peakLED, LOW);
      }
    }
    if (ar.endLEDCounter > 0) {
      ar.endLEDCounter -= 1;
      if (ar.endLEDCounter == 0) {
        digitalWrite(endTrigger, LOW);
        digitalWrite(endLED, LOW);
      }
    }

    counter = 0;

    // checking the mode switch
    uint32_t LFOMode = digitalRead(LFOModeSwitch)  == LOW ? HIGH : LOW;
    uint32_t RandomMode = digitalRead(RandomModeSwitch) == LOW ? HIGH : LOW;
    if (LFOMode == HIGH && RandomMode == LOW && ar.modus != LFO) {
      ar.modus = START;
      ar.state = ATTACK;
      analogWrite(A0, 0);
      digitalWrite(peakTrigger, LOW);
      digitalWrite(peakLED, LOW);
      ar.phase_accumulator = 0;
      ar.random_accumulator = 0;
      digitalWrite(endTrigger, LOW);
      digitalWrite(endLED, LOW);
      ar.modus = LFO;
    }

    else if (LFOMode == LOW && RandomMode == HIGH && ar.modus != RANDOM) {
      ar.modus = START;
      analogWrite(A0, 0);
      digitalWrite(peakTrigger, LOW);
      digitalWrite(peakLED, LOW);
      ar.phase_accumulator = 0;
      ar.random_accumulator = 0;
      digitalWrite(endTrigger, LOW);
      digitalWrite(endLED, LOW);
      ar.generated = false;
      ar.modus = RANDOM;
    }

    else if (LFOMode == LOW && RandomMode == LOW && ar.modus != AR) {
      ar.modus = START;
      ar.state = END;
      analogWrite(A0, 0);
      digitalWrite(peakTrigger, LOW);
      digitalWrite(peakLED, LOW);
      ar.phase_accumulator = 0;
      ar.random_accumulator = 0;
      digitalWrite(endTrigger, LOW);
      digitalWrite(endLED, LOW);
      ar.modus = AR;
    }
  }
}
