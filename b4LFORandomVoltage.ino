#include <Arduino.h>

#define SAMPLE_RATE 48000
#define PHASE_RESOLUTION 32  // Phase accumulator uses 32-bit resolution
#define ADC_MAX 4095         // 12-bit ADC max value
#define DAC_MAX 1023         // 10-bit DAC max value

volatile uint32_t phase = 0;             // Phase accumulator (Q16.16 fixed-point)
volatile uint32_t phaseIncrement = 0;   // Phase increment (controls frequency)
volatile int16_t waveformTilt = 0;      // Tilt factor: -32768 (down-ramp) to +32768 (up-ramp)

void setup() {
  analogReadResolution(12);  // 12-bit ADC
  analogWriteResolution(10); // 10-bit DAC
  analogWrite(A0, 0);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  // Configure timer interrupt for LFO generation
  tcConfigure(SAMPLE_RATE);
}

void loop() {
  // Read potentiometer values
  uint16_t potFreq = analogRead(A1);  // Frequency control
  uint16_t potTilt = analogRead(A2); // Waveform tilt control

  // Map potentiometer values to frequency and tilt range
  float lfoFreq = map(potFreq, 0, ADC_MAX, 0.1f * (1 << 16), 30.0f * (1 << 16)); // Q16.16
  waveformTilt = map(potTilt, 0, ADC_MAX, -32768, 32767);                        // Q15.16

  // Update phase increment
  phaseIncrement = (uint32_t)(lfoFreq * (1.0f / SAMPLE_RATE));
}

void TC4_Handler() {
  

  // Update phase
  phase += phaseIncrement;

  // Generate waveform
  uint16_t dacValue = generateWaveform(phase, waveformTilt);

  // Output waveform to DAC
  DAC->DATA.reg = dacValue;

  // Clear the interrupt flag
  TC4->COUNT16.INTFLAG.bit.OVF = 1;
}

uint16_t generateWaveform(uint32_t phase, int16_t tilt) {
  // Phase mapped to 0-1 as a Q16.16 fraction
  uint16_t phaseFraction = (phase >> 16) & 0xFFFF;

  // Calculate waveform value based on tilt
  int32_t waveformValue;
  if (tilt < 0) {
    // Down-ramp to triangle
    waveformValue = (phaseFraction * (32768 + tilt)) >> 15;
  } else {
    // Triangle to up-ramp
    waveformValue = (phaseFraction * (32768 - tilt)) >> 15;
  }

  // Wrap waveformValue to DAC range
  waveformValue = constrain(waveformValue, 0, DAC_MAX);
  return (uint16_t)waveformValue;
}

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
