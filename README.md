# MSP430 LOADCELL ADC12b
Project code for embedded board MSP430F5529LP to measure loadcell weighting and sendback to serial USB onboard.

## 1. Requirements
- Install VS Code with PlatformIO.
- Install MSP430 FET driver to load fw through USB cable.
- Launchkit Pack MSP430F5529LP

##. 2. Library
- <Arduino.h>  Energia/ Arduino Framework for Serial Comm.
- <msp430.h>   Register Configuration for GPIO and Peripherals.

##. 3. Pinout Config
- P4.7 GREEN LED for blinking.
- P1.0 RED LED toggled by button at P1.1.
- ADC12 at P6.6:
  * Vref+ = 5V.
  * Vref- = GND.
  * Vsense+ = P6.6
  * Vsense- = GND