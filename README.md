# MSP430 LOADCELL ADC12b
Project code for embedded board MSP430F5529LP to measure loadcell weighting and sendback to serial USB onboard.

## 1. Requirements
- Install VS Code with PlatformIO.
- Install MSP430 FET driver to load fw through USB cable.
- Launchkit Pack MSP430F5529LP
- HX711 ADC 12bit Converter Board

## 2. Library
- <Arduino.h>  Energia/ Arduino Framework for Serial Comm.
- <msp430.h>   Register Configuration for GPIO and Peripherals.

## 3. Pinout Config
- P4.7 GREEN LED for blinking.
- P1.0 RED LED toggled by button at P1.1.
- Loadcell ADC readback via HX711 24bit ADC Convert Board
  * DATA = P4.2 /DI
  * SCLK = P4.1 /DO
- Loadcell ADC readback via HX711 24bit ADC Convert Board
  * DATA = P6.2 /DI
  * SCLK = P6.1 /DO