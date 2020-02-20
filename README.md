# CommsTests
Simple I2C and SPI drivers written in C for Atmel 8-bit microcontroller.
This is a learning project, so the aim is to keep things simple, not perfect.

## Hardware Setup
Hardware consists of an Atmel ATTiny 817 (8-bit microcontroller) Xplained Mini evaluation board, and two I/O expanders, one SPI and one I2C.
The expanders are from Microchip, part numbers MCP23017 and MCP23S17.

- [ATtiny Xplained Mini Evaluation board](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/ATTINY817-XMINI) ([ATtiny817](https://www.microchip.com/wwwproducts/en/ATTINY817) processor)
- [MCP23017](https://www.microchip.com/wwwproducts/en/MCP23017) I2C I/O expander
- [MCP23S17](https://www.microchip.com/wwwproducts/en/MCP23017) SPI I/O expander

Aim is to have LEDs and switches driven and read by the MCU via I2C and SPI. Not particularly groundbreaking but you have to start somewhere.
Connections are shown in the schematic.
![Schematic](schematic.png)

## Software Tools

* [Atmel Studio 7](https://www.microchip.com/mplab/avr-support/atmel-studio-7)
* [git](https://git-scm.com/)

## Notes

* Haven't had the luxury of any test equipment on this one. No multimeter, no scope. Getting things to work has been interesting.
* I2C driver is based loosely on the Atmel version, but with a more simplifed state machine and a less complicated callback system.
* SPI driver is fairly bare-bones, so can still suffer from the odd data collision if you mash the buttons hard enough.
* At this stage it appears the I/O expanders can update the INTCAP register after an MCU read without triggering another interrupt. This means if you mash several buttons at once the MCU won't read the correct values from the INTCAP registers, and won't receive an additional interrupt. This has not been addressed yet... Can try reading from GPIO registers, or failing that take multiple reads.
