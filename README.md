# CommsTests
Basic drivers for I2C and SPI for Atmel 8-bit microcontroller.

## Hardware Setup
Hardware consists of Atmel ATTiny 817 Xplained Mini evaluation board, and two I/O expanders.
The expanders are from Microchip, part numbers MCP23017 and MCP23S17 (I2C and SPI respectively).

- [ATtiny Xplained Mini Evaluation board](https://www.microchip.com/DevelopmentTools/ProductDetails/PartNO/ATTINY817-XMINI) ([ATtiny817](https://www.microchip.com/wwwproducts/en/ATTINY817) processor)
- [MCP23017](https://www.microchip.com/wwwproducts/en/MCP23017) I2C I/O expander
- [MCP23S17](https://www.microchip.com/wwwproducts/en/MCP23017) SPI I/O expander

LEDs and switches are operated via expander ships, and driven and read by the MCU via I2C and SPI.
Connections are shown in the schematic.
![Schematic](schematic.png)

## Software Tools

* [Atmel Studio 7](https://www.microchip.com/mplab/avr-support/atmel-studio-7)
* [git](https://git-scm.com/)

## ToDo

* At this stage it appears the I/O expanders sometimes update the INTCAP register after an MCU read without triggering another interrupt. This means multiple simultaneous button presses can cause incorrect values to be read from the expander INTCAP registers without triggering an interrupt. The easiest fix for this would be an additional verification read.
