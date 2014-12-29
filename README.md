M24M02_EEPROM
=============
ST Microelectronics' M24M02DRC 2 Mbit I2C EEPROM

Simple sketch to write data to and read data from two ST Microelectronic's M24M02 2 Mbit EEPROMS
mounted on a small [breakout board](https://www.tindie.com/products/onehorse/m24m02-2-x-256-kbyte-eeprom-add-on-for-teensy-31/) and hardwired for I2C communication:

Teensy 3.1 pin      Breakout Board
16                      SDA
17                      SCL
3V3                     VIN
GND                     GND

The EEPROMS are very low power devices and can handle voltages from 1.8 to 5.5 V. 
This means that the breakout board can be used with an Arduino UNO without logic level converters
but more interestingly can be powered via digitalWrite(HIGH) to VIN and digitalWrite(LOW) to GND
from any convenient GPIO pins. For the Teensy 3.1, this means that the board can be mounted on the top
to access I2C pins 16/17 or on the bottom to access I2C pins 18/19 and the power and ground can be
taken from pins 8 and 9, or 7 and 6, respectively. 

The two EEPROM devices have different I2C addresses, and the data page access is via part of the device address.
This means that the first EEPROM has I2C addresses 0x50, 0x51, 0x52 and 0x53 (0x50 | 0x00, 0x50 | 0x01,
0x50 | 0x02 and 0x50 | 0x03) while the second has I2C device addresses 0x54, 0x55, 0x56, and 0x57
(0x54 | 0x00, 0x54 | 0x01, 0x54 | 0x02 and 0x54 | 0x03). In addition, each EEPROM has an ID page
accessed via I2C address 0x58 and 0x5C, respectively. This ID page can be locked to permanently store
data that can be read only.

The breakout board has two solder jumpers that control the write protect of each EEPROM. If the trace 
between the solder jumpers is cut. the EEPROM can be read from butnot written to. Resoldering the jumper
returns the EEPROM to its writeable state.

These EEPROMs are in a wafer level package, meaning they are essentially silicon wafers mounted on some 
solder balls attached to the pc board. They don't like rough handling nor bright light, as the latter can 
damage the silicon lattice and affect the memory cells of the chip.

The 2097152 bits of each EEPROM are organized as 1024 pages (pages 0 - 1023), and 256 bytes (0 - 255) per page. The data are
accessed via an 18-bit address. This sketch will use four quadrants determined by the 2-bit XSadd, 
and then the MSadd and LSadd of the remaining 65536 bytes. Thus the 18-bit address of a byte will be
XSadd << 16 | MSadd << 8 | LSadd. The XSadd will be appended to the device i2C address in the read 
and write functions.

This sketch demonstrates simple byte, multiple bytes, and page reads and writes.

Here is the M24M02DRC [data sheet.](http://www.st.com/web/en/resource/technical/document/datasheet/CD00290537.pdf)
