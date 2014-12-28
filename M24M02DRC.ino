/*
ST Microelectronics' M24M02DRC 2 Mbit I2C EEPROM

Simple sketch to write data to and read data from two ST Microelectronic's M24M02 2 Mbit EEPROMS
mounted on a small breakout board and hardwired for i2C communication:

Teensy 3.1 pin      Breakout Board
16                      SDA
17                      SCL
3V3                     VIN
GND                     GND

The EEPROMS are very low power devices and can handle voltages from 1.8 to 5.5 V. 
This means that the breakout board can be used with an Arduino UNO without logic level converters
but more interestingly can be powered via digitalWrite(HIGH) to VIN and digitalWrite(LOW) to GND
from any convenient GPIO pins. For the Teensy 3.1, this means that the board can be mounted on the top
to access I2C pins 16/17 or on the bottom to access I2C pins 18/19a and the power and ground can be
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
XSadd << 18 | MSadd << 8 | LSadd. The XSadd will be appended to the device i2C address in the read 
and write functions.

This sketch demonstrates simple byte, multiple bytes, and page reads and writes.

Here is the M24M02DRC data sheet:
http://www.st.com/web/en/resource/technical/document/datasheet/CD00290537.pdf
*/

#include <i2c_t3.h>

#define M24M02DRC_1_DATA_ADDRESS   0x50   // Address of the first 1000 page M24M02DRC EEPROM data buffer, 2048 bits (256 8-bit bytes) per page
#define M24M02DRC_1_IDPAGE_ADDRESS 0x58   // Address of the single M24M02DRC lockable ID page of the first EEPROM
#define M24M02DRC_2_DATA_ADDRESS   0x54   // Address of the second 1000 page M24M02DRC EEPROM data buffer, 2048 bits (256 8-bit bytes) per page
#define M24M02DRC_2_IDPAGE_ADDRESS 0x5C   // Address of the single M24M02DRC lockable ID page of the second EEPROM
#define myVIN 19
#define myGND 6

#define myLed 13
uint8_t data[256], output[256], XSadd, MSadd, LSadd;

void setup() {

  // Setup for Master mode, pins 18/19, external pullups, 400kHz for Teensy 3.1
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_400);
  
  Serial.begin(9600);
  delay(4000);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  pinMode(myVIN, OUTPUT);    // Can use traditional 3V3 and GND from the Teensy 3.1 or Arduino Pro Mini
  digitalWrite(myVIN, HIGH); // or use GPIO pins for power and ground since tha max current drawn is 2 mA!
  pinMode(myGND, OUTPUT);
  digitalWrite(myGND, LOW);

  // Make sure we have a device to talk to!
  I2Cscan();
  
  // First write and then read a single byte to EEPROM 1
  data[0] = 0x34;
  XSadd = 0x01; // 2-bits only, 0 - 3 for four quadrants
  MSadd = 0x76; // 256 bits only, 0 - 255
  LSadd = 0x21; // 256 bits only, 0 - 255
  M24M02DRCwriteByte(M24M02DRC_1_DATA_ADDRESS | XSadd, MSadd, LSadd, data[0]); // write data starting at first byte of page MSadd
  // It takes a maximum of 10 ms for a read or write operation; the EEPROM won't respond until the operation is done
  // There are ways to speed this up by polling, consult the data sheet
  delay(10); 
  
  output[0] = 0x00;
  output[0] = M24M02DRCreadByte(M24M02DRC_1_DATA_ADDRESS | XSadd, MSadd, LSadd);
  Serial.print("Read a byte, should be 0x34, it is 0x"); Serial.println(output[0], HEX); Serial.println(" ");
 
 
  // Next write 10 bytes to and then read ten bytes from EEPROM 2
  data[0] = 0x34;
  data[1] = 0x31; 
  data[2] = 0x09;
  data[3] = 0xBA;
  data[4] = 0xC7;
  data[5] = 0xDE;
  data[6] = 0x03;
  data[7] = 0x8A;
  data[8] = 0x00;
  data[9] = 0xFF;
  XSadd = 0x03; // 2-bits only, 0 - 3 for four quadrants
  MSadd = 0x06; // 256 bits only, 0 - 255
  LSadd = 0x10; // 256 bits only, 0 - 255
  // Make sure there are enough bytes left on the page (a page is the collection of bytes with the
  // same XSadd and MSadd) for the number you are writing otherwise they will wrap to the start of the
  // page and overwrite whetver was there. This is why 256 (or less)-byte page write is so useful.
  M24M02DRCwriteBytes(M24M02DRC_2_DATA_ADDRESS | XSadd, MSadd, LSadd, 10, data); // write data starting at first byte of page MSadd
  // It takes a maximum of 10 ms for a read or write operation; the EEPROM won't respond until the operation is done
  // There are ways to speed this up by polling, consult the data sheet
  delay(10); 
  
  output[0] = 0x00;
  output[1] = 0x00;
  output[2] = 0x00;
  output[3] = 0x00;
  output[4] = 0x00;
  output[5] = 0x00;
  output[6] = 0x00;
  output[7] = 0x00;
  output[8] = 0x00;
  output[9] = 0x00;

  M24M02DRCreadBytes(M24M02DRC_2_DATA_ADDRESS | XSadd, MSadd, LSadd, 10, output);
  Serial.println("Read 10 bytes, they should be 0x34, 0x31, 0x09, 0xBA, 0xC7, 0xDE, 0x03, 0x8A, 0x00, 0xFF:"); 
  Serial.print("0x"); Serial.println(output[0], HEX);
  Serial.print("0x"); Serial.println(output[1], HEX); 
  Serial.print("0x"); Serial.println(output[2], HEX); 
  Serial.print("0x"); Serial.println(output[3], HEX);
  Serial.print("0x"); Serial.println(output[4], HEX); 
  Serial.print("0x"); Serial.println(output[5], HEX); 
  Serial.print("0x"); Serial.println(output[6], HEX);
  Serial.print("0x"); Serial.println(output[7], HEX); 
  Serial.print("0x"); Serial.println(output[8], HEX); 
  Serial.print("0x"); Serial.println(output[9], HEX); Serial.println(" ");
  
  delay(10);

// Lastly, write a page to EEPROM 1 and then read it out
  for (int i = 0; i < 256; i++) {          // generate the "data"
   data[i] = (byte) i;
  }

   XSadd = 0x03; // 2-bits only, 0 - 3 for four quadrants
   MSadd = 0xFF; // 256 bits only, 0 - 255
   LSadd = 0x00;// page starts at LSadd = 0x00 and shouldn't be more than 256 bytes long
   M24M02DRCwriteBytes(M24M02DRC_1_DATA_ADDRESS | XSadd, MSadd, LSadd, 256, data); // write data starting at first byte of page XSadd << 8 | MSadd
   delay(100);
  
  // Read 256 byte page from EEPROM 1
   M24M02DRCreadBytes(M24M02DRC_1_DATA_ADDRESS | XSadd, MSadd, LSadd, 256, output);
   Serial.println("Reading page "); Serial.print(XSadd << 8 | MSadd); Serial.println(" from EEPROM 1"); 

   for (int i = 0; i < 16; i++) {
   Serial.println(" ");
     
   for (int j = 0; j < 16; j++) {
     Serial.print(output[i*16 + j], HEX); Serial.print(" ");
   }
   }
   
  
}

void loop() {
}

////////////////////////////////////////////////////////////////////////////////////////////////
//Useful functions
////////////////////////////////////////////////////////////////////////////////////////////////
//
// I2C read/write functions for motion sensors like the MPU9250 and AK8963 sensors, for example

        void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	Wire.beginTransmission(address);  // Initialize the Tx buffer
	Wire.write(subAddress);           // Put slave register address in Tx buffer
	Wire.write(data);                 // Put data in Tx buffer
	Wire.endTransmission();           // Send the Tx buffer
}

        uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(address);         // Initialize the Tx buffer
	Wire.write(subAddress);	                 // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
	Wire.beginTransmission(address);   // Initialize the Tx buffer
	Wire.write(subAddress);            // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);  // Read bytes from slave register address 
        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }         // Put read results in the Rx buffer
}


// I2C communication with the M24M02DRC EEPROM is a little different from I2C communication with the usual motion sensor
// since the address is defined by two bytes

        void M24M02DRCwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data)
{
	Wire.beginTransmission(device_address);   // Initialize the Tx buffer
	Wire.write(data_address1);                // Put slave register address in Tx buffer
	Wire.write(data_address2);                // Put slave register address in Tx buffer
	Wire.write(data);                         // Put data in Tx buffer
	Wire.endTransmission();                   // Send the Tx buffer
}


       void M24M02DRCwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint16_t count, uint8_t * dest)
{
	if(count > 256) {
        count = 256;
        Serial.print("Page count cannot be more than 256 bytes!");
        }
        
        Wire.beginTransmission(device_address);   // Initialize the Tx buffer
	Wire.write(data_address1);                // Put slave register address in Tx buffer
	Wire.write(data_address2);                // Put slave register address in Tx buffer
	for(uint16_t i=0; i < count; i++) {
	Wire.write(dest[i]);                      // Put data in Tx buffer
	}
        Wire.endTransmission();                   // Send the Tx buffer
}


        uint8_t M24M02DRCreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2)
{
	uint8_t data; // `data` will store the register data	 
	Wire.beginTransmission(device_address);         // Initialize the Tx buffer
	Wire.write(data_address1);                // Put slave register address in Tx buffer
	Wire.write(data_address2);                // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.requestFrom(address, 1);  // Read one byte from slave register address 
	Wire.requestFrom(device_address, (size_t) 1);   // Read one byte from slave register address 
	data = Wire.read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

        void M24M02DRCreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint16_t count, uint8_t * dest)
{  
	Wire.beginTransmission(device_address);   // Initialize the Tx buffer
	Wire.write(data_address1);                     // Put slave register address in Tx buffer
	Wire.write(data_address2);                     // Put slave register address in Tx buffer
	Wire.endTransmission(I2C_NOSTOP);         // Send the Tx buffer, but send a restart to keep connection alive
//	Wire.endTransmission(false);              // Send the Tx buffer, but send a restart to keep connection alive
	uint8_t i = 0;
//        Wire.requestFrom(address, count);       // Read bytes from slave register address 
        Wire.requestFrom(device_address, (size_t) count);  // Read bytes from slave register address 
	while (Wire.available()) {
        dest[i++] = Wire.read(); }                // Put read results in the Rx buffer
}

// simple function to scan for I2C devices on the bus
void I2Cscan() 
{
    // scan for i2c devices
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
