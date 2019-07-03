#include <SPI.h>

/*
   This program is written by Neil Johari in an attempt to simplify the RFID library into a few important function calls to the MFRC522.
   
   The goal is to interact with an MF1 IC S50 card, which is compliant with parts 2 and 3 of the ISO/IEC 14443A. Thus, we only implement
   Type A anticollision in this program.

   For any real control over the sensor, please see miguelbalboa/rfid (GitHub)

   This program has two main parts: Basic interaction with MFRC522 according to the datasheet, and then implementation of commands to
   allow the sensor to interact with the MF1 IC S50.

   Useful reading:
   - http://xionghuilin.com/iso-iec-14443-type-ab-summary/
   - https://www.nxp.com/docs/en/application-note/AN10833.pdf
   - https://cdn-shop.adafruit.com/datasheets/S50.pdf

*/


// 8.1.2 "The interface can handle data speeds up to 10 Mbit/s." -> 10MHz SPI clock speed
// 8.1.2 "Data bytes on both MOSI and MISO lines are sent with the MSB first" -> Most Significant Byte First
// 8.1.2 "Data on both MOSI and MISO lines must be stable on the rising edge of the clock" -> We should sample when stable via SPI Mode 0
const SPISettings SPI_CONFIG(10000000, MSBFIRST, SPI_MODE0);


enum MFRC522Register : byte {
  // 8.1.2.3 "The MSB of the first byte defines the mode used."
  // 8.1.2.3 "LSB is set to logic 0."
  // Thus, the address bits must be within bits 6 to 0, so we shift everything over by 1 bit
  
  //ComIEnReg     = 0x02 << 1, // 9.3.1.3 | Controls which events trigger IRQ pin * IRQ NOT SUPPORTED BY HARDWARE (PIN GOES TO NOTHING)
  CommandReg      = 0x01 << 1, // 9.3.1.2 | Sends a command to the sensor; Commands are described in 10.3
  TxControlReg    = 0x14 << 1, // 9.3.2.5 | Controls antenna driver pins
  TxASKReg        = 0x15 << 1, // 9.3.2.6 | Amplitude Shift Keyed modulation setting
  VersionReg      = 0x37 << 1, // 9.3.4.8 | Returns software version
};

enum MFRC522Command : byte { 
  // 10.3
  
  SoftReset       = B00001111, // Resets the MFRC522. Command automatically terminates. All registers are reset, internal memory unchanged.
};

enum TagCommand : byte {

};


void setup() {
  initReader();


  Serial.begin(9600);
  SPI.begin();

  // 9.3.4.8
  Serial.print("MFRC522 Version detected: ");
  Serial.print(readReg(VersionReg), HEX);
  Serial.println(" [The 9 is the chiptype (MFRC522). '1' stands for MFRC522 version 1.0 and '2' stands for MFRC522 version 2.0.]");
  
  softReset();

  // TODO: Set PICC timeout
  
  writeReg(TxASKReg, B01000000); // Type A uses 100 % ASK modulation (see https://www.rfwireless-world.com/Terminology/10-ASK-modulation-vs-100-ASK-modulation.html). This register value forces 100 % ASK modulation.
  
  // TODO: Alter CRC coprocessor preset value

  Serial.println(readReg(TxControlReg), BIN);
  
  enableAntennas(); 

  Serial.println(readReg(TxControlReg), BIN);

  disableAntennas();
  
  Serial.println(readReg(TxControlReg), BIN);

  
/* miguelbolboa appears to reset these occasionally, so if something goes wrong maybe ensure these are at def values??
  Serial.println(readReg(0x12 << 1), HEX); // tx baud  -> 0h
  Serial.println(readReg(0x13 << 1), HEX); // rx baud  -> 0h
  Serial.println(readReg(0x24 << 1), HEX); // modwidth setting? -> 26h
*/
}

void loop() {
}


void writeReg(byte addr, byte val) {
  // The MFRC522 follows the most common SPI communication pattern
  // Thus, we can follow https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPI_CONFIG);

  digitalWrite(SS, LOW); // Choose slave

  // 8.1.2.3 "The MSB of the first byte defines the mode used. ...  To write data to the MFRC522 the MSB must be set to logic 0."
  // 8.1.2.3 "LSB is set to logic 0."
  // We could just pass in the address formatted correctly (and we should), but this helps ensure we are in writing mode
  SPI.transfer(B01111110 & addr);

  SPI.transfer(val);

  digitalWrite(SS, HIGH);      // Release slave
  SPI.endTransaction();
}

byte readReg(byte addr) {
  // The MFRC522 follows the most common SPI communication pattern
  // Thus, we can follow https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPI_CONFIG);

  digitalWrite(SS, LOW); // Choose slave

  // 8.1.2.3 "The MSB of the first byte defines the mode used. To read data from the MFRC522 the MSB is set to logic 1. "
  SPI.transfer(B10000000 | addr);

  byte response = SPI.transfer(0);

  digitalWrite(SS, HIGH);      // Release slave
  SPI.endTransaction();

  return response;
}

void clearRegBitMask(byte addr, byte mask) {
  writeReg(addr, readReg(addr) & (~mask));
}


void setRegBitMask(byte addr, byte mask) {
  writeReg(addr, readReg(addr) | mask);
}


void initReader() {
  pinMode(SS, OUTPUT); // SS defined in pins_arduino.h
  digitalWrite(SS, HIGH);
}


void softReset() {
  writeReg(CommandReg, SoftReset);
  delay(150);  
}

void enableAntennas() {
  // Table 62 under 9.3.2.5: Last two bits control whether the antenna driver pins (TX1 and TX2) are on or off
  // We want to preserve bits 7:2 as they are settings, so we will just check what the state of the last two bits are and flip them on if necessary
    
  setRegBitMask(TxControlReg, B11);
}

void disableAntennas() {
  // Table 62 under 9.3.2.5: Last two bits control whether the antenna driver pins (TX1 and TX2) are on or off
  // As with enableAntennas(), we want to preserve bits 7:2. This is accomplished by doing an AND with 1s, and forcing the last two bits off
  clearRegBitMask(TxControlReg, B11);
}
