#include "sensor.h"

namespace MFRC522 {

const SPISettings SPI_CONFIG(10000000, MSBFIRST, SPI_MODE0);

Sensor::Sensor() {}

void Sensor::initReader() {
  pinMode(SS, OUTPUT); // SS defined in pins_arduino.h
  digitalWrite(SS, HIGH);

  SPI.begin();

  // 9.3.4.8
  Serial.print("MFRC522 Version detected: ");
  Serial.print(readReg(VersionReg), HEX);
  Serial.println(" [The 9 is the chiptype (MFRC522). '1' stands for MFRC522 version 1.0 and '2' stands for MFRC522 version 2.0.]");
  
  softReset();

  writeReg(TModeReg, B10000000); // Start timer at end of tramission (TAuto = 1)


  /*
     Lets set the timer to be a countdown for 25 milliseconds.  Equation (5)
     states that t_d1 = timer_period * ticks, where ticks is TReloadVal+1. 
     Thus, timer_period = (TPrescaler*2+1)/(13.56 MHz).  To make it easy, lets
     just have the period be 25 microseconds, and then have 1000 ticks (reload
     value) to make our timer be 25 miliseconds.
   */
  
  // For a timer_period of 25μs, we need a TPrescaler of ( 25μs*13.56MHz - 1 )/2 =
  //  169 = 0xA9 (solved using the timer_period equation)
  writeReg(TPrescalerReg, 0xA9); 
  // By reloading the timer with 0x3E8 = 1000, we effectively have 1000 ticks of
  // our timer before we hit 0. 
  writeReg(TReloadRegH, 0x03);   
  writeReg(TReloadRegL, 0xE8);

  // Type A uses 100 % ASK modulation (see
  // https://www.rfwireless-world.com/Terminology/10-ASK-modulation-vs-100-ASK-modulation.html).
  // This register value forces 100 % ASK modulation
  writeReg(TxASKReg, B01000000); 

  // According to ISO/IEC 14443-3 6.1.6, "the initial register content shall be '6363'" w.r.t. the CRC coprocessor preset value
  // This register's last 2 bits determine the preset value. By default the register is 0x3F, we want the last values to be 01
  //  so we use 0x3D
  writeReg(ModeReg, 0x3D);
  
  enableAntennas(); // Antennas are disabled by a soft reset
}

void Sensor::writeReg(byte addr, byte val) {
  // The MFRC522 follows the most common SPI communication pattern
  // Thus, we can follow https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPI_CONFIG);

  digitalWrite(SS, LOW); // Choose slave

  /*
  8.1.2.3 "The MSB of the first byte defines the mode used. ...  To write
  data to the MFRC522 the MSB must be set to logic 0." 
  8.1.2.3 "LSB is set to logic 0."
  We could just pass in the address formatted correctly (and we should), but
  this helps ensure we are in writing mode
  */

  SPI.transfer(B01111110 & addr);

  SPI.transfer(val);

  digitalWrite(SS, HIGH);      // Release slave
  SPI.endTransaction();
}

byte Sensor::readReg(byte addr) {
  // The MFRC522 follows the most common SPI communication pattern
  // Thus, we can follow https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPI_CONFIG);

  digitalWrite(SS, LOW); // Choose slave

  // 8.1.2.3 "The MSB of the first byte defines the mode used. To read data from
  // the MFRC522 the MSB is set to logic 1. "
  SPI.transfer(B10000000 | addr);

  byte response = SPI.transfer(0);

  digitalWrite(SS, HIGH);      // Release slave
  SPI.endTransaction();

  return response;
}

/*
* This implements 8.1.2.1 of the MFRC522 datasheet specificially for the FIFODataReg
* 
* As expected, it repeatedly reads the register and discards the first received data byte.
* It ends reading by sending 0x00.
* 
* The rxAlign parameter controls which bit (from the least significant bit in MSB form) of the first 
* byte the data begins being written from. The other bits are preserved.
*/

void Sensor::readFIFOData(uint8_t numBytes, byte *backData, byte rxAlign) {
  // The MFRC522 follows the most common SPI communication pattern
  // Thus, we can follow https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPI_CONFIG);

  digitalWrite(SS, LOW); // Choose slave

  // 8.1.2.3 "The MSB of the first byte defines the mode used. To read data from
  // the MFRC522 the MSB is set to logic 1. "
  SPI.transfer(B10000000 | FIFODataReg);

    
  // Table 6 shows the order of MOSI and MISO
  // The reads and returned data are staggered
  for(int i = 0 ; i < numBytes - 1 ; i++) {
    byte bufferDataReturned = SPI.transfer(B10000000 | FIFODataReg);
    if(i == 0 && rxAlign) {
      byte mask = 0xFF << rxAlign;
      backData[0] = (backData[0] & ~mask) | (bufferDataReturned & mask);
    } else {
      backData[i] = bufferDataReturned;
    }
  }
  
  backData[numBytes-1] =  SPI.transfer(0x00);

  digitalWrite(SS, HIGH);      // Release slave
  SPI.endTransaction();
}


void Sensor::enableAntennas() {
  // Table 62 under 9.3.2.5: Last two bits control whether the antenna driver
  //  pins (TX1 and TX2) are on or off
  Utils::setRegBitMask(this, TxControlReg, B11);
}

void Sensor::disableAntennas() {
  // Table 62 under 9.3.2.5: Last two bits control whether the antenna driver
  //  pins (TX1 and TX2) are on or off
    Utils::clearRegBitMask(this, TxControlReg, B11);
}



void Sensor::softReset() {
  writeReg(CommandReg, SoftReset);
  delay(150); // Give plenty of time for oscillator to boot up 
}


void Sensor::flushFIFOBuffer() {
    Utils::setRegBitMask(this, FIFOLevelReg, B10000000);
}

void Sensor::clearMarkedIRQBits() {
    Utils::clearRegBitMask(this, ComIrqReg, B10000000);
}

void Sensor::clearLoggedCollisionBits() {
    Utils::clearRegBitMask(this, CollReg, B10000000); 
}

// Stops Crypto1 used for Mifare protocol
void Sensor::stopEncryptedCommunication() {
    Utils::clearRegBitMask(this, Status2Reg, B00001000);
}

/*
 * Uses the CRC Co-processor on the MFRC522 to compute a CRC type A checksum on a given block of data
 * 
 * This function takes in up to 64 bytes of data (size of the FIFO buffer), computes the checksum, and returns it in the result byte array.
 * Note: the result of this function is 2 bytes, so the result byte pointer should allocate at least 2 bytes. 
 */
StatusCode Sensor::calculateCRC_A(byte *data, byte dataLen, byte *result) {
  writeReg(CommandReg, Idle); // Idle halts any currently running commands
  writeReg(DivIrqReg, B00000100); // First bit indicates we want to clear marked bit, marked bit is CRC interrupt
  flushFIFOBuffer();
  // 8.3.1 "Writing to this register stores one byte in the FIFO buffer and
  //  increments the internal FIFO buffer write pointer"
  for (int i = 0; i < dataLen; i++) {
    writeReg(FIFODataReg, data[i]);    
  }
  
  writeReg(CommandReg, CalcCRC_A);
  
  long start = millis();

  while(millis() < start + 100) {
    byte markedIrqFlags = readReg(DivIrqReg);
    // DivIrqReg return bits:
    //  Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
    //  b7   b6       b5       b4         b3       b2     b1       b0
    
    if(markedIrqFlags & B00000100) { // If the CRCIrq is set, the CRC coprocessor is done with our calculation
      writeReg(CommandReg, Idle); // Idle halts any currently running commands
      result[0] = readReg(CRCResultRegL);
      result[1] = readReg(CRCResultRegH);
      return STATUS_OK;
    }
  }
  
  return STATUS_TIMEOUT;
}

/*
 * Transfers data to FIFO buffer, executes command, and returns data back from
 * the buffer. 
 */
StatusCode Sensor::executeDataCommand(PCDCommand cmd, byte successIrqFlag, 
                              byte *sendData, byte sendLen, byte *backData, 
                              byte *backLen, byte *validBitsInLastByte, byte rxAlign) { 
  writeReg(CommandReg, Idle); // Idle halts any currently running commands
  clearMarkedIRQBits();
  flushFIFOBuffer();
  
  // 8.3.1 "Writing to this register stores one byte in the FIFO buffer and
  //  increments the internal FIFO buffer write pointer"
  for (int i = 0; i < sendLen; i++) {
    writeReg(FIFODataReg, sendData[i]);    
  }


  // Frame adjustment for BitFramingReg
  byte txLastBits = validBitsInLastByte ? *validBitsInLastByte : 0;
  byte bitFraming = (rxAlign << 4) + txLastBits; // RxAlign is defined in bits 4 to 6, and TxLastBits is 2 to 0
  
  writeReg(BitFramingReg, bitFraming);

  // Execute command
  writeReg(CommandReg, cmd); 

  // Typically, commands that need data will immediately process FIFO, except
  //  for Transceive: 10.2 "Transceive command. Using this command, transmission
  //  is started with the BitFramingReg register’s StartSend bit."
  if(cmd == Transceive)
    Utils::setRegBitMask(this, BitFramingReg, B10000000); // StartSend = 1

  // Since we set TAuto = 1, the Timer unit has started now that transmission is
  //  over.
   
  // From my tests, on an Arduino Nano it takes ~20 microseconds to perform a
  //  register read and 2 bit comparison operations
  uint16_t i;
  for (i = 2000; i > 0; i--) {

    // ComIrqReg return bits:
    //  Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
    //  b7 . b6 .  b5 .  b4 .    b3 .       b2 .       b1 .   b0
    byte markedIrqFlags = readReg(ComIrqReg); 

    if (markedIrqFlags & successIrqFlag) // Command successful 
      break;

    // 8.4.1 TimeIrq is fired when the timer is decremented from 1 to 0
    // Since the entire timer has finished, we know the time elapsed is 25 ms.
    // See where we configure the timer during initialization for an
    //  explanation.
    if (markedIrqFlags & B00000001) 
      return STATUS_TIMEOUT; 
  }

  
  Utils::clearRegBitMask(this, BitFramingReg, B10000000); // Stop forcing data transmission

  // 20 microseconds * 2000 = 40 ms elapsed and we didn't get a successful flag
  //  or even a timer timeout. 
  // Probably means we can't communicate with the sensor anymore.
  if (i == 0) 
    return STATUS_TIMEOUT;

  // ErrorReg return bits:
  //  WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
  //  b7 .  b6 .    b5 .     b4 .       b3 .    b2 .   b1 .      b0
  if(readReg(ErrorReg) & B00010011) // BufferOvfl, ParityErr, and ProtocolErr
    return STATUS_ERROR;
  
  if (backData && backLen) {
    byte fifoByteCount = readReg(FIFOLevelReg); 
    
    if (fifoByteCount > *backLen) 
      return STATUS_NO_ROOM;
      
    *backLen = fifoByteCount;
    readFIFOData(fifoByteCount, backData, rxAlign);

   
    // Some of the data in the last byte might not actually be part of the data we want.
    // ControlReg.RxLastBits (b2 to b0) indicates the number of valid bits in
    //  the last byte, "if this value is 000b, the whole byte is valid"
    if (validBitsInLastByte) 
      *validBitsInLastByte = readReg(ControlReg) & B00000111; // extracts the last 3 bits 
    
  }

  if (readReg(ErrorReg) & B00001000)  // CollErr, see ErrorReg return bits comment above
    return STATUS_COLLISION;
    
  
  return STATUS_OK;
}

}
