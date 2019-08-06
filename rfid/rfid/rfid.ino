#include <SPI.h>

/*
   This program is written by Neil Johari in an attempt to simplify the RFID
   library into a few important function calls to the MFRC522.
   
   The goal is to interact with an MF1 IC S50 card, which is compliant with
   parts 2 and 3 of the ISO/IEC 14443A. Thus, we only implement Type A
   anticollision in this program.

   For any real control over the sensor, please see miguelbalboa/rfid (GitHub)

   This program has two main parts: Basic interaction with MFRC522 according to
   the datasheet, and then implementation of commands to allow the sensor to
   interact with the MF1 IC S50.

   Useful reading:
   - http://xionghuilin.com/iso-iec-14443-type-ab-summary/
   - https://www.nxp.com/docs/en/application-note/AN10833.pdf
   - https://www.nxp.com/docs/en/application-note/AN1304.pdf
   - https://cdn-shop.adafruit.com/datasheets/S50.pdf
   - https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf

   Program based on multiple other RFID libraries:
   - https://github.com/pimylifeup/MFRC522-python/blob/master/mfrc522/MFRC522.py
   - https://github.com/sumotoy/RFID/blob/master/RFID.cpp
   - https://github.com/miguelbalboa/rfid/blob/master/src/MFRC522.cpp
*/


// 8.1.2 "The interface can handle data speeds up to 10 Mbit/s." -> 10MHz SPI clock speed
// 8.1.2 "Data bytes on both MOSI and MISO lines are sent with the MSB first" 
// 8.1.2 "Data on both MOSI and MISO lines must be stable on the rising edge of
//       the clock" -> We should sample when stable via SPI Mode 0

const SPISettings SPI_CONFIG(10000000, MSBFIRST, SPI_MODE0);


enum MFRC522Register : byte {
  // 8.1.2.3 "The MSB of the first byte defines the mode used."
  // 8.1.2.3 "LSB is set to logic 0."
  // Thus, the address bits must be within bits 6 to 0, so we shift everything
  // over by 1 bit
  
  //ComIEnReg     = 0x02 << 1, // 9.3.1.3  | Controls which events trigger IRQ pin * IRQ NOT SUPPORTED BY HARDWARE (PIN GOES TO NOTHING)
  CommandReg      = 0x01 << 1, // 9.3.1.2  | Sends a command to the sensor; Commands are described in 10.3
  ComIrqReg       = 0x04 << 1, // 9.3.1.5  | Interrupt request bits
  DivIrqReg       = 0x05 << 1, // 9.3.1.6  | Interrupt request bits
  ErrorReg        = 0x06 << 1, // 9.3.1.7  | Error flags from last command executed 
  Status2Reg      = 0x08 << 1, // 9.3.1.9  | Contains status bits of the receiver, transmitter and data mode detector
  FIFODataReg     = 0x09 << 1, // 9.3.1.10 | I/O for FIFO buffer
  FIFOLevelReg    = 0x0A << 1, // 9.3.1.11 | Indicates # bytes in FIFO buffer, also can clear buffer
  ControlReg      = 0x0C << 1, // 9.3.1.13 | Miscallaneous control bits; we use it to figure out # of valid bits in a byte at the end of a frame
  BitFramingReg   = 0x0D << 1, // 9.3.1.14 | Bit-oriented frame settings. Used for adjusting # of valid bits in a frame, and for forcing transmission start using StartSend
  CollReg         = 0x0E << 1, // 9.3.1.15 | First bit collision detected
  ModeReg         = 0x11 << 1, // 9.3.2.2  | TX and RX settings. We change the CRC coprocessor preset value.
  TxControlReg    = 0x14 << 1, // 9.3.2.5  | Controls antenna driver pins
  TxASKReg        = 0x15 << 1, // 9.3.2.6  | Amplitude Shift Keyed modulation setting
  CRCResultRegH   = 0x21 << 1, // 9.3.3.2  | CRC calculation result higher bits.
  CRCResultRegL   = 0x22 << 1, // 9.3.3.2  | CRC calculation result lower bits.
  TModeReg        = 0x2A << 1, // 9.3.3.10 | Timer settings
  TPrescalerReg   = 0x2B << 1, // 9.3.3.10 | Timer prescaler (how many cycles of input clock do we count as a single tick)
  TReloadRegH     = 0x2C << 1, // 9.3.3.11 | Timer reload value higher bits. When the timer hits 0, these are the higher 8 bits for the 16 bit value we start at.
  TReloadRegL     = 0x2D << 1, // 9.3.3.11 | Timer reload value lower bits. When the timer hits 0, these are the lower 8 bits for the 16 bit value we start at.
  VersionReg      = 0x37 << 1, // 9.3.4.8  | Returns software version
};

enum MFRC522Command : byte { 
  // 10.3
  Idle            = B00000000, // 10.3.1.1  | Puts into Idle mode, halting all currently running commands.
  CalcCRC_A        = B00000011, // 10.3.1.4  | Actives the CRC co-processor
  Transmit        = B00000100, // 10.3.1.5  | FIFO buffer is transmitted. Relevant registers must be set for data transmission.
  Receive         = B00001000, // 10.3.1.7  | Activates receiver and listens for data until frame end or exceeded length. If RxModeReg.RxMultiple = 1 then receive won't auto terminate. 
  Transceive      = B00001100, // 10.3.1.8  | Continuously repeat FIFO buffer transmit and then receive from field. 
                               //             Each transmit process must be started by setting the BitFramingReg register’s StartSend bit to logic 1. 
                               //             This command must be cleared by writing any command to the CommandReg register.
                               //             If RxModeReg.RxMultiple = 1 then transcieve doesn't leave receive state.
  MFAuthent       = B00001110, // 10.3.1.9  | Manages 3 pass authentication with Mifare cards.
  SoftReset       = B00001111, // 10.3.1.10 | Resets the MFRC522. Command automatically terminates. All registers are reset, internal memory unchanged.
  
};

enum StatusCode : byte {
  STATUS_OK             , // Success
  STATUS_ERROR          , // Error in communication
  STATUS_COLLISION      , // Collision detected
  STATUS_TIMEOUT        , // Timeout in communication.
  STATUS_NO_ROOM        , // A buffer is not big enough.
  STATUS_INTERNAL_ERROR , // Internal error in the code. -> shouldn't happen
  STATUS_INVALID        , // Invalid argument.
  STATUS_CRC_WRONG      , // The CRC_A does not match
  STATUS_MIFARE_NACK      // A MIFARE PICC responded with NAK.
};

enum TagCommand : byte {
  PICC_CMD_REQA       = 0x26, // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
  PICC_SEL_CL1        = 0x93, // SEL command for cascade level 1
  PICC_CMD_HLTA       = 0x50, // HALT command, Type A.
  // MF prefix means Mifare Classic Operation Command. See MF1S50YYX_V1 Document by NXP for more details.
  PICC_MF_AUTH_KEY_A  = 0x60, // Authenticate access to the block that follows with Key A
  PICC_MF_AUTH_KEY_B  = 0x61, // Authenticate access to the block that follows with Key B
  PICC_MF_READ        = 0x30, // Reads a block of an authenticated sector
};


enum TagType : byte {
    PICC_TYPE_UNKNOWN,
    PICC_TYPE_ISO_14443_4, // PICC compliant with ISO/IEC 14443-4  
    PICC_TYPE_MIFARE_1K, // MIFARE Classic protocol, 1KB 
};


void setup() {
  Serial.begin(9600);
  initReader();
}

void loop() {
  if(isNewCardPresent()) {
    byte serNum[5];

    StatusCode anticollisionStatus = performAnticollision(PICC_SEL_CL1, serNum);

    if(anticollisionStatus == STATUS_OK) {

      char uid[11];
      sprintf(uid,"%02X:%02X:%02X:%02X", serNum[0], serNum[1], serNum[2], serNum[3]);
      Serial.println("UID of card targeted: " + String(uid));

      byte sak;
      selectCard(serNum, sak);

      TagType cardType = getTagType(sak);
      
      if(cardType == PICC_TYPE_MIFARE_1K) {
        Serial.println("Attempting Mifare Classic Operation (MIFARE Read)");

        byte targetBlock = 0x03;
        
        byte sectorKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        StatusCode mfauthentStatus = mifareAuthenticate(PICC_MF_AUTH_KEY_A, targetBlock, sectorKey, serNum);

        if(mfauthentStatus == STATUS_OK) {
          Serial.println("Successfully authenticated target block");

          byte blockData[18] = { 0 };
          byte blockDataSize = 18;
          StatusCode mfReadStatus = mifareRead(targetBlock, blockData, &blockDataSize);

          if(mfReadStatus == STATUS_OK) {
            Serial.println("Successfully read the target block's data:");
            for(int i = 0 ; i < 16 ; i++) {
              Serial.print(blockData[i], HEX);
              Serial.print(" ");
            }
            Serial.println();
            
          } 
          
          // According to MF1S50YYX_V1, "The HLTA command needs to be sent encrypted to the PICC after a successful 
          //  authentication in order to be accepted."
          // Thus, we send HALT A before stopping encrypted communication
          sendHLTA(); 
        }
      }
    } else {
      Serial.print("Anticollision failed with status code: "); Serial.println(anticollisionStatus);
    }
  }

  stopEncryptedCommunication();

  delay(2000);
}

void initReader() {
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


void writeReg(byte addr, byte val) {
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

byte readReg(byte addr) {
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

void readFIFOData(uint8_t numBytes, byte *backData, byte rxAlign) {
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

/*
* Using bit masks allow us to alter only specific bits in a binary value.  This
* helps us preserve bits that may be settings and still change what we want.
* 
* See antennas for a good usage of this.
*/


/*
* Alters a register's value such that the values selected in the mask are turned
* off 
*/
void clearRegBitMask(byte addr, byte mask) {
  writeReg(addr, readReg(addr) & (~mask));
}

/*
* Alters a register's value such that the values selected in the mask are turned
* on 
*/
void setRegBitMask(byte addr, byte mask) {
  writeReg(addr, readReg(addr) | mask);
}

void softReset() {
  writeReg(CommandReg, SoftReset);
  delay(150); // Give plenty of time for oscillator to boot up 
}

void enableAntennas() {
  // Table 62 under 9.3.2.5: Last two bits control whether the antenna driver
  //  pins (TX1 and TX2) are on or off
  setRegBitMask(TxControlReg, B11);
}

void disableAntennas() {
  // Table 62 under 9.3.2.5: Last two bits control whether the antenna driver
  //  pins (TX1 and TX2) are on or off
  clearRegBitMask(TxControlReg, B11);
}

/*
 * Functions to communicate with PCD (MFRC522)
 */

void flushFIFOBuffer() {
  setRegBitMask(FIFOLevelReg, B10000000);
}

void clearMarkedIRQBits() {
  clearRegBitMask(ComIrqReg, B10000000);
}

void clearLoggedCollisionBits() {
  clearRegBitMask(CollReg, B10000000); 
}

// Stops Crypto1 used for Mifare protocol
void stopEncryptedCommunication() {
  clearRegBitMask(Status2Reg, B00001000);
}

/*
 * Uses the CRC Co-processor on the MFRC522 to compute a CRC type A checksum on a given block of data
 * 
 * This function takes in up to 64 bytes of data (size of the FIFO buffer), computes the checksum, and returns it in the result byte array.
 * Note: the result of this function is 2 bytes, so the result byte pointer should allocate at least 2 bytes. 
 */
StatusCode calculateCRC_A(byte *data, byte dataLen, byte *result) {
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
StatusCode executeDataCommand(byte cmd, byte successIrqFlag, 
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
    setRegBitMask(BitFramingReg, B10000000); // StartSend = 1

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

  
  clearRegBitMask(BitFramingReg, B10000000); // Stop forcing data transmission

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

/*
 * Functions to communicate with PICCs
 */

StatusCode sendREQA(byte *bufferATQA, byte *bufferSize) {
  // The ATQA response is 2 bytes long
  if (bufferATQA == nullptr || *bufferSize < 2) 
    return STATUS_NO_ROOM;

  clearLoggedCollisionBits();
  
  // REQA and WUPA are specially crafted to only be 7 bits
  // so that they can't be confused for any other command.
  uint8_t validBits = 7;                  

  byte command = PICC_CMD_REQA;
  StatusCode status = executeDataCommand(Transceive, B00110000, &command, 1, bufferATQA, bufferSize, &validBits, 0);

  if (status != STATUS_OK) 
    return status;
    
  if (*bufferSize != 2 || validBits != 0)    // ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  
  return STATUS_OK;
}

StatusCode sendHLTA() {
  // "The HLTA Command consists of two bytes followed by CRC_A and shall be transmitted within Standard Frame." (ISO/IEC 14443-3 6.3.3)
  byte cmdFrame[4];
  cmdFrame[0] = PICC_CMD_HLTA;
  cmdFrame[1] = 0x00;

  // A CRC is computed for all data bits in the frame
  // The result is stored in the 2nd and 3rd bytes in our command frame
  StatusCode crcStatus = calculateCRC_A(cmdFrame, 2, &cmdFrame[3]);
  
  if(crcStatus != STATUS_OK)
   return crcStatus;

  // This command will result in STATUS_OK for receiving any data from the PICC, and STATUS_TIMEOUT otherwise
  StatusCode status = executeDataCommand(Transceive, B00110000, cmdFrame, 4, nullptr, nullptr, nullptr, 0);

  // ISO/IEC 14443-3 6.3.3 states "If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the HLTA
  //  Command, this response shall be interpreted as 'not acknowledge'."
  switch(status) {
    case STATUS_OK: // The fact that the PICC responded means it NACKed our halt request
      return STATUS_ERROR;
    case STATUS_TIMEOUT: // Timeout means we stopped communicating sucessfully
      return STATUS_TIMEOUT;
    default: // Who knows what happened here
      return status;
  }
}

/*
 * This is a very versatile implementation of the SELECT command described in the ISO/IEC 1444-3
 * It can execute a SEL command for any cascade level, and send any number of bits out.
 * 
 * The integrity of the backData will be verified with a BCC checksum.
 * 
 * Parameters:
 * inputs:
 *  - cascadeCommand: specifies which cascade level SEL should be executed
 *  - nvb: Number Valid Bits encoding as described by the ISO/IEC 1444-3
 *  - sendData: any number of bits of UID you wish to send
 * outputs:
 *  - backData: data returned from PICC. Could be either the remaining bits of a UID for a PICC + BCC, or a SAK + CRC_A
 *  - backLen: length in bytes of data returned
 *  - validReturnBits: number of valid bits in last byte returned
 */
StatusCode sendSEL(byte cascadeCommand, byte nvb, byte *sendData, byte *backData, byte *backLen, byte *validReturnBits) {

  // NVB description:
  //  nvbByteCount: "The upper 4 bits are called “Byte count” and specify the integer part of the number of all valid data bits transmitted
  //   by the PCD (including SEL and NVB) divided by 8" (ISO/IEC 1444-3 6.4.3.3)
  //  nvbBitCount: "The lower 4 bits are called “bit count” and specify the number of all valid data bits transmitted by the PCD
  //   (including SEL and NVB) modulo 8."
  const byte nvbByteCount = (nvb & B11110000) >> 4;
  const byte nvbBitCount = (nvb & B00001111);
  const byte totalKnownBitsForCascadeLevel = nvbByteCount * 8 + nvbBitCount;

  // nvbByteCount accounts for SEL and NVB (so it is always >= 2) and also the integer part of all valid bits transmitted
  //  additionally, if there are some bits in the last byte being sent, we'll need to allocate another byte
  //  additionally, if we are sending the whole UID we will need another 2 bytes for the CRC_A
  const byte cmdBufferSize = nvbByteCount + (nvbBitCount == 0 ? 0 : 1) + (nvb == 0x70 ? 2 : 0);

  byte cmdFrame[cmdBufferSize]; 
  
  cmdFrame[0] = cascadeCommand; // SEL command for corresponding cascade level
  cmdFrame[1] = nvb; // NVB. See ISO/IEC 14443-3 6.4.3.3 for "Coding of NVB"

  // Send the known portion of the Uid
  for(int i = 0 ; i < cmdBufferSize ; i++) {
    cmdFrame[i+2] = sendData[i];
  }

  if(nvb == 0x70) {
   uint8_t crcDataSize = cmdBufferSize - 2;
   // A CRC is computed for all data bits currently in the frame
   // The result is stored in the last 2 bytes in our command frame
   StatusCode crcStatus = calculateCRC_A(cmdFrame, crcDataSize, &cmdFrame[crcDataSize]);

   if(crcStatus != STATUS_OK)
    return crcStatus;
  }

  uint8_t validBits = nvbBitCount; // Transmit only the number of bits specified by NVB in the last byte of sendUid

  clearLoggedCollisionBits();
  StatusCode status = executeDataCommand(Transceive, B00110000, cmdFrame, cmdBufferSize, backData, backLen, &validBits, totalKnownBitsForCascadeLevel % 8);
  *validReturnBits = validBits;
  
  // The last 4 bits of CollReg are the position of the bit collision, b5 is collision invalid/not detected
  byte collisions = readReg(CollReg) & B00111111; 

  if(~collisions & B00100000) { // If b5 is logic 0, a collision was detected
    return STATUS_COLLISION;
  }
  
  if(status == STATUS_OK && backLen && *backLen == 5) {
    // "UID CLn check byte, calculated as exclusive-or over the 4 previous bytes, Type A" (ISO/IEC 1444-3 4)
    byte byte0 = backData[0];
    byte byte1 = backData[1];
    byte byte2 = backData[2];
    byte byte3 = backData[3];
    byte BCC   = backData[4];
    
    byte checksum = byte0 ^ byte1 ^ byte2 ^ byte3;

    if(checksum != BCC)
      return STATUS_ERROR;
  }
  
  return status;
  
}

bool isNewCardPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  StatusCode result = sendREQA(bufferATQA, &bufferSize); 
  
  return (result == STATUS_OK || result == STATUS_COLLISION);
}

/**
 * Attempts to classify a PICC based off the SAK
 */
TagType getTagType(byte SAK) {
  // ISO/IEC 14443-3 6.4.3.4 Table 8 gives basic coding of SAK
  // AN10833 MIFARE Type Identification Procedure Table 5 specifies SAK encodings for NXP cards
  
  // Here, we check for XX1XX0XX, which means "UID complete, PICC compliant with ISO/IEC 14443-4"
  if( (SAK & B00100000) && (~SAK & B00000100) ) { 
    return PICC_TYPE_ISO_14443_4;
  } 
  // Here, we check for XX0XX0XX, which means "UID complete, PICC compliant with ISO/IEC 14443-4"
  else if(~SAK & B00100100) {
    if(SAK == B00001000) // NXP SAK 
      return PICC_TYPE_MIFARE_1K;
  } 
  return PICC_TYPE_UNKNOWN;
}


StatusCode performAnticollision(byte cascadeCommand, byte *uidBuffer) {
  uint8_t knownUidLen = 0;

  bool collision = false;
  uint8_t collisionPosition = 0; // This is the bit within this cascade level that a collision occurred
  byte NVB = 0x20; // Start off with minimum valid bits

  Serial.println("\n performAnticollision() called");
  do {
    if (collision) {
      Serial.println("**Collision detected. Attempting to resolve it.**");
      
      if (readReg(CollReg) & B00100000) { // CollPosNotValid
        Serial.println("collpos not valid. aborting this run at anticollision");
        return STATUS_COLLISION; 
      }
      
      collisionPosition = readReg(CollReg) & B00011111; // The CollPos is b4..b0, this handles bit values 0-31
      if(collisionPosition == 0x00) collisionPosition = 32; // bit collision at 32nd bit is represented as 0x00

      Serial.print("A collision was detected at this bit: "); Serial.println(collisionPosition);
      
      // There are 4 UID bytes for a given cascade level. Since integral division is truncation, this gives us a zero-indexed 
      //  number representing the whole number of bytes needed to represent this collision
      byte collisionByte = collisionPosition / 8; 
      // This is the actual bit within the above specified byte that has collided
      byte collisionBitWithinByte = collisionPosition % 8; 
  
      // Now we craft an NVB code using ISO/IEC 14443-3 6.4.3.3 Table 7
      NVB += (collisionByte << 4) + collisionBitWithinByte;
      Serial.print("Our NVB frame is now 0x"); Serial.println(NVB, HEX);

      Serial.println("Altering collision byte");
      Serial.print("Original byte: "); Serial.println(uidBuffer[collisionByte], BIN);
      
      // We set the collided bit to 1b
      uidBuffer[collisionByte] |= 1 << (collisionBitWithinByte-1); 
      
      Serial.print("Altered byte: "); Serial.println(uidBuffer[collisionByte], BIN);
    }

    byte backDataLen = 5;
    byte validReturnBits;
    StatusCode commandStatus = sendSEL(cascadeCommand, NVB, uidBuffer, uidBuffer, &backDataLen, &validReturnBits);
        
    collision = (commandStatus == STATUS_COLLISION);

    if(commandStatus != STATUS_COLLISION && commandStatus != STATUS_OK) {
      Serial.print("sendSEL did something unexpected! It returned status code: "); Serial.println(commandStatus);
      return commandStatus;
    }
    
  } while (collision == true);
 

  return STATUS_OK;
}

/*
 * This is a very basic naive implementation of anticollision.
 * 
 * It does not handle bit collisions, nor does it escalate cascade levels to retrieve a UID larger than type single. 
 * Additionally, it does not verify that we have received the SAK (Select AcKnowledge) frame which would indicate a full UID.
 */
StatusCode performAnticollision(byte *serialNumber) {
  byte validReturnBits = 0;
  byte backLen = 5;
  
  return sendSEL(PICC_SEL_CL1, 0x20, serialNumber, serialNumber, &backLen, &validReturnBits);
}

/*
 * Given a complete Uid, this function will transition a PICC in the READY state to ACTIVE state
 */
StatusCode selectCard(byte *serialNumber, byte &SAK) {
   byte validReturnBits = 0;
   uint8_t backLen = 3; // We expect the following bytes to come back: SAK (1 byte), CRC_A (2 bytes)
   byte backData[3]; // Allocate the byte array

   StatusCode status = sendSEL(PICC_SEL_CL1, 0x70, serialNumber, backData, &backLen, &validReturnBits);
   
   if(status == STATUS_OK && backLen == 3) {
    SAK = backData[0];
    // The SAK encodes a few states. See ISO/IEC 14443-3 6.4.3.4 Table 8 for coding of SAK

    // Here we are just checking to see if we have a full UID
    if(SAK & B00000100) {
      Serial.println("Cascade bit set: UID not complete");
      return STATUS_INVALID;
    }
   }

   return STATUS_OK;
}

/*
 * The following are MIFARE Classic Operation commands
 */

/*
 * Executes the MFRC522's MFAuthent command
 * 
 * authType should be PICC_MF_AUTH_KEY_A or PICC_MF_AUTH_KEY_B
 * blockAddr is the block address and must be between 0x00 and 0xFF (0x3F for a 1 kb card)
 * sectorKey must be 5 bytes long
 * serNum must be 4 bytes long
 */
StatusCode mifareAuthenticate(TagCommand authType, byte blockAddr, byte *sectorKey, byte *serNum) {
  byte cmdFrame[12];
 
  cmdFrame[0] = authType;
  cmdFrame[1] = blockAddr;
  
  for(int i = 0 ; i < 6 ; i++)
    cmdFrame[i+2] = sectorKey[i];
  
  for(int i = 0 ; i < 4 ; i++)
    cmdFrame[i+8] = serNum[i];

  // Regarding MFAuthent: "This command automatically terminates when the MIFARE card is authenticated
  //  and the Status2Reg register’s MFCrypto1On bit is set to logic 1" (10.3.1.9)
  
  // We check IdleIrq for automatic command termination
  StatusCode cmdStatus = executeDataCommand(MFAuthent, B00010000, cmdFrame, 12, nullptr, nullptr, nullptr, 0);
  // Just in case, we also check Status2Reg register’s MFCrypto1On bit which 
  //  "can only be set to logic 1 by a successful execution of the MFAuthent command"
  bool mfCrypto1On = readReg(Status2Reg) & B00001000;
  
  if(cmdStatus == STATUS_OK) {
    if(mfCrypto1On)
      return STATUS_OK;
    else
      return STATUS_ERROR; // This indicates communication is not encrypted with the card
  } else {
    return cmdStatus;
  }
}

/*
 * Executes MIFARE Read as documented in MF1S50YYX_V1 12.2 
 * 
 * blockAddr is the block address and must be between 0x00 and 0xFF (0x3F for a 1 kb card)
 * data should be an array of size at least 18 elements; this is where the block's data will be stored
 * dataSize is the amount of the array populated
 */
StatusCode mifareRead(byte blockAddr, byte *data, byte *dataSize) {
  if (data == nullptr || *dataSize < 18) 
    return STATUS_NO_ROOM;

  byte cmdFrame[4];
  cmdFrame[0] = PICC_MF_READ;
  cmdFrame[1] = blockAddr;
  
  // A CRC is computed for all data bits in the frame
  // The result is stored in the 2nd and 3rd bytes in our command frame
  StatusCode crcStatus = calculateCRC_A(cmdFrame, 2, &cmdFrame[2]);
  
  if(crcStatus != STATUS_OK)
    return crcStatus;  
    
  return executeDataCommand(Transceive, B00110000, cmdFrame, 4, data, dataSize, nullptr, 0);
}
