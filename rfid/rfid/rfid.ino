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
   - https://cdn-shop.adafruit.com/datasheets/S50.pdf

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
  ErrorReg        = 0x06 << 1, // 9.3.1.7  | Error flags from last command executed 
  FIFODataReg     = 0x09 << 1, // 9.3.1.10 | I/O for FIFO buffer
  FIFOLevelReg    = 0x0A << 1, // 9.3.1.11 | Indicates # bytes in FIFO buffer, also can clear buffer
  ControlReg      = 0x0C << 1, // 9.3.1.13 | Miscallaneous control bits; we use it to figure out # of valid bits in a byte at the end of a frame
  BitFramingReg   = 0x0D << 1, // 9.3.1.14 | Bit-oriented frame settings. Used for adjusting # of valid bits in a frame, and for forcing transmission start using StartSend
  CollReg         = 0x0E << 1, // 9.3.1.15 | First bit collision detected
  ModeReg         = 0x11 << 1, // 9.3.2.2  | TX and RX settings. We change the CRC coprocessor preset value.
  TxControlReg    = 0x14 << 1, // 9.3.2.5  | Controls antenna driver pins
  TxASKReg        = 0x15 << 1, // 9.3.2.6  | Amplitude Shift Keyed modulation setting
  TModeReg        = 0x2A << 1, // 9.3.3.10 | Timer settings
  TPrescalerReg   = 0x2B << 1, // 9.3.3.10 | Timer prescaler (how many cycles of input clock do we count as a single tick)
  TReloadRegH     = 0x2C << 1, // 9.3.3.11 | Timer reload value higher bits. When the timer hits 0, these are the higher 8 bits for the 16 bit value we start at.
  TReloadRegL     = 0x2D << 1, // 9.3.3.11 | Timer reload value lower bits. When the timer hits 0, these are the lower 8 bits for the 16 bit value we start at.
  VersionReg      = 0x37 << 1, // 9.3.4.8  | Returns software version
};

enum MFRC522Command : byte { 
  // 10.3
  Idle            = B00000000, // 10.3.1.1  | Puts into Idle mode, halting all currently running commands.
  Transmit        = B00000100, // 10.3.1.5  | FIFO buffer is transmitted. Relevant registers must be set for data transmission.
  Receive         = B00001000, // 10.3.1.7  | Activates receiver and listens for data until frame end or exceeded length. If RxModeReg.RxMultiple = 1 then receive won't auto terminate. 
  Transceive      = B00001100, // 10.3.1.8  | Continuously repeat FIFO buffer transmit and then receive from field. 
                               //             Each transmit process must be started by setting the BitFramingReg register’s StartSend bit to logic 1. 
                               //             This command must be cleared by writing any command to the CommandReg register.
                               //             If RxModeReg.RxMultiple = 1 then transcieve doesn't leave receive state.
  SoftReset       = B00001111, // 10.3.1.10 | Resets the MFRC522. Command automatically terminates. All registers are reset, internal memory unchanged.
  
};

enum StatusCode : byte {
  STATUS_OK             , // Success
  STATUS_ERROR          , // Error in communication
  STATUS_COLLISION      , // Collission detected
  STATUS_TIMEOUT        , // Timeout in communication.
  STATUS_NO_ROOM        , // A buffer is not big enough.
  STATUS_INTERNAL_ERROR , // Internal error in the code. -> shouldn't happen
  STATUS_INVALID        , // Invalid argument.
  STATUS_CRC_WRONG      , // The CRC_A does not match
  STATUS_MIFARE_NACK      // A MIFARE PICC responded with NAK.
};

enum TagCommand : byte {
  PICC_CMD_REQA   = 0x26 // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
};

void setup() {
  Serial.begin(9600);
  initReader();
}

void loop() {
  if(isNewCardPresent())
    Serial.println("New card detected");
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

/*
 * Transfers data to FIFO buffer, executes command, and returns data back from
 * the buffer. 
 */
StatusCode executeDataCommand(byte cmd, byte successIrqFlag, 
                              byte *sendData, byte sendLen, byte *backData, 
                              byte *backLen, byte *validBitsInLastByte) { 
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
  writeReg(BitFramingReg, txLastBits);

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

  // Serial.println(readReg(ErrorReg),BIN);

  if (backData && backLen) {
    byte fifoByteCount = readReg(FIFOLevelReg); 

    if (fifoByteCount > *backLen)
      return STATUS_NO_ROOM;
    
    
    *backLen = fifoByteCount;

    for (int i = 0; i < fifoByteCount; i++) 
      backData[i] = readReg(FIFODataReg);  

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
  StatusCode status = executeDataCommand(Transceive, B00110000, &command, 1, bufferATQA, bufferSize, &validBits);

  if (status != STATUS_OK) 
    return status;
    
  if (*bufferSize != 2 || validBits != 0)    // ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  
  return STATUS_OK;
}

bool isNewCardPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
   
  StatusCode result = sendREQA(bufferATQA, &bufferSize); 
  return (result == STATUS_OK || result == STATUS_COLLISION);
}
