#ifndef Sensor_h
#define Sensor_h

#include <Arduino.h>
#include <SPI.h>
#include "iso14443_3.h"
#include "mifare.h"
#include "utils.h"


namespace MFRC522 {

enum Register : byte {
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

enum PCDCommand : byte { 
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


class Sensor {
    public: 
        Sensor();
        void initReader();

        void writeReg(byte addr, byte val);
        byte readReg(byte addr);
        void readFIFOData(uint8_t numBytes, byte *backData, byte rxAlign);

        void enableAntennas();
        void disableAntennas();

        void softReset();
        void flushFIFOBuffer();
        void clearMarkedIRQBits();
        void clearLoggedCollisionBits();
        void stopEncryptedCommunication();

        StatusCode calculateCRC_A(byte *data, byte dataLen, byte *result);
        StatusCode executeDataCommand(PCDCommand cmd, byte successIrqFlag, 
                byte *sendData, byte sendLen, byte *backData, 
                byte *backLen, byte *validBitsInLastByte, byte rxAlign);
};

}
#endif
