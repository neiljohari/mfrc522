#include "iso14443_3.h"

StatusCode ISO14443_3::sendREQA(MFRC522* mfrc522, byte *bufferATQA, byte *bufferSize) {
  // The ATQA response is 2 bytes long
  if (bufferATQA == nullptr || *bufferSize < 2) 
    return STATUS_NO_ROOM;

  mfrc522->clearLoggedCollisionBits();
  
  // REQA and WUPA are specially crafted to only be 7 bits
  // so that they can't be confused for any other command.
  uint8_t validBits = 7;                  

  byte command = PICC_CMD_REQA;
  StatusCode status = mfrc522->executeDataCommand(Transceive, B00110000, &command, 1, bufferATQA, bufferSize, &validBits, 0);

  if (status != STATUS_OK) 
    return status;
    
  if (*bufferSize != 2 || validBits != 0)    // ATQA must be exactly 16 bits.
    return STATUS_ERROR;
  
  return STATUS_OK;
}

StatusCode ISO14443_3::sendHLTA(MFRC522* mfrc522) {
  // "The HLTA Command consists of two bytes followed by CRC_A and shall be transmitted within Standard Frame." (ISO/IEC 14443-3 6.3.3)
  byte cmdFrame[4];
  cmdFrame[0] = PICC_CMD_HLTA;
  cmdFrame[1] = 0x00;

  // A CRC is computed for all data bits in the frame
  // The result is stored in the 2nd and 3rd bytes in our command frame
  StatusCode crcStatus = mfrc522->calculateCRC_A(cmdFrame, 2, &cmdFrame[3]);
  
  if(crcStatus != STATUS_OK)
   return crcStatus;

  // This command will result in STATUS_OK for receiving any data from the PICC, and STATUS_TIMEOUT otherwise
  StatusCode status = mfrc522->executeDataCommand(Transceive, B00110000, cmdFrame, 4, nullptr, nullptr, nullptr, 0);

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
StatusCode ISO14443_3::sendSEL(MFRC522* mfrc522, byte cascadeCommand, byte NVB, byte *sendData, byte *backData, byte *backLen, byte *validReturnBits) {

  // NVB description:
  //  nvbByteCount: "The upper 4 bits are called “Byte count” and specify the integer part of the number of all valid data bits transmitted
  //   by the PCD (including SEL and NVB) divided by 8" (ISO/IEC 1444-3 6.4.3.3)
  //   Note: b8 isn't used in NVB 
  //  nvbBitCount: "The lower 4 bits are called “bit count” and specify the number of all valid data bits transmitted by the PCD
  //   (including SEL and NVB) modulo 8."
  //   Note: b4 isn't used in NVB
  const byte nvbByteCount = (NVB & B01110000) >> 4;
  const byte nvbBitCount = (NVB & B00000111);
  const byte totalKnownBitsForCascadeLevel = nvbByteCount * 8 + nvbBitCount;

  // We need 2 bytes for SEL and NVB
  //  Then we need bytes for the whole UID. This is nvbByteCount and then an extra byte for any bits after the last whole byte.
  //  additionally, if we are sending the whole UID we will need another 2 bytes for the CRC_A 
  const uint8_t bytesForWholeUid = (nvbByteCount - 2) + (nvbBitCount == 0 ? 0 : 1); // We subtract 2 because NVB accounts for SEL and NVB itself
  const uint8_t cmdBufferSize = 2 + bytesForWholeUid + (NVB == 0x70 ? 2 : 0); 
  
  // The buffer can be at most 9 bytes long: SEL (1) + NVB (1) + UID (1-4) + BCC (0-1) + CRC_A (2)
  // Not all of this will always be used though depending on UID length
  byte cmdFrame[9]; 
  cmdFrame[0] = cascadeCommand; // SEL command for corresponding cascade level
  cmdFrame[1] = NVB; // NVB. See ISO/IEC 14443-3 6.4.3.3 for "Coding of NVB"

  // Send the known portion of the Uid
  for(int i = 0 ; i < bytesForWholeUid ; i++) {
    cmdFrame[i+2] = sendData[i];
  }

  if(NVB == 0x70) {
   uint8_t crcDataSize = cmdBufferSize - 2;
   // A CRC is computed for all data bits currently in the frame
   // The result is stored in the last 2 bytes in our command frame
   StatusCode crcStatus = mfrc522->calculateCRC_A(cmdFrame, crcDataSize, &cmdFrame[crcDataSize]);

   if(crcStatus != STATUS_OK)
    return crcStatus;
  }

  uint8_t validBits = nvbBitCount; // Transmit only the number of bits specified by NVB in the last byte of sendUid

  mfrc522->clearLoggedCollisionBits();
    
  StatusCode status = mfrc522->executeDataCommand(Transceive, B00110000, cmdFrame, cmdBufferSize, backData, backLen, &validBits, totalKnownBitsForCascadeLevel % 8);
  *validReturnBits = validBits;
  
  // The last 4 bits of CollReg are the position of the bit collision, b5 is collision invalid/not detected
  byte collisions = mfrc522->readReg(CollReg) & B00111111; 

  if(~collisions & B00100000) { // If b5 is logic 0, a collision was detected
    return STATUS_COLLISION;
  }

  if(status == STATUS_OK && backLen && *backLen == 5) {
    byte mask = 0xFF << nvbBitCount;
    
    // "UID CLn check byte, calculated as exclusive-or over the 4 previous bytes, Type A" (ISO/IEC 1444-3 4)
    // The BCC is not recalculated for partially sent Uid bytes, so for byte0 we merge in the portion of the UID sent from cmdFrame
    //    More info on this: cmdFrame index 0 and 1 are SEL and NVB, then the following indices are UID bytes. 1 + bytesForWholeUid gets the last byte with bits being sent.
    //                       The mask takes only the bits actually being sent and merges it with bits from first Uid byte coming in
    byte byte0 = (cmdFrame[1 + bytesForWholeUid] & ~mask) | (backData[0] & mask); 
    byte byte1 = backData[1];
    byte byte2 = backData[2];
    byte byte3 = backData[3];
    byte BCC   = backData[4];
    
    byte checksum = ((byte0 ^ byte1) ^ byte2) ^ byte3;

    if(checksum != BCC) {
      Serial.println("FAILED CHECKSUM");
      Serial.print("Got: "); Serial.print(checksum, BIN); Serial.print(", expected:"); Serial.println(BCC, BIN);
      return STATUS_ERROR;
    }
  }
  
  return status;
  
}


StatusCode ISO14443_3::performAnticollision(MFRC522* mfrc522, byte cascadeCommand, byte *uidBuffer) {
  bool collision = false;
  uint8_t collisionPosition = 0; // This is the bit within this cascade level that a collision occurred
  byte NVB = 0x20; // Start off with minimum valid bits
  
  do {
    if (collision) {
      Serial.println("**Collision detected. Attempting to resolve it.**");
      
      if (mfrc522->readReg(CollReg) & B00100000) { // CollPosNotValid
        Serial.println("CollPos not valid. Aborting this anticollision.");
        return STATUS_COLLISION; 
      }
      
      collisionPosition = mfrc522->readReg(CollReg) & B00011111; // The CollPos is b4..b0, this handles bit values 0-31
      if(collisionPosition == 0x00) collisionPosition = 32; // bit collision at 32nd bit is represented as 0x00

      Serial.print("A collision was detected at this bit: "); Serial.println(collisionPosition);
      
      // There are 4 UID bytes for a given cascade level. Since integral division is truncation, this gives us a zero-indexed 
      //  number representing the whole number of bytes needed to represent this collision
      byte collisionByte = collisionPosition / 8; 
      // This is the actual bit within the above specified byte that has collided, it is one-indexed
      byte collisionBitWithinByte = collisionPosition % 8; 
  
      // Now we craft an NVB code using ISO/IEC 14443-3 6.4.3.3 Table 7
      NVB += (collisionByte << 4) + collisionBitWithinByte;
      Serial.print("Our NVB frame is now 0x"); Serial.println(NVB, HEX);

      // We set the collided bit to 1b
      uidBuffer[collisionByte] |= 1 << (collisionBitWithinByte-1); 
    }


    int knownBytes = ((NVB & B11110000) >> 4) - 2; // Subtract out 2 bytes, one for SEL and one for NVB
      
    byte backData[5] = {0}; // backData can be at most 5 bytes long, but we might not use all of this allocated memory
    byte backDataLen = 5 - knownBytes;
    byte validReturnBits;
    StatusCode commandStatus = sendSEL(mfrc522, cascadeCommand, NVB, uidBuffer, backData, &backDataLen, &validReturnBits);

    // sendSEL populates backData with the remaining Uid bits for CLn
    // The following code will update uidBuffer with the new Uid bits while preserving the existing known bits
    // This is effectively a Uid merge
    for(int i = knownBytes ; i < 5 ; i++) {
      if(i == knownBytes) {
        byte mask = 0xFF << (NVB & B00001111);
        uidBuffer[knownBytes] = (uidBuffer[knownBytes] & ~mask) | (backData[0] & mask);
      } else {
        uidBuffer[i] = backData[i - knownBytes];
      }
    }

    collision = (commandStatus == STATUS_COLLISION);

    if(commandStatus != STATUS_COLLISION && commandStatus != STATUS_OK) {
      return commandStatus;
    }
    
  } while (collision == true);
 

  return STATUS_OK;
}

/*
 * Given a complete Uid, this function will transition a PICC in the READY state to ACTIVE state
 */
StatusCode ISO14443_3::selectCard(MFRC522* mfrc522, byte *serialNumber, byte *SAK) {
   byte validReturnBits = 0;
   uint8_t backLen = 3; // We expect the following bytes to come back: SAK (1 byte), CRC_A (2 bytes)
   byte backData[3]; // Allocate the byte array

   StatusCode status = sendSEL(mfrc522, PICC_SEL_CL1, 0x70, serialNumber, backData, &backLen, &validReturnBits);
   
   if(status == STATUS_OK && backLen == 3) {
    *SAK = backData[0];
    // The SAK encodes a few states. See ISO/IEC 14443-3 6.4.3.4 Table 8 for coding of SAK

    // Here we are just checking to see if we have a full UID
    if(*SAK & B00000100) {
      Serial.println("Cascade bit set: UID not complete");
      return STATUS_INVALID;
    }
   }

   return STATUS_OK;
}
