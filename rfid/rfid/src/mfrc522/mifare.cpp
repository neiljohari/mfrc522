#include "mifare.h"


/*
 * Executes the MFRC522's MFAuthent command
 * 
 * authType should be PICC_MF_AUTH_KEY_A or PICC_MF_AUTH_KEY_B
 * blockAddr is the block address and must be between 0x00 and 0xFF (0x3F for a 1 kb card)
 * sectorKey must be 5 bytes long
 * serNum must be 4 bytes long
 */
StatusCode MIFARE::mifareAuthenticate(MFRC522* mfrc522, TagCommand authType, byte blockAddr, byte *sectorKey, byte *serNum) {
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
  StatusCode cmdStatus = mfrc522->executeDataCommand(MFAuthent, B00010000, cmdFrame, 12, nullptr, nullptr, nullptr, 0);
  // Just in case, we also check Status2Reg register’s MFCrypto1On bit which 
  //  "can only be set to logic 1 by a successful execution of the MFAuthent command"
  bool mfCrypto1On = mfrc522->readReg(Status2Reg) & B00001000;
  
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
StatusCode MIFARE::mifareRead(MFRC522* mfrc522, byte blockAddr, byte *data, byte *dataSize) {
  if (data == nullptr || *dataSize < 18) 
    return STATUS_NO_ROOM;

  byte cmdFrame[4];
  cmdFrame[0] = PICC_MF_READ;
  cmdFrame[1] = blockAddr;
  
  // A CRC is computed for all data bits in the frame
  // The result is stored in the 2nd and 3rd bytes in our command frame
  StatusCode crcStatus = mfrc522->calculateCRC_A(cmdFrame, 2, &cmdFrame[2]);
  
  if(crcStatus != STATUS_OK)
    return crcStatus;  
    
  return mfrc522->executeDataCommand(Transceive, B00110000, cmdFrame, 4, data, dataSize, nullptr, 0);
}
