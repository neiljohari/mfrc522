#include "utils.h"

/*
 * Alters a register's value such that the values selected in the mask are turned
 * off 
 */
void Utils::clearRegBitMask(MFRC522* mfrc522, byte addr, byte mask) {
    mfrc522->writeReg(addr, mfrc522->readReg(addr) & (~mask));
}

/*
 * Alters a register's value such that the values selected in the mask are turned
 * on 
 */
void Utils::setRegBitMask(MFRC522* mfrc522, byte addr, byte mask) {
    mfrc522->writeReg(addr, mfrc522->readReg(addr) | mask);
}

bool Utils::isNewCardPresent(MFRC522* mfrc522) {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  StatusCode result = ISO14443_3::sendREQA(mfrc522, bufferATQA, &bufferSize); 
  
  return (result == STATUS_OK || result == STATUS_COLLISION);
}

/**
 * Attempts to classify a PICC based off the SAK
 */
TagType Utils::getTagType(byte SAK) {
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
