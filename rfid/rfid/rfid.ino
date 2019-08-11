#include <SPI.h>
#include "src/mfrc522/sensor.h"

using namespace MFRC522;

MFRC522::Sensor mfrc522;  // Create MFRC522 instance

void setup() {
  Serial.begin(9600);
  mfrc522.initReader();
}

void loop() {
  if(Utils::isNewCardPresent(&mfrc522)) {
    byte serNum[5] = {0x00};

    StatusCode anticollisionStatus = ISO14443_3::performAnticollision(&mfrc522, PICC_SEL_CL1, serNum);

    if(anticollisionStatus == STATUS_OK) {

      char uid[11];
      sprintf(uid,"%02X:%02X:%02X:%02X", serNum[0], serNum[1], serNum[2], serNum[3]);
      Serial.println("UID of card targeted: " + String(uid));
      Serial.print("The BCC for the UID targeted is "); Serial.println(serNum[4], BIN);

      byte sak = 0;
      StatusCode selectStatus = ISO14443_3::selectCard(&mfrc522, serNum, &sak);

      TagType cardType = Utils::getTagType(sak);
      
      if(selectStatus != STATUS_OK) {
        Serial.print("selectCard failed. Error code "); Serial.println(selectStatus);
      }
      
      if(cardType == PICC_TYPE_MIFARE_1K) {
        Serial.println("Attempting Mifare Classic Operation (MIFARE Read)");

        byte targetBlock = 0x03;
        
        byte sectorKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        StatusCode mfauthentStatus = MIFARE::mifareAuthenticate(&mfrc522, PICC_MF_AUTH_KEY_A, targetBlock, sectorKey, serNum);

        if(mfauthentStatus == STATUS_OK) {
          Serial.println("Successfully authenticated target block");

          byte blockData[18] = { 0 };
          byte blockDataSize = 18;
          StatusCode mfReadStatus = MIFARE::mifareRead(&mfrc522, targetBlock, blockData, &blockDataSize);

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
          ISO14443_3::sendHLTA(&mfrc522); 
        }
      }
    } else {
      Serial.print("Anticollision failed with status code: "); Serial.println(anticollisionStatus);
    }
  }

  mfrc522.stopEncryptedCommunication();

  Serial.println();
  delay(2000);  
}
