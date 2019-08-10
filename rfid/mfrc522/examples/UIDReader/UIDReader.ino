#include <mfrc522.h>
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

      byte sak = 0;
      StatusCode selectStatus = ISO14443_3::selectCard(&mfrc522, serNum, &sak);

      if(selectStatus == STATUS_OK) {
        Serial.println("Card was successfully selected.");
      }
    }
  }
  delay(1000);  
}
