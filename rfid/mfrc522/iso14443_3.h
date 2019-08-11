#ifndef ISO14443_3_h
#define ISO14443_3_h

#include "sensor.h"

namespace MFRC522 {

class Sensor;
enum StatusCode : byte;

enum TagCommand : byte {
  PICC_CMD_REQA       = 0x26, // REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
  PICC_SEL_CL1        = 0x93, // SEL command for cascade level 1
  PICC_CMD_HLTA       = 0x50, // HALT command, Type A.
  // MF prefix means Mifare Classic Operation Command. See MF1S50YYX_V1 Document by NXP for more details.
  PICC_MF_AUTH_KEY_A  = 0x60, // Authenticate access to the block that follows with Key A
  PICC_MF_AUTH_KEY_B  = 0x61, // Authenticate access to the block that follows with Key B
  PICC_MF_READ        = 0x30, // Reads a block of an authenticated sector
};

class ISO14443_3 {
    public:
        static StatusCode sendREQA(Sensor* mfrc522, byte *bufferATQA, byte *bufferSize);
        static StatusCode sendHLTA(Sensor* mfrc522);
        static StatusCode sendSEL(Sensor* mfrc522, byte cascadeCommand, byte NVB, byte *sendData, byte *backData, byte *backLen, byte *validReturnBits);
        static StatusCode performAnticollision(Sensor* mfrc522, byte cascadeCommand, byte *uidBuffer);
        static StatusCode selectCard(Sensor* mfrc522, byte *serialNumber, byte *SAK);

    private:
};

}
#endif
