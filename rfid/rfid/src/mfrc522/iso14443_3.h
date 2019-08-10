#ifndef ISO14443_3_h
#define ISO14443_3_h

#include "mfrc522.h"

class MFRC522;
enum StatusCode : byte;

class ISO14443_3 {
    public:
        static StatusCode sendREQA(MFRC522* mfrc522, byte *bufferATQA, byte *bufferSize);
        static StatusCode sendHLTA(MFRC522* mfrc522);
        static StatusCode sendSEL(MFRC522* mfrc522, byte cascadeCommand, byte NVB, byte *sendData, byte *backData, byte *backLen, byte *validReturnBits);
        static StatusCode performAnticollision(MFRC522* mfrc522, byte cascadeCommand, byte *uidBuffer);
        static StatusCode selectCard(MFRC522* mfrc522, byte *serialNumber, byte *SAK);

    private:
};

#endif
