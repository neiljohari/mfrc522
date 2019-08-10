#ifndef MIFARE_H
#define MIFARE_H

#include "mfrc522.h"

class MFRC522;

enum TagCommand : byte;
enum StatusCode : byte;

class MIFARE {
    public:
        static StatusCode mifareAuthenticate(MFRC522* mfrc522, TagCommand authType, byte blockAddr, byte *sectorKey, byte *serNum);
        static StatusCode mifareRead(MFRC522* mfrc522, byte blockAddr, byte *data, byte *dataSize);
};

#endif
