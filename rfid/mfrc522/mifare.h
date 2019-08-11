#ifndef MIFARE_H
#define MIFARE_H

#include "sensor.h"

namespace MFRC522 {

class Sensor;

enum TagCommand : byte;
enum StatusCode : byte;

class MIFARE {
    public:
        static StatusCode mifareAuthenticate(Sensor* mfrc522, TagCommand authType, byte blockAddr, byte *sectorKey, byte *serNum);
        static StatusCode mifareRead(Sensor* mfrc522, byte blockAddr, byte *data, byte *dataSize);
};

}

#endif
