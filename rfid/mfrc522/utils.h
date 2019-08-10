#ifndef Utils_h
#define Utils_h

#include "sensor.h"

namespace MFRC522 {

class Sensor;

enum TagType : byte {
    PICC_TYPE_UNKNOWN,
    PICC_TYPE_ISO_14443_4, // PICC compliant with ISO/IEC 14443-4  
    PICC_TYPE_MIFARE_1K, // MIFARE Classic protocol, 1KB 
};

class Utils {
    public:
        static void clearRegBitMask(Sensor* mfrc522, byte addr, byte mask);
        static void setRegBitMask(Sensor* mfrc522, byte addr, byte mask);
        static bool isNewCardPresent(Sensor* mfrc522);
        static TagType getTagType(byte SAK);
};

}

#endif
