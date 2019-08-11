#ifndef Utils_h
#define Utils_h

#include "mfrc522.h"

class MFRC522;

enum TagType : byte {
    PICC_TYPE_UNKNOWN,
    PICC_TYPE_ISO_14443_4, // PICC compliant with ISO/IEC 14443-4  
    PICC_TYPE_MIFARE_1K, // MIFARE Classic protocol, 1KB 
};

class Utils {
    public:
        static void clearRegBitMask(MFRC522* mfrc522, byte addr, byte mask);
        static void setRegBitMask(MFRC522* mfrc522, byte addr, byte mask);
        static bool isNewCardPresent(MFRC522* mfrc522);
        static TagType getTagType(byte SAK);
};

#endif
