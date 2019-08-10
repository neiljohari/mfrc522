#ifndef Utils_h
#define Utils_h

#include "mfrc522.h"

class MFRC522;
enum TagType : byte;

class Utils {
    public:
        static void clearRegBitMask(MFRC522* mfrc522, byte addr, byte mask);
        static void setRegBitMask(MFRC522* mfrc522, byte addr, byte mask);
        static bool isNewCardPresent(MFRC522* mfrc522);
        static TagType getTagType(byte SAK);
};

#endif
