#ifndef BRDCANLIB_H
#define BRDCANLIB_H

#include <Arduino.h>  // always include Arduino core

class BDRCANLib {
public:
    BDRCANLib();   // constructor
    void begin();         // initialization

    struct CanMessage {
        uint32_t id;       // CAN ID
        uint8_t data[8];   // Data payload (up to 8 bytes)
        uint8_t length;    // Length of data payload
    };

private:
};

#endif
