#ifndef __JRLIN_HELPER_H
#define __JRLIN_HELPER_H


#include "Arduino.h"

class JRLINHelper {
  public:
    // For Lin 1.X "start" should = 0, for Lin 2.X "start" should be the addr byte.
    static uint8_t dataChecksum(const uint8_t* message, char nBytes, uint16_t start = 0);
    static uint8_t addrParity(uint8_t addr);
};

#endif
