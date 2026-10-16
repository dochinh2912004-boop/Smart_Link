#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <Arduino.h>

inline uint8_t calc_checksum(uint8_t dataType, uint8_t length, const uint8_t *data) {
    uint32_t sum = dataType + length;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum ^ 0xFF);
}

#endif 
