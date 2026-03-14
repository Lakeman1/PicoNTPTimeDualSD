#ifndef GLUTILS_H
#define GLUTILS_H


#include <Arduino.h>

#include <stdint.h>
#include <math.h>
// ======================================================
// USER CONFIG
// ======================================================

// Enable / Disable debug utilities
#ifndef DEBUG_ENABLE
#define DEBUG_ENABLE 1
#endif


// ======================================================
// FUNCTION DECLARATIONS  Prototype(s)
// ======================================================

// CRC
uint8_t crc8_xor(const uint8_t *data, size_t len);

// Hex dump
void dumpHexPro(const char *label,
                const uint8_t *data,
                size_t len);

// String helper
void dumpHexStr(const char *label,
                const char *str);

// Day of Week in C (Zeller’s Congruence)
const char *getDayOfWeek(int day, int month, int year);

// is it leap year?
int leapyear(int year);

// Time functions Julian Date and LeapYear
int julian(int day, int month, int year);

#endif  // must be at the end to guard the whole GLUTILS_H
// don't use pragma with this method