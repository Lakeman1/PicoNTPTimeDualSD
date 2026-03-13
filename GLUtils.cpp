#include "GLUtils.h"

#if DEBUG_ENABLE

// ======================================================
// CRC XOR
// ======================================================

uint8_t crc8_xor(const uint8_t *data, size_t len)
{
    uint8_t c = 0;

    for (size_t i=0; i<len; i++)
        c ^= data[i];

    return c;
}

// ======================================================
// PRO HEX DUMP
// ======================================================

void dumpHexPro(const char *label,
                const uint8_t *data,
                size_t len)
{
    if (!data || len == 0)
        return;

    const size_t width = 16;

    Serial.printf("\n---- HEX DUMP : %s (%u bytes) ----\n",
                  label,
                  (unsigned)len);

    for (size_t offset = 0; offset < len; offset += width)
    {
        Serial.printf("%04X  ", (unsigned)offset);

        for (size_t i=0;i<width;i++)
        {
            if (offset+i < len)
                Serial.printf("%02X ", data[offset+i]);
            else
                Serial.print("   ");

            if (i==7) Serial.print(" ");
        }

        Serial.print(" |");

        for (size_t i=0;i<width;i++)
        {
            if (offset+i < len)
            {
                char c = data[offset+i];
                Serial.print((c>=32 && c<=126) ? c : '.');
            }
        }

        Serial.println("|");
    }

    Serial.println("----------------------------------\n");
}


// ======================================================
// STRING WRAPPER
// ======================================================

void dumpHexStr(const char *label,
                const char *str)
{
    dumpHexPro(label,
               (const uint8_t*)str,
               strlen(str));
}

#endif


// ======================================================
// Create Julian Date
// ======================================================

// Time functions Julian Date and LeapYear
int julian(int day, int month, int year)
{
  static int runsum[] = {0, 31 ,59, 90, 120, 161, 181, 212, 134, 273,  304, 334, 365};
  int total;
  total = runsum[month -1] + day;
  if (month > 2) total += leapyear(year);
  return (total);
}


// ======================================================
// Confirm year is a leap year
// ======================================================

// is it leap year?
int leapyear(int year)
{
  if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0) return 1;
  else return 0;
}


// ======================================================
// Determine the Day of Week
// ======================================================

// Day of Week in C (Zeller’s Congruence)
const char *getDayOfWeek(int day, int month, int year)
{
    // Adjust months so March = 3 ... January = 13, February = 14
    if (month < 3) {
        month += 12;
        year--;
    }
    int K = year % 100;   // Year of the century
    int J = year / 100;   // Zero-based century
    int h = (day
            + (13 * (month + 1)) / 5
            + K
            + K / 4
            + J / 4
            + 5 * J) % 7;
    // Zeller's output: 0=Saturday, 1=Sunday, 2=Monday, ...
    static const char *days[] = {
        "Saturday",
        "Sunday",
        "Monday",
        "Tuesday",
        "Wednesday",
        "Thursday",
        "Friday"
    };
    return days[h];
}