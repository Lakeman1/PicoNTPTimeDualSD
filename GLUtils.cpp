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
        "Sat",
        "Sun",
        "Mond",
        "Tue",
        "Wed",
        "Thu",
        "Fri"
    };
    return days[h];
}

// A alarm timer
// // alarm interupt callback
// void alarm_callback() {
//     char buffer[64];
//     // Serial.print("Alarm Call Back: ");
//     snprintf(buffer, sizeof(buffer),
//              "ALARM TRIGGERED! ");

//     // Serial.print("Alarm Call Back: ");

//     Serial.println(buffer);
//     flash(LED_PIN1,10,60);
//     // Display on OLED
//     display.stopscroll();
//     display.clearDisplay();
//     display.display();
//     delay(1000);
//     display.setTextSize(2);
//     display.setCursor(0, 0);
//     display.println("Alarm!!:");
//     display.setTextSize(1);
//     display.setCursor(0, 30);
//     display.println(buffer);
//     display.display();
//     delay(1000);
//     display.startscrollright(0x00,0x01);
//     // Serial.println("ALARM TRIGGERED! It is 9:00 AM!");
// }

// // function to set the alarm interupt 
// void set_alarm(int8_t aHour, int8_t aMinute) // (int aHour, int aMinute)
// {
//   char buffer[64];
//   // Configure alarm
//   datetime_t alarm = {
//       .year  = -1,   // wildcards = every day
//       .month = -1,
//       .day   = -1,
//       .dotw  = -1,
//       .hour  = aHour,  // at 6 pm  // aHour
//       .min   = aMinute,   // aMinute
//       .sec   = 0
//   };
 
//   rtc_set_alarm(&alarm, alarm_callback);
    
//   Serial.print("Setting Alarm Call Back: ");
//   snprintf(buffer, sizeof(buffer),"ALARM set for %02d:%02d\n",aHour,aMinute);
//   Serial.println(buffer);
// }

//  helper to standardize formatting DS3231 time print
// bool formatTimestamp(char *buf, size_t len) {
//     if (!rtc_ok) return false;

//     return snprintf(buf, len,
//         "DS:%02d-%02d-%02d %02d:%02d:%02d\n %s\n",
//         rtc_cache.y, rtc_cache.mo, rtc_cache.d,
//         rtc_cache.h, rtc_cache.m, rtc_cache.s,t) > 0;
// }