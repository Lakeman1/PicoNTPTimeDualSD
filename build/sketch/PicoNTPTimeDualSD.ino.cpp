#line 1 "/home/nor/PicoNTPTimeDualSD/PicoNTPTimeDualSD/PicoNTPTimeDualSD.ino"
// DEBUGING CODE   ----- Preprocessor code ------------
#define DEBUG_ENABLE 1   // ← flip to 0 for production

#if DEBUG_ENABLE
  #define DBG(x)        Serial.print(x)
  #define DBGLN(x)      Serial.println(x)
  #define DBGPF(...)    Serial.printf(__VA_ARGS__)
  #define DBGSNPF(...) snprintf(__VA_ARGS__)
  #define SP(...) if (SPrint) {Serial.print(__VA_ARGS__);}
  #define SPF(...) if (SPrint) {Serial.printf(__VA_ARGS__);}
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGPF(...)
  #define DBGSNPF(...)
  #define SP(...)
  #define SPF(...)
#endif

// if (SPrint) {Serial.print("\nRTC_Read\t");}

//  turn on or off test printing
#define SERIAL_PRINT true // ← flip to 0 or false  for production
bool SPrint = SERIAL_PRINT;

// #define TURNONPRT     // clear comment  TO turn on printing again

// #if DEBUG_ENABLE
//   #define DBF(fmt, ...) printf("[DBF] " fmt "\n", ##__VA_ARGS__)
//   #define DBN(fmt, ...) printf(fmt,__VA_ARGS__)
// #else
//   #define DBF(fmt, ...)
//   #define DBN(fmt, ...)
// #endif

#define DBG_ONCE(...) \
  do { static bool _done=false; if(!_done){ _done=true; DBGPF(__VA_ARGS__); } } while(0)


#define LFCYCLE 6  //every 1 hours so 6 - 10 minute cycles so 1 is 10 minutes, 2 is 20 minutes etc. 
#define INFO_ENABLE 1  //  ← flip to 0 for production
#if INFO_ENABLE
  #define LF(...) listFiles(__VA_ARGS__)
#else
  #define LF(...)
#endif



// -------------------------
#include "assert.h"  //for preprocessor in program folder

#include <Arduino.h>

// fence WiFi
#define WIFI

#ifdef WIFI
#include <WiFi.h>
#endif

#include "pico/stdlib.h"
// #include <pico/cyw43_arch.h>

// #include "pico/util/datetime.h"

// Watchdog Libraries
#include "hardware/watchdog.h"

#define WATCHDOG_TIMEOUT_MS 120000  // 2 minutes

// #include <Arduino.h>
#include "hardware/rtc.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
// #include "pico/util/datetime.h"
#include <uRTCLib.h>
// For eeprom on the DS3231
#include "uEEPROMLib.h"

#define ETEMP 4   // location for eTemp storage in EEPROM
#define CDATE 8   // location for compDate storage in EEPROM

// Instantiate RTC object on default I2C (GP0=SDA, GP1=SCL)
uRTCLib rtc(0x68);  // 0x68 is DS3231 I2C address
// uEEPROMLib eeprom
// const uint8_t EEPROM_ADDR = 0x57;
uEEPROMLib eeprom(0x57);

const uint8_t DS3231_ADDR = 0x68;  // RTC component
const uint8_t EEPROM_ADDR = 0x57;  //eEPROM component

// --------------------------------------------- SD --------------------------------------------------
#include "SdFat.h"  // better than sd.h
#include <SPI.h>    // SPI bus 

// -------------------  SPI pins
// sd SPI bus 1
#define PIN_SD1_SS   17
#define PIN_SD1_SCK  18
#define PIN_SD1_MISO 16
#define PIN_SD1_MOSI 19
#define PIN_SD1_PWR  26

// sd SPI bus 2
#define PIN_SD2_SS   13
#define PIN_SD2_SCK  10
#define PIN_SD2_MISO 12
#define PIN_SD2_MOSI 11
#define PIN_SD2_PWR  27

// Create two separate SdFat instances  have two SD modules separately wired 
SdFat sdFatA;
SdFat sdFatB;

typedef struct  {       // template for the structure; no allocation done
    SdFat*    sd;       // <-- REQUIRED
    SPIClass* spi;        // pointers 

    uint8_t pinMOSI;      // intergers
    uint8_t pinMISO;
    uint8_t pinSCK;
    uint8_t pinSS;
    uint8_t pinPWR;
} sd_bus_t;

sd_bus_t sdA = {        // struct allocated using sd_bus_t template or typedef and allocating memory
    .sd   = &sdFatA,
    .spi  = &SPI,

     .pinMOSI = PIN_SD1_MOSI, //19
     .pinMISO = PIN_SD1_MISO, //16
     .pinSCK  = PIN_SD1_SCK,  //18
     .pinSS   = PIN_SD1_SS,   //17
     .pinPWR  = PIN_SD1_PWR   //26
};

sd_bus_t sdB = {        // dito to above
     .sd  = &sdFatB,
     .spi = &SPI1,

     .pinMOSI = PIN_SD2_MOSI, //11
     .pinMISO = PIN_SD2_MISO, //12
     .pinSCK  = PIN_SD2_SCK,  //10
     .pinSS   = PIN_SD2_SS,   //13
     .pinPWR  = PIN_SD2_PWR   //27
};

// Define Chip Select pins for both modules
 uint8_t SD1_CS = 17; 
 uint8_t SD2_CS = 13;

// Define GPIO for power switch // critical for SD power via MOSFED
 uint8_t SD1_PWR = 26;
 uint8_t SD2_PWR = 27;  //9

const uint8_t rtc_PWR = 15; //DS3231 power control via n-mosfed gnd switch

const uint8_t rtc_LED = 14; // DS3231 activity LED
const uint8_t pi_LED  = 22; // PICO activity LED
const uint8_t eP_LED  =  8; // 24C32 EEPROM activity LED

#define DEGREE "\xC2\xB0"

// File objects for logging
FsFile csvFile1;
FsFile csvFile2;

bool sdA_Ready = false;
bool sdB_Ready = false;
// bool sdOK = false;

// SD Activity LED
const uint8_t SD1_LED = 28;
const uint8_t SD2_LED = 21;

// for sd recovery
bool sdOK = false;
uint8_t sdFailCount = 0;
const uint8_t SD_FAIL_LIMIT = 3;

bool sd1Healthy = true;
bool sd2Healthy = true;

// --------------------------------- sd end --------------------------------------------------------

#include <time.h>
#include <Wire.h>

// const int SDA_PIN = 4;
// const int SCL_PIN = 5;

// --------------------- SSD1306 ----------------------------------------------
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define TIMECYCLE 300 // seconds // this call back will be used to compare DS3231 time to PICO time and reset PICO time if needed.
#define SDTIMECYCLE 160  //160 Seconds 
// Start the repeating timer call back AFTER setup
// 40,000 µs = 25 FPS smooth fades
// #define RAINBOW 40000    // 40000 microseconds = .04 sec

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ------------------------------------SSD1306 end -----------------------------------------------------------------
//  may not be needed 
#define PICOW  //pi pico w 

// LED's - not currently used
// #define LED_PIN 0  // Built-in LED on Pico W Red LED
// #define LED_PIN1 1  // GPI01 on Pico W White LED
// #define LED_PIN2 2  // GPI02 on Pico W Yellow LED

//-------------- BMP or BME includes
// #define BMP280
#ifdef BME280
  #include "Adafruit_BMP280.h"
  //Setup connection of the sensor
  Adafruit_BMP280 bmp;  // I2C 
#else
  #include "Adafruit_BME280.h"
  //Setup connection of the sensor
  Adafruit_BME280 bme;  // I2C
#endif

#define SEALEVELPRESSURE_HPA (1013.25)
#define GUNLAKE_HPA (1013.25)    // TO BE SET
#define CURRENT_HPA (1013)    // RICHMOND --- TO BE SET


// Global  Variables: ------------------------------------------------------------ Globals -------------------------
// Watch_dog  variables  --  things to track and then act upon with Watch_dog
volatile uint32_t i2c_error_count;
volatile uint32_t i2c_timeout_count;

volatile uint32_t fs_fail_count;
volatile uint32_t watchdog_kicks;

// expose them  in 1 hour file list or other ways
// DBG("health: i2c=%lu fs=%lu wd=%lu",
//     i2c_error_count, fs_fail_count, watchdog_kicks);


// loop times
unsigned long now;
unsigned long previousLF,previous,cycles,dispCycle = 0;
bool firstLoop, firstCycle = true;

bool flagDateTimeDisp = true;

long t = 600000;
int  f = 10;

// Globals for SD sdData
// sd Data
char sdData[100] = "";  
// ASSERT(sizeof(sdData) >= 100);

// sd write status
bool sdWriteStatus = true;

// --------------------------Globals for PICO
struct {
  uint16_t y;
  uint8_t mo,d,dow,h,m,s;
} pico_rtc_cache;

// ------------------------ Globals for DS3231

// DS3231 --- cache variables for rtc.  
struct {
  uint8_t y, mo, d, dow, h, m, s;
  float t;
} rtc_cache;

bool rtc_ok = false;


// RTC checking variables
volatile bool rtcCheckFlag = false;
volatile bool rtcMismatchFlag = false; // = 0

uint8_t rtcMismatchCount = 0;

// RTC control variables
bool rtcRecovered = false;

volatile bool checkRTC = false;

// RTC data
bool rtcOK = false;
char rtcData[42] = "CCYY,MM,DD,HH,MM,SS,FFFFFFFF,\n"; //       increased rtcData from 32 to 42
char rtcTemp[10];   // 8 chars and NULL
// ASSERT(sizeof(rtcData) >= 42);
// ASSERT(sizeof(rtcTemp) >= 10);


int currentHour = 0;
// int flushHour   = 0;
int currentMth  = 0;
int currentYear = 0;

//  Globals for BME or BMP Sensors
// ---------------------------------------------- BMP-E data

bool bmAvailable = false;  // set by bmInit()
bool bmInitialized = false;

char bmData[37] = "FFFFFFFF,FFFFFFFF,FFFFFFFF,FFFFFFFF\n";
// ASSERT(sizeof(bmData) >= 37);

// BM(EP)280 temperature and string version
float bmTemp = 0;
// character float
char bmCTemp[10] = "";
// ASSERT(sizeof(bmCTemp) >= 10);

// declare the symbol for degree for temperature
const char degree[] = "\u00b0";


// -------------- coldestTemp contains the coldest temperature recorded.  
float coldestTemp = 129.9;  // set to force an coldest file update right at the beginning
char  coldestCTemp[10] = "129.9"; // character float variable for the coldest tempStr
// ASSERT(sizeof(coldestCTemp) >= 10);
// coldest data
char coldestData[100] = "";
// ASSERT(sizeof(coldestData) >= 100);
// ----------------- uEEPROM Data // 24C32 EEPROM variables
float eTemp;
char e_string[80] = ""; //51 //61 FOR BME
// ASSERT(sizeof(e_string) >= 80);
int string_length = 0;

char bufferpi[114];
// ASSERT(sizeof(bufferpi) >= 114);
char bufferds[114];
// ASSERT(sizeof(bufferds) >= 114);

// timer structure for the SD Update
struct repeating_timer sdUpdateTimer;

// RTCLib RTC;

// extern "C" 
// {
// #include "pico/utl/datetime.h"
// }

#ifdef WIFI
// my wifi
const char* ssid = "SunorGuest";
const char* password = "Guest11760";

// time server
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = -28800;
const int   daylightOffset_sec = 3600;
#endif

struct tm timeinfo;  // made global  from NTP Server


// global  print buffer
char buffer[180];
// ASSERT(sizeof(buffer) >= 180);

// --  uEEPROM Data --  stores a julian date
long Global_comDate = 0;

//  -------------------------------------- functions -------------------------



// DS3231 Read Routine
// ---------------------------- I2C Routines -------------------------------------------
// RTC Read Function

// scan for I2C Bus devices
void scanI2C() {
  DBGLN("I2C scan start");

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();

    if (err == 0) {
      DBG("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
    delay(2);
  }

  DBGLN("I2C scan done");
}

void flLED(int8_t led) // call when ds3231 rtc activity to show activity
{
    digitalWrite(led,HIGH);
    delay(15);
    digitalWrite(led,LOW);
}

//  ------------ Wire begin
void i2cInit(void) 
{
    URTCLIB_WIRE.begin();
    URTCLIB_WIRE.setClock(100000); // could be 400000

    // REQUIRED: prevent I2C bus deadlock
    URTCLIB_WIRE.setTimeout(100, true);
    URTCLIB_WIRE.clearTimeoutFlag();

#ifdef DEBUG
  // Sanity: timeout mechanism must start clean
  ASSERT(URTCLIB_WIRE.getTimeoutFlag() == false);
#endif
#ifdef DEBUG
  DBG("I2C timeout set to %d ms", 25);
#endif

}

// power up the DS3231
bool rtcPowerOn(uint8_t Pwr) // DS3231
{
  bool pin_state = LOW;
  char buffer[80] = "";

  // Ensure I2C pins are NOT driving
  // pinMode(SDA, INPUT);
  // pinMode(SCL, INPUT);
  pinMode(Pwr, OUTPUT);
  digitalWrite(Pwr, HIGH);     // P-MOSFET ON
  
  delay(500);
  pin_state = gpio_get(Pwr); 
    // Test the state using an if statement
  if (pin_state) {
      snprintf(buffer,sizeof(buffer),"Power %d is HIGH (3.3V)\n", Pwr);
      DBG(buffer);
      return true;
  } else {
      snprintf(buffer,sizeof(buffer),"Power %d is LOW (0V)\n",  Pwr);
      DBG(buffer);
      return false;
  }
}

// routines to control DS3231 power and I2C bus
bool rtcPowerOff(uint8_t Pwr) //DS3231
{
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);

  bool pin_state = LOW;
  char buffer[80] = "";
  pinMode(Pwr, OUTPUT);

  digitalWrite(Pwr, LOW);  // P-MOSFET OFF
  delay(200); 
    pin_state = gpio_get(Pwr);
  // Test the state using an if statement
  if (pin_state) {
      snprintf(buffer,sizeof(buffer),"Power Pin %d is HIGH (3.3V)\n", Pwr);
      DBG(buffer);
      return false;
  } else {
      snprintf(buffer,sizeof(buffer),"Power Pin %d is LOW (0V)\n",  Pwr);
      DBG(buffer);
      return true;
  }                  // ensure full collapse
}

// rtc power cycle 
void rtcPowerCycle()
 {
    pinMode(SDA, INPUT);
    pinMode(SCL, INPUT);

    rtcPowerOff(rtc_PWR);
    delay(300);

    rtcPowerOn(rtc_PWR);
    delay(500);
}


// I2C  bus and DS3231 Recovery
bool i2cRecover() 
{   // To reset and recover the i2c bus from a potential bad condition
    Serial.printf("\n--- Attempting I2C Recovery ---\n");
    pinMode(SCL, OUTPUT);
    pinMode(SDA, INPUT);

    if (digitalRead(SDA) == LOW)
    {
      pinMode(SCL, OUTPUT);
      pinMode(SDA, INPUT_PULLUP);  // reset below 
    }

    // If SDA is LOW, clock it free
    for (int i = 0; i < 9; i++) {
        digitalWrite(SCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(SCL, LOW);
        delayMicroseconds(5);
    }

    // Generate STOP
    pinMode(SDA, OUTPUT);
    digitalWrite(SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(SDA, HIGH);

    pinMode(SDA, INPUT);        // reset
    pinMode(SCL, INPUT);

    delay(1000); // long delay to bleed CAPS

    return true;
}

//  I2C bus responding 
// Check if I2C device at addr responds, count errors, and report (DEBUG only)
bool i2cDevicePresent(uint8_t addr)
{
    static bool last_ok = true;  // track previous state to count transitions
    bool ok = true;

    // Step A: handle timeout first
    if (URTCLIB_WIRE.getTimeoutFlag()) 
    {
        i2c_timeout_count++;
        URTCLIB_WIRE.clearTimeoutFlag();
        ok = false; // bus is blocked
    }

    // Step B: probe the device if bus not blocked
    if (ok) 
    {
        URTCLIB_WIRE.beginTransmission(addr);
        uint8_t err = URTCLIB_WIRE.endTransmission();
        ok = (err == 0);

        // Count only new failures
        if (!ok && last_ok) 
        {
            i2c_error_count++;
        }

#ifdef DEBUG
        char buffer[80];
        snprintf(buffer, sizeof(buffer), "I2C addr 0x%02X endTransmission: %d\n", addr, err);
        DBG(buffer);

        if (!ok) 
        {
            char errorMSG[80];
            switch(err)
            {
                case 1: snprintf(errorMSG,sizeof(errorMSG),"Data too long\n"); break;
                case 2: snprintf(errorMSG,sizeof(errorMSG),"NACK on address\n"); break;
                case 3: snprintf(errorMSG,sizeof(errorMSG),"NACK on data\n"); break;
                case 4: snprintf(errorMSG,sizeof(errorMSG),"Other bus error\n"); break;
                default: snprintf(errorMSG,sizeof(errorMSG),"Unknown error\n"); break;
            }
            DBG(errorMSG);
        }
#endif
    }

    last_ok = ok;
    return ok;
}


// use to read the DS3231 in place of rtc.refresh()  at appropriate places  has recovery built in
bool rtcRefresh() {
  static uint8_t rtcFailCount = 0;
  bool rtcRecoveryFlag = true;
  char status[10] = "";
  char buffer[200] = "";

  if (rtc.refresh()) 
  {
    //  stop call the module and use cache causes less problems
    // use cache variable not rtc variables
    rtc_cache.y  = rtc.year();
    rtc_cache.mo = rtc.month();
    rtc_cache.d  = rtc.day();
    rtc_cache.dow = rtc.dayOfWeek();
    rtc_cache.h  = rtc.hour();
    rtc_cache.m  = rtc.minute();
    rtc_cache.s  = rtc.second();
    rtc_cache.t  = rtc.temp();
    rtc_ok = true;
 
    // check the rtc and it lostPower status
    bool rtcok = rtcOk();
    bool rtclostPower = rtc.lostPower();

    DBGSNPF(buffer, sizeof(buffer),
    "rtcOK:%d rtclostPower:%d \n",rtcok,rtclostPower);
 
    if (!rtcok)
    {  
      bool PicoOK = rtcPICOOK();
      DBGPF("DS rtc failed RTC sanity check:%d\n",rtcok);
      if (rtclostPower) {DBGPF("RTC lost power, time is invalid!:%d\n",rtclostPower);}

      if (PicoOK)
      {
        DBGPF("DS need re-sync-ed with PICO RTC -- PicoOK return:%d\n",PicoOK);
        syncPicoToDS3231();  // sync PICO to DS3231
        rtcFailCount = 0;
        // return rtc.refresh();
        return true;
      }
      //   No valid time fail
      rtcFailCount++;
    }
    // rtcFailCount = 0; NOT needed
    return true;
  }

  rtcFailCount++;
  DBG("RTC fail count: ");
  DBGLN(rtcFailCount);
  if (rtcFailCount < 3) 
  {
    return false;
    // tolerate transient failures
  }

  // problems recycling DS3231 power and the I2C bus
  DBGLN("Power-cycling RTC...");
  // Disable I2C pins to avoid phantom power
  pinMode(SDA, INPUT); // NEEDed here
  pinMode(SCL, INPUT); // NEEDed here

  rtcRecoveryFlag = rtcSafeRefresh(); // powercycle, i2cRecovery, i2cInit
    // Build a display for number of cycles remaining 
 
  // snprintf(status, sizeof(status), "%s", rtcRecoveryFlag ? "worked" : "failed");
  
  snprintf(buffer, sizeof(buffer),"DS3231 recovery attempt %s\n 12c Bus ERROR-Check for MSG above or power status",rtcRecoveryFlag ? "worked" : "failed");
  DBG(buffer);

  rtcFailCount = 0;
  return rtc.refresh();
}

// the actual POWER CYCLE
bool rtcSafeRefresh() {
    if (i2cDevicePresent(0x68) == 0) {
        rtc.refresh();
        flLED(rtc_LED);
        return true;
    }

    rtcPowerCycle();
    i2cRecover();
    i2cInit();

    if (i2cDevicePresent(0x68) == 0) {
        rtc.refresh();
        flLED(rtc_LED);
        return true;
    }

    return false;
}

// ----------------------------------------------the actual DS3231 read for recording on the SD
bool RTC_Read() 
{

  // RTC Refresh
  // RTC stores the time in integers
  // RTC stores temperature in float

  // temp variables to store date for comparison, snprintf, etc
  char buffer[100];
  // char errorBuffer[100] = "";
  char rtcbuffer[100] = "";
  char s[15];  // 14 chars and NULL
  char t[10];   // 8 chars and NULL
  int j;       // length of the sprintf

  ASSERT(sizeof(buffer) >= 100);
  ASSERT(sizeof(rtcbuffer) >= 100);
  ASSERT(sizeof(s) >= 15);
  ASSERT(sizeof(t) >= 10);
  // temp work variable
  float temp;

  // get updated reading from the RTC
  // if (!rtcSafeRefresh()) {
  //     Serial.println("RTC unavailable this cycle. Skipping...");
  //     // Optional: increment failed RTC counter
  //     return;
  // }
  // rtc.refresh();
  if (!rtcRefresh()) 
  {
    DBGLN("RTC refresh FAILED");
    rtcOK = false;
    // needs hardening here ----------------- working on it.--------------------------------------------------------------
    return false;
  }
  else rtcOK = true;  // need to be looked at
  // flLED(rtc_LED);
  // give the module time to complete
  // delay(50);

  // check if new month if so list files on SD
  // if (currentMth != rtc_cache.mo) 
  // {
  //   // list files on SD's
  //   listFiles(*sdA.sd, "SD Card 1 (SPI0)");
  //   listFiles(*sdB.sd, "SD Card 2 (SPI1)");
  // }

    // save current Hour, Month and Year for comparisons to work with
  // currentHour = rtc_cache.h;  // for the file flush
  // currentMth  = rtc_cache.mo; // for monthly files
  // currentYear = rtc_cache.y;  // for monthyear file

  // store rtc temp in a local variable
  temp = rtc_cache.t;
  ASSERT(!isnan(temp));

  // convert the float temp to string
  // dtostrf(float, min_width, num_of_digits after decimal, string where to store the converted string)
  dtostrf( rtc_cache.t / 100, 8, 2, t);

  ASSERT(sizeof(rtcTemp) >= sizeof(t));
  ASSERT(strlen(t) < sizeof(rtcTemp));
  strncpy(rtcTemp,t,sizeof(rtcTemp));

  // build a print line for monitor with snprintf rather than many serial.print lines
  j = DBGSNPF(rtcbuffer,sizeof(rtcbuffer),"RTC DT:%02d/%02d/%02d  %02d:%02d:%02d %s%sC\t",
  rtc_cache.y,rtc_cache.mo,rtc_cache.d,rtc_cache.h,rtc_cache.m,rtc_cache.s,t,"\xC2\xB0");  // use cache variables not module calls
  // if (SPrint) {Serial.print(rtcbuffer);} // only print when debugging
  SP(rtcbuffer)
  return true;
}

// ----------------------------- eePROM routines ----------------------------------------------
void FirsteePROM() 
{
  bool statusEEPROM = false;
  // flash(LED_PIN1,2,200);
  eTemp = 129.9;
  // set the temp to a high to force an lower temperture update as coldestTemp will not be set
  coldestTemp = eTemp; // force a update
  // statusEEPROM = eepromWriteWithAck(ETEMP, eTemp);
  statusEEPROM = eeprom.eeprom_write(ETEMP, eTemp);
  // delay(90);
  // statusEEPROM = eepromWriteByte(ETEMP, eTemp); // current going to old

  if (!statusEEPROM) DBGLN(F("I2C Bus Error - check wiring"));
 
  // now test the write by re-reading it in
  flLED(eP_LED);
  eeprom.eeprom_read(ETEMP, &eTemp);
  delay(50);
  if (isnan(eTemp))  // invalid float re initialize
  {
    DBG("eTemp is NaN -- problem with the eeprom read of a float variable  SOLUTION NEEDED\n");
  }
  else
  {
    DBG("eTemp read good \t");
    // print to check data
    DBG("Read eEPROM Temp=");
    DBGLN((float)eTemp);
    // eeprom.eeprom_read(4, &eTemp);
    // delay(10); 
    // Serial.print(F("\t eTemp re-read :"));
    // Serial.println((float)eTemp);
  }
}

// ----------------------------------------------------------------------------------------------
void RecoveryeePROM() 
{
  bool statusEEPROM = false;

  // float eTemp;  // do not initialize
  char e_string[100] = "";
  // bool  eReadOK = false;  // = 0
  bool  nan = false; // = 0

  char eTempbuf[10] = "";
  ASSERT(sizeof(eTempbuf) >= 10);
  // update variable to store in EEPROM

  DBG(F("Recovery because of restart\t"));
  // power recovery

  // ********** we are here because of a power outage so read the eePROM for coldestTemp and coldestData *** NOT FIRST TIME ***
  // Serial.print(F(" Read eeprom for float\t"));
  // flash(LED_PIN1,2,200);
  flLED(eP_LED);
  eeprom.eeprom_read(ETEMP, &eTemp);
  // eeprom.eeprom_read(4, &eTemp);
  delay(90);
  // print to check data
  DBG("Read eEPROM Temp=");
  DBGLN((float)eTemp);
  ASSERT(!isnan(eTemp));
  if (nan = (isnan(eTemp))) // > 0  // isnan return true > 0 if not valid float
  {
    DBGPF("eTemp is NaN  -- so re initializing -- nan:%d \n",nan);
    FirsteePROM();  // data is bad so reinitialize // = 0
    // eReadOK = false;  // invalid float re initialize // = 0
  } else
  {
    // eReadOK = true; // > 0
    dtostrf(eTemp, 8, 1, eTempbuf); 
    DBGPF("Recovered eTemp from ePROM :%s\n",eTempbuf);
    DBG("eTemp:");
    DBGLN((float) eTemp);
    // store in global
    // if (eTemp == 0.0) eTemp = 99.1;  // seem wrong force an coldest update //found bug not needed
    coldestTemp = eTemp;  // copy coldest temp to coldestTemp to reset because of recovery
    dtostrf(coldestTemp, 8, 2, coldestCTemp); // Also update the char float variable
  }

  // now the string data
  // Serial.print(F(" Read eeprom for string data:"));
  // flash(LED_PIN1,2,200);
  // safeEEPROMRead(33, e_string, string_length);
  // Serial.print("EEProm String:");
  // Serial.println(e_string);
  // statusEEPROM = eeprom.eeprom_read(33, (byte *)e_string, string_length);
  // delay(800);
  // if (!statusEEPROM) 
  // { 
  //   Serial.println(F("I2C Bus Error - check wiring"));
  // }
  // else 
  // {
  //   Serial.println(e_string);
  //   strcpy(coldestData, e_string);      // copy the estring to coldestData string;
  //   strcpy(sdData, e_string);  // copy the estring to sdData string;
  // }
}

// wait for eeprom write to finish
// bool eepromWaitReady(uint8_t addr)
// {
//     for (int i = 0; i < 50; i++) {       // ~100 ms max
//         URTCLIB_WIRE.beginTransmission(addr);
//         if (URTCLIB_WIRE.endTransmission() == 0) {
//             return true;
//         }
//         delay(2);  // short poll interval
//     }
//     return false;
// }

// bool eepromWriteWithAck(uint16_t addr, uint8_t data)
// {
//     if (!eeprom.eeprom_write(addr, data))
//         return false;

//     // Wait for EEPROM internal write cycle to finish
//     return eepromWaitReady(EEPROM_ADDR);
// }

// ACK poll helper
bool eepromWaitReady(uint8_t addr)
{
    for (int i = 0; i < 50; i++) {       // max ~100ms
        URTCLIB_WIRE.beginTransmission(addr);
        if (URTCLIB_WIRE.endTransmission() == 0) {
            return true; // EEPROM ready
        }
        delay(2); // short poll
    }
    return false; // timed out
}

// Full write routine
bool eepromWriteByte(uint16_t memAddr, uint8_t data)
{
    // 1️⃣ Write the byte to EEPROM
    URTCLIB_WIRE.beginTransmission(EEPROM_ADDR);
    URTCLIB_WIRE.write((uint8_t)(memAddr >> 8));   // high byte address
    URTCLIB_WIRE.write((uint8_t)(memAddr & 0xFF)); // low byte address
    URTCLIB_WIRE.write(data);
    if (URTCLIB_WIRE.endTransmission() != 0) {
        return false; // transmission failed
    }

    // 2️⃣ Wait for internal write cycle to finish
    return eepromWaitReady(EEPROM_ADDR);
}




void WriteeePROM() 
{
  // digitalWrite(LED_PIN2, HIGH);
  // digitalWrite(LED_PIN, HIGH);
  bool statusEEPROM = false;
  char e_string[100] = "";
  char eTempbuf[10] = "";
  ASSERT(sizeof(e_string) >= 100);
  ASSERT(sizeof(eTempbuf) >= 10);
  // update variable to store in EEPROM
  eTemp = coldestTemp;
  ASSERT(!isnan(eTemp));
  dtostrf(eTemp, 8, 1, eTempbuf); 
  DBGPF("eProm-Data to be written eTemp(coldestTerm):%s\n",eTempbuf);
  // strcpy(eCTemp,coldestCTemp);  // also update the string copy
  // write eeprom float --- syntax -- write() prom location, variable (the data))
  // Serial.println(F("Writing into eeprom memory..."));
  // Serial.println(F("Now writing eTemp to eeprom"));
  // flash(LED_PIN1,2,200);
  flLED(eP_LED);

  statusEEPROM = eeprom.eeprom_write(ETEMP, eTemp);  //<------------------
  // statusEEPROM = eepromWriteWithAck(ETEMP, eTemp);


  // statusEEPROM = eepromWriteByte(ETEMP, eTemp);  // current going to old

  if (!statusEEPROM) 
  {  // write a float value to the eePROM  (4 chars)
    DBG(F("I2C Bus Error - check wiring -- eTemp: "));
    DBGLN((float)eTemp);
  } else 
  {
    //waitReady(); // wait for the transfer success
    DBG(F("eTemp stored correctly :"));
    DBG((float)eTemp);
    // check read it back
    flLED(eP_LED);
    eeprom.eeprom_read(ETEMP, &eTemp);
    delay(70);
    DBG(F("\t eTemp re-read :"));
    DBGLN((float)eTemp);
  }

  // write sdData to eeprom
  // string_length = strlen(coldestData);
  // strcpy(e_string,coldestData);
  // // Serial.println(F("Now writing e_string to eeprom"));
  // flash(LED_PIN1,2,200);

  // // use safeEEPROMWrite to avoid 32-byte page boundaries
  // // safeEEPROMWrite(33, e_string);

  // // startEEPROM = millis();
  // statusEEPROM = eeprom.eeprom_write(33, (byte *)e_string, strlen(e_string));
  // delay(500); // while (millis() - startEEPROM < 500) { delay(10); }
  // if (!statusEEPROM) 
  // {
  //   Serial.println(F("I2C Bus Error - check wiring"));
  // } else 
  // {
  //   // wait for transfer success
  //   // waitReady();
  //   Serial.println(F("eString was stored correctly."));
  // }
  // digitalWrite(LED_PIN2, LOW);
  // digitalWrite(LED_PIN, LOW);
}

void promUpdate(float currentTemp, char *currentData) 
{
  char buffer[120] = "";
  char currentCTemp[10] = "";
  ASSERT(sizeof(buffer) >= 120);
  ASSERT(sizeof(currentCTemp) >= 10);
  dtostrf(currentTemp, 8, 2, currentCTemp);
  DBGPF("currentCTemp:%s coldestCTemp:%s",currentCTemp,coldestCTemp);
  dtostrf(coldestTemp, 8, 2, coldestCTemp);
  // Serial.print("Coldest Temp check called  --promUpdate\n");
  // check if it is colder than previous coldest temperature  -- keep the coldest also in the EEPROM
  if (currentTemp < coldestTemp) {  // found a colder temperature -- up date the coldest record -- update the eeprom

    DBGSNPF(buffer,sizeof(buffer),"Colder -- %s Temp than %s  eePROM updated\n",bmCTemp,coldestCTemp);
    DBG(buffer);

    // update the coldest global variable coldestTemp with new coldest --- and the char float variable too
    coldestTemp = currentTemp;
    dtostrf(coldestTemp, 8, 2, coldestCTemp); // update the char float variable //5 is mininum width, 1 is precision; float value is copied onto buff
    DBGSNPF(buffer,sizeof(buffer),"2nd time Colder -- %s Temp than %s  eePROM updated\n",bmCTemp,coldestCTemp);
    DBG(buffer);


    strcpy(coldestData, currentData);
    dtostrf(currentTemp, 8, 2, currentCTemp);
    DBGSNPF(buffer,sizeof(buffer),"Checking for coldest currentCTemp: %s coldestCTemp %s\n",currentCTemp,coldestCTemp);
    DBG(buffer);
    // write to the PROM
    WriteeePROM();
  }
}

// Read the BME or BMP
// -------------------------------------------- BMx routines -----------------------------------

#define BME_ADDR 0x76

void bmeSoftReset(void)
{
    URTCLIB_WIRE.beginTransmission(BME_ADDR);
    URTCLIB_WIRE.write(0xE0);   // reset register
    URTCLIB_WIRE.write(0xB6);   // soft reset
    URTCLIB_WIRE.endTransmission();
    delay(10);                 // datasheet >2ms
}


uint8_t bmxReadChipID(uint8_t addr)
{
    URTCLIB_WIRE.beginTransmission(addr);
    URTCLIB_WIRE.write(0xD0);           // Chip ID register
    URTCLIB_WIRE.endTransmission(false);
    URTCLIB_WIRE.requestFrom(addr, (uint8_t)1);

    if (URTCLIB_WIRE.available()) {
        return URTCLIB_WIRE.read();
    }
    return 0xFF;  // indicates failure
}


// uint8_t bmxReadChipID(uint8_t addr)
// {
//     URTCLIB_WIRE.beginTransmission(addr);
//     URTCLIB_WIRE.write(0xD0);   // chip ID register
//     URTCLIB_WIRE.endTransmission(false);
//     URTCLIB_WIRE.requestFrom(addr, (uint8_t)1);
//     return URTCLIB_WIRE.read();
// }

#define BME_ADDR 0x76

void bmInit()
{
    if (!i2cDevicePresent(BME_ADDR)) {
        DBG("BME not responding on I2C\n");
        bmAvailable = false;
        return;
    }

    bmeSoftReset();
    delay(50);

    uint8_t chipID = bmxReadChipID(BME_ADDR);

#ifdef DEBUG_ENABLE
    char buf[40];
    snprintf(buf, sizeof(buf), "BME chip ID: 0x%02X\n", chipID);
    DBG(buf);
#endif

    if (chipID != 0x60) {
        DBG("Unexpected BME chip ID\n");
        bmAvailable = false;
        return;
    }

    // IMPORTANT: force same Wire instance
    if (!bme.begin(BME_ADDR, &URTCLIB_WIRE)) {
        DBG("BME begin() failed\n");
        bmAvailable = false;
        return;
    }

    bmInitialized = true;
    bmAvailable   = true;
    DBG("BME initialized OK\n");
}




// void bmInit()
// {
//     // Sensor present on bus?
//     if (!i2cDevicePresent(BME_ADDR)) 
//     {
//         DBG("BMx not responding on I2C\n");
//         bmAvailable = false;
//         return;
//     }


// #ifndef BMP280
//     // Force sensor into known state after Pico reset
//     bmeSoftReset();
//     delay(50);
// #endif

// #ifdef BMP280
//     if (!bmp.begin()) 
// #else


//     if (!bme.begin(0x76, &URTCLIB_WIRE))
// #endif
//     {
//         DBG("BMx begin() failed — will retry later\n");
//         bmAvailable = false;
//         return;
//     }

//     bmInitialized = true;
//     bmAvailable   = true;

//     DBG("BMx initialized OK\n");
// }

bool bmTask()
{
    // Retry init if Pico was reset but sensor wasn't
    if (!bmInitialized || !bmAvailable) {
        static uint32_t last_try = 0;
        if (millis() - last_try > 5000) {   // retry every 5 seconds
            last_try = millis();
            bmInit();
        }
        return true;
    }

    BM_Read();
    return true;
}



// void bmInit()
// {
//     // Step 1: check I2C bus and device
//     if (!i2cDevicePresent(0x76)) {
//         DBG("BMx sensor not found on I2C bus!\n");
//         // Optionally: set a flag to skip reads
//         bmAvailable = false;
//         return;
//     }
//     bmAvailable = true;

// #ifdef BMP280
//     if (!bmp.begin()) {
//         DBG("Failed to initialize BMP280\n");
//         bmAvailable = false;
//         return;
//     }
// #else
//     if (!bme.begin()) {
//         DBG("Failed to initialize BME280\n");
//         bmAvailable = false;
//         return;
//     }
// #endif

//     // Step 2: Configure sensor if needed (oversampling, filter, etc.)
//     // Example:
//     // bme.setSampling(...);

//     DBG("BMx sensor initialized successfully\n");
// }

// BM Read Function

bool BM_Read()
{
  // if (!bmAvailable) return false; Not needed as tested in bmTask()

  // declare the symbol for degree
  char degree[] = "\u00b0";

  char t_buffer[10] = "";
  char p_buffer[10] = "";
  char a_buffer[10] = "";
  char h_buffer[10] = "";
  char f_buffer[10] = "";
  char bm_buffer[100] = "";

  float temperature, pressure, altimeter, humidity = 0;

  flLED(pi_LED);
  #ifdef BMP280
    pressure = bmp.readPressure() / 100.0;   // hPa
    temperature = bmp.readTemperature();
    altimeter = bmp.readAltitude(CURRENT_HPA);
  #else
    pressure = bme.readPressure() / 100.0;
    temperature = bme.readTemperature();
    altimeter = bme.readAltitude(CURRENT_HPA);
    humidity = bme.readHumidity();
    ASSERT(sizeof(h_buffer) >= 10);
    dtostrf(humidity, 8, 2, h_buffer);
  #endif



  

    // Convert to strings
    ASSERT(sizeof(p_buffer) >= 10);
    dtostrf(pressure, 8, 2, p_buffer);
    ASSERT(sizeof(t_buffer) >= 10);
    dtostrf(temperature, 8, 2, t_buffer);

    // Copy for display
    ASSERT(sizeof(bmCTemp) >= sizeof(t_buffer));
    strcpy(bmCTemp, t_buffer);

    ASSERT(sizeof(a_buffer) >= 10);
    dtostrf(altimeter, 8, 1, a_buffer);

    float fahrenheit = 1.8 * temperature + 32;
    ASSERT(sizeof(f_buffer) >= 10);
    dtostrf(fahrenheit, 8, 2, f_buffer);

    // copy t_buffer into a global variable bmCTemp -- for OLED display
    // store in global variable and convert to a string for printing
    bmTemp = temperature;
    ASSERT(sizeof(bmCTemp) >= sizeof(t_buffer));
    ASSERT(sizeof(bmCTemp) >= sizeof(t_buffer));
    strcpy(bmCTemp,t_buffer);

    // Build BM string for SD / display
#ifdef BMP280
    snprintf(bm_buffer, sizeof(bm_buffer), "%s,%s,%s", t_buffer, p_buffer, a_buffer);
#else
    snprintf(bm_buffer, sizeof(bm_buffer), "%s,%s,%s,%s", t_buffer, p_buffer, a_buffer, h_buffer);
#endif

    ASSERT(sizeof(bmData) >= sizeof(bm_buffer));
    strcpy(bmData, bm_buffer);

#ifdef TURNONPRT
    if (SPrint) {
#ifdef BMP280
        char bm_pbuffer[100];
        snprintf(bm_pbuffer, sizeof(bm_pbuffer), "BM P: %s Pa T: %s°C %s°F A: %s m\n",
                 p_buffer, t_buffer, f_buffer, a_buffer);
        Serial.print(bm_pbuffer);
#else
        char bm_pbuffer[120];
        snprintf(bm_pbuffer, sizeof(bm_pbuffer), "BM P: %s Pa T: %s°C %s°F A: %s m H: %s%%\n",
                 p_buffer, t_buffer, f_buffer, a_buffer, h_buffer);
        Serial.print(bm_pbuffer);
#endif
    }
#endif
return true;
}


// void BM_Read() {
//   // declare the symbol for degree
//   //char degree[] = "\u00b0";

//   // string buffers for dtostrf function
//   char bm_buffer[70] = "";
//   // char errorBuffer[100] = "";
//   char t_buffer[10] = "";
//   char p_buffer[10] = "";
//   char a_buffer[10] = "";
//   char h_buffer[10] = "";
//   char f_buffer[10] = "";

//   ASSERT(sizeof(t_buffer) >= 10);
//   ASSERT(sizeof(p_buffer) >= 10);
//   ASSERT(sizeof(a_buffer) >= 10);
//   ASSERT(sizeof(a_buffer) >= 10);
//   ASSERT(sizeof(f_buffer) >= 10);


//   char bm_pbuffer[100] = "";
//   int j, ej = 0;
  
//   // string buffer 
//   // char tempStr[10] = "";
 
//   // temp variables for bme data
//   // float pressure, temperature, altimeter, humidity, fahrenheit = 0;

//   //Read values from the sensor: Convert the float values to char string for printing
//   flLED(pi_LED);
//   #ifdef BMP280
//     // pressure
//     float pressure = bmp.readPressure() / 100;  // put into hPa units from Pa
//     // temperature
//     float temperature = bmp.readTemperature();
//     // altimeter
//     float altimeter = bmp.readAltitude(CURRENT_HPA);
//     ASSERT(!isnan(pressure));
//     ASSERT(!isnan(temperature));
//     ASSERT(!isnan(altimeter));  

//   #else
//     // pressure
//     float pressure = bme.readPressure() / 100;  // put into hPa units from Pa
//     // temperature
//     float temperature = bme.readTemperature();
//     // altimeter
//     float altimeter = bme.readAltitude(CURRENT_HPA);
//     // humidity
//     float humidity = bme.readHumidity(); 
   
//     dtostrf(humidity, 8, 2, h_buffer); 

//     ASSERT(!isnan(pressure));
//     ASSERT(!isnan(temperature));
//     ASSERT(!isnan(altimeter)); 
//     ASSERT(!isnan(humidity));  
//   #endif

//   // convert to a string for printing
//   dtostrf(pressure, 8, 2, p_buffer);
//   dtostrf(temperature, 8, 2, t_buffer); 
//   dtostrf(altimeter, 8, 1, a_buffer); 

//   // store in global variable and convert to a string for printing
//   bmTemp = temperature;

//   // copy t_buffer into a global variable bmCTemp -- for OLED display
//   ASSERT(sizeof(bmCTemp) >= sizeof(t_buffer));
//   ASSERT(sizeof(bmCTemp) >= sizeof(t_buffer));
//   strcpy(bmCTemp,t_buffer);

//   //Change the "1050.35" to your city current barrometric pressure (weather off iPhone app source The Weather Channel)
 
//   // temperature in fahrenheit -- not recorded as it can be calculated
//   float fahrenheit = 1.8 * temperature + 32;
//   dtostrf(fahrenheit, 8, 2, f_buffer); 

//   //  Adjust the data length for the various sensors BMP BME on Board ones, etc
// 	//Print values to serial monitor:  build a print line with sprinf rather than many serial.print lines
//   #ifdef BMP280
//     snprintf(bm_pbuffer,sizeof(bm_pbuffer),"BM P: %s Pa T: %s%sC %s%sF A: %s m\n",p_buffer,t_buffer,
//     "\xC2\xB0",f_buffer,"\xC2\xB0",a_buffer);

//     // build a string to store on the SD Card from BMP data with delimiter for a CSV file
//     j = snprintf(bm_buffer,sizeof(bm_buffer),"%s,%s,%s",t_buffer,p_buffer,a_buffer);
//     ASSERT(j > 0);
//     ASSERT(j < sizeof(bm_buffer));

//     ej = 25;  // must be >= to ej or missing data
//   #else //BME280
//     DBGSNPF(bm_pbuffer,sizeof(bm_pbuffer),"BM P: %s Pa T: %s%sC %s%sF A: %s m H: %s%c \n",p_buffer,t_buffer,
//     "\xC2\xB0",f_buffer,"\xC2\xB0",a_buffer,h_buffer,'%');
    
//     // build a string to store on the SD Card from BME data with delimiter for a CSV file
//     j = snprintf(bm_buffer,sizeof(bm_buffer),"%s,%s,%s,%s",t_buffer,p_buffer,a_buffer,h_buffer);
//     ASSERT(j > 0);
//     ASSERT(j < sizeof(bm_buffer));
//     ej  = 35;  // must be >= to ej or missing data 
//   #endif

//   // ----- stop the printing turn on for debugging
//   #ifdef TURNONPRT 
//     if (SPrint) {Serial.print(bm_pbuffer);} // only print if debugging
//   #endif 
//   // DispData(bm_pbuffer,NULL);  // display on OLED screen

//   // store in global variable bmData
//   ASSERT(sizeof(bmData) >= sizeof(bm_buffer));
//   ASSERT(strlen(bm_buffer) < sizeof(bmData));
//   strcpy(bmData, bm_buffer);

//   // Ensure we have the correct amount of sensor data - should be 22 + NULL - so 23 chars
//   ASSERT(j >= ej);
//   // if (j < ej) {
//   //   char errorBuffer[100] = "";
//   //   DBGSNPF(errorBuffer,sizeof(errorBuffer),"*-*-* Error BM data is too small %d! Data is:%s:  Length of rtcData is %d \n",j,bmData,strlen(bmData));
//   //   Serial.print(errorBuffer);
//   //   // flash RED for error
//   //   // flash(LRED, 12,100);
//   // }
// }

// --------------------------------------------------- SD Routines ------------------------------------------------------

// Hi-Z the selected bus only
void sdSpiHiZ(const sd_bus_t &bus) {
    pinMode(bus.pinMOSI, INPUT);
    pinMode(bus.pinSCK,  INPUT);
    pinMode(bus.pinSS,   INPUT);
    delay(10);
}

// Restore SPI pins
void sdSpiRestore(const sd_bus_t &bus) {
    pinMode(bus.pinSS, OUTPUT);
    digitalWrite(bus.pinSS, HIGH);   // deselect
}

// map SPI pins
void mapSpiPins(const sd_bus_t &bus)
{
    // RP2040: pin mux is GLOBAL
    SPI.setTX(bus.pinMOSI);
    SPI.setRX(bus.pinMISO);
    SPI.setSCK(bus.pinSCK);
}

// SD LED controls
void sdOn(int8_t led) { digitalWrite(led,HIGH);}
void sdOff(int8_t led) { delay(15); digitalWrite(led,LOW);}

// void sdPowerOFF(const sd_bus_t &bus) {
//     digitalWrite(bus.pinPWR, LOW);
// }

// void sdPowerON(const sd_bus_t &bus) {
//     digitalWrite(bus.pinPWR, HIGH);
// }
void spiPinMapping(sd_bus_t &busA,sd_bus_t &busB)
{
  SD1_CS  = sdA.pinSS;
  SD1_PWR = sdA.pinPWR;

  SD2_CS  = sdB.pinSS;
  SD2_PWR = sdB.pinPWR;
}

bool sdHardRecover(sd_bus_t &bus) {
    DBGLN("SD: HARD RECOVER");

    // 1. Stop SPI driving for THIS bus only
    sdSpiHiZ(bus);

    // 2. Power off SD
    sdPowerOff(bus);
    delay(250);

    // 3. Reset SPI peripheral
    bus.spi->end();
    delay(50);

    // 4. Power on SD
    sdPowerOn(bus);
    delay(300);

    // 5.1 Map SPI pins
    mapSpiPins(bus);

    // 5.2 Restore SPI pins
    sdSpiRestore(bus);

    // 6. Restart SPI
    bus.spi->begin();
    delay(100);

    // 7.  Init SD using SdFat (CORRECT)
    if (!bus.sd->begin(
            SdSpiConfig(bus.pinSS, DEDICATED_SPI, SD_SCK_MHZ(12), bus.spi)
        )) {
        Serial.println("SD init FAILED");
        fs_fail_count++;
        return false;
    }

    DBGLN("SD init OK");
    return true;
}


// sd Power routines 
bool sdPowerOn(const sd_bus_t &bus) {
  bool pin_state = LOW;
  char buffer[80] = "";
  pinMode(bus.pinPWR, OUTPUT);
  digitalWrite(bus.pinPWR, HIGH);
  delay(200);
  pin_state = gpio_get(bus.pinPWR);
  // Test the state using an if statement
  if (pin_state) {
      DBGSNPF(buffer,sizeof(buffer),"Power %d is HIGH (3.3V)\n", bus.pinPWR);
      DBG(buffer);
      return true;
  } else {
      DBGSNPF(buffer,sizeof(buffer),"Power %d is LOW (0V)\n",  bus.pinPWR);
      DBG(buffer);
      return false;
  }
}

bool sdPowerOff(const sd_bus_t &bus) 
{
  bool pin_state = HIGH;
  char buffer[80] = "";
  pinMode(bus.pinPWR, OUTPUT);
  digitalWrite(bus.pinPWR, LOW);
  delay(300);
  pin_state = gpio_get(bus.pinPWR);
  // Test the state using an if statement
  if (pin_state) {
      DBGSNPF(buffer,sizeof(buffer),"Power %d is HIGH (3.3V)\n", bus.pinPWR);
      DBG(buffer);
      return true;
  } else {
      DBGSNPF(buffer,sizeof(buffer),"Power %d is LOW (0V)\n",  bus.pinPWR);
      DBG(buffer);
      return false;
  }
}


// flashs an LED  -- not currently used
void flash(int ledPin, int numFlashes, int d) // Pin, No of flashes, delay between flashes
{
  for (int j = 0; j < numFlashes; j++) 
  {
    digitalWrite(ledPin, HIGH);
    delay(d);
    digitalWrite(ledPin, LOW);
    delay(d);
  }
}


// Routine to write sdData to two SD Cards
bool writeSDFile(char *sdData)
{
  char buffer[299] = "";
  char fileName1[15] = "";
  char fileName2[15] = "";
  ASSERT(sizeof(fileName1) >= 15);
  ASSERT(sizeof(fileName2) >= 15);

  int written1, written2 = 0;
  int sdDataLen = 0; // excluding \0 character
  bool cardOpenError1, cardOpenError2 = false; 
  bool cardWriteError1, cardWriteError2 = false;
  bool cardSyncError1, cardSyncError2 = false;
  bool sdARecovError, sdBRecovError = false;

  // Logging routine for SD 1
  // SD1 or sdA
  snprintf(fileName1,sizeof(fileName1),"csv1%02d%02d.txt",currentMth,currentYear); 
  DBGSNPF(buffer,sizeof(buffer),"New file name is %s\n ",fileName1);
  // Serial.print(buffer);

  // --- WRITE TO CARD 1 ---
  if (!sdARecovError)  // not in a recovery state or just coming out of one  = 0
  {
    sdA.sd->chvol(); // FORCE focus to SD1
    if (csvFile1.open(sdA.sd, fileName1, O_RDWR | O_CREAT | O_AT_END)) 
    {   // open good write data
        sdDataLen = strlen(sdData); // not including \0
        written1 = (csvFile1.println(sdData) - 2);  //including CR and NL 
        if((written1 + 2) > 0)
        { // write good now sync
          // DBGPF("SD1 sdDataLen:%d  written:%d\n",sdDataLen,written1);  // debugging
          // delay(10);
          if (csvFile1.sync())
          { // sync good now close the file clean up SfFat
            csvFile1.close();
            // Serial.println("Wrote to csvFile1 on SD 1");
            // flash(LED_PIN,2,60);
            sdA_Ready = true;
          }
          else // Sync failure
          {
            cardSyncError1 = true; // > 0
            sd1Healthy = false;    // = 0
            DBGSNPF(buffer,sizeof(buffer),"Card:%d sync failed - card possible removal mid-write\n",1);
            DBG(buffer);
          } //sync failure else close
        } else // write failure 
        { 
          sdA_Ready = false; 
          cardWriteError1 = true;
          sd1Healthy = false;
          DBGSNPF(buffer,sizeof(buffer),"Error writing %s\n ",fileName1);
          DBG(buffer);
        } // write failure else close
    } else // Open failure
    {
      sdA_Ready = false; // Mark error
      cardOpenError1 = true;
      sd1Healthy = false;
      DBGSNPF(buffer,sizeof(buffer),"Error opening %s\n ",fileName1);
      DBG(buffer);
    }
  } // end of !sdARecovError otherwise end of we are good

  // Open, Write, or Sync error occurred need to recover
  if (!sd1Healthy) //  not true = 0 = false
  {
    // csvfile1.close();          // safe even if already bad
    // sdFatA.end();  
    if (!sdHardRecover(sdA))  // = 0
    {
      DBGLN("SD1 offline — using SD2 only");
      sdA_Ready = false; // = 0
      sdARecovError = true; // > 0
    } else // HardRecover good  now clear up SfFat by closing open files
    {
      if (cardSyncError1){csvFile1.close(); delay(10);}
      if (cardWriteError1){csvFile1.close(); delay(10);}
      DBGLN("SD1 Recovered!");
      sdA_Ready = true; // > 0
      sdARecovError = false; // > 0
      sd1Healthy = true;
    }

  } 

  // SD2 or sdB
  snprintf(fileName2,sizeof(fileName2),"csv2%02d%02d.txt",currentMth,currentYear); 
  DBGSNPF(buffer,sizeof(buffer),"New file name is %s\n ",fileName2);
  // Serial.print(buffer);

// SD2 New Start
  // --- WRITE TO CARD 1 ---
  if (!sdBRecovError)  // not in a recovery state or just coming out of one  = 0
  {
    sdB.sd->chvol(); // FORCE focus to SD2
    if (csvFile2.open(sdB.sd, fileName2, O_RDWR | O_CREAT | O_AT_END)) 
    {   // open good write data
        sdDataLen = strlen(sdData); // not including \0
        written2 = (csvFile2.println(sdData) - 2);  //including CR and NL 
        if((written2 + 2) > 0)
        { // write good now sync
          // DBGPF("SD2 sdDataLen:%d  written:%d\n",sdDataLen,written2);  // debugging
          delay(10);
          if (csvFile2.sync())
          { // sync good now close the file clean up SfFat
            csvFile2.close();
            // Serial.println("Wrote to csvFile1 on SD 1");
            // flash(LED_PIN,2,60);
            sdB_Ready = true;
          }
          else // Sync failure
          {
            cardSyncError2 = true; // > 0
            sd2Healthy = false;    // = 0
            DBGSNPF(buffer,sizeof(buffer),"Card:%d sync failed - card possible removal mid-write\n",2);
            DBG(buffer);
          } //sync failure else close
        } else // write failure 
        { 
          sdB_Ready = false; 
          cardWriteError2 = true;
          sd2Healthy = false;
          DBGSNPF(buffer,sizeof(buffer),"Error writing %s\n ",fileName2);
          DBG(buffer);
        } // write failure else close
    } else // Open failure
    {
      sdB_Ready = false; // Mark error
      cardOpenError2 = true;
      sd2Healthy = false;
      DBGSNPF(buffer,sizeof(buffer),"Error opening %s\n ",fileName2);
      DBG(buffer);
    }
  } // end of !sdARecovError otherwise end of we are good

  // Open, Write, or Sync error occurred need to recover
  if (!sd2Healthy) //  not true = 0 = false
  {
    // csvfile1.close();          // safe even if already bad
    // sdFatA.end();  
    if (!sdHardRecover(sdB))  // = 0
    {
      DBGLN("SD2 offline — using SD1 only");
      sdB_Ready = false; // = 0
      sdBRecovError = true; // > 0
    } else // HardRecover good  now clear up SfFat by closing open files
    {
      if (cardSyncError2){csvFile2.close(); delay(10);}
      if (cardWriteError2){csvFile2.close(); delay(10);}
      DBGLN("SD2 Recovered!");
      sdB_Ready = true; // > 0
      sdBRecovError = false; // > 0
      sd2Healthy = true;
    }

  } 
  // SD2 New End

  if (sdDataLen != written1) //LFCR
  {
    cardWriteError1 = true;  // > 0
    DBGSNPF(buffer,sizeof(buffer),"Card:%d write size:%d not same as data size:%d\n",1,written1,sdDataLen);
    DBG(buffer);
  }
  if (sdDataLen != written2) //LFCR // inplies card may be removed mi-write, card removed or SPi failure
  {
    cardWriteError2 = true;  // > 0
    DBGSNPF(buffer,sizeof(buffer),"Card:%d write size:%d not same as data size:%d\n",2,written2,sdDataLen);
    DBG(buffer);
  }
  
  // indicate finish SD operations
  // digitalWrite(LED_PIN2, LOW);
  if (sdA_Ready || sdB_Ready)

  // if (sdA_Ready && sdB_Ready)  // > 0 && > 0
  {
    sdWriteStatus = true;      // > 0
    return true;               // > 0
  }
  else 
  {
    DBGSNPF(buffer,sizeof(buffer),"** error SD Status on 1: %d and on 2: %d\n",sdA_Ready,sdB_Ready);
    DBG(buffer);
    // flash(LED_PIN2,12,60);
    sdWriteStatus = false;    // = 0
    return false;             // = 0
  }
}

// ----Call Back ---- from a timer interupt
bool timer_callback(struct repeating_timer *t) 
{
  // check the call back interval  Checking for mismatch with DS3231 RTC time (if so flag it)
  // datetime_t ptt;

  char buffer[199];
  int16_t picoYear  = 2000;
  int16_t dsYear = 2000;
  int16_t year2026 = 2026;
  
  readRTC();  //  read PICO RTC  -- then use cache variables

  bool rtcPicoOk = rtcPICOOK();   //check out timing order  placed here after the PICO rtc read
  
  // Build
  DBGSNPF(buffer, sizeof(buffer),
  "Print PICO  Time Call Back invoked:%02d:%02d:%02d\n\n ",
  pico_rtc_cache.h, pico_rtc_cache.m, pico_rtc_cache.s);
  DBG(buffer);
  snprintf(bufferpi, sizeof(bufferpi),
  "%04d-%02d-%02d %02d:%02d:%02d\n (DOW=%d)\n\n",
  pico_rtc_cache.y,
  pico_rtc_cache.mo,
  pico_rtc_cache.d,
  pico_rtc_cache.h,
  pico_rtc_cache.m,
  pico_rtc_cache.s,
  pico_rtc_cache.dow);
  //   old   version ptt.year, ptt.month, ptt.day, ptt.hour, ptt.min, ptt.sec, ptt.dotw);

  picoYear = pico_rtc_cache.y;
  // Serial.printf("currentYear:%d\t",currentYear);
  dsYear = currentYear + 2000;
  // Serial.printf("dsYear:%d\n",dsYear);

  int16_t diff = picoYear - dsYear;
  if (diff < 0) diff = -diff;

  bool mismatch = (diff > 1) ||  (picoYear < 2026);
  if (!mismatch)
  {
    rtcMismatchCount = 0;
    rtcCheckFlag = false;
  }
  else
  {
    rtcMismatchCount++;
    // flash(LED_PIN,2,60);
    if (rtcMismatchCount >= 2)
    {
        rtcMismatchFlag = true;
        DBGPF("rtcMismatchFlag > 2: %d \t",rtcMismatchFlag);
        DBGPF(" diff:%d picoYear:%d dsYear:%d rtcPICOOK returned:%s\n",diff,picoYear,dsYear,rtcPicoOk?"true":"false");        
    }
  }
    
  // print the time in the two clocks
  printTime();  //  which call DS3231 rtc Read
  return true;
}


struct repeating_timer timer;


//  function to read PICO onboard RTC and print value  ############### Need work #######
bool readRTC() { // PICO onboard clock
    datetime_t t;

    // julian Dates
    int jul = 0;
    long compDate = 0;

    rtc_get_datetime(&t);
    flLED(LED_BUILTIN);

    // copy PICO RTC data to pico_rtc_cache  Global struct
    pico_rtc_cache.y = t.year;
    pico_rtc_cache.mo = t.month;
    pico_rtc_cache.d = t.day;
    pico_rtc_cache.dow = t.dotw;
    pico_rtc_cache.h = t.hour;
    pico_rtc_cache.m = t.min;
    pico_rtc_cache.s = t.sec;

        // now test the copy
    // INSP("\ncallback - copying to pico_rtc_cache ");
    DBGSNPF(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d\n",
    pico_rtc_cache.y,
    pico_rtc_cache.mo,
    pico_rtc_cache.d,
    // pico_rtc_cache.dow,
    pico_rtc_cache.h,
    pico_rtc_cache.m,
    pico_rtc_cache.s); 
    // INSP(buffer);

      // check if new month if so list files on SD
  if (currentMth != pico_rtc_cache.mo) 
  {
    // list files on SD's
    DBG("New Month - List SD files");
    listFiles(*sdA.sd, "SD Card 1 (SPI0)");
    listFiles(*sdB.sd, "SD Card 2 (SPI1)");
  }

    // save current Hour, Month and Year for comparisons and file name construction
    currentHour = t.hour;  // for the file flush
    currentMth  = t.month; // for monthly files
    currentYear = (t.year - 2000);  // for monthyear file only year not century


    // if (formatPicoTimeStamp(bufferpi,sizeof(bufferpi))) Serial.print(bufferpi);
    // else Serial.print("formatPicoTimeStamp error ??");
    snprintf(bufferpi, sizeof(bufferpi),  // for the OLED
             "%04d-%02d-%02d %02d:%02d:%02d\n (DOW=%d)\n\n",
             t.year, t.month, t.day, t.hour, t.min, t.sec, t.dotw);

     //  get julian date
    jul = julian(t.day,t.month,t.year);
    // add year component
    compDate = t.year * 1000 + jul;

    #ifdef TURNONPRT
      DBGSNPF(buffer, sizeof(buffer),"jul:%d compDate:%ld\t",jul, compDate);
      DBG(buffer);

      DBGSNPF(buffer, sizeof(buffer),"Global_comDate:%ld\n",Global_comDate);
      DBG(buffer);
    #endif

    if (Global_comDate == 0) writeEEPROM(compDate);
    if (Global_comDate > 2030001) writeEEPROM(compDate);
    // compare it to today
    if ((Global_comDate != compDate) && ((Global_comDate < 2030001) && (compDate <2030001))) // not equal so update necessary
    {
      DBGSNPF(buffer, sizeof(buffer),"Global_comDate:%ld compDate:%ld\n",Global_comDate,compDate);
      DBG(buffer);
      writeEEPROM(compDate);
    } 

    #ifdef TURNONPRT
      DBG("PICO RTC Time:\n");
      DBGLN(bufferpi);
    #endif

    // Display on OLED  -- is this a dup of PrintTime()  call in LOOP
    // display.stopscroll();
    // display.clearDisplay();
    // display.display();
    // delay(300);
    // display.setTextSize(2);
    // display.setCursor(0, 0);
    // display.println("Both RTCs:");
    // display.setTextSize(1);
    // display.setCursor(0, 18);
    // display.println(bufferpi);
    // display.setCursor(0, 42);
    // display.println(bufferds);
    // display.display();
    // delay(200);
    // display.startscrollright(0x00,0x01);
    // display.startscrolldiagright(0x00,0x00);
  return true;
}

// // Day of Week in C (Zeller’s Congruence)
// const char *getDayOfWeek(int day, int month, int year)
// {
//     // Adjust months so March = 3 ... January = 13, February = 14
//     if (month < 3) {
//         month += 12;
//         year--;
//     }

//     int K = year % 100;   // Year of the century
//     int J = year / 100;   // Zero-based century

//     int h = (day
//             + (13 * (month + 1)) / 5
//             + K
//             + K / 4
//             + J / 4
//             + 5 * J) % 7;

//     // Zeller's output: 0=Saturday, 1=Sunday, 2=Monday, ...
//     static const char *days[] = {
//         "Saturday",
//         "Sunday",
//         "Monday",
//         "Tuesday",
//         "Wednesday",
//         "Thursday",
//         "Friday"
//     };

//     return days[h];
// }

// // wifi status routines
// void checkWiFiAvailability() {
//   // Use a cast to convert uint8_t to wl_status_t to avoid the compiler error
//   wl_status_t status = (wl_status_t)WiFi.status();

//   if (status != WL_CONNECTED) {
//     Serial.print("WiFi is NOT available. Status: ");
//     Serial.println((int)status); // Print the raw number for diagnostics

//     switch(status) {
//       case WL_NO_SSID_AVAIL:
//         Serial.println("Error: SSID not found.");
//         break;
//       case WL_CONNECT_FAILED:
//         Serial.println("Error: Connection failed.");
//         break;
//       case WL_CONNECTION_LOST:
//         Serial.println("Error: Connection lost.");
//         break;
//       case WL_DISCONNECTED:
//         Serial.println("Error: Disconnected.");
//         break;
//       default:
//         Serial.println("Error: WiFi is idle or in an unknown state.");
//         break;
//     }
//   } else {
//     Serial.println("WiFi is available and connected.");
//   }
// }

// Time functions Julian Date and LeapYear
int julian(int day, int month, int year)
{
  static int runsum[] = {0, 31 ,59, 90, 120, 161, 181, 212, 134, 273,  304, 334, 365};
  int total;
  total = runsum[month -1] + day;
  if (month > 2) total += leapyear(year);
  return (total);
}

// is it leap year?
int leapyear(int year)
{
  if (year % 4 == 0 && year % 100 != 0 || year % 400 == 0) return 1;
  else return 0;
}

// -------------------------- EEPROM routines to store julian date ---------------------------------------------------
// write to the EEPROM the julDate
void writeEEPROM(long julDate)
{
  // digitalWrite(LED_PIN2, HIGH);
  // digitalWrite(LED_PIN, HIGH);
  // flash(LED_PIN1,2,60);
  bool statusEEPROM = false;
  flLED(eP_LED);
  // statusEEPROM = eepromWriteWithAck(CDATE, julDate);
  statusEEPROM  = eeprom.eeprom_write(CDATE, julDate);
  // delay(30);
  // statusEEPROM = eepromWriteByte(CDATE, julDate);  // current going back to old

  if (!statusEEPROM) 
  {
    DBGLN(F("I2C Bus Error - check wiring"));
  } else 
  {
    // wait for transfer success
    // waitReady();
    // Serial.println(F("New Date  ->  long was stored correctly."));
    // update the Global_comDate
    Global_comDate = julDate;
    DBGSNPF(buffer, sizeof(buffer),"New Date %ld ->  long was stored correctly.\n",Global_comDate);
    DBGLN(buffer);
  }
  // digitalWrite(LED_PIN2, LOW);
  // digitalWrite(LED_PIN, LOW);
}

// read from the EEPROM the julDate
long readEEPROM()
{
  // digitalWrite(LED_PIN2, HIGH);
  // digitalWrite(LED_PIN, HIGH);
  // flash(LED_PIN1,2,60);
  long comDate = 0;
  flLED(eP_LED);
  eeprom.eeprom_read(CDATE, &comDate);
  delay(30);
  DBGSNPF(buffer, sizeof(buffer),"EEPROM Data:%ld \n",comDate);
  DBG(buffer);
  // digitalWrite(LED_PIN2, LOW);
  // digitalWrite(LED_PIN, LOW);
  return comDate;

}

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


// print the current DS3231 time
void printTime() //DS3231 RTC module  clock
{

    char t[10];   // 8 chars and NULL for float temp to char 
    ASSERT(sizeof(t) >= 10);

    // Serial.printf("DS3231 RTC time: ");
    if (!rtcRefresh())  // this will replace all rtc.refresh() calls  after testing -----------------------------------------
    {
      DBGPF("DS3231 failed to read time after a refresh cycle");
      return;
    } else
    {
      // rtc.refresh();  // Read DS3231 values
      flLED(rtc_LED); // blue led

      // convert float temp to character for printing
      dtostrf(rtc_cache.t / 100, 8, 2, t);
      // Serial.printf(" %02d-%02d-%02d %02d:%02d:%02d  %s%s\t",
                // rtc.year(), rtc.month(), rtc.day(),
                // rtc.hour(), rtc.minute(), rtc.second(),t,"\xC2\xB0");
      // if (formatTimestamp(bufferds,sizeof(bufferds))) Serial.print(bufferds);
      // else Serial.print("formatTimeStamp error ??");

      //  we will add the BMx char temperature to the second line.


      snprintf(bufferds, sizeof(bufferds), // for the OLED
              "DS:%02d-%02d-%02d %02d:%02d:%02d\nD%s B%s\n",
              rtc_cache.y, rtc_cache.mo, rtc_cache.d,
              rtc_cache.h, rtc_cache.m, rtc_cache.s,t,bmCTemp);
      // #ifdef TURNONPRT
        DBGPF("bmCTemp:%s\n",bmCTemp);
        DBG(bufferds);
      // #endif
      flagDateTimeDisp = true;
    }
  }

  bool setDateTimeDisp(){
     // Display on OLED DS3231 data  have to build buffer yet
    // display.clearDisplay();
    // display.setTextSize(2);
    // display.setCursor(0, 0);
    // display.println("DS3231 RTC:");
    display.stopscroll();
    display.clearDisplay();
    display.display();
    
    delay(300);
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.println("Both RTCs:");
    display.setTextSize(1);
    display.setCursor(0, 18);
    display.println(bufferpi);
    display.setCursor(0, 42);
    display.println(bufferds);
    display.display();
    delay(300);
    display.startscrollright(0x00,0x01);

    // displayed turn off flag;
    flagDateTimeDisp = false;
  return true;
}

// Display cycle remaining on OLED
void setCycleDisp(int f, long t) 
{
  // Build a display for number of cycles remaining 
  // char status[10] = "";
  // snprintf(status, sizeof(status), "%s", sdWriteStatus ? "good" : "error");
  char buffer[194];
  snprintf(buffer, sizeof(buffer),"Cycles Left:    %d\nMS Left:     %ld\n\nSD Write Status:%s",f, t,sdWriteStatus ? "good" : "error");

  //clear OLED screen
  display.stopscroll();
  display.clearDisplay();
  display.display();
  delay(500);
  // display on OLED
  // display.clearDisplay();
  // display.display();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println("Cycles:");
  display.setTextSize(1);
  display.setCursor(0, 18);
  display.println(buffer);
  display.display();
  display.startscrollright(0x00,0x01);
  // return true;  not needed
}

#ifdef WIFI
//  geLocalTime routine from the NTP time server
bool getLocalTime(struct tm * info, uint32_t ms = 5000) {
  time_t now = time(nullptr);
  uint32_t start = millis();
  while (now < 8 * 3600 * 2 && millis() - start < ms) {
    delay(10);
    now = time(nullptr);
  }
  if (now < 8 * 3600 * 2) {
    return false;
  }
  *info = *localtime(&now);
  return true;
}
#endif

//  set up the SdFat time link call back  // for the date time record related to files via SfFat
// 1. PLACE CALLBACK HERE (Before setup)
// rtc routines 

// RTC sanity check
bool rtcPICOOK()
{
    datetime_t t;
    // Get time from Pico internal RTC
    rtc_get_datetime(&t); 
    flLED(LED_BUILTIN);
    bool rtcPICOrtN = true;  // will return true unless sanity check changes 
  // rtc.set(
  //   t.sec,
  //   t.min,
  //   t.hour,
  //   dayOfWeekLib, // Use the adjusted Day of Week
  //   t.day,
  //   t.month,
  //   t.year % 100 // Use only the last two digits of the year (0-99)
  // );

    int year = t.year;  
    int month = t.month;
    int day = t.day;

   
    // Basic sanity check
    if (year < 2023 || year > 2100) rtcPICOrtN = false;
    if (month < 1 || month > 12)    rtcPICOrtN = false;
    if (day < 1 || day > 31)        rtcPICOrtN = false;
    if (!rtcPICOrtN) 
    {
       DBGPF("Pico date check:%04d-%02d-%02d %02d:%02d:%02d\n", t.year, t.month, t.day, t.hour, t.min, t.sec);
    }

    return rtcPICOrtN;
}

// RTC sanity check
bool rtcOk()
{
    char t[10], buffer[200] = "";
    ASSERT(sizeof(t) >= 10);

    bool rtcRefreshRTNCode = rtc.refresh();
    if (rtcRefreshRTNCode) 
    {
      // use cache variable not rtc variables
      rtc_cache.y  = rtc.year();
      rtc_cache.mo = rtc.month();
      rtc_cache.d  = rtc.day();
      rtc_cache.dow = rtc.dayOfWeek();
      rtc_cache.h  = rtc.hour();
      rtc_cache.m  = rtc.minute();
      rtc_cache.s  = rtc.second();
      rtc_cache.t  = rtc.temp();
      rtc_ok = true;
    } else DBGPF("ERROR-Could NOT read DS3231 RTC - Return code:%d",rtcRefreshRTNCode);

    flLED(rtc_LED);

    int year = rtc_cache.y + 2000;   // uRTCLib gives year offset
    int month = rtc_cache.mo;
    int day = rtc_cache.d;

    // convert float temp to character for printing
    dtostrf(rtc.temp() / 100, 8, 2, t);

    if (!rtc_ok) 
    {
    DBGSNPF(buffer, sizeof(buffer),
      "rtcOK check DS:%02d-%02d-%02d %02d:%02d:%02d\n %s rtcOK year:%d month:%d day:%d \n",
      rtc_cache.y, rtc_cache.mo, rtc_cache.d,
      rtc_cache.h, rtc_cache.m, rtc_cache.s,t,year,month,day);
      // rtc.year(), rtc.month(), rtc.day(),
      // rtc.hour(), rtc.minute(), rtc.second(),t);
      // DBG(buffer);
    }

    // Basic sanity check
    if (year < 2023 || year > 2100) return false;
    if (month < 1 || month > 12)    return false;
    if (day < 1 || day > 31)        return false;

    return true;
}

// for SfFat file time records
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) 
{
  datetime_t t;
  rtc_get_datetime(&t);
  flLED(LED_BUILTIN);
  *date = FS_DATE(t.year, t.month, t.day);
  *time = FS_TIME(t.hour, t.min, t.sec);
  *ms10 = 0;
}


// Function to synchronize the DS3231 time to the Pico's internal RTC time
void syncPicoToDS3231() {
  datetime_t t;
  // Get time from Pico internal RTC
  rtc_get_datetime(&t);
  flLED(LED_BUILTIN); 

  // Set the DS3231 RTC using uRTCLib's set function
  // The uRTCLib set function expects parameters in a specific order: 
  // second, minute, hour, dayOfWeek (1=Sun, 7=Sat), dayOfMonth, month, year (0-99)
  // The Pico's 'day of week' (t.dotw) is 0-6 (Sunday is 0), so we adjust it for uRTCLib (Sunday is 1)
  uint8_t dayOfWeekLib = (t.dotw == 0) ? 1 : t.dotw + 1;

  rtc.set(
            t.sec,
            t.min,
            t.hour,
            dayOfWeekLib, // Use the adjusted Day of Week
            t.day,
            t.month,
            t.year % 100 // Use only the last two digits of the year (0-99)
          );
}

// Routine to call to resync PICO time after power failure
bool syncDS3231ToPico() 
{
  // Update internal library variables from the physical DS3231
  rtcRefresh();
  flLED(rtc_LED);
  bool rtcok = rtcOk();
  if (!rtcok)
  {  
    char buffer[120] = "";
    DBGPF("DS rtc failed RTC sanity check:%d",rtcok);
    DBGSNPF(buffer, sizeof(buffer), // for the OLED
    "DS:%02d-%02d-%02d %02d:%02d:%02d\n %s\n",
    rtc_cache.y,rtc_cache.mo,rtc_cache.d,rtc_cache.h,rtc_cache.m,rtc_cache.s,t);
    // rtc.year(), rtc.month(), rtc.day(),
    // rtc.hour(), rtc.minute(), rtc.second(),t);
    DBG(buffer);
    DBGLN("Error: rtcOk routine rejected the DS3231 rtc time.");
    return false;
  } 
  else // let try
  {
    // Prepare the Pico internal RTC format
    // uRTCLib returns year as offset from 2000 (e.g., 26 for 2026)
    datetime_t t = 
      {
        .year  = (int16_t)(2000 + rtc_cache.y),
        .month = (int8_t)rtc_cache.mo,
        .day   = (int8_t)rtc_cache.d,
        // Pico 0 is Sunday; uRTCLib 1 is Sunday
        .dotw  = (int8_t)(rtc_cache.dow - 1), 
        .hour  = (int8_t)rtc_cache.h,
        .min   = (int8_t)rtc_cache.m,
        .sec   = (int8_t)rtc_cache.s
      };
  
    // Set the Pico's hardware RTC to the DS3231 time
    if (rtc_set_datetime(&t))
    {
      DBG("Pico RTC successfully synced from DS3231: ");
      DBGPF("%04d-%02d-%02d %02d:%02d:%02d\n", t.year, t.month, t.day, t.hour, t.min, t.sec);
      return true;
    } else 
    {
      DBGLN("Error: Pico RTC rejected the datetime.");
      return false;
    }
  }
}
// --------------------------------------- Get time --------------------------------------------------------------
// function to get NTP time then set the PICO RTC and the DS3231 RTC and to print or display the time, update the EEPROM if necessary and to set an alarm if requested
void getTime() 
{
    // julian Dates

    int jul = 0;
    long compDate = 0;
  

    // EEPROM data
   
#ifdef WIFI
    // GET TIME FROM NTP SERVER
    Serial.print("Waiting for NTP server");
    while (!getLocalTime(&timeinfo)) {
        // Serial.println("Waiting for time");
        Serial.print(".");
        delay(250);
    }

    // Set internal PICO RTC from NTP Time
    datetime_t t = {
        .year  = static_cast<int16_t>(timeinfo.tm_year + 1900), 
        .month = static_cast<int8_t>(timeinfo.tm_mon + 1),
        .day   = static_cast<int8_t>(timeinfo.tm_mday),
        .dotw  = static_cast<int8_t>(timeinfo.tm_wday),
        .hour  = static_cast<int8_t>(timeinfo.tm_hour),
        .min   = static_cast<int8_t>(timeinfo.tm_min),
        .sec   = static_cast<int8_t>(timeinfo.tm_sec)
    };
#endif

    // update the PICO onboard RTC
    rtc_init();
    flLED(LED_BUILTIN);

#ifndef WIFI
  // now that it is initialized sync with the DS3231
  if (syncDS3231ToPico())
  {
    Serial.print("PICO RTC Updated from DS3231\n");
  } else
  {
    Serial.print("Sync of PICO RTC with DS3231 failed\n");
  }
#endif

    // copy PICO RTC data to pico_rtc_cache  Global struct
    pico_rtc_cache.y = t.year;
    pico_rtc_cache.mo = t.month;
    pico_rtc_cache.d = t.day;
    pico_rtc_cache.dow = t.dotw;
    pico_rtc_cache.h = t.hour;
    pico_rtc_cache.m = t.min;
    pico_rtc_cache.s = t.sec;
    // now test the copy
    // DBGLN("After copy to pico_rtc_cache ");
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d\n",
    pico_rtc_cache.y,
    pico_rtc_cache.mo,
    pico_rtc_cache.d,
    // pico_rtc_cache.dow,
    pico_rtc_cache.h,
    pico_rtc_cache.m,
    pico_rtc_cache.s); 
    // INSP(buffer);


    // Test PICO  time PRINT IT from the PICO RTC time server
    DBGLN("After rtc_init  ----Now print the PICO RTC time again");
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d\n",
            t.year, t.month, t.day, t.hour, t.min, t.sec);
    // DBGLN(buffer);
    // rtc_set_datetime(&t);
    // Test PICO  time PRINT IT from the PICO RTC time server
    DBGLN("After rtc_set_datetime ---Now print the PICO RTC time again");
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d\n",
            t.year, t.month, t.day, t.hour, t.min, t.sec);
    DBGLN(buffer);

    //  NWG set DS3231
    // NWG change to rtc set
    t.dotw = t.dotw + 1;  // fix for DS3231 RTC DOW
    if (t.dotw > 7){t.dotw = 1;}

    //  snprintf(buffer,sizeof(buffer),"Day of week: %s\n", getDayOfWeek(t.day,t.month,t.year));
    //  Serial.print(buffer);
    
#ifdef WIFI
    // Only set the DS3231 if the NTP has set the datetime_t struct.  So year will not be 2021 otherwise use the DS3231 time
    // correct year to exclude century and set the DS3231
    // if (t.year != 2021)
     rtc.set(t.sec,t.min,t.hour,t.dotw,t.day,t.month,(t.year-2000)); // use PICO RTC TIME to set DS3231
    // sync to DS3231 time if PICO rtc has not been set
    // if (t.year <= 2021) syncDS3231ToPico(); // sync PICO RTC to DS3231 RTC
#endif

    
     // 3. Register the callback with SdFat
    // This ensures ALL SdFat instances (sdA, sdB) use this timestamp source
    FsDateTime::setCallback(dateTime);

    rtc_set_datetime(&t);
    flLED(LED_BUILTIN);
    DBGLN("FsStat Callback set up ---Now print the PICO RTC time again");
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d\n",
             t.year, t.month, t.day, t.hour, t.min, t.sec);
    DBGLN("PICO Time  synchronized and PICO RTC set.");

     //  get julian date
    jul = julian(t.day,t.month,t.year);
    // add year component
    compDate = t.year * 1000 + jul;
    // fix Global_comDate


    // print time from the onboard PICO time server
    snprintf(buffer, sizeof(buffer), "PICO RTC: %04d-%02d-%02d %02d:%02d:%02d Julian:%d Compare Date:%ld Global comDate:%ld\n",
            t.year, t.month, t.day, t.hour, t.min, t.sec,jul,compDate,Global_comDate);

    DBGLN(buffer);
    DBGPF("Time zone: %s\n", tzname[0]);  // PST or PDT



    // update EEPROM if necessary
    Global_comDate = readEEPROM(); // read current EEPROM date
        // fix Global_comDate before the bad Global is read
    if (Global_comDate > 2030000)
    {
      writeEEPROM(compDate);          // FIX bad date
      Global_comDate = readEEPROM();  // RE-read current EEPROM date

    } 
    // print it
    snprintf(buffer, sizeof(buffer),"Global EEPROM Date Data:%ld \n",Global_comDate);
    Serial.print(buffer);

    // compare it to today
    if (Global_comDate != compDate) // not equal so update necessary
    {
      snprintf(buffer, sizeof(buffer),"Global_comDate:%ld compDate:%ld\n",Global_comDate,compDate);
      Serial.print(buffer);
      writeEEPROM(compDate);
    }
 
    // now test the alarm call back
    //  set_alarm(15,25);

    readRTC();   // from the PICO RTC
    DBGLN("Now print the PICO RTC time again");
    snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d",
            t.year, t.month, t.day, t.hour, t.min, t.sec);
    DBGLN(buffer);
    printTime(); // from the DS3231
    // Now read the DS3231 RTC back
    rtc.refresh();  // Read back the DS3231 values 
    flLED(rtc_LED);
    DBGPF("NWG PICO RTC and NTP Year:%d\n",t.year);  //from the pico not the DS3231

    // save current Hour, Month and Year for comparisons and file name construction
    currentHour = t.hour;  // for the file flush
    currentMth  = t.month; // for monthly files
    currentYear = (t.year - 2000);  // for monthyear file only year not century
    // Serial.printf("currentYear:%d\n",currentYear);
  
    // See if files exist or is this the first time

    // readRTC();   // from the PICO RTC
    // printTime(); // from the DS3231
 
    // print time from the PICO RTC time server
    DBGLN("Now print the PICO or DS3231 RTC time again");
    snprintf(buffer, sizeof(buffer), "%02d-%02d-%02d %02d:%02d:%02d",
            t.year, t.month, t.day, t.hour, t.min, t.sec);
    DBGLN(buffer);

    Serial.println("Time Setup  Done!");
    // flash(LED_PIN2,4,300);
}

// ------------------------------------- Routine to print SD Card directory and File sizes


bool listFiles(SdFat &sdInstance, const char* label) {
  FsFile root;
  FsFile file;
  char fileName[256];

  Serial.printf("\n--- %s ---\n", label);

  sdInstance.chvol(); // needed to tell SdFat to handle two SD Cards
  if (!root.open("/",O_RDONLY)) {
    Serial.printf("Error opening %s root\n", label);
    return false;
  }

  while (file.openNext(&root, O_RDONLY)) {
    file.getName(fileName, sizeof(fileName));
    if (file.isDir()) {
      Serial.printf("[DIR]  %s/\n", fileName);
    } else {
      Serial.printf("FILE   %-20s", fileName);
    }

    // --- NEW: Display Date and Time ---
    Serial.print("  Modified: ");
    file.printModifyDateTime(&Serial); // Built-in formatter: YYYY-MM-DD HH:MM:SS
    
    if (!file.isDir()) {
      Serial.printf("  %llu bytes", file.fileSize());
    }
    
    Serial.println();
    file.close();
  }
  root.close();
  return true;
}




// ------------------------------------------------- Setup ------------------------------------------------------
void setup() {
  Serial.begin(9600);

  // let everything start first -- wait five seconds 
  unsigned long start = millis();
  while (!Serial && (millis() - start < 5000)) { delay(10); } // need both conditions to fail to loop
  delay(15);
  Serial.print("\n--------------------------------------------- Pico SD Boot Sequence --------------------------------\n");
  Serial.print(F("Processor: "));

  #ifdef PICOW
    Serial.println(F("Raspberry Pi Pico W"));
  #endif

  // Start the I2C bus
    // Must power on DS3231 and any other I2C module first
  // ----------------------------------------------  powering on the DS3231  and I2C bus  ---------------------------------------------------
    delay(500);
    DBG("Ensure I2C pins are NOT driving\n");
    pinMode(SDA, INPUT);      //I2C
    pinMode(SCL, INPUT);      //I2C
    pinMode(rtc_LED, OUTPUT); // DS3231 Time activity
    pinMode(pi_LED, OUTPUT);  // BMx  Time activity
    pinMode(eP_LED, OUTPUT);  // 24C32  EEPROM activity
    pinMode(LED_BUILTIN, OUTPUT); //pico Time
  
    delay(500);

    rtcPowerCycle();  // power off the back on the DS3231
    delay(59);

    // Recover the I2C bus 
    if (i2cRecover())
    {
      Serial.print("I2C Recovered\n");
    }

    i2cInit(); // -------------------------- initialize Wire
    delay(100);

    //  Presence test
    if (i2cDevicePresent(0x68)) {
        DBGLN("RTC present and OK");
        rtc.refresh();
        flLED(rtc_LED);
    } else {
        Serial.println("RTC bus error at boot  --- TROUBLE \n");
    }
  // I2C Started

  // ---------------------------------------------------------- OLED setup ---------------------------------------------------------
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setRotation(0);
  display.display(); // display the logo
  delay(4000);
    //clear OLED screen
  display.stopscroll();
  display.clearDisplay();
  display.display();
  
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("OLED INIT");
  display.display();
  delay(4000);
  display.clearDisplay();

#ifdef WIFI
// --------------------------------- access wifi ------------------------------------------------------------------
  Serial.print("Trying to Connect to WiFi\n");
  display.println("WiFi INIT");
  display.display();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {  // try for 50 or so attempts if no connection 
    delay(500);
    Serial.print(".");
  }

// connection extablished
  Serial.println("\nWiFi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

// set time to PST or PDT
  setenv("TZ", "PST8PDT,M3.2.0/2,M11.1.0/2", 1);
  tzset();
  configTime(0, 0, ntpServer1, ntpServer2);

  Serial.print("Requesting time from NTP Server\n");
  // struct tm timeinfo;  // made global
  display.println("NTP Sync");
  display.display();
#endif

  // routine to handle PICO and DS3231 time

  //  ------------------ CALL getTime to sync all time events -----------------------------------------------------
  getTime();

   // test LED
  flLED(pi_LED);
  flLED(rtc_LED);
  flLED(eP_LED);
  flLED(LED_BUILTIN);


  // --------------------------------------------------- Create repeating timer callback :--------------------------------------------
  // Interval = TIMECYCLE or initial 120 seconds = 120,000 ms = 120,000,000 µs

  add_repeating_timer_us(TIMECYCLE * 1000000, timer_callback, NULL, &timer);   //NEEDED

  // ----------------------------------------------- Initialize the BME or BMP --------------------------------------------
  bmInit();

  //------------------------------------- Scan for I2C devices ---------------------------------------------------------
  scanI2C();

  // check the EEPROM
  DBG("EEPROM ready check: ");
  DBG(i2cDevicePresent(0x57) ? "OK\n" : "FAIL\n");


//-------------------------------------------------- SD Startup -------------------------------------------------------------
    sdA.sd = &sdFatA;
    sdB.sd = &sdFatB;

  // Start up the SD modules SD have a MOSFET which switches on power when a PICO pin is set HIGH
  // 0. SPI pin mapping
  spiPinMapping(sdA,sdB);

  // 1. Power on


  // set up the LED
  pinMode(SD1_LED, OUTPUT);
  pinMode(SD2_LED, OUTPUT);


 sdOn(SD1_LED);
  if (!sdPowerOn(sdA)) {
    Serial.print("Trouble with powering on SD1 -- Check wiring, and MOSFET\n");
  };
  sdOff(SD1_LED);

  pinMode(SD1_CS, OUTPUT);
  digitalWrite(SD1_CS, HIGH);
  delay(100); // Give hardware time to settle

  // 2. Init
  //  Initialization: Initialize each SD object with its CS pin
  // --- INITIALIZE SD1 ON SPI0 ---
   sdOn(SD1_LED);
  SPI.setRX(16); SPI.setTX(19); SPI.setSCK(18);
  if (!sdA.sd->begin(SdSpiConfig(sdA.pinSS, DEDICATED_SPI, SD_SCK_MHZ(16), sdA.spi))) {
    Serial.println("sdA (SPI0) Failed to initialize!");
  } else {
    DBGLN("sdA Init OK");
  }
  sdOff(SD1_LED);

  // 1. Power on
  sdOn(SD2_LED);
  if (!sdPowerOn(sdB)) {
    Serial.print("Trouble with powering on SD2 -- Check wiring, and MOSFET\n");  
  };
  sdOff(SD2_LED);

  pinMode(SD2_CS, OUTPUT);
  digitalWrite(SD2_CS, HIGH);
  delay(100); // Give hardware time to settle

  // 2. Init
  // sd2 uses SPI1
  sdOn(SD2_LED);
  SPI1.setRX(12); SPI1.setTX(11); SPI1.setSCK(10);
  if (!sdB.sd->begin(SdSpiConfig(sdB.pinSS, DEDICATED_SPI, SD_SCK_MHZ(16), sdB.spi))) {
    Serial.println("sdB (SPI1) Failed to initialize!");
  } else {
    DBGLN("sdB Init OK");
  }
  sdOff(SD2_LED);

  // List all files and directories recursively from the root directory

  Serial.println("\nSD Card 1 Listing files: Built in lib");
   sdOn(SD1_LED);
  sdA.sd->ls(LS_R); // LS_R flag enables recursive listing
  sdOff(SD1_LED);
  Serial.println("sdA Listed!");

     // List all files and directories recursively from the root directory
  Serial.println("\nSD Card 1 Listing files:");
  // sdA.sd->ls(LS_R); // LS_R flag enables recursive listing
   sdOn(SD1_LED);
  if (sdA.sd->exists("/")) listFiles(*sdA.sd, "SD Card 1 (SPI0)");
  sdOff(SD1_LED);
  Serial.println("SD A done!");

  Serial.println("\nSD Card 2 Listing files: Built in lib");
  sdOn(SD2_LED);
  sdB.sd->ls(LS_R); // LS_R flag enables recursive listing
  sdOff(SD2_LED);
  Serial.println("sdB  Built in lib!");

       // List all files and directories recursively from the root directory
  Serial.println("\nSD Card 2 Listing files:");
  // sd2.sd->ls(LS_R); // LS_R flag enables recursive listing
  sdOn(SD2_LED);
  if (sdB.sd->exists("/")) listFiles(*sdB.sd, "SD Card 2 (SPI1)");
  Serial.println("sd B done!");
  sdOff(SD2_LED);

  char fileExists[15] = "";
  // check if this is a Recovery by reading the DS3231 EEPROM data
  snprintf(fileExists,sizeof(fileExists),"csv1%02d%02d.txt",currentMth,currentYear); 

      // first time executed as CFile.txt does not exist   on SD  
  
  
  if (!sdA.sd->exists(fileExists)) // Name set in DateTest
  {
      snprintf(buffer,sizeof(buffer),"file %s not found using FirsteePROM\n",fileExists);
      eTemp = 128.1;
      Serial.print(buffer);    
      FirsteePROM();                  // set coldest to very high temp for force the update in LOOP

  } else {
      // ********** we are here because of a restart so read the eePROM for cTemp and cD *** NOT FIRST TIME ***
      snprintf(buffer,sizeof(buffer),"file %s found using RecoveryeePROM\n",fileExists);
      Serial.print(buffer);
      RecoveryeePROM();   
  }
 
  // Enable Watchdog
  watchdog_enable(WATCHDOG_TIMEOUT_MS, 1);


  // ----------------------------------------------watchdog enable fix for PICO RTC -------------------------- Temp????
   // now that watchdog corrupted PICO RTC Time sync with the DS3231
  delay(100);
  if (bool syncRtCode = syncDS3231ToPico())
  {
    Serial.printf("PICO RTC Updated from DS3231 -- return code:%s\n",syncRtCode?"true":"false");
  } else
  {
    Serial.printf("Sync of PICO RTC with DS3231 failed -- return code:%s\n",syncRtCode?"true":"false");
  }
   // ----------------------------------------------watchdog enable fix for PICO RTC -------------------------- Temp????

  // --------------------------------------------------
  // Start repeating timer AFTER WiFi/NTP, if needed
  // --------------------------------------------------
  // Start the repeating timer call back AFTER setup
  // 40,000 µs = 25 FPS smooth fades
  // add_repeating_timer_us(RAINBOW,               // 250ms = smooth visible cycle
  //       rainbowTimerCallback,                   // your callback
  //       NULL,
  //       &rainbowTimer
  //   );

  // // Update timer every 90 seconds
  // add_repeating_timer_us(
  //   SDTIMECYCLE * 1000 * 1000,   // 90,000,000 µs
  //   sdUpdate_timer_callback,
  //   NULL,
  //   &sdUpdateTimer
  // );

}
// ------------------------------------------- Loop ----------------------------------------
void loop() 
{
   bool cycle_ok = true;   // fresh verdict every cycle

  char buffer[150] = "";
  now = millis();
  if (firstLoop || (now - previous >= 600000))  // Ten minutes
  {
    // Read BMP280 or BME280
    firstLoop = false;
    previous  = now;
    t = 600000; //  Reset loop time to 10 minutes
    f = 10; // Reset cycles to 10

    // display dates from timer callback
    if (flagDateTimeDisp) setDateTimeDisp();
    // check if RTC years are the same
    if (rtcMismatchFlag) // fix it
    {
      rtcRecovered = syncDS3231ToPico(); // sync PICO RTC to DS3231 RTC
      if (!rtcRecovered) 
      {
        DBGPF("RTC sync refresh failed, will retry later rtcMismatchCount:%d rtcRecovered:%d\n",rtcMismatchCount,rtcRecovered);
      }
      rtcMismatchCount = 0;
      rtcMismatchFlag = false;
    } 


    // Read the DS3231
    // if (SPrint) {Serial.print("\nRTC_Read\t");}
    DBG("\n");
    if (!RTC_Read()) cycle_ok = false;
    // Now read the PICO RTC
    if (!readRTC()) cycle_ok = false;
    // Use PICO RTC as timestamp so change sdData So rebuild rtcData here
    // Use PICO RTC to control filenames

      // build a RTC data string with delimiter be written to the SD Chip
    int j = snprintf(buffer,sizeof(buffer), "%02d,%02d,%02d,%02d,%02d,%02d,%02d,%08s,", 20, // century, plus data
    (pico_rtc_cache.y - 2000),pico_rtc_cache.mo,pico_rtc_cache.d,pico_rtc_cache.h,pico_rtc_cache.m,pico_rtc_cache.s,rtcTemp);
    // DBGPF("size of the rtcData buffer:%d\n",j);
    // store in the global variable rtcData
    strcpy(rtcData, buffer);  // adds a \0 at the end
    j = strlen(rtcData);
    // DBGPF("strlen of the rtcData:%d\n",j);

    // build a print line for monitor with snprintf rather than many serial.print lines
    char rtcbuffer[100] = "";
    j = snprintf(rtcbuffer,sizeof(rtcbuffer),"RTC DT:%02d/%02d/%02d  %02d:%02d:%02d %s%sC\n",
    (pico_rtc_cache.y - 2000),pico_rtc_cache.mo,pico_rtc_cache.d,pico_rtc_cache.h,pico_rtc_cache.m,pico_rtc_cache.s,rtcTemp,"\xC2\xB0");  // use cache variables not module calls
    // if (SPrint) {Serial.print(rtcbuffer);} // only print when debugging
    // INSP(rtcbuffer);
    // DBGPF("snprintf size of the rtcbuffer:%d\n",j);


    // -------------------------------------- Do it HERE --------------------------------------------

    // Read the BMP or BME 280
    // if (SPrint) {Serial.print("\nBM_Read\t\n");}
    // Serial.print("\n");
    if (!bmTask()) cycle_ok = false;
    // BM_Read();in bmTask

    //Build the data record
    // if (SPrint) {Serial.println("build sdDATA");}
    // put in a SD delimiter
    // remember we are building a CSV file

    // ASSERT(sizeof(sdData) >= (sizeof(rtcData) + sizeof(bmData)  + 4);
    // ASSERT((strlen(rtcData) + strlen(bmData) + 4) < sizeof(sdData));


    strcpy(sdData,"SD,");
    strcat(sdData, rtcData);
    strcat(sdData, bmData);
    // moved to the listfiles print loop below
    // snprintf(buffer,sizeof(buffer),"%s\n",sdData);
    // INSP(buffer);

    // Write data to two SD's for backup
     sdOn(SD1_LED); sdOn(SD2_LED);
    if(!writeSDFile(sdData))
    {
      snprintf(buffer,sizeof(buffer),"** error SD Status on 1: %d and on 2: %d\n",sdA_Ready,sdB_Ready);
      Serial.print(buffer);
      cycle_ok = false;
      // flash(LED_PIN2,12,60);  // Indicate problems
    }
     sdOff(SD1_LED); sdOff(SD2_LED);

    // check if colder and update EEPROM
    promUpdate(bmTemp,sdData);

    #ifdef LISTFILES
      // list files on each SD card with size and time to see if updating occurs
      DBGLN("\nSD Card 1 Listing files:");
      // sdA.sd->ls(LS_R); // LS_R flag enables recursive listing
      sdOn(SD1_LED);
      listFiles(*sdA.sd, "SD Card 1 (SPI0)");  // if (sdA.sd->exists("/"))
      sdOff(SD1_LED);
      DBGLN("SD 1 listed!");
      LF(*sdA.sd, "SD Card 1 (SPI0)");

      DBGLN("\nSD Card 2 Listing files:");
      // sd2.sd->ls(LS_R); // LS_R flag enables recursive listing
      sdOn(SD2_LED);
      listFiles(*sdB.sd, "SD Card 2 (SPI1)");
      sdOff(SD2_LED);
      DBGLN("SD 2 listed!");
      LF(*sdB.sd, "SD Card 2 (SPI1)");
    #endif
  }
  // One minute cycle -- Display
  if (firstCycle || (now - cycles >= 60000))
  {
    setCycleDisp(f,t);
    firstCycle = false;
    cycles = now;
    // digitalWrite(LED_PIN2, HIGH);
    // flash(LED_PIN,f,120); //flash the number of cycle left to go
    // digitalWrite(LED_PIN2, LOW);
    f--; // reduce loop count
    t = t - 60000; //reduce remaining time by 10%
  }

  // check if we have an data to display on OLED
  if (now - dispCycle >= 30000) 
  {
    dispCycle = now;
    // display dates from timer callback
    if (flagDateTimeDisp) setDateTimeDisp();
  }
  
  if (now - previousLF >= 600000*LFCYCLE) // ever 1 hour  ie 6 * 10 minute intervals
  {
    previousLF = now;
    // display file list on cycle defined by LFCYCLE 

    snprintf(buffer,sizeof(buffer),"%s\n",sdData);
    DBG(buffer);

    DBGLN("SD Cycle list!");
    // SD 1
    sdOn(SD1_LED);
    LF(*sdA.sd, "SD Card 1 (SPI0)");
    sdOff(SD1_LED);
    // SD 2
    sdOn(SD2_LED);
    LF(*sdB.sd, "SD Card 2 (SPI1)");
    sdOff(SD2_LED);

    // display health counters
    DBGPF("Health: ******  i2cerr=%lu i2cto=%lu fs=%lu wd=%lu ****** \n",
    i2c_error_count, i2c_timeout_count,fs_fail_count, watchdog_kicks);
  }
  // --- End of successful 10-minute cycle ---

  // watchdog update
  if (cycle_ok) 
  {
    watchdog_update();
  }
}
