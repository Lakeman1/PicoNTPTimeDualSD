#define FW_VERSION "1.0.0"
#include "BuildInfo.h"

// DEBUGING CODE   ----- Preprocessor code ------------
#define DEBUG_ENABLE 1   // ← flip to 0 for production

#if DEBUG_ENABLE
  #define DBG(x)        Serial.print(x)
  #define DBGLN(x)      Serial.println(x)
  #define DBGPF(...)    Serial.printf(__VA_ARGS__)
  #define DBGSNPF(...)  snprintf(__VA_ARGS__)
  #define SP(...)       if (SPrint) {Serial.print(__VA_ARGS__);}
  #define SPF(...)      if (SPrint) {Serial.printf(__VA_ARGS__);}
  #define DBG_BOOL(label,b) DBGPF("%s: %s\n", label, ((b)?"true":"false"))
  // EXAMPLE: DBG_BOOL("\nreadRTC => BAD RETURN CODE:",rtncode);
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGPF(...)
  #define DBGSNPF(...)
  #define SP(...)
  #define SPF(...)
  #define DBG_BOOL(label,b)
#endif

// MACROS
// #define STAGE(s) \
//     stage = s; \
//     Serial.printf("Stage -> %s (%d)\n", stageName(stage), stage); \
//     writeStageEEPROM(stage)

/*
Breadboard wiring 
POWER
SD1 pin 26
SD2 pin 27
DS  pin 15

LED's 
DS  pin 14
24C32 pin 8 (eeprom)
SD1 pin 28
SD2 pin 21
BMP pin 22
Pico built-in



*/




#define STAGE_EARLY(s,msg) do { \
    stage = s; \
    DBGPF("Stage:%d %s\n",stage,stageToStr(stage)); \
} while(0)

#define STAGE(s,msg) do { \
    stage = s; \
    wdRecord(__LINE__); \
    DBGPF("Stage:%d %s\n",stage,stageToStr(stage)); \
    writeStageEEPROM(stage); \
    oledEvent(msg,stage); \
} while(0)

#define WD() wdRecord(__LINE__)

// #define PRODUCTION_BUILD     ← flip to 0 for production  ←  ←  ←  ←  ←  ←  ←  ←  ←  ← 

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

// Breadboard ID
#define NODE_ID_DEFAULT 1
#define NODE_ID_ADDR 24

// define NODE_ID 1
// #define NODE_ID 2
// #define NODE_ID 3

// # define LISTFILES
// -------------------------
#include "assert.h"  //for preprocessor in program folder
#include "GLUtils.h"

#include <Arduino.h>
#include <inttypes.h>

// | Type       | Format        |
// | ---------- | ------------- |
// | `uint8_t`  | `%" PRIu8 "`  |
// | `int8_t`   | `%" PRId8 "`  |
// | `uint16_t` | `%" PRIu16 "` |
// | `int16_t`  | `%" PRId16 "` |
// | `uint32_t` | `%" PRIu32 "` |
// | `int32_t`  | `%" PRId32 "` |
// | `float`    | `%f`          |
// | hex byte   | `%02X`        |


// #include <stdint.h>

// fence WiFi
#define WIFI

#ifdef WIFI
#include <WiFi.h>
#endif

#include "pico/stdlib.h"
// #include <pico/cyw43_arch.h>

// ---------------- Watchdog ----------------------------
// Watchdog Libraries
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"

#define WATCHDOG_TIMEOUT_MS 120000  // 2 minutes
#define WD_FEED_INTERVAL_MS 250


// #define WD_STAGE(s)
//   do {stage = s; wdRecordStage(stage);} while(0)

#include "hardware/rtc.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"

// ------------------------------- DS3231 ------------------
#include <uRTCLib.h>
// For eeprom  24C32 EEPRO on the DS3231
#include "uEEPROMLib.h"
// EEPROM MEMORY LAYOUT
// Addr   Size   Description
// ----   ----   -----------------------------
// 0      4      stage of processing
// 4      8      wd_COUNT addr
// 16     12     TimeSnapshot structure
// 24     1      NODE_ID
// 28     4      new bootCount 2 bytes plus
// 40     8      eTemp eTemp  coldest temp
// 48     8      eTemp2 eTemp2  coldest temp

#define EEPROM_STAGE_ADDR  0
#define EEPROM_LINE_ADDR  1

#define WD_MAGIC 0x5A
#define EEPROM_WD_MAGIC_ADDR 3
#define ETEMP 40   // location for eTemp storage in EEPROM
#define ETEMP2 48   // location for eTemp2 storage in EEPROM
// #define CDATE 8   // location for compDate storage in EEPROM

#define SNAP_ADDR  16

// #define WD_STAGE_ADDR   32
// #define WD_COUNT_ADDR   33 
// #define EEPROM_WD_LOG_BASE   32      // you said stage uses 32/33
// #define EE_WD_SLOTS  4       // because using 32/33 only  // changed from 2 to 4

#define EEPROM_WD_COUNT_ADDR  4

#define BOOTCOUNT_ADDR 28  // eePROM ADDR for 2 bytes
// #define EEPROM_BOOTCOUNT_ADDR 28
#define BOOTCOUNT_MAGIC_ADDR     (BOOTCOUNT_ADDR + sizeof(uint16_t))

// ON the DS3231 module
// Instantiate RTC object on default I2C (GP0=SDA, GP1=SCL)
uRTCLib rtc(0x68);  // 0x68 is DS3231 I2C address
// uEEPROMLib 24C32 EEPROM
// const uint8_t EEPROM_ADDR = 0x57;
uEEPROMLib eeprom(0x57);

const uint8_t DS3231_ADDR = 0x68;  // RTC component
const uint8_t EEPROM_ADDR = 0x57;  //eEPROM component

// ------------------------ SD -----------------------------------------
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

// Node Global
uint8_t NODE_ID = 1;

// -------------- DS3231 ----------  constants 
const uint8_t rtc_PWR = 15; //DS3231 power control via n-mosfed gnd switch

// const uint8_t rtc_LED = 14; // DS3231 activity LED
// const uint8_t pi_LED  = 22; // PICO activity LED // BME
// const uint8_t eP_LED  =  8; // 24C32 EEPROM activity LED

#define DEGREE "\xC2\xB0" // degree symbol

// -------------------- SD ------- variables
// File objects for logging
FsFile csvFile1;
FsFile csvFile2;


// fileSizes
uint32_t log1Size = 0;
uint32_t log2Size = 0;
uint32_t csv1Size = 0;
uint32_t csv2Size = 0;
uint32_t evt1Size = 0;
uint32_t evt2Size = 0;

// fileNames
// csv files
char fileName1[15] = "";
char fileName2[15] = "";

// Heartbeat globals log files
char logFile1[15] = "";
char logFile2[15] = "";
char logEntry[150] = ""; 

// Event globals event files
#define EVENT_LINE_LEN 128
#define OLED_EVENT_LEN 48

// Event fileNames
char evtFile1[15] = "";
char evtFile2[15] = "";
char eventLine[EVENT_LINE_LEN] = ""; 

char oledEventLine[OLED_EVENT_LEN];

bool eventPending = false;

// SD ready flags
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

// ------------------ sd end -------------------------

#include <time.h>
#include <Wire.h>    //  may be included in the uRTCLib.h

// const int SDA_PIN = 4;
// const int SCL_PIN = 5;

// --------------------- SSD1306 ----------------------------------------------
#include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
#include <Adafruit_SSD1306.h>

// ------------  timer cycle -----------------------
#define TIMECYCLE 300 // seconds // this call back will be used to compare DS3231 time to PICO time and reset PICO time if needed.
#define SDTIMECYCLE 160  //160 Seconds 
// Start the repeating timer call back AFTER setup
// 40,000 µs = 25 FPS smooth fades
// #define RAINBOW 40000    // 40000 microseconds = .04 sec

// SDD1306
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------------SSD1306 end --------------------------------------------

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

// Global  Variables: ------------------- Globals -------------------------



// watchdog

bool pendingWatchdogEvent = false;

// OLED Display
// globals
bool i2c_ok = true;
bool bm_ok  = true;
bool ds_ok  = true;

enum SysEvent {   // this may not be useful or displayed  -- future removal
    EVT_NONE,
    EVT_BOOT,
    EVT_SENSOR_READ,
    EVT_SD_WRITE,
    EVT_NTP_SYNC,
    EVT_ERROR,
};

enum DisplayPage  // may be called enum Page
{
    PAGE_EVENT = 0,
    PAGE_CLOCK,
    PAGE_CYCLE,
    PAGE_HEALTH
};

DisplayPage page = PAGE_EVENT;
#define PAGE_COUNT 4

// uint32_t pageStart = 0;

#define TITLE_Y 0 //  was 2
#define BODY_Y  22

// Timing State
unsigned long lastPageChange = 0;

// Watch_dog  variables  --  things to track and then act upon with Watch_dog
// ----- Failure Classification Controls -----

int8_t timeConfidence = 0;

//  --- stored in EEPROM -- used possible to built  clock time should battery fail or no WiFi
struct TimeSnapshot
{
    uint8_t  year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;

    uint8_t crc;
};

// health snapshot struct ----- NEEDS MORE DEFINITION
struct SystemHealth {
    bool sd_ok;
    bool sensor_ok;
    bool ntp_ok;
    bool cycle_ok;
    uint32_t cycle_count;
    uint32_t last_error;
};

// Globals 

struct LedPulse {
    int pin;
    bool active;
    unsigned long startTime;
};

LedPulse ledRTC    = {14, false, 0};
LedPulse ledEEPROM = {8, false, 0};
LedPulse ledBME    = {22, false, 0};
LedPulse ledPICO   = {LED_BUILTIN, false, 0};

// OLED Event System  ----- NEEDS MORE DEFINITION
#define EVENT_LINES 5
#define EVENT_LEN   28   // tuned to fit nicely on 128px width

char eventLog[EVENT_LINES][EVENT_LEN];  // this may not be useful or displayed  -- future remova
uint8_t eventHead = 0;
bool eventWrapped = false;

int32_t driftSeconds = 0;  // this may not be useful or displayed  -- future remova
int8_t  timeDriftConfidence = 0;   // -10 to +10

// Boot Counter
#define EEPROM_LAYOUT_MAGIC 0x42
// #define EEPROM_MAGIC_ADDR   6

uint16_t bootCount = 0;

// ---- Min Heap Tracker (Memory Leak Detector)
uint32_t minHeap = 0xFFFFFFFF;
uint32_t lastHeap = 0;

// NEEDS MORE DEFINITION  // this may not be useful or displayed  -- future remova
SystemHealth health;
SysEvent currentEvent = EVT_BOOT;

// Timing constants  -- NEEDS MORE DEFINITION
const uint32_t PAGE_SHORT = 20000;   // 20 sec
const uint32_t PAGE_LONG  = 65000;   // 65 sec

// Rotation state  --- NEEDS MORE DEFINITION
uint32_t pageStart = 0;

// Heartbeat Indicator  -- NEEDS CLARITY
uint8_t hbFrame = 0;
uint32_t hbLast = 0;

// Snapshop enum  -- defines the stage of current processing -- needed for debugging long term

uint8_t lastSnapshotDay = 255;

// watch dog stages   // builds enum
#define STAGE_LIST \
    X(STAGE_NONE = 0) \
    X(STAGE_SETUP) \
    X(STAGE_BOOT_I2C) \
    X(STAGE_BOOT_RTC) \
    X(STAGE_BOOT_SD) \
    X(STAGE_BOOT_TIME) \
    X(STAGE_BOOT_EEPROM) \
    X(STAGE_START) \
    X(STAGE_SENSORS) \
    X(STAGE_SD1) \
    X(STAGE_SD2) \
    X(STAGE_PROM) \
    X(STAGE_DONE) \
    X(STAGE_IDLE) \
    X(STAGE_UNKNOWN)

// build enum automatically
enum CycleStage
{
#define X(a) a,
    STAGE_LIST
#undef X
};

volatile CycleStage stage = STAGE_SETUP;

// watchdog globals 
// uint8_t watchdogStage = 0;
const char* bootResetReason = "POWERON";
uint16_t watchdogLine = 0;

CycleStage watchdogStage;

// #define LOG_INTERVAL_MS 10*60*1000  //10 minutes * 60 seconds/minute * 1000 miliseconds/ second
 
volatile bool cycle_ok = true;   // fresh verdict every cycle  NEEDS MORE DEFINITION

// BMx recovery variables
#define BM_REINIT_THRESHOLD 10
#define BM_FAIL_THRESHOLD 3
volatile uint32_t bm_fail_total = 0;
volatile uint16_t bm_fail_streak = 0;

// Pico RTC recovery variables
#define PICORTC_FAIL_THRESHOLD 3
volatile uint32_t PicoRtc_fail_streak = 0;

#define RTC_FAIL_THRESHOLD 3
volatile uint32_t rtc_fail_streak = 0;

// NEEDS MORE DEFINITION
#define LOOP_MAX_MS 2000
#define LOOP_FAIL_RESET_THRESHOLD 3

volatile uint32_t loop_start_ms = 0;               // this may not be being used. 
volatile uint32_t loop_overrun_streak = 0;          // this may not be being used.

// I2C recovery variables
#define I2C_FAIL_RESET_THRESHOLD 3

volatile uint32_t i2c_fail_streak = 0;
volatile uint32_t i2c_fail_total = 0;

#define SD_FAIL_RESET_THRESHOLD 3   // <-- tune later if needed

// SD recovery variables
volatile uint32_t bothSD_fail_streak = 0;
volatile uint32_t bothSD_fail_total = 0;

//  I2C error counts
volatile uint32_t i2c_error_count;
volatile uint32_t i2c_timeout_count;

//  NEEDS MORE DEFINITION
volatile uint32_t fs_fail_count;
volatile uint32_t watchdog_kicks;           // this may not be being used.

// RTC resync counts
volatile uint32_t rtc_resync_events;

// NEEDS MORE DEFINITION AND WORK ON THE HEALTH VARIABLES
// expose them  in 1 hour file list or other ways
// DBG("health: i2c=%lu fs=%lu wd=%lu",
//     i2c_error_count, fs_fail_count, watchdog_kicks);

// loop  CONTROL times
unsigned long now;
unsigned long previousLF = 0; 
unsigned long previous = 0;
unsigned long cycles = 0;
unsigned long dispCycle = 0;
bool firstLoop = true; 
bool firstCycle = true;

// FLAG TO RESYNC PICO TIME IN LOOP
// bool flagDateTimeDisp = true; //NOT USED

// loop cycle variables
long t = 600000;
int8_t  f = 10;

// Globals for SD sdData  -- sdData contains the temperature and date string to be recorded on the SD's
// sd Data
char sdData[100] = "";  
// ASSERT(sizeof(sdData) >= 100);

// a test variable not needed in production
char line[160] = "";  // test line for adding crc 

// snprintf variable
int8_t n = 0;

// crc variable
uint16_t crc;

// sd write status
bool sdWriteStatus = true;

// -----CACHE'S---------------------Globals cache variables for PICO
struct {
  uint16_t y;
  uint8_t mo,d,dow,h,m,s;
} pico_rtc_cache;

// ------------------------ Globals cache variables for DS3231

// DS3231 --- cache variables for rtc.  
struct {
  uint8_t y, mo, d, dow, h, m, s;
  float t;
} rtc_cache;

// ONE OF THE MANY RTC OK VARIABLES -- NEEDS TO BE CONSOLIDATED
bool rtc_ok = false;

// RTC checking variables
// volatile bool rtcCheckFlag = false;
volatile bool rtcMismatchFlag = false; // = 0

uint8_t rtcMismatchCount = 0;

// RTC control variables
bool rtcRecovered = false;

volatile bool checkRTC = false;  //volatile because can change often - compiler indicator

// RTC data
bool rtcOK = false; // another RTC OK variable

// Global rtcData variable that is used to contribute data to sdData to be written to the SD's
char rtcData[100] = "CCYY,MM,DD,HH,MM,SS,FFFFFFFF,\n"; 

// char variable to hold float data in char format.  used with xxxxxxx routine to convert float to char not needed with Pico as Pico supports float variable in printf snprintf.
char rtcTemp[10];   // 8 chars and NULL
// ASSERT(sizeof(rtcData) >= 42);
// ASSERT(sizeof(rtcTemp) >= 10);

// variables used to build filenames that reflect month and year - thus reducing failure exposure
int8_t currentHour = 0;
int8_t currentMth  = 0;
int8_t currentYear = 0;

//  Globals for BME or BMP Sensors -- recovery and data
// ---------------------------------------------- BMP-E data

bool bmAvailable = false;  // set by bmInit()
bool bmInitialized = false;

// variable used to contribute to sdData to be written to the SD's
char bmData[37] = "FFFFFFFF,FFFFFFFF,FFFFFFFF,FFFFFFFF\n";
// ASSERT(sizeof(bmData) >= 37);

// BM(EP)280 temperature and string version
float bmTemp = 0;

// character float variable used to build bmData
char bmCTemp[10] = "";
// ASSERT(sizeof(bmCTemp) >= 10);

// declare the symbol for degree for temperature
const char degree[] = "\u00b0";

// -------------- coldestTemp contains the coldest temperature recorded.  
float coldestTemp = 99.9;  // set to force an coldest file update right at the beginning
float coldestTemp2 = 89.9;  // set to force an coldest file update right at the beginning
char  coldestCTemp[10] = "99.9"; // character float variable for the coldest tempStr
// ASSERT(sizeof(coldestCTemp) >= 10);
// coldest data
char coldestData[100] = "";
// ASSERT(sizeof(coldestData) >= 100);

// ----------------- uEEPROM Data // 24C32 EEPROM variables
uint16_t etempAddr = 40;
uint16_t etempAddr2 = 48;
float eTemp;
float eTemp2;
char e_string[80] = ""; //51 //61 FOR BME // not currently used
char e2_string[80] = ""; //51 //61 FOR BME // not currently used
// ASSERT(sizeof(e_string) >= 80);
int string_length = 0;   // not currently used

// globals to store OLED data
char bufferpi[114];
// ASSERT(sizeof(bufferpi) >= 114);
char bufferds[114];
// ASSERT(sizeof(bufferds) >= 114);
char bufferhe[114];
// ASSERT(sizeof(bufferhe) >= 114);

// PICO timer structure for the SD Update
// struct repeating_timer sdUpdateTimer; // NOT CURRENTLY USED

// RTCLib RTC;

// extern "C" // POTENTIAL PICO 2 USAGE
// {
// #include "pico/utl/datetime.h"
// }

#ifdef WIFI
const char* ssidList[] = {
    "SunorGuest",
    "Cabin435",
    "Sunor11760"
};

const char* passList[] = {
    "Guest11760",
    "GIGABYTE",
    "GIGABYTE11760"
};

#define WIFI_COUNT 3
#define WIFI_RETRIES 3

// my wifi
const char* ssid = "SunorGuest";
const char* password = "Guest11760";

// time server  NTP SERVER
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = -28800;
const int   daylightOffset_sec = 3600;
#endif

struct tm timeinfo;  // made global  from NTP Server


// global  print buffer
char buffer[200];

//  -------------------------------------- functions -------------------------
// tiny recorder     records the current stage of processing for future failure debugging
inline void wdRecord(uint16_t line)
{
    led_pulse_start(ledEEPROM);
    eeprom_write_u8(EEPROM_WD_MAGIC_ADDR, WD_MAGIC);
    led_pulse_start(ledEEPROM);
    eeprom.eeprom_write(EEPROM_STAGE_ADDR, stage);
    led_pulse_start(ledEEPROM);
    eeprom.eeprom_write(EEPROM_LINE_ADDR, line);
}

// OLED functions -------------------------------------------------------------------
// Uptime  - ROUTINE TO BUILD THE LENGTH OF TIME APPLICATION HAS BEEN RUNNING IN HUMAN READABLE FORM
// GOOD CANDITDATE FOR MY LIBRARY
void formatUptime(char* buffer, size_t len)
{
    uint32_t seconds = millis() / 1000;

    uint32_t days    = seconds / 86400;
    seconds         %= 86400;
    uint32_t hours   = seconds / 3600;
    seconds         %= 3600;
    uint32_t minutes = seconds / 60;

    snprintf(buffer, len, "%ud %02uh %02um", days, hours, minutes);
}

// OLED PAGE BUILD ROUTINES
void drawClockPage()
{
    // DBG("drawClockPage");
    display.setTextSize(1);

    // ----- TITLE -----
    display.setCursor(0, TITLE_Y);
    display.print("CLOCK ");

    // RHS indicators (exact positions)
    // display.setCursor(32, TITLE_Y);   // space for 2 symbols only  //NO NO need room for the drawHeartbeat
    // display.setCursor(56, TITLE_Y); 
    bool rtcSyncOKRtncode =  rtcSyncOK(); // NEEDS TO BE CORRECTLY UPDATED TO A MEANINGFUL VALUE
    display.printf("Sync:%c ", rtcSyncOK() ? 'Y' : 'N');
    display.printf("FNam:%c\n", filenameDateOK() ? 'Y' : 'N');
    display.printf("  sdA:%c    sdB:%c", sdA_Ready? 'G' : 'N',sdB_Ready? 'G' : 'N');

    // ----- BODY -----

    // display.fillRect(0, 14, 128, 2, SSD1306_BLACK); // I believe it draws a line

    display.setCursor(0, BODY_Y);
    display.println(bufferpi);

    display.setCursor(0, 38);
    display.println(bufferds);
}

// OLED Event Display
void oledEventMessage(const char *msg)  // part of logEvent
{
    display.clearDisplay();
    display.setCursor(0,0);
    display.print("SYSTEM EVENT");

    display.setCursor(0,16);
    display.print(msg);

    display.display();
}

// ROUTINE TO RETURN THE STAGE OF CURRENT PROCESSING 
const char* stageNames[] =
{
#define X(a) #a,
    STAGE_LIST
#undef X
};

const char* stageToStr(CycleStage s)
{
    if (s >= 0 && s < STAGE_UNKNOWN)
        return stageNames[s];

    return "?";
}

// OLED ROUTINE TO BUILD THE CYCLE PAGE
void drawCyclePage()
{
  char line[30] = "";
    // DBG("drawCyclePage");
    display.setCursor(0, TITLE_Y);
    display.printf("CYCLE ");
    display.printf("%1d \n",f);
    // display.setCursor(60, TITLE_Y);    // changed from 80
        snprintf(line, sizeof(line),
         "I2C=%c BM=%c DS=%c",
         i2c_ok ? 'G' : 'N',
         bm_ok  ? 'G' : 'N',
         ds_ok  ? 'G' : 'N');
    display.printf("%s\n", line);
   
    display.setCursor(0, BODY_Y);
    display.printf("Count: %lu\n", health.cycle_count); // what is it telling me?
    display.printf("Stage: %d %s\n", stage, stageToStr(stage)); // fixed I believe
}

// update Routine  // NOT MEANINGFULL IN CURRENT FORM AS DS3231 AND PICO RTC ARE READ AT DIFFERENT TIMES
void updateRtcHealth()    // not being used
{
    driftSeconds = rtcDriftSeconds();

    if(abs(driftSeconds) <= 2)
    {
        if(timeDriftConfidence < 10)
            timeDriftConfidence++;
    }
    else
    {
        if(timeDriftConfidence > -10)
            timeDriftConfidence--;
    }
}

// Health Page 
void drawHealthPage()
{
    // DBG("drawHealthPage");
    // Yellow
    display.setCursor(0, TITLE_Y);
    display.print("HEALTH");  // was println
    // display.setTextSize(2);
    display.setCursor(80, TITLE_Y);
    display.printf("%.1fC\n", coldestTemp);   //****
    display.printf("EL 1:%lu 2:%lu", evt1Size,evt2Size);
    // Blue
    display.setCursor(0, BODY_Y);
    // uptime
    char uptimeStr[32];
    formatUptime(uptimeStr, sizeof(uptimeStr));
    display.printf("Up: %s\n ", uptimeStr);
    // DBGPF("Up Time: %s\n", uptimeStr);

    // Add bootCount   
    display.printf("Boots:%" PRIu16 "\n ", (uint16_t)bootCount);
    // Memory Leaks
    display.printf("Heap:%lu\n Min: %lu\n", lastHeap,lastHeap);
    // display.printf("Min : %lu\n", lastHeap);
    // DBGPF("Heap: %lu Min : %lu\n", lastHeap,lastHeap);
    // event Logs  Leaks
    display.printf("  sdA:%c    sdB:%c", sdA_Ready? 'G' : 'N',sdB_Ready? 'G' : 'N');
    // display.printf("  BME:%c    DS3231:%c", bmAvailable? 'G' : 'N',rtcOK? 'G' : 'N');
   


    int y = BODY_Y + 24 + 28; // added 10 for bootCount
    // display.setTextSize(1);
    bool any = false;

    // VARIABLES BELOW ARE NOT GOOD INDICATORS OF HEALTH STATUS IN CURRENT FORM **************
    if (!health.sd_ok)
    {
        display.setCursor(0,y);
        display.println("SD FAIL");
        y += 10;
        any = true;
    }

    if (!health.sensor_ok)
    {
        display.setCursor(0,y);
        display.println("SNS FAIL");
        y += 10;
        any = true;
    }

    if (!health.cycle_ok)
    {
        display.setCursor(0,y);
        display.println("CYCLE FAIL");
        y += 10;
        any = true;
    }

    if (!any)
    {
        display.setCursor(0,BODY_Y);
        display.println("ALL OK");
    }
}

// oled Single Display Scheduler
void updateDisplayScheduler()
{
    static uint32_t last = 0;

    rotateDisplayPage();

    if (millis() - last >= 200)
    {
        last = millis();
        renderCurrentPage();
    }
}

// OLED TOP RH CORNER HEARTBEAT INDICATOR
void drawHeartbeat()
{
    // DBG("drawHeartbeat");
    if (millis() - hbLast > 500)
    {
        hbLast = millis();
        hbFrame = (hbFrame + 1) & 3;
    }

    int x = SCREEN_WIDTH - 8;
    int y = 0;

    switch(hbFrame)
    {
        case 0: display.fillCircle(x,y+3,1,SSD1306_WHITE); break;
        case 1: display.fillCircle(x,y+3,2,SSD1306_WHITE); break;
        case 2: display.fillCircle(x,y+3,3,SSD1306_WHITE); break;
        case 3: display.fillCircle(x,y+3,2,SSD1306_WHITE); break;
    }
}

// PAGE DISPLAY TIME CONTROL 
const uint32_t pageDurations[PAGE_COUNT] =
{
    3000,  // PAGE_EVENT
    3000,  // PAGE_CLOCK
    3000,  // PAGE_CYCLE
    3000   // PAGE_HEALTH
};

// Page Rotation Timing
void rotateDisplayPage()
{
    uint32_t now = millis();

    if (now - pageStart >= pageDurations[page])
    {
        page = (DisplayPage)((page + 1) % PAGE_COUNT);
        pageStart = now;
    }
}


// Page Rendering Functions  // NEED WORK
void drawEventPage()
{
    display.setTextSize(1);

    display.setCursor(0, TITLE_Y);
    display.println("EVENT LOG");
    display.printf("EL 1:%lu 2:%lu\n", evt1Size,evt2Size);

    // display.drawLine(0, 15, 127, 15, SSD1306_WHITE);

    display.setCursor(0, BODY_Y);
    // display.setCursor(0, 30);

    uint8_t lines = eventWrapped ? EVENT_LINES : eventHead;
    uint8_t idx = eventHead;

    for(uint8_t i = 0; i < lines; i++)
    {
        idx = (idx == 0) ? lines - 1 : idx - 1;
        display.println(eventLog[idx]);
    }
}

// OLED RENDERING ROUTINE
void renderCurrentPage()  //This is the rendering routine
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    
    switch(page)
    {
        case PAGE_EVENT:
            drawEventPage();
            break;

        case PAGE_CLOCK:
            drawClockPage();
            break;

        case PAGE_CYCLE:
            drawCyclePage();
            break;

        case PAGE_HEALTH:
            drawHealthPage();
            break;
    }

    drawHeartbeat();
    display.display();
}

// Event Recorder (FINAL) //NEEDS CLEARER DEFINITION OF USE
void oledEvent(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);

    vsnprintf(eventLog[eventHead], EVENT_LEN, fmt, args);

    va_end(args);

    eventHead = (eventHead + 1) % EVENT_LINES;

    if(eventHead == 0)
        eventWrapped = true;
}

// RTC Sync Check for OLED // NEEDS CLEARER DEFINITION
static inline uint32_t secondsOfDay(uint8_t h, uint8_t m, uint8_t s)
{
    return (h * 3600UL) + (m * 60UL) + s;
}


// Drift calculator // NEEDS CLEARER DEFINITION
int32_t rtcDriftSeconds()       // need investigation
{
    uint32_t t1 = secondsOfDay(rtc_cache.h,
                               rtc_cache.m,
                               rtc_cache.s);

    uint32_t t2 = secondsOfDay(pico_rtc_cache.h,
                               pico_rtc_cache.m,
                               pico_rtc_cache.s);

    int32_t diff = (int32_t)t1 - (int32_t)t2;

    // Handle midnight rollover
    if(diff > 43200)      diff -= 86400;
    else if(diff < -43200) diff += 86400;

    return diff;
}

// the test.  // NEEDS CLEARER DEFINITION
bool rtcSyncOK()
{
    return abs(driftSeconds) <= 5;
}

// Filename Date Check
bool filenameDateOK()
{
    return (currentYear > 23 && currentMth >=1 && currentMth <=12); //just year not century year
}


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
    delay(300);
}

// Add a helper for OLED later:
bool i2cHealthy()
{
    return (i2c_fail_streak == 0);
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

    delay(10); // long delay to bleed CAPS

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

// rtc Pico cache print routine
void rtcPrintPicoCache()
{
      DBGPF("Pico Cache =>:%04d-%02d-%02d %02d:%02d:%02d\n",
      pico_rtc_cache.y,
      pico_rtc_cache.mo,
      pico_rtc_cache.d,
      // pico_rtc_cache.dow,
      pico_rtc_cache.h,
      pico_rtc_cache.m,
      pico_rtc_cache.s); 
}

// rtc cache print routine
void rtcPrintCache()
{
    DBGPF("RTC %02d-%02d-%02d %02d:%02d:%02d T=%.2f\n",
        rtc_cache.y,
        rtc_cache.mo,
        rtc_cache.d,
        rtc_cache.h,
        rtc_cache.m,
        rtc_cache.s,
        rtc_cache.t);
}

// Validation routine
bool rtcValidateCache()
{
    if (rtc_cache.mo < 1 || rtc_cache.mo > 12) return false;
    if (rtc_cache.d  < 1 || rtc_cache.d  > 31) return false;
    if (rtc_cache.h  > 23) return false;
    if (rtc_cache.y  < 20 || rtc_cache.y > 90) return false;

    return true;
}

// Cache routine
void rtcCacheUpdate()
{
    rtc_cache.y   = rtc.year();
    rtc_cache.mo  = rtc.month();
    rtc_cache.d   = rtc.day();
    rtc_cache.dow = rtc.dayOfWeek();
    rtc_cache.h   = rtc.hour();
    rtc_cache.m   = rtc.minute();
    rtc_cache.s   = rtc.second();
    rtc_cache.t   = rtc.temp() / 100.0;
}

// use to read the DS3231 in place of rtc.refresh()  at appropriate places  has recovery built in
bool rtcRefresh() 
{
    static uint8_t rtcFailCount = 0;
    bool rtcRecoveryFlag = true;
    // char status[10] = "";
    char buffer[200] = "";

    led_pulse_start(ledRTC);
    if (rtc.refresh()) 
    {  // Good
      rtcCacheUpdate();  // cache update
      // check time data
      if (rtcValidateCache())  // Data Good
      {
        rtcFailCount = 0;
        rtc_ok = true;
        ds_ok = true;
        return true; // good get OUT
      }
      else // rtc read good DATA IS Bad  let try PICO RTC
      {
        // rtcFailCount++;
        rtc_ok = false;
        DBGPF("DS3231 rtc failed RTC sanity check:%d\n",rtc_ok);
        // now check PICO RTC
        bool PicoOK = rtcPICOOK();  // test Pico RTC for sanity 
        
        if (PicoOK)
        {  // PICO good resync and recover
          DBGPF("DS need re-sync-ed with PICO RTC -- PicoOK return:%d\n",PicoOK);
          syncPicoToDS3231();  // sync PICO to DS3231
          rtc_resync_events++;
          ds_ok = true;
          rtc_ok = true;  // DS3231 has been refreshed
          // return rtc.refresh();
          return true;  // good get OUT
        }
        //   No valid time fail
        // rtcFailCount++;
      } // default through  to I2C bus
    } 
    // Try to repair I2C bus and DS3231 via power recycle -- Try repair
  // rtcFailCount++;   // no already counted 
  DBG("RTC fail count: "); DBGLN(rtcFailCount); // print the rtcFailCount
  if (++rtcFailCount < 3) 
  {   // fail tolarance 3 times before power cycling  I2C bus and DS3231
    return false;
    // tolerate transient failures
  }

  // problems recycling DS3231 power and the I2C bus
  DBGLN("Power-cycling RTC..."); // NEED TO DISPLAY ON OLED
  // Disable I2C pins to avoid phantom power
  pinMode(SDA, INPUT); // NEEDed here
  pinMode(SCL, INPUT); // NEEDed here

  rtcRecoveryFlag = rtcSafeRefresh(); // powercycle, i2cRecovery, i2cInit
  i2c_ok = rtcRecoveryFlag;
  if (!rtcRecoveryFlag)
  {  // did't recover the I2C bus  // cycle again // big problems MAY HAVE TO FORCE A WATCHDOG REBOOT (TO BE CODED)
      i2c_fail_streak++;
      i2c_fail_total++;
  }
  else  // We recoverED the I2C bus  try the read again below
  {
      i2c_fail_streak = 0; // bus is good
  }
  
  snprintf(buffer, sizeof(buffer),"DS3231 recovery attempt %s\n I2c Bus ERROR-Check for MSG above or power status",rtcRecoveryFlag ? "worked" : "failed");
  DBG(buffer);

  // rtcFailCount = 0;  // NO NO do not want to clear count yet  need success to clear
  // Try again to read DS3231 rtc
  led_pulse_start(ledRTC);
  bool finalOk = rtc.refresh(); // try DS3231 again

  if (!finalOk) // NEED TO DISPLAY ON OLED
  {  // still no good   will return with a false  ==> a fail
      i2c_fail_streak++;
      i2c_fail_total++;
  }
  else
  { // read good 
      i2c_fail_streak = 0;
      rtcFailCount = 0;
      ds_ok = true;
      rtc_ok = true;
  }

  return finalOk;
}

// the actual POWER CYCLE
bool rtcSafeRefresh() {
    if (i2cDevicePresent(0x68) == 0) {
        rtc.refresh();
        led_pulse_start(ledRTC);
        return true;
    }

    rtcPowerCycle();
    i2cRecover();
    i2cInit();

    if (i2cDevicePresent(0x68) == 0) {
        rtc.refresh();
        led_pulse_start(ledRTC);
    
        return true;
    }

    return false;
}

uint8_t snapshotCRC(const uint8_t *buf)
{
    uint8_t c = 0;
    // for (int i=0;i<5;i++)
    for (int i=0;i<4;i++)     // only the first 4 as buf[5] is the CRC record and not included in the calc
        c ^= buf[i];

    // DBGPF("CRC calc:%02X\n",c);
    return c;
}

// // Save Snapshot routine at NOON
// #define SNAP_ADDR 16  // see line 86

// void writeTimeSnapshot(uint8_t confidence, int16_t bootCount)
void writeTimeSnapshot()
{
    uint8_t buf[5];

    buf[0] = rtc.year();
    buf[1] = rtc.month();
    buf[2] = rtc.day();
    buf[3] = rtc.hour();

    // buf[4] = (uint8_t)(bootCount & 0xFF);
    // buf[5] = (uint8_t)((bootCount >> 8) & 0xFF);

    // buf[6] = confidence;

    buf[4] = snapshotCRC(buf);

    DBG("Once in 24 hours Write TimeSnapshot EEPROM Struct:HEX->:");
    for(uint8_t i=0;i<5;i++)
    {
        led_pulse_start(ledEEPROM);
        eeprom.eeprom_write(SNAP_ADDR+i, buf[i]);
        delay(5);
        DBGPF("%02X ",buf[i],buf[i]);
    }
    DBG("\t");
    DBG("Struct:DEC->:");
    for(uint8_t i=0;i<5;i++){DBGPF("%02u ",buf[i],buf[i]); }
    DBG("\n");
}

bool readTimeSnapshot(TimeSnapshot &snap)
{
    uint8_t buf[5];

    DBG("In setup Read TimeSnapshot EEPROM Struct:HEX DEC->:");
    for(uint8_t i=0;i<5;i++)
    {
        led_pulse_start(ledEEPROM);
        buf[i] = eeprom.eeprom_read(SNAP_ADDR+i);
        DBGPF("%02X %02u ",buf[i],buf[i]);
    }
    DBG("\n");
    // for (uint8_t j=0;j<5;J++)  {printf("%02X ",buf[j]);}
    // CRC
    if(snapshotCRC(buf) != buf[4])
    {
        DBG("Snapshot CRC FAIL: CRC=>:");
        DBGPF("%02X \n",buf[4]);
        return false;
    }

    // Populate struct explicitly
    snap.year  = buf[0];
    snap.month = buf[1];
    snap.day   = buf[2];
    snap.hour  = buf[3];

    // snap.bootCountLSB = buf[4];
    // snap.bootCountMSB = buf[5];

    // snap.confidence = buf[6];
    snap.crc        = buf[4];

    // Sanity checks
    if(snap.month < 1 || snap.month > 12) return false;
    if(snap.day   < 1 || snap.day   > 31) return false;
    if(snap.hour  > 23) return false;
    // if(snap.confidence > 2) return false;

    return true;
}

// Recovery Apply (+12 hr model) -- this is artificial time but works to recover from a failed battery
bool recoverRTCfromSnapshot()
{
    TimeSnapshot snap;

    if(!readTimeSnapshot(snap))
        return false;

    uint8_t y = snap.year;
    uint8_t m = snap.month;
    uint8_t d = snap.day;
    uint8_t h = snap.hour;

    // Advance estimate (+12h model)
    h += 12;
    if(h >= 24)   // adjust the day if generated time is > 24 hours
    {
        h -= 24;
        d += 1;
    }

    rtc.set(0, 0, h, 0, d, m, y);   //sec, min, hour, dow, day, month, year

    DBG("RTC rebuilt from snapshot");
    return true;
}

// Minimal RTC Validation (Setup-Safe)
bool rtcReadValid()
{
    uint8_t y = 0;
    uint8_t m = 0;
    uint8_t d =  0;
  
    led_pulse_start(ledRTC);
    bool rtcrefreshRtnCode = rtc.refresh();

    // set temp variables
    y  = rtc.year();
    m  = rtc.month();
    d   = rtc.day();

    if (!rtcrefreshRtnCode) 
    {
        Serial.printf("rtcReadValid rtc.refresh rtn code: %s\n",rtcrefreshRtnCode?"true":"false");
        return false;
    }

    // Test is DS3231 RTC time is close to real time  /// This test may HAVE TO BE RESET WITH NEW DS3231 AND DIFFERENT TIME PERIODS
    if (y != 2026) return false;
    if (m < 1 || m > 4) return false; // Feb–May window

    rtcCacheUpdate();

    bool rtcValidateCacheRtnCode = rtcValidateCache();
    if (!rtcValidateCacheRtnCode)
    {
        Serial.printf("rtcValidateCache rtn code: %s\n",rtcValidateCacheRtnCode?"true":"false");
        return false;
    }

    rtc_ok = true;
    return true;

}

// ----------------------------------------------the actual DS3231 read for recording on the SD
bool RTC_Read() 
{

  // RTC Refresh
  // RTC stores the time in integers
  // RTC stores temperature in float

  // temp variables to store date for comparison, snprintf, etc
  // char buffer[100];
  // char errorBuffer[100] = "";
  char rtcbuffer[200] = "";
  char s[15];  // 14 chars and NULL
  char t[10];   // 8 chars and NULL
  int j;       // length of the sprintf

  ASSERT(sizeof(buffer) >= 100);
  ASSERT(sizeof(rtcbuffer) >= 200);
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
  if (!rtcRefresh())   // rtcRefresh() has a lot of recovery code in it should not fail.
  {
    DBGLN("RTC refresh FAILED");
    rtcOK = false;
    // needs hardening here ----------------- working on it.--------------------------------------------------------------
    return false;
  }
  else rtcOK = true;  // need to be looked at

  // store rtc temp in a local variable
  temp = rtc_cache.t;
  ASSERT(!isnan(temp));

  // convert the float temp to string
  // dtostrf(float, min_width, num_of_digits after decimal, string where to store the converted string)
  dtostrf( temp, 8, 2, t);  // error was dividing by 100  fixed

  ASSERT(sizeof(rtcTemp) >= sizeof(t));
  ASSERT(strlen(t) < sizeof(rtcTemp));
  // strncpy(rtcTemp,t,sizeof(rtcTemp));
  snprintf(rtcTemp, sizeof(rtcTemp), "%s", t);
  // DBGPF("rtcTemp:%s line 1907 , t:%s  temp:%f",rtcTemp,t,temp);

  // build a print line for monitor with snprintf rather than many serial.print lines
  j = DBGSNPF(rtcbuffer,sizeof(rtcbuffer),"DS RTC DT:%02d/%02d/%02d  %02d:%02d:%02d %s%sC\t\n",
  rtc_cache.y,rtc_cache.mo,rtc_cache.d,rtc_cache.h,rtc_cache.m,rtc_cache.s,t,"\xC2\xB0");  // use cache variables not module calls
  SP(rtcbuffer)
  return true;
}
#define EEPROM_MAGIC 0xA5
// eTemp Struct & Union
#pragma pack(push,1)
typedef struct
{
    float    temp;
    uint8_t  crc;
    uint8_t  magic;
} ETempStruct;
#pragma pack(pop)

typedef union
{
    ETempStruct data;
    uint8_t raw[sizeof(ETempStruct)];
} ETempUnion;

// CRC eTemp Calculation Routine
uint8_t calcCRC(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
        crc ^= data[i];

    return crc;
}

// eePROM Write Routine
// change ETEMP to pAddr in the routine

// void writeETemp(float temp)
void writeETemp(float temp, uint16_t pAddr )
{
    ETempUnion block;

    block.data.temp = temp;
    block.data.magic = EEPROM_MAGIC;

    block.data.crc = calcCRC(
        (uint8_t*)&block.data.temp,
        sizeof(float)
    );

    // Verification of the data
    // ETempUnion verify;
    DBGPF("Pre Write VERIFY temp=%.2f crc=%02X magic=%02X\n",
      block.data.temp,
      block.data.crc,
      block.data.magic);

    led_pulse_start(ledEEPROM);
    // bool status = eeprom.eeprom_write(pAddr, block.raw);   // changed from &block.data
    // bool status = eeprom.eeprom_write(pAddr,(uint8_t*)&block.data,sizeof(block.data));
    bool status = eeprom.eeprom_write(pAddr,(uint8_t*)block.raw,sizeof(block.raw));

    delay(10);
    DBGPF("\neTemp Dump of after write pAddr ADDR:%d\n",pAddr);
    
    for (int i=0;i<8;i++)
    {
        led_pulse_start(ledEEPROM);
        uint8_t b = eeprom.eeprom_read(pAddr+i);
    DBGPF("%02X ", b);
    }
    DBGLN("\n");

    DBGPF("Dump 2: Write block: ");
    for(int i=0;i<sizeof(block.raw);i++)
        DBGPF("%02X ", block.raw[i]);
    DBGLN("");

    if (!status)
        DBGLN(F("EEPROM write failed"));

    DBGPF("EEPROM write temp=%.2f crc=%02X\n\n",
          block.data.temp,
          block.data.crc);

    // Verification
    ETempUnion verify;

    eeprom.eeprom_read(pAddr, (uint8_t*)verify.raw, sizeof(verify.raw));

    DBGPF("Post VERIFY read temp=%.2f crc=%02X magic=%02X\n",
          verify.data.temp,
          verify.data.crc,
          verify.data.magic);
}


// New eTemp READ Routine with validation
// change ETEMP to pAddr in the routine

// bool readETemp(float *temp)
bool readETemp(float *temp,uint16_t pAddr)
{
    ETempUnion block;
    // DBGPF("\neTemp Dump of pAddr after read ADDR:%d\n",pAddr);
    // led_pulse_start(ledEEPROM);
    // for (int i=0;i<8;i++)
    // {
    //     uint8_t b = eeprom.eeprom_read(pAddr+i);
    //     DBGPF("%02X ", b);
    // }
    // DBGLN("\n");

    DBGPF("Sizeof block = %u\n", sizeof(block.data)); // changed from block.data
    // eeprom.eeprom_read(pAddr, (uint8_t*)&block.data, sizeof(block.data));
    led_pulse_start(ledEEPROM);
    eeprom.eeprom_read(pAddr, (uint8_t*)block.raw, sizeof(block.raw));
    // delay(15);

    uint8_t crcCalc = calcCRC(
        (uint8_t*)&block.data.temp,
        sizeof(float)
    );

    DBGPF("Post Read VERIFY temp=%.2f crc=%02X magic=%02X\n",
      block.data.temp,
      block.data.crc,
      block.data.magic);

    if (block.data.magic != EEPROM_MAGIC)
    {
        DBGLN("eTemp EEPROM magic invalid");
        return false;
    }

    if (crcCalc != block.data.crc)
    {
        DBGLN("EEPROM CRC mismatch");
        return false;
    }

    if (isnan(block.data.temp))
    {
        DBGLN("EEPROM temp NaN");
        return false;
    }

    if (block.data.temp < -40.0 || block.data.temp > 85.0)
    {
        DBGLN("EEPROM temp out of range");
        return false;
    }

    *temp = block.data.temp;

    DBGPF("EEPROM read temp=%.2f crc=%02X OK\n",
          block.data.temp,
          block.data.crc);

    DBGPF(
      "temp=%.2f crc=%02X magic=%02X\n",
      block.data.temp,
      block.data.crc,
      block.data.magic
    );

    return true;
}

// inspectFloat tool
void inspectFloat(float value)
{
    uint8_t *p = (uint8_t*)&value;

    DBGPF("Value: %.3f  HEX:", value);

    for(int i=0;i<4;i++)
        DBGPF(" %02X", p[i]);

    DBGLN();
}

void promUpdate(float currentTemp) 
{
  // char buffer[200] = "";
  uint16_t promAddr = 40;
  float curTemp = 0.0;
  float curTemp2 = 0.0;
  char currentCTemp[10] = "";
  curTemp = curTemp2 = currentTemp;
  dtostrf(currentTemp, 8, 2, currentCTemp);
 
  // dtostrf(coldestTemp, 8, 2, coldestCTemp);
  DBGPF("currentTemp1=%.2f   coldestTemp=%.2f\n",curTemp,coldestTemp); 
  if (curTemp < coldestTemp) 
  {
    coldestTemp = curTemp;
    promAddr = etempAddr;
    writeETemp(coldestTemp,promAddr);
    DBGPF("New coldest temp stored %.2f\n", coldestTemp);
   
    // dtostrf(currentTemp, 8, 2, currentCTemp); // character version of float
  }
   
  DBGPF("currentTemp2=%.2f   coldestTemp=%.2f\n",curTemp2,coldestTemp2); 
  if (curTemp2 < coldestTemp2) 
  {
    coldestTemp2 = curTemp2;
    promAddr = etempAddr2;
    writeETemp(coldestTemp2,promAddr);
    DBGPF("New coldest2 temp stored %.2f\n", coldestTemp2);
   
    // dtostrf(currentTemp, 8, 2, currentCTemp); // character version of float
  }
}

// Read the BME or BMP
// ----------------------------- BMx routines ------------------------------

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

#define BME_ADDR 0x76

bool bmInit()
{
    if (!i2cDevicePresent(BME_ADDR)) {
        DBG("BME not responding on I2C\n");
        bmAvailable = false;
        return true;
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
        return true;
    }

    // IMPORTANT: force same Wire instance
    if (!bme.begin(BME_ADDR, &URTCLIB_WIRE)) {
        DBG("BME begin() failed\n");
        bmAvailable = false;
        oledEvent("Sensor Timeout");
        return true;
    }

    bmInitialized = true;
    bmAvailable   = true;
    DBG("BME initialized OK\n");
    oledEvent("BMP280 OK");
  return true;
}

bool bmTask()
{
    // Retry init if Pico was reset but sensor wasn't
    if (!bmInitialized || !bmAvailable) {
        static uint32_t last_try = 0;
        if (millis() - last_try > 5000) {   // retry every 5 seconds
            last_try = millis();
            bmAvailable = bmInit();
        }
        return true;
    }

    BM_Read();
    return true;
}

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

  float temperature = 0;
  float pressure = 0;
  float altimeter = 0;
  float humidity = 0;

  led_pulse_start(ledBME);  //?
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
    // strcpy(bmData, bm_buffer);  
    snprintf(bmData,sizeof(bmData),"%s",bm_buffer); // garantees not buffer over runs

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

// -------------------------------- SD Routines ------------------------------------

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
void sdOff(int8_t led) { digitalWrite(led,LOW);}

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
    watchdog_update();

    // 3. Reset SPI peripheral
    bus.spi->end();
    delay(50);

    // 4. Power on SD
    sdPowerOn(bus);
    delay(300);
    watchdog_update();

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

// update global filesize
bool checkFile(char *name, uint32_t filesize) 
{
  // strncmp compares up to 'n' characters
  // char patcsv1[6] = "";
  //DBGSNPF(patcsv1,sizeof(patcsv1),"c%cv1",NODE_ID);  //* Decided against the node in the file name */
  // build a pattern with snprintf(patcsv1,sizeof(patcsv1),"c%cv1",NODE_ID));
  // if (strncmp(name, patcsv1, 4) == 0) 
  if (strncmp(name, "csv1", 4) == 0) 
  {
    // DBGPF("Matches: File starts with csv1 - Size:%lu\n",filesize);
    csv1Size = filesize;
    return true;
  }   // build a pattern with snprintf(patcsv2,sizeof(patcsv2),"c%cv2",NODE_ID));
    else if (strncmp(name, "csv2", 4) == 0) 
  {
    // DBGPF("Matches: File starts with csv2 - Size:%lu\n",filesize);
    csv2Size = filesize;
    return true;
  }  // build a pattern with snprintf(patlog1,sizeof(patlog1),"l%cg1",NODE_ID);
    else if (strncmp(name, "log1", 4) == 0) 
  {
    // DBGPF("Matches: File starts with log1 - Size:%lu\n",filesize);
    log1Size = filesize;
    return true;
  } // build a pattern with snprintf(patlog2,sizeof(patlpg2),"l%cg2",NODE_ID);
    else if (strncmp(name, "log2", 4) == 0) 
  {
    // DBGPF("Matches: File starts with log2 - Size:%lu\n",filesize);
   log2Size = filesize;
   return true;
  } 
   else if (strncmp(name, "evt1", 4) == 0) 
  {
    // DBGPF("Matches: File starts with log2 - Size:%lu\n",filesize);
   evt1Size = filesize;
   return true;
  } 
   else if (strncmp(name, "evt2", 4) == 0) 
  {
    // DBGPF("Matches: File starts with log2 - Size:%lu\n",filesize);
   evt2Size = filesize;
   return true;
  } 
    else
  {
    DBGLN("The filename Does not match.");
    return false;
  }
}

// Routine to write sdData to two SD Cards
bool writeSDFile(char *Data, char *File1, char *File2)
{
  char buffer[299] = "";

  // fileSize work variable
  uint32_t fileSize = 0;

  int written1 = 0, written2 = 0;
  int sdDataLen = 0; // excluding \0 character
  bool cardOpenError1 = false; 
  bool cardOpenError2 = false; 
  bool cardWriteError1 = false;
  bool cardWriteError2 = false;
  bool cardSyncError1 = false; 
  bool cardSyncError2 = false;
  bool sdARecovError  = false; 
  bool sdBRecovError = false;

  // --- WRITE TO CARD 1 ---
  if (!sdARecovError)  // not in a recovery state or just coming out of one  = 0
  {
    // watch_dog ENUM 
    // stage = STAGE_SD1;
    // wdRecordStage(stage);
    STAGE(STAGE_SD1,"SD1 write");
    // oledEvent("SD1 Write",stage);

    sdA.sd->chvol(); // FORCE focus to SD1

    // Watch_dog   update strengthening
    watchdog_update();        // <- important

    if (csvFile1.open(sdA.sd, File1, O_RDWR | O_CREAT | O_AT_END)) 
    {   // open good write data
        sdDataLen = strlen(Data); // not including \0
        written1 = (csvFile1.write(Data));  // including CR and NL 
        if((written1) > 0)   //good write as not zero return
        { // write good now sync
          // DBGPF("SD1 sdDataLen:%d  written:%d\n",sdDataLen,written1);  // debugging
          // delay(10);
          if (csvFile1.sync())
          { // sync good now close the file clean up SfFat

            // Watch_dog   update strengthening
            watchdog_update();        // <- important

            // determine the current file size for logging
            fileSize = csvFile1.fileSize();
            if (!checkFile(File1,fileSize)) DBGPF("Filename not found %s",File1);
            // close the file
            csvFile1.close();
            // Serial.println("Wrote to csvFile1 on SD 1");
            // flash(LED_PIN,2,60);
            sdA_Ready = true;
            oledEvent("SD1 Write OK");
          }
          else // Sync failure
          {
            cardSyncError1 = true; // > 0
            sd1Healthy = false;    // = 0
            DBGSNPF(buffer,sizeof(buffer),"**Error: Card:%d sync failed - card possible removal mid-write\n",1);
            DBG(buffer);
            logEvent("SD_SYNC_FAIL card=%d", 1);
          } //sync failure else close
        } else // write failure 
        { 
          sdA_Ready = false; 
          cardWriteError1 = true;
          sd1Healthy = false;
          DBGSNPF(buffer,sizeof(buffer),"**Error writing %s\n ",File1);
          DBG(buffer);
          logEvent("SD_WRITE_FAIL card=%d", 1);
        } // write failure else close
    } else // Open failure
    {
      sdA_Ready = false; // Mark error
      cardOpenError1 = true;
      sd1Healthy = false;
      DBGSNPF(buffer,sizeof(buffer),"**Error opening %s\n ",File1);
      DBG(buffer);
      logEvent("SD_OPEN_FAIL card=%d", 1);
    }
  } // end of !sdARecovError otherwise end of we are good

  // Open, Write, or Sync error occurred need to recover
  if (!sd1Healthy) //  not true = 0 = false
  {
    // csvfile1.close();          // safe even if already bad
    // sdFatA.end();  
    if (!sdHardRecover(sdA))  // = 0
    {
      oledEvent("SD1 ERROR");
      DBGLN("SD1 offline — using SD2 only");
      sdA_Ready = false; // = 0
      sdARecovError = true; // > 0
    } else // HardRecover good  now clear up SfFat by closing open files
    {
      if (cardSyncError1){csvFile1.close(); delay(10);}
      if (cardWriteError1){csvFile1.close(); delay(10);}
      DBGLN("** GOOD: SD1 Recovered!");
      oledEvent("SD1 Recovered");
      sdA_Ready = true; // > 0
      sdARecovError = false; // > 0
      sd1Healthy = true;
      logEvent("**SD_RECOVER card=%d", 1);
    }

  } 

// SD2 New Start
  // --- WRITE TO CARD 1 ---
  if (!sdBRecovError)  // not in a recovery state or just coming out of one  = 0
  {
    // watch_dog ENUM
    // stage = STAGE_SD2;
    // wdRecordStage(stage);
    STAGE(STAGE_SD2,"SD2 Write");
    // oledEvent("SD2 Write",stage);

    sdB.sd->chvol(); // FORCE focus to SD2

    // Watch_dog   update strengthening
    watchdog_update();        // <- important

    if (csvFile2.open(sdB.sd, File2, O_RDWR | O_CREAT | O_AT_END)) 
    {   // open good write data
        sdDataLen = strlen(Data); // not including \0
        written2 = (csvFile2.write(Data));  //including CR and NL 
        if((written2) > 0)   //good write as not zero return
        { // write good now sync
          // DBGPF("SD2 sdDataLen:%d  written:%d\n",sdDataLen,written2);  // debugging
          delay(10);
          if (csvFile2.sync())
          { 
            // sync good now close the file clean up SfFat
            // Watch_dog   update strengthening
            watchdog_update();        // <- important

            // determine the current file size for logging
            fileSize = csvFile2.fileSize();
            if (!checkFile(File2,fileSize)) DBGPF("Filename not found %s",File2);
            // checkFile(File2,fileSize);

            // Now close the file
            csvFile2.close();

            // Watch_dog   update strengthening
            watchdog_update();        // <- important

            // Serial.println("Wrote to csvFile1 on SD 1");
            // flash(LED_PIN,2,60);
            sdB_Ready = true;
            oledEvent("SD2 Write OK");
          }
          else // Sync failure
          {
            cardSyncError2 = true; // > 0
            sd2Healthy = false;    // = 0
            DBGSNPF(buffer,sizeof(buffer),"**Error Card:%d sync failed - card possible removal mid-write\n",2);
            DBG(buffer);
            logEvent("SD_SYNC_FAIL card=%d", 2);
          } //sync failure else close
        } else // write failure 
        { 
          sdB_Ready = false; 
          cardWriteError2 = true;
          sd2Healthy = false;
          DBGSNPF(buffer,sizeof(buffer),"**Error writing %s\n ",File2);
          DBG(buffer);
          logEvent("SD_WRITE_FAIL card=%d", 2);
        } // write failure else close
    } else // Open failure
    {
      sdB_Ready = false; // Mark error
      cardOpenError2 = true;
      sd2Healthy = false;
      DBGSNPF(buffer,sizeof(buffer),"**Error opening %s\n ",File2);
      DBG(buffer);
      logEvent("SD_OPEN_FAIL card=%d", 2);
    }
  } // end of !sdARecovError otherwise end of we are good

  // Open, Write, or Sync error occurred need to recover
  if (!sd2Healthy) //  not true = 0 = false
  {
    // csvfile1.close();          // safe even if already bad
    // sdFatA.end();  
    if (!sdHardRecover(sdB))  // = 0
    {
      oledEvent("SD2 ERROR");
      DBGLN("SD2 offline — using SD1 only");
      sdB_Ready = false; // = 0
      sdBRecovError = true; // > 0
    } else // HardRecover good  now clear up SfFat by closing open files
    {
      if (cardSyncError2){csvFile2.close(); delay(10);}
      if (cardWriteError2){csvFile2.close(); delay(10);}
      DBGLN("**GOOD: SD2 Recovered!");
      oledEvent("SD2 Recovered");
      sdB_Ready = true; // > 0
      sdBRecovError = false; // > 0
      sd2Healthy = true;
      logEvent("**SD_RECOVER card=%d", 2);
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

  cycle_ok = true;

}

// ----Call Back ---- from a timer interupt ---- reads PICO RTC - displays PICO time - compares PICO & DS3231 time
bool timer_callback(struct repeating_timer *t) 
{
  // check the call back interval  Checking for mismatch with DS3231 RTC time (if so flag it)
  // datetime_t ptt;

  char buffer[199];
  int16_t picoYear  = 2000;
  int16_t dsYear = 2000;
  int16_t year2026 = 2026;
  
  readRTC();  //  read PICO RTC  -- then updates cache variables - updates currentDates

  bool rtcPicoOk = rtcPICOOK();   //check out timing order  placed here after the PICO rtc read
  if(!rtcPicoOk) DBG_BOOL("rtcPicoOk rtn code:",rtcPicoOk);
  // Build
  DBGSNPF(buffer, sizeof(buffer),
  "5 min CallBack invoked - (DS & PI) PI =>:%02d:%02d:%02d\t",
  pico_rtc_cache.h, pico_rtc_cache.m, pico_rtc_cache.s);
  DBG(buffer);
  snprintf(bufferpi, sizeof(bufferpi),
  "P:%04d-%02d-%02d %02d:%02d:%02d\n",
  pico_rtc_cache.y,
  pico_rtc_cache.mo,
  pico_rtc_cache.d,
  pico_rtc_cache.h,
  pico_rtc_cache.m,
  pico_rtc_cache.s);
  // pico_rtc_cache.dow);
 
  // print the time in the two clocks
  printTime();  //  which call DS3231 rtc Read - update bufferds and flags

  // TEST PICO RTC AGAINS DS3231 RTC FOR ERRORS OR CORRUPTION
  picoYear = pico_rtc_cache.y;    //PICO current Time
  // Serial.printf("currentYear:%d\t",currentYear);
  dsYear = rtc_cache.y + 2000;    //DS current Time  // rtcRefresh() IS NOT NEEDED TO UPDATE THE rtc_cache AS WE ARE ONLY USING rtc_cache.y THE YEAR WHICH 

  int16_t diff = picoYear - dsYear;
  if (diff < 0) diff = -diff;

  bool mismatch = (diff > 1) ||  (picoYear < 2026);
  if (!mismatch)
  {
    rtcMismatchCount = 0;
    // rtcCheckFlag = false;  //FIX IN LOOP  // does nothing
    DBG("currentDates Updated - times compared - times Good\n");
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
  return true;
}

// interupt timer structure storage allocation
struct repeating_timer timer;

// RTC sanity check
bool rtcPICOBad(datetime_t &b)
{
    bool rtcPICOBadrtN = true;  // will return true unless sanity check changes 

    int year = b.year;  
    int month = b.month;
    int day = b.day;

    // Basic sanity check
    if (year < 2023 || year > 2026) rtcPICOBadrtN = false;
    if (month < 1 || month > 12)    rtcPICOBadrtN = false;
    if (day < 1 || day > 31)        rtcPICOBadrtN = false;
    if (!rtcPICOBadrtN) 
    {
      DBGPF("Pico date check:%04d-%02d-%02d %02d:%02d:%02d\n", b.year, b.month, b.day, b.hour, b.min, b.sec);
    }

    return rtcPICOBadrtN;
}

// routine to  check and set currentMth, Year, Hour
bool currentDates()  // needed for SD file name build
{
  // check if new month if so list files on SD ---- use the pico_cache to update 
  // DBGPF("Before test - CurrentMth:%d currentYear:%d pico_rtc_cache.mo:%d\n",currentMth,currentYear,pico_rtc_cache.mo);
  if ((currentMth != pico_rtc_cache.mo) && (currentMth != 0))
  {
    DBG("****** New Month - List SD files *******");
    listFiles(*sdA.sd, "SD Card 1 (SPI0)");
    listFiles(*sdB.sd, "SD Card 2 (SPI1)");
  }
      
  // save current Hour, Month and Year for comparisons and file name construction
  currentHour = pico_rtc_cache.h;           // for the file flush
  currentMth  = pico_rtc_cache.mo;          // for monthly files
  currentYear = (int8_t)(pico_rtc_cache.y % 100);  // for monthyear file only year not century

  // Serial.printf("After update - CurrentMth:%d currentYear:%d pico_rtc_cache.mo:%d (pico_rtc_cache.y % 100):%d \n",currentMth,currentYear, pico_rtc_cache.mo,(pico_rtc_cache.y % 100) ); 
  return true; 
}

//  function to read PICO onboard RTC - check RTC date - loads pico_rtc_cache - builds bufferpi
bool readRTC() 
{ 
  // PICO onboard clock
    datetime_t t;
    char buffer[199] = "";
    
    // julian Dates
    int jul = 0;
    long compDate = 0;
    // DBG("\n\n*****recdRTC ****** \n");  
  
    led_pulse_start(ledPICO);
    bool rtcgetRtnCode = rtc_get_datetime(&t);
    delay(100);
    if (!rtcgetRtnCode) DBG_BOOL("readRTC() => PICO rtc_get_datetime BAD RtnCode:",rtcgetRtnCode);
    // rtc_get_datetime(&t);

    // test date  
    bool rtncode = rtcPICOBad(t);
    if(!rtncode)
    {
      DBG_BOOL("\nreadRTC => BAD RETURN CODE:",rtncode);
      snprintf(buffer, sizeof(buffer),  
      "readRTC =>:%04d-%02d-%02d %02d:%02d:%02d\n (DOW=%d)\n\n",
      t.year, t.month, t.day, t.hour, t.min, t.sec, t.dotw);
      DBG(buffer);
      return false;
    }

    // refresh current  PICO RTC data to pico_rtc_cache  Global struct   // maybe put following routine into a function
    pico_rtc_cache.y = t.year;
    pico_rtc_cache.mo = t.month;
    pico_rtc_cache.d = t.day;
    pico_rtc_cache.dow = t.dotw;
    pico_rtc_cache.h = t.hour;
    pico_rtc_cache.m = t.min;
    pico_rtc_cache.s = t.sec;

    // update currentDates for filenames
    bool currentDatesRtnCode = currentDates();
    if (!currentDatesRtnCode) 
    {
      DBG_BOOL("\ncurrentDatesRtnCode => BAD RETURN CODE:",currentDatesRtnCode);
      return false;
    }

    // now print the copy
    
    // rtcPrintPicoCache();

    snprintf(bufferpi, sizeof(bufferpi),  // Build bufferpi   for the OLED display
             "P:%04d-%02d-%02d %02d:%02d:%02d\n",
             t.year, t.month, t.day, t.hour, t.min, t.sec);

    #ifdef TURNONPRT
      DBG("PICO RTC Time:\n");
      DBGLN(bufferpi);
    #endif

  return true;
}


// print the current DS3231 time
void printTime() //Reads DS3231 RTC module  clock - converts floats to char - loads bufferds - flags OLED display
{
    // added the BMx char temperature to the second line with var bmCTemp
    snprintf(bufferds, sizeof(bufferds), // for the OLED
            "DS: %02d-%02d-%02d %02d:%02d:%02d\nD:%7.2fC B:%7.2fC \n",
            rtc_cache.y, rtc_cache.mo, rtc_cache.d,
            rtc_cache.h, rtc_cache.m, rtc_cache.s,rtc_cache.t,bmTemp);
    // DBGPF("bmCTemp:%s\n",bmCTemp);
    // DBG(bufferds);
  }

//  geLocalTime routine from the NTP time server  --- for NTP server request via WiFi
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

// RTC sanity check
bool rtcPICOOK()
{
    datetime_t t;

    // Get time from Pico internal RTC
    bool rtcgetRtnCode = rtc_get_datetime(&t);
    delay(20);
    if (!rtcgetRtnCode)
      DBG_BOOL("rtcPICOOK PICO get time RtnCode:",rtcgetRtnCode);
    // rtc_get_datetime(&t); 

    led_pulse_start(ledPICO);  //flash LED

    bool rtcPICOrtN = true;  // will return true unless sanity check changes 

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

// for SfFat file time records -- for callback 
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) 
{
  datetime_t t;
  // rtc_get_datetime(&t);
  bool rtcgetRtnCode = rtc_get_datetime(&t);
  delay(100);
  // Serial.printf("SfFat PICO get time RtnCode: %s\n",rtcgetRtnCode?"true":"false");
  // DBG_BOOL("SfFat PICO get time RtnCode:",rtcgetRtnCode);
  led_pulse_start(ledPICO);
  *date = FS_DATE(t.year, t.month, t.day);
  *time = FS_TIME(t.hour, t.min, t.sec);
  *ms10 = 0;
}

// Function to synchronize the DS3231 time to the Pico's internal RTC time
bool syncPicoToDS3231() {
  datetime_t t;
  // Get time from Pico internal RTC
  bool rtcgetRtnCode = rtc_get_datetime(&t);
  delay(100);
  DBG_BOOL("syncPicotoDS3231 PICO get time RtnCode:",rtcgetRtnCode);
  // rtc_get_datetime(&t);
  led_pulse_start(ledPICO); 

  // Set the DS3231 RTC using uRTCLib's set function
  // The uRTCLib set function expects parameters in a specific order: 
  // second, minute, hour, dayOfWeek (1=Sun, 7=Sat), dayOfMonth, month, year (0-99)
  // The Pico's 'day of week' (t.dotw) is 0-6 (Sunday is 0), so we adjust it for uRTCLib (Sunday is 1)
  uint8_t dayOfWeekLib = (t.dotw == 0) ? 1 : t.dotw + 1;

  // sanity check before resyncing
  if (t.year < 2024 || t.year > 2100)
  {
    return false;
  }
  // DS3231 rtc set from PICO values
  rtc.set(
            t.sec,
            t.min,
            t.hour,
            dayOfWeekLib, // Use the adjusted Day of Week
            t.day,
            t.month,
            t.year % 100 // Use only the last two digits of the year (0-99)
          );
  rtc.refresh();

  Serial.printf("syncPicoToDS3231 => DS3231 RTC after rtc.set: DS:%02d-%02d-%02d %02d:%02d:%02d\n %s\n",
     rtc.year(),
     rtc.month(),
     rtc.day(),
    // tc.dayOfWeek(),
     rtc.hour(),
     rtc.minute(),
     rtc.second()
     //= rtc.temp();
    );

  // count resync's
  rtc_resync_events++;

  // validate
  if (rtc.year() != (t.year % 100))
  {
    return false;
  }

  // resync th;e cache vars too
  rtc_cache.y  = t.year % 100;
  rtc_cache.mo = t.month;
  rtc_cache.d  = t.day;
  rtc_cache.dow = dayOfWeekLib;
  rtc_cache.h  = t.hour;
  rtc_cache.m  = t.min;
  rtc_cache.s  = t.sec;

  // all is good
  return true;
}

// Routine to call to resync PICO time after power failure
bool syncDS3231ToPico() 
{
   char buffer[200] = "";
  // Update internal library variables from the physical DS3231
  rtcRefresh();
  led_pulse_start(ledRTC);
  if (!rtc_ok)
  {  
    DBGPF("DS rtc failed RTC sanity check:%d",rtc_ok);

    rtcPrintCache();
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
        .dotw  = (int8_t)(rtc_cache.dow - 1), // adjust for Sunday being 0 in PICO and 1 in DS3231
        .hour  = (int8_t)rtc_cache.h,
        .min   = (int8_t)rtc_cache.m,
        .sec   = (int8_t)rtc_cache.s
      };
 
    // Set the Pico's hardware RTC to the DS3231 time
    bool rtcsetRtnCode = rtc_set_datetime(&t);
    delay(65);
    if (rtcsetRtnCode)
    {
      DBG("Pico RTC successfully synced from DS3231: ");
      DBGPF("%04d-%02d-%02d %02d:%02d:%02d\n", t.year, t.month, t.day, t.hour, t.min, t.sec);
      bool rtcgetRtnCode = rtc_get_datetime(&t);
      delay(100);
      Serial.printf("PICO set time RtnCode: %s PICO get time RtnCode: %s\n",rtcsetRtnCode?"true":"false",rtcgetRtnCode?"true":"false");
      DBG("Pico RTC get datetime from PICO: ");
      DBGPF("%04d-%02d-%02d %02d:%02d:%02d\n", t.year, t.month, t.day, t.hour, t.min, t.sec);

      // copy PICO RTC data to pico_rtc_cache  Global struct
      pico_rtc_cache.y = t.year;
      pico_rtc_cache.mo = t.month;
      pico_rtc_cache.d = t.day;
      pico_rtc_cache.dow = t.dotw;
      pico_rtc_cache.h = t.hour;
      pico_rtc_cache.m = t.min;
      pico_rtc_cache.s = t.sec;

      rtcPrintPicoCache();

      return true;
    } else 
    {
      DBGLN("Error: Pico RTC rejected the datetime.");
      // display.setCursor(0,0); 
      // display.println("PICO RTCSet Failed");
      // display.display();
      return false;
    }
  }
}
// ------------------------ Get time --------------------------------------------

void getTime() // GET THE RTC's in SYNC AND CACHE's updated
{
    char buffer[200] = "";
    int jul = 0;  // not used
    long compDate = 0; // not used
    // DBG("\ngetTime:\n");
    bool rtcRefreshRtnCd = rtcRefresh();  // DS3231 time refresh - tests - recovers bus and DS3231
    if (!rtcRefreshRtnCd) DBG_BOOL("DS3231 rtc failed to refresh:\n",rtcRefreshRtnCd);
    else
    {
      rtcPrintCache();
      
      bool curDatesRtnCode = currentDates();    // set current Mth, Year, Hours from cache - used in filename(s)
      if (!curDatesRtnCode) 
      {
        DBG("Error  ***  currentDates not set!");
        DBG_BOOL("currentDates => curDatesRtnCode:",curDatesRtnCode);
      }
      // Register the callback with SdFat This ensures ALL SdFat instances (sdA, sdB) use this timestamp source
      FsDateTime::setCallback(dateTime);   //for SfFat date time stamp

      bool readRTCRtnCd = readRTC();   // from the PICO RTC  build bufferpi for OLED - updates cache
      if (!readRTCRtnCd) DBG_BOOL("PICO rtc failed to read:\n",readRTCRtnCd);
  
      printTime(); // from the DS3231 -  build bufferds for OLED
    }
    DBGLN("**************Time Setup  Done!****************");
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

// try WiFi to see if present   for three SSID's and can be determined with WiFi.status() if SSID is available
#define WIFI_COUNT 3
#define WIFI_RETRIES 3

bool tryWiFiMulti()  
{
    for (int s=0; s<WIFI_COUNT; s++)
    {
        for (int r=0; r<WIFI_RETRIES; r++)
        {
            DBG("WiFi trying SSID...");
            DBG(ssidList[s]);

            WiFi.begin(ssidList[s], passList[s]);

            uint32_t start = millis();
            // OLED display
            // display.println("WiFi INIT");
            // display.display();

            while (millis() - start < 8000)
            {
                if (WiFi.status() == WL_CONNECTED)
                {
                    DBG("WiFi connected!");
                    return true;
                }
                Serial.print(".");
                delay(250);
            }
        }
    }
    DBG("WiFi unavailable");
    return false;
}


// sync from NTP server
bool syncTimeFromNTP()
{
  char buffer[200] = "";

  // set time to PST or PDT -- info for NTP
  setenv("TZ", "PST8PDT,M3.2.0/2,M11.1.0/2", 1);
  tzset();
  configTime(0, 0, ntpServer1, ntpServer2);
 
  // display.println("NTP Sync");
  // display.display();

  // GET TIME FROM NTP SERVER
  Serial.print("Requesting time from NTP Server\n");
  Serial.print("Waiting for NTP server");
  while (!getLocalTime(&timeinfo)) 
  {
      // Serial.println("Waiting for time");
      Serial.print(".");
      delay(250);
  }
  DBGPF("Time zone: %s\n", tzname[0]);  
  // Got the time in struct tm timeinfo;  
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

  DBG("\nsyncTimeFromNTP: from NTP\t");
  snprintf(buffer, sizeof(buffer),  
      "%04d-%02d-%02d %02d:%02d:%02d\n (DOW=%d)\n\n",
      t.year, t.month, t.day, t.hour, t.min, t.sec, t.dotw);
  DBG(buffer);

  // copy PICO RTC data to pico_rtc_cache  Global struct
  pico_rtc_cache.y = t.year;
  pico_rtc_cache.mo = t.month;
  pico_rtc_cache.d = t.day;
  pico_rtc_cache.dow = t.dotw;
  pico_rtc_cache.h = t.hour;
  pico_rtc_cache.m = t.min;
  pico_rtc_cache.s = t.sec;
  DBG("\nsyncTimeFromNTP:print the PICO RTC cache time:");
  snprintf(buffer, sizeof(buffer), "%04d-%02d-%02d %02d:%02d:%02d\n",
          pico_rtc_cache.y, pico_rtc_cache.mo, pico_rtc_cache.d, pico_rtc_cache.h, pico_rtc_cache.m, pico_rtc_cache.s);
  DBG(buffer);

  led_pulse_start(ledPICO);  // flash LED

  DBG("Setting PICO RTC Time - rtc_set_datetime()\n");
  bool rtcsetRtnCode = rtc_set_datetime(&t);
  delay(65);
  Serial.printf("syncTimeFromNTP PICO set time RtnCode: %s\n",rtcsetRtnCode?"true":"false");
  if(!rtcsetRtnCode) 
  {
    DBG("PICO RTC Failed to set Time\n");
    // display.setCursor(0,0); 
    // display.println("PICO RTCSet Failed");
    // display.display();
    return false;
  }

  // DBG("syncTimeFromNTP: After rtc_init() \t");
  // snprintf(buffer, sizeof(buffer),  
  //     "%04d-%02d-%02d %02d:%02d:%02d\n (DOW=%d)\n\n",
  //     t.year, t.month, t.day, t.hour, t.min, t.sec, t.dotw);
  // DBG(buffer);

  // now set the DS3231 
  
  // first fix dow for DS3231 NTP is 0 and DS3231 1 for Sunday
  // uint8_t dayOfWeekLib = (t.dotw == 0) ? 1 : t.dotw + 1;

  // // resync
  // DBG("DS3231 set - rtc.set()\n");
  // rtc.set(t.sec, t.min, t.hour,  dayOfWeekLib, // Use the adjusted Day of Week
  //           t.day, t.month,  t.year % 100 // Use only the last two digits of the year (0-99)
  //         );
  // rtc.refresh();

  // DBG("syncTimeFromNTP: After DS3231 rtc.set() \t");
  // snprintf(buffer, sizeof(buffer),  
  //     "%04d-%02d-%02d %02d:%02d:%02d\n (DOW=%d)\n\n",
  //     t.year, t.month, t.day, t.hour, t.min, t.sec, t.dotw);
  // DBG(buffer);

  // DBG("End of syncTimeFromNTP\n");

  // all good
  return true;
}

// init the OLED
void OLEDinit()
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // display.begin(0x3C, true);   // address, reset
  display.setRotation(0);
  display.display(); // display the logo

  //clear OLED screen
  // display.stopscroll();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.display();
  display.clearDisplay();
}

// Clear WatchdogEscalation
void clearWatchdogEscalation()
{
    led_pulse_start(ledEEPROM);
    eeprom.eeprom_write(EEPROM_WD_COUNT_ADDR, 0);
}


// Boot Detection Logic routine called after i2c started
void watchdogEscalationCheck()
{
    if (!watchdog_caused_reboot())
        return;

    uint8_t lastStage = readStageEEPROM();
    led_pulse_start(ledEEPROM);
    uint8_t count     = eeprom.eeprom_read(EEPROM_WD_COUNT_ADDR);

    count++;

    led_pulse_start(ledEEPROM);
    eeprom.eeprom_write(EEPROM_WD_COUNT_ADDR, count);

    Serial.printf("WD Reboot Stage=%u Count=%u\n", lastStage, count);

    // ---------- Escalation Threshold ----------
    if (count >= 5)
    {
        Serial.println("ESCALATION: repeated failure");

        // Possible actions:
        // Blink LED pattern
        // Disable SD temporarily
        // Force RTC resync
        // Enter safe mode (future)
    }
}

const char* stageName(CycleStage s)
{
    switch(s)
    {
        case STAGE_SETUP:        return "SETUP";
        case STAGE_BOOT_I2C:     return "BOOT I2C";
        case STAGE_BOOT_RTC:     return "BOOT RTC";
        case STAGE_BOOT_TIME:    return "BOOT TIME";
        case STAGE_BOOT_SD:      return "BOOT SD";
        case STAGE_BOOT_EEPROM:  return "BOOT EEPROM";

        case STAGE_START:       return "START";
        case STAGE_SENSORS:     return "SENSORS";
        case STAGE_SD1:         return "SD1";
        case STAGE_SD2:         return "SD2";
        case STAGE_PROM:        return "PROM";
        case STAGE_DONE:        return "DONE";

        default: return "UNKNOWN";
    }
}

// Node Id write routine
void setNodeID(uint8_t id)
{
    if (id < 1 || id > 3)
        return;

    NODE_ID = id;
    led_pulse_start(ledEEPROM);
    eeprom.eeprom_write(NODE_ID_ADDR, (byte*)&NODE_ID, sizeof(NODE_ID));
}

// processing stage write routine
void writeStageEEPROM(uint8_t stage)
{
    static uint8_t lastStage = 255;

    // avoid unnecessary EEPROM wear
    if (stage == lastStage)
        return;

    lastStage = stage;

    led_pulse_start(ledEEPROM);
    eeprom.eeprom_write(EEPROM_STAGE_ADDR, stage);
}

// and read rooutine
uint8_t readStageEEPROM()
{
  led_pulse_start(ledEEPROM);
  return eeprom.eeprom_read(EEPROM_STAGE_ADDR);
}

// ----------------- uint8_t helpers -----------------

uint8_t eeprom_read_u8(uint16_t addr)
{
    uint8_t value = 0;
    eeprom.eeprom_read(addr, (uint8_t*)&value, sizeof(value));
    return value;
}

void eeprom_write_u8(uint16_t addr, uint8_t value)
{
    eeprom.eeprom_write(addr, (uint8_t*)&value, sizeof(value));
}


// ----------------- uint16_t helpers -----------------

uint16_t eeprom_read_u16(uint16_t addr)
{
    uint16_t value = 0;
    eeprom.eeprom_read(addr, (uint8_t*)&value, sizeof(value));
    return value;
}

void eeprom_write_u16(uint16_t addr, uint16_t value)
{
    eeprom.eeprom_write(addr, (uint8_t*)&value, sizeof(value));
}


// ----------------- uint32_t helpers -----------------

uint32_t eeprom_read_u32(uint16_t addr)
{
    uint32_t value = 0;
    eeprom.eeprom_read(addr, (uint8_t*)&value, sizeof(value));
    return value;
}

void eeprom_write_u32(uint16_t addr, uint32_t value)
{
    eeprom.eeprom_write(addr, (uint8_t*)&value);
}

// Replace LED modules


// LED at Modules
// LedPulse ledSD1    = {28, false, 0};
// LedPulse ledSD2    = {21, false, 0};
// LedPulse rtc_LED    = {14, false, 0};
// LedPulse eP_LED = {8, false, 0};
// LedPulse pi_LED    = {22, false, 0};
// LedPulse ledPICO   = {LED_BUILTIN, false, 0};



  // led_init(rtc_LED);
  // led_init(pi_LED);
  // led_init(eP_LED);
  // led_init(LED_BUILTIN);

void led_init(LedPulse &led) {
    pinMode(led.pin, OUTPUT);
    digitalWrite(led.pin, LOW);
}

void led_pulse_start(LedPulse &led) {
    digitalWrite(led.pin, HIGH);
    led.active = true;
    led.startTime = millis();
}

void led_pulse_update(LedPulse &led, unsigned long duration = 50) {
    if (led.active && (millis() - led.startTime >= duration)) {
        digitalWrite(led.pin, LOW);
        led.active = false;
    }
}

#define WATCHDOG_SETUP_TIMEOUT 10000
#define WATCHDOG_RUN_TIMEOUT   3000


// --------------------------------- Setup ------------------------------------
void setup() 
{
  Serial.begin(9600);
  // let everything start first -- wait six seconds 
  unsigned long start = millis();
  while (!Serial && (millis() - start < 6000)) { delay(10); } // need both conditions to fail to loop

  watchdog_update();
  Serial.setTimeout(100);

  Serial.print(F("\n-------------------- Pico SD Boot Sequence ----------------------\n"));
  Serial.print(F("Processor: "));

  #ifdef PICOW
    Serial.println(F("Raspberry Pi Pico W"));
  #endif

  // check magic marker
  led_pulse_start(ledEEPROM);
  uint8_t magic = eeprom_read_u8(EEPROM_WD_MAGIC_ADDR);

  if (watchdog_caused_reboot() && magic == WD_MAGIC)
  {
    pendingWatchdogEvent = true;

    watchdogStage = (CycleStage)readStageEEPROM();
    bootResetReason = "WATCHDOG";

    if (watchdogStage == 0xFF) Serial.println("Stage: UNKNOWN (EEPROM empty)");
    else 
    {
      Serial.printf("\n** WATCHDOG RESET DETECTED ***Stage: %s (%d)\n", stageToStr(watchdogStage), watchdogStage);
      Serial.printf("Watchdog reason reg: %08lX\n", watchdog_hw->reason);
    }
  }

  // enable watchdog after t;he watchdog_caused_reboot() check
  watchdog_enable(WATCHDOG_SETUP_TIMEOUT, true);

  // NOW set runtime stage
  STAGE_EARLY(STAGE_SETUP,"Setup"); 

  // Start the I2C bus
  // Must power on DS3231 and any other I2C module first
  // ------------------  powering on the DS3231  and I2C bus  -----------------------
  delay(100);
  DBG("Ensure I2C pins are NOT driving\n");
  pinMode(SDA, INPUT);      //I2C
  pinMode(SCL, INPUT);      //I2C

  // Init the LED's 
  led_init(ledRTC); // DS3231 Time activity
  led_init(ledBME);  // BMx  Time activity
  led_init(ledEEPROM);  // 24C32  EEPROM activity
  led_init(ledPICO);  //pico Time

  // pinMode(rtc_LED, OUTPUT); 
  // pinMode(pi_LED, OUTPUT);
  // pinMode(eP_LED, OUTPUT);
  // pinMode(LED_BUILTIN, OUTPUT);
 

  delay(100);
  // test LED // turn on then off to test
  // led_pulse_start(pi_LED);
  // led_pulse_start(rtc_LED);
  // led_pulse_start(eP_LED);
  // led_pulse_start(LED_BUILTIN);
  
  // NOW set Boot I2C stage 
  STAGE_EARLY(STAGE_BOOT_I2C,"I2C");  

  rtcPowerCycle();  // power off the back on the DS3231
  delay(59);
  watchdog_update();

  // Recover the I2C bus 
  if (i2cRecover())  // only return true
  {
    Serial.print("I2C Recovered\n");
  }

  i2cInit();    // -------------------------- initialize Wire ---------
  delay(100);

  //  Presence test
  if (i2cDevicePresent(0x68)) 
  {
      DBGLN("RTC present and OK");
      rtc.refresh();
      led_pulse_start(ledRTC);
  } else 
  {
      Serial.println("RTC bus error at boot  --- TROUBLE \n");
  }
  // I2C Started  // I believe wire.begin() has been run
  // eePROM scan to determine at what stage we crashed
  // reportLastResetStage();



  // List of EEPROM ADDRESSES:
  // DBG("EEPROM ADDR:\n");
  // Serial.printf("Stage addr \t\t= %u\n", EEPROM_STAGE_ADDR);
  // Serial.printf("Stage line addr \t= %u\n", EEPROM_LINE_ADDR);
  // Serial.printf("wd count addr \t\t= %u\n", EEPROM_WD_COUNT_ADDR);
  // Serial.printf("Snap addr \t\t= %u\n", SNAP_ADDR);
  // Serial.printf("Node id addr \t\t= %u\n", NODE_ID_ADDR);
  // Serial.printf("BootCount addr \t\t= %u\n", BOOTCOUNT_ADDR);
  // Serial.printf("BootCount magic addr \t= %u\n", BOOTCOUNT_MAGIC_ADDR);
  // Serial.printf("Temp addr \t\t= %u\n", ETEMP);

  // DUMP OF THE EEPROM
  // DBG("\n\nDUMP OF THE EEPROM - FIRST 64 ADDRESSES (HEX):\n");
  // for (int i=0;i<64;i++)
  // {
  //     if (i%16==0) Serial.printf("\n%02d: ",i);
  //     led_pulse_start(eP_LED);
  //     Serial.printf("%02X ", eeprom.eeprom_read(i));
  // }
  // Serial.println();

  // DBG("DUMP OF THE EEPROM - FIRST 64 ADDRESSES (DEC):\n");
  // for (int i=0;i<64;i++)
  // {
  //     if (i%16==0) Serial.printf("\n%02d: ",i);
  //     led_pulse_start(eP_LED);
  //     Serial.printf("%02u ", eeprom.eeprom_read(i));
  // }
  // Serial.println("\n\n");

  // ONE TIME CODE
  //setNodeID(1);   // NEW BREADBOARD SET TO 2 OR 3 DEPENDING
  // load the Node ID 
  
  uint8_t id;
  led_pulse_start(ledEEPROM);
  eeprom.eeprom_read(NODE_ID_ADDR, (byte*)&id, sizeof(id));

  if (id == 0xFF || id < 1 || id > 3)
      NODE_ID = 1;
  else
      NODE_ID = id;   // defaultNODE_ID
  DBGPF("Setup NODE_ID: %d\n",NODE_ID);

  // load boot count

  // ------------------- One time code only
  // eeprom_write_u32(8, 0); // one time only
  // eeprom_write_u16(BOOTCOUNT_ADDR, 0); // one time only
  // ------------------- end of one time code

    // eeprom.eeprom_read(BOOTCOUNT_ADDR, (byte*)&bootCount, sizeof(bootCount));

  // DBGPF("\nBefore write dump of BOOTCOUNT_ADDR: %d\n",BOOTCOUNT_ADDR);
  // led_pulse_start(ledEEPROM);
  // for (int i=0;i<8;i++)
  // {
  //     led_pulse_start(eP_LED);
  //     uint8_t b = eeprom.eeprom_read(BOOTCOUNT_ADDR+i);
  //     DBGPF("%02X ", b);
  // }

  led_pulse_start(ledEEPROM);
  bootCount = eeprom_read_u16(BOOTCOUNT_ADDR);

  DBGPF("Setup bootCount read:%" PRIu16 "\n\n",bootCount);

  // check magic
  led_pulse_start(ledEEPROM);
  magic  = eeprom.eeprom_read(BOOTCOUNT_MAGIC_ADDR);

  if (magic != EEPROM_LAYOUT_MAGIC)
  {
      DBG("Magic failed setting bootCount to 0");
      bootCount = 0;
      // writing bootCount
      led_pulse_start(ledEEPROM);
      eeprom_write_u16(BOOTCOUNT_ADDR, bootCount);
      //  writing bootCount Magic
      led_pulse_start(ledEEPROM);
      eeprom.eeprom_write(BOOTCOUNT_MAGIC_ADDR, EEPROM_LAYOUT_MAGIC);
      DBGPF("Setup Updated MAGIC & New bootCount:%" PRIu16 "\n",bootCount);
      // // dump memory
      // for (int i=0;i<8;i++)
      // {
      //     led_pulse_start(eP_LED);
      //     uint8_t b = eeprom.eeprom_read(BOOTCOUNT_ADDR+i);
      //     DBGPF("%02X ", b);
      // }
      // DBG("\n");
  }

  // Now bootCount
  if (bootCount > 1000)  
  {
    DBG("bootCount over 1000 resetting to 0");
    bootCount = 0;
    led_pulse_start(ledEEPROM);
    eeprom_write_u16(BOOTCOUNT_ADDR, bootCount);
    DBGPF("Setup reset value of bootCount:%" PRIu16 "\n",bootCount);
  } 
  else bootCount++;
  // rewrite bootCount
  DBGPF("Setup bootCount incremented by 1 new value:%" PRIu16 "\n\n",bootCount);
  led_pulse_start(ledEEPROM);
  eeprom_write_u16(BOOTCOUNT_ADDR, bootCount);
  // DBGPF("after write dump of BOOTCOUNT_ADDR ");
  
  // for (int i=0;i<8;i++)
  // {
  //     led_pulse_start(eP_LED);
  //     uint8_t b = eeprom.eeprom_read(BOOTCOUNT_ADDR+i);
  //     DBGPF("%02X ", b);
  // }
  // DBGLN("\n");

  // boot Detection Logic
  watchdogEscalationCheck();
  Serial.printf("\n");      // blank line
  watchdog_update();

  // ------------------------ OLED setup ------------------------------------------
  // The may have to move
  currentEvent = EVT_SD_WRITE;  // Investigate
  health.sd_ok = true;

  // init the OLED
  OLEDinit();

  // NOW set Boot RTC stage 
  STAGE(STAGE_BOOT_RTC,"RTC Time set");   

  // initialize the PICO RTC   --- Must be done here!!
  rtc_init();
  DBG("PICO RTC Initialized rtc_init() called!");

 // ---------------------- Recover Time  ------------------------  

  delay(50); // let things catch up so we get print
  watchdog_update();

  DBG("\n ******************  Time Recovery in Progress ******************\n");
  TimeSnapshot snap;  // alloc memory for struct

  bool snapOK = readTimeSnapshot(snap); // Read the EEPROM
  DBGPF("snapOK rtn code: %s\n",snapOK?"true":"false");

  bool rtcValid = rtcReadValid();  // read DS3231 RTC
  DBGPF("rtcValid rtn code: %s\n",rtcValid?"true":"false");

  if (rtcValid)  // First try the DS3231 battery time
  {
      DBG("SETUP: RTC battery time OK\n");
      timeConfidence = 1;
      rtcPrintCache();
      // NEED TO RESET PICO RTC ---------------------------------------------
      syncDS3231ToPico();  // copy DS3231 RTC to PICO RTC
  }
  else  // try WiFi and NTP server
  {
    DBG("SETUP: RTC invalid");
    if (tryWiFiMulti())
    {
        if (syncTimeFromNTP())  // NEED TO TEST NTP TIME FOR VALID TIME  // NEEDS ITEMS UPDATED TO BE PUT IN A SEPARATE ROUTINE
        {
            DBG("SETUP: NTP acquired");
            syncPicoToDS3231();   // MAKE SURE CACHES ARE SET  syncTimeFromNTP MAY HAVE DONE THIS ALL READY
            timeConfidence = 2;
        }
    }

    if (timeConfidence == 0 && snapOK)  // reset RTC with EEPROM Time may be out a day
    {
        DBG("SETUP: Synthetic rebuild");

        datetime_t t;
        t.year  = 2000 + snap.year;
        t.month = snap.month;
        t.day   = snap.day;
        t.hour  = (snap.hour + 12) % 24;
        t.min   = 0;
        t.sec   = 0;

        // rtc_set_datetime(&t);
        bool rtcsetRtnCode = rtc_set_datetime(&t);
        delay(65);
        Serial.printf("PICO set time RtnCode: %s\n",rtcsetRtnCode?"true":"false");
        syncPicoToDS3231();  // MAKE SURE CACHES ARE SET
    }

    if (timeConfidence == 0 && !snapOK)  // so assume an arbratary time of 2025-1-1  so we have a sequence counter
    {
        DBG("SETUP: Default compile time");

        datetime_t t = {2025,1,1,0,0,0}; //A RECORDING METHOD EVEN IF SNAP FAILS  (I WOULD SET YEAR TO 2030 TO BE DISTINTIVE )
        bool rtcsetRtnCode = rtc_set_datetime(&t);
        delay(65);
        Serial.printf("PICO set time RtnCode: %s\n",rtcsetRtnCode?"true":"false");
        syncPicoToDS3231();  // MAKE SURE CACHES ARE SET
    }
  }

  watchdog_update();

  // clear the watchdogEscalation
  clearWatchdogEscalation();

  // ------------------ Recover Time  ----------- **KEEP UNTIL WIFI TESTED** --------------- 

  // attempt to make wifi connection 
  // bool wifi_ok = connectWiFiWithRetries(3, 8000);  // most likely parms (3,20000) // 3 tries, wait 20 seconds
  // if (wifi_ok)
  // {
  //   syncTimeFromNTP();
  //   // copyPicoRTCtoDS3231();  // done in syncTimeFromNTP
  // }
  // else
  // {
  //   rtc_init();          // initialize PICO RTC
  //   syncDS3231ToPico();  // sync DS3231 To Pico()
  // }

  // #ifdef WIFI
  // // ------ access wifi ------------------------------
  //   Serial.print("Trying to Connect to WiFi\n");
  //   display.println("WiFi INIT");
  //   display.display();
  //   WiFi.mode(WIFI_STA);
  //   WiFi.begin(ssid, password);
  //   Serial.print("Connecting to WiFi");

  //   while (WiFi.status() != WL_CONNECTED) {  // try for 50 or so attempts if no connection 
  //     delay(500);
  //     Serial.print(".");
  //   }

  // // connection extablished
  //   Serial.println("\nWiFi connected.");
  //   Serial.print("IP address: ");
  //   Serial.println(WiFi.localIP());

  // // set time to PST or PDT
  //   setenv("TZ", "PST8PDT,M3.2.0/2,M11.1.0/2", 1);
  //   tzset();
  //   configTime(0, 0, ntpServer1, ntpServer2);

  //   Serial.print("Requesting time from NTP Server\n");
  //   // struct tm timeinfo;  // made global
  //   display.println("NTP Sync");
  //   display.display();
  // #endif

  // routine to handle PICO and DS3231 time
  //  ---------------- CALL getTime to sync all time events ----------------------
  // NOW set Boot Time stage  
  STAGE(STAGE_BOOT_TIME,"RTC Time set");  
  getTime();

  // ---------- Create repeating timer callback -----------------------
  // Interval = TIMECYCLE or initial 120 seconds = 120,000 ms = 120,000,000 µs

  add_repeating_timer_us(TIMECYCLE * 1000000, timer_callback, NULL, &timer);   //NEEDED

  // -------------------- Initialize the BME or BMP ---------------------------
  bmInit();

  watchdog_update();

  //----------------- Scan for I2C devices ---------------------------------------
  scanI2C();

  // check the EEPROM available
  DBG("EEPROM ready check: ");
  DBG(i2cDevicePresent(0x57) ? "OK\n" : "FAIL\n");
  watchdog_update();

  //------------------ SD Startup ----------------------------
  // NOW set Boot SD stage  
  STAGE(STAGE_BOOT_SD,"Init SD's");  

  sdA.sd = &sdFatA;
  sdB.sd = &sdFatB;

  // Start up the SD modules SD have a MOSFET which switches on power when a PICO pin is set HIGH
  // 0. SPI pin mapping
  spiPinMapping(sdA,sdB);

  // Power on
  // set up the LED
  pinMode(SD1_LED, OUTPUT);
  pinMode(SD2_LED, OUTPUT);

  // ------------------------- SD1 -----------------------------------
  sdOn(SD1_LED);
  if (!sdPowerOn(sdA)) {
    Serial.print("Trouble with powering on SD1 -- Check wiring, and MOSFET\n");
  };
  sdOff(SD1_LED);

  pinMode(SD1_CS, OUTPUT);
  digitalWrite(SD1_CS, HIGH);
  delay(100); // Give hardware time to settle
  
  // Init SPI
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

  watchdog_update();

  // ---------------------------------------- SD2 -----------------------------
  // Power on
  sdOn(SD2_LED);
  if (!sdPowerOn(sdB)) {
    Serial.print("Trouble with powering on SD2 -- Check wiring, and MOSFET\n");  
  };
  sdOff(SD2_LED);
  watchdog_update();
  pinMode(SD2_CS, OUTPUT);
  digitalWrite(SD2_CS, HIGH);
  delay(100); // Give hardware time to settle

  // Init SPI
  // sd2 uses SPI1
  sdOn(SD2_LED);
  SPI1.setRX(12); SPI1.setTX(11); SPI1.setSCK(10);
  if (!sdB.sd->begin(SdSpiConfig(sdB.pinSS, DEDICATED_SPI, SD_SCK_MHZ(16), sdB.spi))) {
    Serial.println("sdB (SPI1) Failed to initialize!");
  } else {
    DBGLN("sdB Init OK");
  }
  sdOff(SD2_LED);

  watchdog_update();

  // Test by List all files and directories recursively from the root directory

  Serial.println("\nSD Card 1 Listing files: Built in lib");
   sdOn(SD1_LED);
  sdA.sd->ls(LS_R); // LS_R flag enables recursive listing
  sdOff(SD1_LED);
  Serial.println("sdA Listed!");
  watchdog_update();
     // List all files and directories recursively from the root directory
  Serial.println("\nSD Card 1 Listing files:");
  // sdA.sd->ls(LS_R); // LS_R flag enables recursive listing
   sdOn(SD1_LED);
  if (sdA.sd->exists("/")) listFiles(*sdA.sd, "SD Card 1 (SPI0)");
  sdOff(SD1_LED);
  Serial.println("SD A done!");
  watchdog_update();


  Serial.println("\nSD Card 2 Listing files: Built in lib");
  sdOn(SD2_LED);
  sdB.sd->ls(LS_R); // LS_R flag enables recursive listing
  sdOff(SD2_LED);
  Serial.println("sdB  Built in lib!");
  watchdog_update();
       // List all files and directories recursively from the root directory
  Serial.println("\nSD Card 2 Listing files:");
  // sd2.sd->ls(LS_R); // LS_R flag enables recursive listing
  sdOn(SD2_LED);
  if (sdB.sd->exists("/")) listFiles(*sdB.sd, "SD Card 2 (SPI1)");
  Serial.println("sd B done!");
  sdOff(SD2_LED);

  // NOW set Boot EEPROM stage  
  STAGE(STAGE_BOOT_EEPROM,"Check EEPROM"); 

  // recovery if any eTemp's are bad
  bool ok1 = readETemp(&eTemp, etempAddr);
  bool ok2 = readETemp(&eTemp2, etempAddr2);

  DBGLN("Restoring the coldest temps from EEPROM copies");
  if (ok1 && ok2)
  {
      // Both valid
      if (fabs(eTemp - eTemp2) > 0.01)
      {
          DBGLN("EEPROM copies differ - using eTemp");
          coldestTemp2 = coldestTemp = eTemp2 = eTemp;
          writeETemp(eTemp2, etempAddr2);
      }
      else 
      {
        DBGLN("EEPROM copies same - using both eTemp's");
        coldestTemp = eTemp;
        coldestTemp2 = eTemp2;
      }
  }
  else if (ok1 && !ok2)
  {
      DBGLN("eTemp2 invalid - restoring from eTemp");

      coldestTemp2 = eTemp2 = eTemp;
      writeETemp(eTemp2, etempAddr2);
  }
  else if (!ok1 && ok2)
  {
      DBGLN("eTemp invalid - restoring from eTemp2");

      coldestTemp = eTemp = eTemp2;
      writeETemp(eTemp, etempAddr);
  }
  else
  {
      DBGLN("Both EEPROM copies invalid - initializing");

      eTemp = 79.9;
      eTemp2 = 79.9;

      writeETemp(eTemp, etempAddr);
      writeETemp(eTemp2, etempAddr2);
  }


  // touch watchdog
   watchdog_update();

  // logEvent call  // Second part fo Watchdog_caused Boot
  if (pendingWatchdogEvent)
  {

      //  Read potential crash line
      DBGPF("Setup() - pendingWatchdogEvent EEPROM_STAGE_ADDR  ADDRESS: %d:\n",EEPROM_STAGE_ADDR);
      watchdogStage = (CycleStage)readStageEEPROM();
      led_pulse_start(ledEEPROM);
      watchdogLine = eeprom_read_u16(EEPROM_LINE_ADDR);

      DBGPF("watchdogStage:%",PRIu8 "watchdogLine:%" PRIu16 "\n",watchdogStage,watchdogLine);

      DBGPF("Before - Dump of Stage Addr for 8 chars ADDR:%d",EEPROM_STAGE_ADDR);
      for (int i=0;i<8;i++)
      {
          led_pulse_start(ledEEPROM);
          uint8_t b = eeprom.eeprom_read(EEPROM_STAGE_ADDR+i);
          DBGPF("%02X ", b);
      }
      DBG("\n\n");

      // log event msg variable
      char msg[96];

      // build the message
      snprintf(msg,sizeof(msg),
        "WATCHDOG RESET stage=%s (%d) line=%d",
        stageName(watchdogStage),
        watchdogStage,
        watchdogLine
      );
      DBGPF("\n**logEvent() called msg:%s\n ",msg);
      logEvent(msg);

      // Clear the event
      pendingWatchdogEvent = false;
      // clear the entry
      led_pulse_start(ledEEPROM);
      eeprom_write_u8(EEPROM_WD_MAGIC_ADDR, 0);
      writeStageEEPROM(STAGE_NONE);
      led_pulse_start(ledEEPROM);
      eeprom_write_u16(EEPROM_LINE_ADDR, 0);
      
      DBGPF("After - Dump of Stage Addr for 8 chars ADDR:%d",EEPROM_STAGE_ADDR);
      for (int i=0;i<8;i++)
      {
          led_pulse_start(ledEEPROM);
          uint8_t b = eeprom.eeprom_read(EEPROM_STAGE_ADDR+i);
          DBGPF("%02X ", b);
      }
      DBG("Line reset to 0 \n\n");
  }

   watchdog_update();

  // Enable Watchdog second time in Setup()
  #ifdef PRODUCTION_BUILD
  #define WDT_DEBUG_PAUSE 0 // false
  #else
  #define WDT_DEBUG_PAUSE 1 // true
  #endif

  // watchdog_enable(WATCHDOG_TIMEOUT_MS, WDT_DEBUG_PAUSE);
  watchdog_enable(WATCHDOG_RUN_TIMEOUT, true);

  // ----------watchdog enable fix for PICO RTC ----------------------
   // now that watchdog corrupted PICO RTC Time sync with the DS3231
  delay(10);
  if (bool syncRtCode = syncDS3231ToPico())
  {
    Serial.printf("Setup: PICO sync after watchdog_enable with DS3231 -- return code:%s\n",syncRtCode?"true":"false");
  } else
  {
    Serial.printf("Setup: PICO Sync after watchdog_enable with DS3231 failed -- return code:%s\n",syncRtCode?"true":"false");
  }
   // -------watchdog enable fix for PICO RTC -------------------------- Temp????

  // ------------- Timer build examples **KEEP** -------------------------------------
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
  // ---------------------------- end of Setup() ---------------------------------------------------
}

// ------------------------------------------- Loop ----------------------------------------
void loop() 
{
  // Watch_dog stage ENUMs 
  // NOW set Boot Start stage 
  stage = STAGE_START;

  cycle_ok = true;   // assume success this cycle 
  bool sd_ok = false;
  char buffer[200] = "";
  now = millis();

  /* =====================================================
      1️⃣ 10 Minute Main Cycle (millis controlled)
  ===================================================== */
  if (firstLoop || (now - previous >= 600000))  // Ten minutes
  {
    // Read BMP280 or BME280
    firstLoop = false;
    previous  = now;
    t = 600000; //  Reset loop time to 10 minutes
    f = 10; // Reset cycles to 10

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
  
    // stage = STAGE_SENSORS;
    // WD();
    STAGE(STAGE_SENSORS,"Read Sensors");
    // DBGPF("Stage:%d  %s\n",stage,stageToStr(stage));
    // wdRecordStage(stage);
    // oledEvent("Read Sensors",stage);

    bm_ok = cycle_ok = readSensors(); //********** need test added 
   
    //  better and saves from buffer over runs
    n = snprintf(sdData,sizeof(sdData),"%s,%s%s,","SD",rtcData,bmData); // adds the , at the end and is NULL terminated

    // calc crc for sdData
    crc = crc16_ccitt((uint8_t*)sdData,strlen(sdData));

    n += snprintf(sdData+n,sizeof(sdData)-n,"%04X\r\n",crc);  // pointer arithmetic line+n  n is increased by 6 so n+6 +=
    // DBGPF("\nsdData size:%d\n\n",n);
    // DBG(sdData);

    // Test of sdData build
        // n = snprintf(line,sizeof(line),"%s \n",sdData);  // also add a , to the end of the line for to find for crc validation. 
        // Serial.print(line);
    // ------------------------- 
    // snprintf(fileName1,sizeof(fileName1),"c%1dv1%02d%02d.txt",node,currentMth,(int8_t)currentYear); 
    snprintf(fileName1,sizeof(fileName1),"csv1%02d%02d.txt",currentMth,(int8_t)currentYear); 
    DBGSNPF(buffer,sizeof(buffer),"New file name is %s\n ",fileName1);

    // snprintf(fileName2,sizeof(fileName2),"c%1dv2%02d%02d.txt",node,currentMth,(int8_t)currentYear); 
    snprintf(fileName2,sizeof(fileName2),"csv2%02d%02d.txt",currentMth,(int8_t)currentYear); 
    DBGSNPF(buffer,sizeof(buffer),"New file name is %s\n ",fileName2);

    sdOn(SD1_LED); sdOn(SD2_LED);

    WD();
    sd_ok = writeSDFile(sdData,fileName1,fileName2);
    sdOff(SD1_LED); sdOff(SD2_LED);

    if(!sd_ok)
    {
      bothSD_fail_streak++;
      bothSD_fail_total++;

      DBGPF("CRITICAL: Both SD writes failed — streak=%u\n",
            bothSD_fail_streak);

      if (bothSD_fail_streak >= SD_FAIL_RESET_THRESHOLD)
      {
          DBGLN("SD failure threshold reached -> watchdog reset allowed");
          cycle_ok = false;   // ⭐ triggers WDT reset
      }
    }
    else
    {
      if (bothSD_fail_streak)
      {
        DBGLN("SD subsystem recovered");
        bothSD_fail_streak = 0;
      }
    }

    // check if colder and update EEPROM
    // watch_dog Stage

    // NOW set Boot PROM stage   
    STAGE(STAGE_PROM,"Update EEPROM");
    promUpdate(bmTemp);

    #ifdef LISTFILES
      // list files on each SD card with size and time to see if updating occurs
      DBGLN("\nSD Card 1 Listing files:");
      // sdA.sd->ls(LS_R); // LS_R flag enables recursive listing
      sdOn(SD1_LED);
      listFiles(*sdA.sd, "SD Card 1 (SPI0)");  // if (sdA.sd->exists("/"))
      sdOff(SD1_LED);
      DBGLN("SD 1 listed!");
      // LF(*sdA.sd, "SD Card 1 (SPI0)");

      DBGLN("\nSD Card 2 Listing files:");
      // sd2.sd->ls(LS_R); // LS_R flag enables recursive listing
      sdOn(SD2_LED);
      listFiles(*sdB.sd, "SD Card 2 (SPI1)");
      sdOff(SD2_LED);
      DBGLN("SD 2 listed!");
      // LF(*sdB.sd, "SD Card 2 (SPI1)");
    #endif
    DBG("");

    // Watch_dog ENUM

    // NOW set Boot Done stage   
    // stage = STAGE_DONE;
    STAGE(STAGE_DONE,"10 Min Loop Done!");

    // Reset Watch_dog Stage Counter on Success
    led_pulse_start(ledEEPROM);
    eeprom.eeprom_write(EEPROM_WD_COUNT_ADDR, 0);
    health.cycle_count++;  // what is it telling me?
  }
    /* =====================================================
        2️⃣ One Minute Display Countdown
    ===================================================== */

    if (now - cycles >= 60000UL)
    {
        cycles = now;

        if (f > 0)
            f--;

        if (t >= 60000)
            t -= 60000;

        // call nothing   // setCycleDisp(f, t);
    }

    /* =====================================================
       3️⃣Now  10 Second OLED Update // was 30 seconds 
    ===================================================== */

    if (now - dispCycle >= 10000UL) //was 30000UL
    {
        dispCycle = now;
        updateDisplayScheduler();
    }

    /* =====================================================
       4️⃣ Hourly SD List (if enabled)
    ===================================================== */

    if (now - previousLF >= 600000UL * LFCYCLE)
    {
        previousLF = now;

        sdOn(SD1_LED);
        LF(*sdA.sd, "SD Card 1 (SPI0)");
        sdOff(SD1_LED);

        sdOn(SD2_LED);
        LF(*sdB.sd, "SD Card 2 (SPI1)");
        sdOff(SD2_LED);
    }

    /* =====================================================
       5️⃣ Daily Snapshot
    ===================================================== */

    if (rtc_ok && cycle_ok)
    {
        uint8_t today = rtc_cache.d;

        if (today != lastSnapshotDay)
        {
            writeTimeSnapshot();
            lastSnapshotDay = today;
        }
    }

    /* =====================================================
       6️⃣ Health + Watchdog
    ===================================================== */

    lastHeap = rp2040.getFreeHeap();
    if (lastHeap < minHeap)
        minHeap = lastHeap;

    // if we had an exception event record it
    if (eventPending)
    {
          // Build filenames
       snprintf(evtFile1, sizeof(evtFile1),
             "evt1%02d%02d.log", currentMth, currentYear);
            // "l%1dg1%02d%02d.txt",node, currentMth, currentYear);

       snprintf(evtFile2, sizeof(evtFile2),
             "evt2%02d%02d.log", currentMth, currentYear);
            // "l%1dg2%02d%02d.txt",node, currentMth, currentYear);
        WD();
        writeSDFile(eventLine, evtFile1, evtFile2);

        // oledShowEvent(oledEventLine); // add routine and un comment

        eventPending = false;
    }

    heartbeatLogger();

    // update all LEDs (non-blocking)
    // led_pulse_update(ledSD);
    led_pulse_update(ledRTC);
    led_pulse_update(ledEEPROM);
    led_pulse_update(ledBME);
    led_pulse_update(ledPICO);

    // heartbeat_update();  // Heartbeat LED // TURNED OFF DON'T LIKE THE CONSTANT FLASHING DISTRACTING.
    updateRtcHealth();  // Not needed can be deleted
    // STAGE(STAGE_IDLE,"At end of loop()");
    static uint32_t last_wd = 0;

    if (cycle_ok && (now - last_wd) > WD_FEED_INTERVAL_MS)
    {
        watchdog_update();
        last_wd = now;
    }

    tight_loop_contents();
}   // LOOP END

// Time stamp builder
void getTimestamp(char *buf, size_t len)
{
    int yr  = rtc.year();
    int mon = rtc.month();
    int day = rtc.day();

    int hr  = rtc.hour();
    int min = rtc.minute();
    int sec = rtc.second();

    snprintf(buf, len,
             "%04d-%02d-%02d %02d:%02d:%02d",
             yr, mon, day,
             hr, min, sec);
}


// logEvent main routine
void logEvent(const char *fmt, ...)
{
    char eventMsg[80];
    char timebuf[24];

    /* format user message */
    va_list args;
    va_start(args, fmt);
    vsnprintf(eventMsg, sizeof(eventMsg), fmt, args);
    va_end(args);

    /* build timestamp */
    getTimestamp(timebuf, sizeof(timebuf));

    const char *resetReason = bootResetReason;

    /* build final event log line */
    snprintf(eventLine, EVENT_LINE_LEN,
             "%s %s boot=%u %s\n",
             timebuf,
             resetReason,
             bootCount,
             eventMsg);


    //   Short OLED Event Message
    oledEventMessage(eventMsg);

    /* OLED short message */
    snprintf(oledEventLine, OLED_EVENT_LEN,
             "%s",
             eventMsg);

    eventPending = true;
}


// heartbeat_update  LED display


// void heartbeat_update() {
//     static unsigned long previousMillis = 0;
//     static int state = 0;

//     unsigned long now = millis();

//     switch (state) {
//         case 0:
//             digitalWrite(Heartbeat_LED_PIN, HIGH);
//             if (now - previousMillis >= 100) {
//                 previousMillis = now;
//                 state = 1;
//             }
//             break;

//         case 1:
//             digitalWrite(Heartbeat_LED_PIN, LOW);
//             if (now - previousMillis >= 100) {
//                 previousMillis = now;
//                 state = 2;
//             }
//             break;

//         case 2:
//             digitalWrite(Heartbeat_LED_PIN, HIGH);
//             if (now - previousMillis >= 100) {
//                 previousMillis = now;
//                 state = 3;
//             }
//             break;

//         case 3:
//             digitalWrite(Heartbeat_LED_PIN, LOW);
//             if (now - previousMillis >= 800) {
//                 previousMillis = now;
//                 state = 0;
//             }
//             break;
//     }
// }

// Alternate code for the same thing. 


// void heartbeat_update() {
//     static unsigned long previousMillis = 0;
//     static int state = 0;

//     const int pattern[] = {100, 100, 100, 800};  // ms per state
//     const int maxStates = 4;

//     unsigned long now = millis();

//     if (now - previousMillis >= pattern[state]) {
//         previousMillis = now;

//         state++;
//         if (state >= maxStates) state = 0;

//         // LED ON for states 0 and 2
//         digitalWrite(LED_PIN, (state == 0 || state == 2) ? HIGH : LOW);
//     }
// }


#define HB_INTERVAL 60000UL  // 1 minute for testing
#define HB_SD_DIV   10       // write to SD every 10 minutes

void heartbeatLogger()
{
    static uint32_t lastHB = 0;
    static uint8_t  hbDiv  = 0;

    uint32_t now = millis();

    if (now - lastHB < HB_INTERVAL)  // limits updates to one minute
        return;

    lastHB = now;

    // Build filenames
    snprintf(logFile1, sizeof(logFile1),
             "log1%02d%02d.txt", currentMth, currentYear);
            // "l%1dg1%02d%02d.txt",node, currentMth, currentYear);

    snprintf(logFile2, sizeof(logFile2),
             "log2%02d%02d.txt", currentMth, currentYear);
            // "l%1dg2%02d%02d.txt",node, currentMth, currentYear);

    // Build uptime string
    char uptimeStr[32];
    formatUptime(uptimeStr, sizeof(uptimeStr));

        snprintf(logEntry, sizeof(logEntry),
                "HB N%" PRIu8 " %s Heap:%" PRIu32 " Min:%" PRIu32
                " L1:%" PRIu32 " L2:%" PRIu32
                " C1:%" PRIu32 " C2:%" PRIu32
                " E1:%" PRIu32 " E2:%" PRIu32
                " Boot:%" PRIu16 "\r\n",

                NODE_ID,
                uptimeStr,

                lastHeap,
                minHeap,
                log1Size,
                log2Size,
                csv1Size,
                csv2Size,
                evt1Size,
                evt2Size,
                (uint16_t)bootCount
        );

    // Only write to SD occasionally
    hbDiv++;

    if (hbDiv >= HB_SD_DIV)  // limits SD writes to every 10 minutes
    {
        hbDiv = 0;
        WD();
        writeSDFile(logEntry, logFile1, logFile2);
        DBGLN(logEntry);
    }
}


// crc routine for the data 
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    while (len--)
    {
        crc ^= (uint16_t)(*data++) << 8;

        for (uint8_t i=0;i<8;i++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}
// #define EEPROM_WD_LOG_BASE   32      // you said stage uses 32/33
// #define EE_WD_SLOTS  4       // because using 32/33 only  // changed from 2 to 4

// Loop sensor read function 
bool readSensors()
{
  // NOW set Boot Sensors stage   
  watchdog_update();
 

  bool ok = true;

  // Read the DS3231
    bool RTC_ReadRtnCode = RTC_Read();
    if (!RTC_ReadRtnCode)
    {
      rtc_fail_streak++;
      if (rtc_fail_streak >= RTC_FAIL_THRESHOLD)
          cycle_ok = false;
  
    }
    else
    {
      rtc_fail_streak = 0;
    }
    // update the routine Rtn code
    ok &= RTC_ReadRtnCode;

    // i2c bus failure // tested above
    if (i2c_fail_streak >= I2C_FAIL_RESET_THRESHOLD)
    {
      DBGPF("CRITICAL: I2C unrecoverable — streak=%u\n",
            i2c_fail_streak);
      cycle_ok = false;   // allow watchdog reset
    }

    // Read the PICO RTC
    bool readRTCRtnCode = readRTC();
    if (!readRTCRtnCode)
    {
      PicoRtc_fail_streak++;
      if (PicoRtc_fail_streak >= PICORTC_FAIL_THRESHOLD)
        cycle_ok = false;
    }
    else
    {
      PicoRtc_fail_streak = 0;
    }
    // update the routine Rtn code
    ok &= readRTCRtnCode;

    // Use PICO RTC as timestamp so change sdData So rebuild rtcData here // Use PICO RTC to control filenames

    // RECORD BUILD => build a RTC data string with delimiter be written to the SD Chip
    int j = snprintf(rtcData,sizeof(rtcData), "%02d,%02d,%02d,%02d,%02d,%02d,%02d,%08s,", 20, // century, plus data
    (pico_rtc_cache.y - 2000),pico_rtc_cache.mo,pico_rtc_cache.d,pico_rtc_cache.h,pico_rtc_cache.m,pico_rtc_cache.s,rtcTemp);
    // DBGPF("size of the rtcData buffer:%d\n",j);

    // store in the global variable rtcData
    // strcpy(rtcData, buffer);  // adds a \0 at the end
    // j = strlen(rtcData);
    // DBGPF("strlen of the rtcData:%d\n",j);

    // PRINT ONLY  build a print line for monitor with snprintf rather than many serial.print lines
    char rtcbuffer[200] = "";
    j = snprintf(rtcbuffer,sizeof(rtcbuffer),"DS RTC DT ::%02d/%02d/%02d  %02d:%02d:%02d %s%sC\n",
    (pico_rtc_cache.y - 2000),pico_rtc_cache.mo,pico_rtc_cache.d,pico_rtc_cache.h,pico_rtc_cache.m,pico_rtc_cache.s,rtcTemp,"\xC2\xB0");  // use cache variables not module calls

    bool bmTaskRtnCode = bmTask();
    if (!bmTaskRtnCode) 
    {
      bm_fail_total++;
      bm_fail_streak++;
      if (bm_fail_streak >= BM_FAIL_THRESHOLD)
        cycle_ok = false;
    }
    else
    {
      bm_fail_total = 0;
    }
    ok &= bmTaskRtnCode;
    static uint32_t last_bm_reinit = 0;

    if (bm_fail_total >= BM_REINIT_THRESHOLD)
    {
        if (millis() - last_bm_reinit > 30000)   // 30 sec guard
        {
            DBGLN("BM sensor reinit triggered");

            bmInit();
            last_bm_reinit = millis();
            bm_fail_total = 0;
        }
    }

    return ok;
}
// The last line