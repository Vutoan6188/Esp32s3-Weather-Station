// Dual_core demo 13/6/2026
// edit range heap, min, block 21/6
// replace Timelib 22/6
// Edit json for API updateWeatherData && updateAstronomyData 18/6
// add esp32_temp function
// edit code for blynk server error block TFT 24/6
// add Windy Station 27/6
// edit readSDData 29/6
// sendWindy() core 0 30/6
// add WeatherUnderground 1/7
// not use commont client, dont use char for URL update weather/astronomy... 6/7
#define AA_FONT_SMALL "fonts/NotoSansBold15"
#define AA_FONT_LARGE "fonts/NotoSansBold36"
#define AA_FONT_10 "fonts/NotoSans-Bold10"
#define AA_FONT_13 "fonts/NotoSans-Bold13"
#define AA_FONT_20 "fonts/NotoSans-Bold20"
#define AA_FONT_30 "fonts/NotoSans-Bold30"
#define AA_FONT_40 "fonts/NotoSans-Bold40"
#define AA_FONT_70 "fonts/NotoSans-Bold70"
#define FIRMWARE_VERSION "0.2.7"

/**                         Load the libraries and settings
***************************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>  // https://github.com/Bodmer/TFT_eSPI
#include <TJpg_Decoder.h>
#include <FS.h>
#include <LittleFS.h>
#include "GfxUi.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <BlynkSimpleEsp32.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "TimeESP32.h"
#include <time.h>
#include "VietnameseLunar.h"
#include "Wire.h"

//AHT30
#include <Adafruit_AHTX0.h>
//BH1750
#include <BH1750.h>
uint32_t lastBacklightCheck = 0;
int currentBacklightPWM = -1;
// Ebyte E220
#include "LoRa_E220.h"
//debugMemory
#include "esp_heap_caps.h"
//Esp32_temp
#include "driver/temperature_sensor.h"
temperature_sensor_handle_t temp_handle = NULL;
/***************************************************************************************
**                          Define the globals and class instances
***************************************************************************************/
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprTime = TFT_eSprite(&tft);

volatile bool booted = false;
volatile bool GetWeatherData = false;
volatile bool GetAstronomy = false;
volatile bool firstWeatherRun = true;
volatile bool firstAstronomyRun = true;
volatile bool weatherReady = false;
volatile bool astronomyReady = false;
volatile bool SendPWSData = false;
volatile int windyLastHttpCode = -1;
volatile bool windyResultPending = false;
volatile int WULastHttpCode = -1;
volatile bool WUResultPending = false;
String WUResponse = "";

bool newLoRaData = false;
//blynkservererror
bool blynkEnabled = false;
unsigned long blynkLostMillis = 0;
bool blynkReconnectScheduled = false;
int lastReconnectMinute = -1;

WiFiManager wifiManager;
GfxUi ui = GfxUi(&tft);

int last_WriteSD = -1;
// Sensor on Board PCB
Adafruit_AHTX0 aht;
BH1750 lightMeter;

#define LORA_RX_PIN 4
#define LORA_TX_PIN 5
#define ENABLE_RSSI true
LoRa_E220 e220ttl(LORA_TX_PIN, LORA_RX_PIN, &Serial2, UART_BPS_RATE_9600, SERIAL_8N1);

WiFiClientSecure client;


/***************************************************************************************
**                          TaskHandle
***************************************************************************************/

TaskHandle_t WeatherTaskHandle = NULL;
/***************************************************************************************
**                          Declare prototypes
***************************************************************************************/
void updateWeatherData(time_t local_time);
void updateAstronomy(time_t local_time);
void drawProgress(uint8_t percentage, const char* text);
void drawTime(time_t local_time);
void updateBlynk();
void sendWindy();
void sendWindyDone();
void sendWU();
void sendWUDone();
void enterLightSleep();
void GetLoRa();
void drawTemHumIndoor();
void drawTemHumOutdoor();
void drawAngle();
void drawWindSpeed();
void drawRain();
void drawSignal();
void ResetValue(time_t local_time);
void connectWiFi();
void drawData(time_t local_time);
void GraphWindRain();
void GraphTemp();
void GraphHum();
void drawCurrentWeather(time_t local_time);
void checkFirmwareUpdate();
void checkButton(time_t local_time);
void saveSDData(time_t local_time);
void readSDData(time_t local_time);
void updateGraphWindRain();
void updateGraphTemp();
void updateGraphHum();
void GraphWindRainSD();
void GraphTempSD();
void GraphHumSD();
void debugMemoryTFT(int baseY);
void updateBacklight();
void espTemp(int baseY);

const char* getMeteoconIcon(const char* weatherIcon, int Cloudcover, int isday);
void drawAstronomy(time_t local_time);
void fillSegment(int x, int y, int start_angle, int sub_angle, int r, unsigned int colour);

int leftOffset(const char* text, const char* sub);
int rightOffset(const char* text, const char* sub);
int splitIndex(const char* text);

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (y >= tft.height()) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}


//  syncTime
bool syncTimeESP32() {
  configTime(
    7 * 3600,  // UTC+7
    0,
    "time.google.com",
    "pool.ntp.org",
    "time.nist.gov");
  struct tm timeinfo;
  for (int i = 0; i < 30; i++) {
    if (getLocalTime(&timeinfo)) {
      return true;
    }
    delay(500);
  }
  return false;
}
int lastMinute = -1;
int lastSyncHour = -1;
/***************************************************************************************
**                          Structs & Data
***************************************************************************************/
StaticJsonDocument<256> weatherFilter;
struct CurrentWeather {
  float temperature;
  float pressure;
  float windSpeedKmph;
  int humidity;
  int windDirectionDegrees;
  int UVindex;
  int Cloudcover;
  char weatherIcon[32];
  char timeupdate[32];
  float rainmm;
  int isday;
};
CurrentWeather weatherData;

StaticJsonDocument<128> astronomyFilter;
struct DailyWeather {
  char sunrise[16];
  char sunset[16];
  char moonrise[16];
  char moonset[16];
  float moonage;
};
DailyWeather astronomyData;

/***************************************************************************************
**                          Hardware Pins & Settings
***************************************************************************************/
#define LEDpin 3
#define BUTTON_PIN 0  // Pin 0 -> BOOT Button
#define SLEEP_PIN 2
#define BATTERY_PIN 1

#define SD_CS 21
#define SD_SCK 36
#define SD_MOSI 37  //SD_DIN
#define SD_MISO 38  //SD_DO

SPIClass spiSD(FSPI);

/***************************************************************************************
**                          Sensor Variables
***************************************************************************************/
// E220 & Network
int rssi, dbm, valueLoRaStrongE220, valueLoRaStrong, valueWiFiStrong, WiFiRSSI;

// Power & Solar
float fvalueBat, valueBattery, valueBat, gBat, valueBattery1;
float fvalueSol, valueSolar, valueSol, gSol;

// TemHum OUTDOOR & INDOOR
float TempOutDoor, HumiOutDoor, maxTempOutDoor, minTempOutDoor, maxHumiOutDoor, minHumiOutDoor;
float TempInDoor, HumiInDoor, maxTempInDoor, minTempInDoor, maxHumiInDoor, minHumiInDoor;

// Wind & Rain
float fWindSpeed, maxWindHour, winddy;
float RainGauge, mmHourly, mmDaily, mmGraph, mmTotal = 0, prevTotal = 0;

float fTimeConnect;
float Pressurebme, Pressurecal, Altitude;
const float PRESSURE_OFFSET = 6.8f;
#define SEALEVELPRESSURE_HPA (1013.25)

/***************************************************************************************
**                          Graph Buffers & Timers
***************************************************************************************/
float graphWindSpeed, graphWindMax, graphWinddy, graphRain;
float graphHumOUT, graphTemOUT, graphHumIN, graphTemIN;

int16_t x[800], y[800], z[800], a[800], b[800], g[800];
int16_t c[800], d[800], v[800], f[800];
int16_t n[800], h[800], m[800], j[800];

uint32_t currentMillis = 0;
uint32_t connectMillis = 0;
uint32_t LoRaMillis = 0;
uint32_t lastCheckPower = 0;
//uint32_t logCount = 0;

/***************************************************************************************
**                          Compass Math
***************************************************************************************/
#define CENTER_X 398
#define CENTER_Y 82
#define RADIUS 50

int lastArrowHeadX1 = CENTER_X;
int lastArrowHeadY1 = CENTER_Y;
int lastArrowHeadX2 = CENTER_X;
int lastArrowHeadY2 = CENTER_Y;
int lastArrowTailX = CENTER_X;
int lastArrowTailY = CENTER_Y;

/***************************************************************************************
**                          Config Data (WiFi/GitHub)
***************************************************************************************/
char locationKey[40] = "";
char apiKey[48] = "";
char authKey[40] = "";

// Windy
char windyStationID[32] = "";
char windyPassword[96] = "";

// WeatherUnderground - ID và Key thực tế ngắn (VD: "KKSSCOTT94", "MPpPODMf")
char wuStationID[16] = "";   // dư ra 15 ký tự + null, đủ cho mọi format WU ID
char wuStationKey[16] = "";  // dư ra 15 ký tự + null, key WU thường ~8 ký tự

// EEPROM Address
#define ADDR_LOCATION 0      // 40 bytes (0 -> 39)
#define ADDR_API 40          // 48 bytes (40 -> 87)
#define ADDR_BLYNK 88        // 40 bytes (88 -> 127)
#define ADDR_WINDY_ID 128    // 32 bytes (128 -> 159)
#define ADDR_WINDY_PASS 160  // 96 bytes (160 -> 255)
#define ADDR_WU_ID 256       // 16 bytes (256 -> 271)
#define ADDR_WU_KEY 272      // 16 bytes (272 -> 287)

const char* GITHUB_USER = "Vutoan6188";
const char* GITHUB_REPO = "Esp32s3-Weather-Station";
const char* FIRMWARE_FILENAME = "Esp32s3_Weather_Station.ino.bin";

/***************************************************************************************
**                          Config ReadSDData
***************************************************************************************/
bool sdReady = false;

byte currentPage = 0;

bool lastButtonState = HIGH;
bool buttonState = HIGH;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
// =====================================================
// GLOBAL VARIABLE
// =====================================================

#define MAX_SD_POINTS 800
#define MAX_LINE_LENGTH 180

int16_t sdTempOut[MAX_SD_POINTS];
int16_t sdTempIn[MAX_SD_POINTS];

int16_t sdHumOut[MAX_SD_POINTS];
int16_t sdHumIn[MAX_SD_POINTS];

int16_t sdWindGust[MAX_SD_POINTS];
int16_t sdWindDir[MAX_SD_POINTS];
int16_t sdRain[MAX_SD_POINTS];

int16_t sdValidPoints = 0;

// =====================================================
// STATIC BUFFER FOR SD
// =====================================================

int16_t tempOutBuffer[MAX_SD_POINTS];
int16_t tempInBuffer[MAX_SD_POINTS];

int16_t humOutBuffer[MAX_SD_POINTS];
int16_t humInBuffer[MAX_SD_POINTS];

int16_t windGustBuffer[MAX_SD_POINTS];
int16_t windDirBuffer[MAX_SD_POINTS];

int16_t rainBuffer[MAX_SD_POINTS];


/***************************************************************************************
**                          WeatherTask
***************************************************************************************/

void WeatherTask(void* pvParameters) {
  while (true) {
    time_t local_time = getLocalTimeSafe();

    int h = getHour(local_time);
    int m = getMinute(local_time);
    int s = getSecond(local_time);
    int d = getDay(local_time);

    // Windy upload — 6 minutes
    if (m % 6 == 0 && s >= 2 && !SendPWSData) {
      sendWindy();
      sendWU();
      SendPWSData = true;
    }
    //if (s == 0)
    if (m % 6 != 0)
      SendPWSData = false;

    // WeatherData
    if (firstWeatherRun || (m == 0 && s >= 7 && !GetWeatherData)) {
      firstWeatherRun = false;
      updateWeatherData(local_time);
      GetWeatherData = true;
    }
    if (m != 0)
      GetWeatherData = false;

    // AstronomyData
    if (firstAstronomyRun || (h == 0 && m == 0 && s >= 10 && !GetAstronomy)) {
      firstAstronomyRun = false;
      updateAstronomy(local_time);
      GetAstronomy = true;
    }
    if (h != 0)
      GetAstronomy = false;

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


/***************************************************************************************
**                          Esp32_temp
***************************************************************************************/
void initESPTemp() {
  temperature_sensor_config_t temp_sensor =
    TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);

  ESP_ERROR_CHECK(
    temperature_sensor_install(
      &temp_sensor,
      &temp_handle));

  ESP_ERROR_CHECK(
    temperature_sensor_enable(
      temp_handle));
}


/***************************************************************************************
**                          Setup
***************************************************************************************/
void setup() {
  //Serial.begin(115200);
  // Tắt hoàn toàn Bluetooth để tiết kiệm pin
  btStop();
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(SLEEP_PIN, INPUT_PULLDOWN);

  // set the resolution to 12 bits (0-4095)
  analogReadResolution(12);

  // DIM
  ledcAttach(LEDpin, 1000, 8);
  ledcWrite(LEDpin, 255);
  tft.setRotation(3);
  tft.begin();
  sprTime.createSprite(323, 56);
  tft.fillScreen(TFT_BLACK);

  if (!LittleFS.begin()) {
    while (1) yield();  // Stay here twiddling thumbs waiting
  }

  EEPROM.begin(512);  // Khởi tạo EEPROM

  // Check BOOT Button
  if (digitalRead(BUTTON_PIN) == LOW) {
    tft.loadFont(AA_FONT_SMALL, LittleFS);
    tft.setTextDatum(BC_DATUM);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextPadding(240);
    tft.drawString(" ", 400, 220);
    tft.drawString("Clearing the old configuration", 400, 240);
    // Delete config
    wifiManager.resetSettings();
    EEPROM.write(0, 0);
    EEPROM.commit();
  } else {
    // If not... read from EEPROM
    EEPROM.get(ADDR_LOCATION, locationKey);
    EEPROM.get(ADDR_API, apiKey);
    EEPROM.get(ADDR_BLYNK, authKey);

    EEPROM.get(ADDR_WINDY_ID, windyStationID);
    EEPROM.get(ADDR_WINDY_PASS, windyPassword);

    EEPROM.get(ADDR_WU_ID, wuStationID);
    EEPROM.get(ADDR_WU_KEY, wuStationKey);
  }

  // Add custom parameters for WiFiManager
  WiFiManagerParameter custom_locationKey("locationKey", "Location", locationKey, 40);
  WiFiManagerParameter custom_apiKey("apiKey", "API_key", apiKey, 48);
  WiFiManagerParameter custom_authKey("authKey", "BLYNK_AUTH_TOKEN", authKey, 40);
  WiFiManagerParameter custom_windyID("windyID", "Windy Station ID", windyStationID, 32);
  WiFiManagerParameter custom_windyPass("windyPass", "Windy Password", windyPassword, 96);
  WiFiManagerParameter custom_wuID("wuID", "WU Station ID", wuStationID, 32);
  WiFiManagerParameter custom_wuKey("wuKey", "WU Station Key", wuStationKey, 96);

  wifiManager.addParameter(&custom_locationKey);
  wifiManager.addParameter(&custom_apiKey);
  wifiManager.addParameter(&custom_authKey);
  wifiManager.addParameter(&custom_windyID);
  wifiManager.addParameter(&custom_windyPass);
  wifiManager.addParameter(&custom_wuID);
  wifiManager.addParameter(&custom_wuKey);

  // Auto connect WiFi
  if (!wifiManager.autoConnect("Weather_Station_Config")) {
    ESP.restart();
  }

  // Clear bottom section of screen
  tft.fillRect(0, 100, 800, 200, TFT_BLACK);
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    sdReady = false;
    tft.drawString("SD init failed", 0, 80);
  } else {
    sdReady = true;
    tft.drawString("SD ready", 0, 80);
  }

  char wifiBuf[64];
  snprintf(wifiBuf, sizeof(wifiBuf), "Connecting to WiFi SSID: %s", WiFi.SSID().c_str());
  tft.drawString(wifiBuf, 0, 100);

  checkFirmwareUpdate();

  // Save config data in EEPROM
  strcpy(locationKey, custom_locationKey.getValue());
  strcpy(apiKey, custom_apiKey.getValue());
  strcpy(authKey, custom_authKey.getValue());
  strcpy(windyStationID, custom_windyID.getValue());
  strcpy(windyPassword, custom_windyPass.getValue());
  strcpy(wuStationID, custom_wuID.getValue());
  strcpy(wuStationKey, custom_wuKey.getValue());

  EEPROM.put(ADDR_LOCATION, locationKey);
  EEPROM.put(ADDR_API, apiKey);
  EEPROM.put(ADDR_BLYNK, authKey);
  EEPROM.put(ADDR_WINDY_ID, windyStationID);
  EEPROM.put(ADDR_WINDY_PASS, windyPassword);
  EEPROM.put(ADDR_WU_ID, wuStationID);
  EEPROM.put(ADDR_WU_KEY, wuStationKey);

  EEPROM.commit();

  char cfgBuf[64];
  snprintf(cfgBuf, sizeof(cfgBuf), "Location: %s", locationKey);
  tft.drawString(cfgBuf, 0, 200);
  snprintf(cfgBuf, sizeof(cfgBuf), "Api Key: %s", apiKey);
  tft.drawString(cfgBuf, 0, 220);
  snprintf(cfgBuf, sizeof(cfgBuf), "Blynk Auth Token: %s", authKey);
  tft.drawString(cfgBuf, 0, 240);

  Wire.begin(6, 7);
  aht.begin();
  lightMeter.begin();
  e220ttl.begin();

  tft.fillScreen(TFT_BLACK);

  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);
  TJpgDec.setSwapBytes(true);

  // Draw splash screen
  if (LittleFS.exists("/splash/weather_icon.jpg") == true) {
    TJpgDec.drawFsJpg(160, 0, "/splash/weather_icon.jpg", LittleFS);
  }
  delay(1000);

  // Clear bottom section of screen
  tft.fillRect(0, 200, 800, 400, TFT_BLACK);

  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.drawString("Original by: blog.squix.org", 400, 260);
  tft.drawString("Adapted by: Bodmer", 400, 280);

  tft.fillRect(0, 200, 800, 400, TFT_BLACK);

  Blynk.config(authKey);
  tft.setTextDatum(BC_DATUM);
  tft.drawString("Fetching weather data...", 400, 240);

  // Fetch the time
  drawProgress(20, "Updating time...");

  //syncTime
  syncTimeESP32();

  if (sdReady) {
    time_t local_time = getLocalTimeSafe();
    char bootFileName[32];

    snprintf(bootFileName, sizeof(bootFileName), "/log_%04d_%02d.csv",
             getYear(local_time),
             getMonth(local_time));

    if (!SD.exists(bootFileName)) {
      File f = SD.open(bootFileName, FILE_WRITE);
      if (f) {
        f.println(
          "DateTime,"
          "TempOutMax,TempOutMin,TempOut,"
          "TempInMax,TempInMin,TempIn,"
          "HumOutMax,HumOutMin,HumOut,"
          "HumInMax,HumInMin,HumIn,"
          "WindSpeed,WindGustHour,WindDirection,"
          "Rain,RainHour,RainDay,Solar_V");
        f.close();

        tft.drawString("Created new month file successfully!", 0, 100);
      }
    } else {
      //Serial.print(">>>> File thang nay da co san: ");
      //Serial.println(bootFileName);
    }
  }

  drawProgress(50, "Update conditions...");
  tft.unloadFont();

  // Default value INDOOR / OUTDOOR / Rain / Wind
  maxTempInDoor = 0;
  minTempInDoor = 99;
  maxHumiInDoor = 0;
  minHumiInDoor = 100;
  maxTempOutDoor = 0;
  minTempOutDoor = 99;
  maxHumiOutDoor = 0;
  minHumiOutDoor = 100;
  mmHourly = 0;
  mmDaily = 0;
  mmGraph = 0;
  maxWindHour = 0;
  valueLoRaStrong = 0;
  valueWiFiStrong = 0;
  valueBattery = 0;
  valueSolar = 0;

  // Weatherfilter
  weatherFilter["currentConditions"]["datetime"] = true;
  weatherFilter["currentConditions"]["icon"] = true;
  weatherFilter["currentConditions"]["feelslike"] = true;
  weatherFilter["currentConditions"]["humidity"] = true;
  weatherFilter["currentConditions"]["winddir"] = true;
  weatherFilter["currentConditions"]["windgust"] = true;
  weatherFilter["currentConditions"]["precip"] = true;
  weatherFilter["currentConditions"]["uvindex"] = true;
  weatherFilter["currentConditions"]["cloudcover"] = true;
  // Astronomyfilter
  astronomyFilter["days"][0]["sunrise"] = true;
  astronomyFilter["days"][0]["sunset"] = true;
  astronomyFilter["days"][0]["moonrise"] = true;
  astronomyFilter["days"][0]["moonset"] = true;
  astronomyFilter["days"][0]["moonphase"] = true;

  for (int i = 799; i >= 0; i--) {
    x[i] = 9999;
    y[i] = 9999;
    z[i] = 9999;
    a[i] = 9999;
    b[i] = 9999;
    g[i] = 9999;
    c[i] = 9999;
    d[i] = 9999;
    v[i] = 9999;
    f[i] = 9999;
    n[i] = 9999;
    h[i] = 9999;
    m[i] = 9999;
    j[i] = 9999;
  }

  drawProgress(100, "Done...");
  tft.fillScreen(TFT_BLACK);

  drawInterfaceSkeleton();

  //Esp32_temp //
  initESPTemp();

  xTaskCreatePinnedToCore(
    WeatherTask,
    "WeatherTask",
    12000,
    NULL,
    1,
    &WeatherTaskHandle,
    0);

  booted = true;
}


void drawInterfaceSkeleton() {
  tft.drawLine(10, 163, 790, 163, 0x4228);
  tft.drawLine(310, 58, 310, 250, 0x4228);
  tft.drawLine(10, 250, 790, 250, 0x4228);
  tft.drawLine(10, 57, 320, 57, 0x4228);
  tft.drawLine(476, 57, 618, 57, 0x4228);
  tft.drawLine(440, 163, 440, 250, 0x4228);
  tft.drawLine(622, 163, 622, 250, 0x4228);
  tft.drawLine(619, 0, 619, 98, 0x4228);
  ui.drawBmp("/icon50/Altitude.bmp", 472, 4);
  //  ui.drawBmp("/icon50/cushin.bmp", 704, 72);
  ui.drawBmp("/icon50/calendar.bmp", 255, 178);
  ui.drawBmp("/icon50/sunrise.bmp", 22, 167);
  ui.drawBmp("/icon50/sunset.bmp", 22, 207);
  ui.drawBmp("/icon50/moonrise.bmp", 207, 169);
  ui.drawBmp("/icon50/moonset.bmp", 207, 209);
  ui.drawBmp("/icon50/cloud.bmp", 178, 69);
  ui.drawBmp("/icon50/uv.bmp", 259, 63);
  ui.drawBmp("/icon50/wind.bmp", 184, 120);
  ui.drawBmp("/icon50/rain.bmp", 266, 119);
  ui.drawBmp("/icon50/rain.bmp", 565, 68);

  char weatherBuf[64];
  char shortKey[6];
  char shortLocation[6];

  snprintf(shortKey, sizeof(shortKey), "%.5s", apiKey);
  snprintf(shortLocation, sizeof(shortLocation), "%.5s", locationKey);
  snprintf(weatherBuf, sizeof(weatherBuf), "Api:%s|location:%s|fw:%s", shortKey, shortLocation, FIRMWARE_VERSION);

  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("API:EN9AS|location:12.78|fw:0.0.2"));
  tft.drawString(weatherBuf, 624, 90);
  // Rain bar
  tft.setTextDatum(BR_DATUM);
  tft.setTextPadding(tft.textWidth("88"));
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("20", 516, 150);
  tft.drawString("40", 516, 130);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("60", 516, 110);
  tft.drawString("80", 516, 90);
  tft.drawString("100", 516, 70);
  tft.drawLine(518, 113, 518, 163, 0x4228);
  tft.drawLine(518, 63, 518, 113, TFT_RED);
  tft.drawLine(551, 113, 551, 163, 0x4228);
  tft.drawLine(551, 63, 551, 113, TFT_RED);

  // System
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Wifi"));
  tft.drawString("WiFi", 624, 0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("LoRa"));
  tft.drawString("LoRa", 624, 10);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Battery:"));
  tft.drawString("Battery", 624, 20);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Solar"));
  tft.drawString("Solar", 624, 30);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Heap"));
  tft.drawString("Heap", 624, 40);
  tft.setTextPadding(tft.textWidth("Min"));
  tft.drawString("Min", 624, 50);
  tft.setTextPadding(tft.textWidth("Block"));
  tft.drawString("Block", 624, 60);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Station Runtime"));
  tft.drawString("Station runtime", 624, 80);

  // Forecast & Outdoor & Indoor
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("Forecasts"));
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.drawString("FORECAST", 321, 169);
  tft.setTextPadding(tft.textWidth("OUTDOOR"));
  tft.drawString("OUTDOOR", 454, 169);
  tft.setTextPadding(tft.textWidth("INDOOR"));
  tft.drawString("INDOOR", 636, 169);

  // Wind km/h
  tft.loadFont(AA_FONT_13, LittleFS);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(tft.textWidth("km/h"));
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("km/h", CENTER_X, CENTER_Y + 50);

  tft.setTextPadding(0);
  tft.unloadFont();
}


// Esp32_temp
float getESPTemp() {
  float temp = 0;
  if (temperature_sensor_get_celsius(
        temp_handle,
        &temp)
      == ESP_OK) {

    return temp;
  }
  return -99;
}


void debugMemoryTFT(int baseY) {
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth("60000+"));
  tft.setTextColor(TFT_RED, TFT_BLACK);

  // =========================
  // HEAP
  // =========================

  uint32_t freeHeap = ESP.getFreeHeap();
  uint32_t minHeap = ESP.getMinFreeHeap();

  uint32_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  /*
  int barHeap = map(constrain(freeHeap, 40000, 120000), 40000, 120000, 0, 50);
  int barMin = map(constrain(minHeap, 20000, 120000), 20000, 120000, 0, 50);
  int barBlock = map(constrain(largestBlock, 10000, 80000), 10000, 80000, 0, 50);
*/
  int barHeap = map(
    constrain(freeHeap, 50000, 100000),
    50000, 100000,
    0, 50);

  int barMin = map(
    constrain(minHeap, 15000, 40000),
    15000, 40000,
    0, 50);

  int barBlock = map(
    constrain(largestBlock, 10000, 50000),
    10000, 50000,
    0, 50);
  // =========================
  // PRINT TFT
  // =========================

  char buf[64];

  snprintf(buf, sizeof(buf), "%.1fk", freeHeap / 1000.0);
  tft.drawString(buf, 748, baseY);

  snprintf(buf, sizeof(buf), "%.1fk", minHeap / 1000.0);
  tft.drawString(buf, 748, baseY + 10);

  snprintf(buf, sizeof(buf), "%.1fk", largestBlock / 1000.0);
  tft.drawString(buf, 748, baseY + 20);

  // Free Heap
  tft.fillRect(750, baseY, 50, 8, 0x4228);
  tft.fillRect(750, baseY, barHeap, 8, TFT_RED);

  // Min Heap
  tft.fillRect(750, baseY + 10, 50, 8, 0x4228);
  tft.fillRect(750, baseY + 10, barMin, 8, TFT_RED);

  // Largest Block
  tft.fillRect(750, baseY + 20, 50, 8, 0x4228);
  tft.fillRect(750, baseY + 20, barBlock, 8, TFT_RED);
}


/***************************************************************************************
**                          Loop
***************************************************************************************/
void loop() {
  time_t local_time = getLocalTimeSafe();
  int h = getHour(local_time);
  int m = getMinute(local_time);
  int s = getSecond(local_time);
  currentMillis = millis();

  // Cache status
  bool isAwake = digitalRead(SLEEP_PIN) == HIGH;
  bool wifiOK = WiFi.status() == WL_CONNECTED;

  // CheckButton
  checkButton(local_time);

  // SaveSDData && UpdateData for Graph && drawGraph
  if (m % 6 == 0 && s >= 1 && m != last_WriteSD) {
    last_WriteSD = m;
    tft.loadFont(AA_FONT_10, LittleFS);
    saveSDData(local_time);
    updateGraphWindRain();
    updateGraphTemp();
    updateGraphHum();
    if (currentPage == 0) {
      GraphWindRain();
      GraphTemp();
      GraphHum();
    }
    tft.setTextPadding(0);
    tft.unloadFont();
  }

  // Reset Value /h /d
  ResetValue(local_time);

  // Check Power for LightSleep
  if (currentMillis - lastCheckPower >= 1000) {
    lastCheckPower = currentMillis;
    if (!isAwake) enterLightSleep();
  }

  // Check WiFi connect()
  if (!wifiOK && currentMillis - connectMillis >= 600000) {
    connectMillis = currentMillis;
    WiFi.disconnect();
    WiFi.reconnect();
  }

  //==================================================
  // BLYNK
  //==================================================
  if (isAwake && wifiOK) {
    if (s == 0 && m != lastReconnectMinute) {
      lastReconnectMinute = m;
      blynkReconnectScheduled = true;
    }
    if (!blynkEnabled) {
      if (blynkReconnectScheduled) {
        blynkReconnectScheduled = false;
        if (Blynk.connect(5000)) {
          blynkEnabled = true;
          blynkLostMillis = 0;
        }
      }
    } else {
      Blynk.run();
      if (Blynk.connected()) {
        blynkLostMillis = 0;
      } else {
        if (blynkLostMillis == 0)
          blynkLostMillis = currentMillis;
        if (currentMillis - blynkLostMillis >= 5000) {
          Blynk.disconnect();
          blynkEnabled = false;
          blynkLostMillis = 0;
          // Draw Blynk Error //
          tft.loadFont(AA_FONT_10, LittleFS);
          tft.setTextDatum(TL_DATUM);
          tft.setTextColor(TFT_BLACK, TFT_RED);
          tft.setTextPadding(tft.textWidth("BLYNK"));
          tft.drawString("BLYNK", 654, (currentPage == 0) ? 70 : 0);
          tft.unloadFont();
        }
      }
    }
  } else {
    if (blynkEnabled || Blynk.connected()) {
      Blynk.disconnect();
      blynkEnabled = false;
      blynkLostMillis = 0;
    }
  }

  // Get LoRa Data
  if (currentMillis - LoRaMillis >= 50) {
    LoRaMillis = currentMillis;
    GetLoRa();
  }

  // drawData
  if (newLoRaData) {
    if (currentPage == 0) drawData(local_time);
    if (blynkEnabled) updateBlynk();
    newLoRaData = false;
  }

  // sendWindyDone
  if (windyResultPending) {
    sendWindyDone();
  }

  // sendWUDone
  if (WUResultPending) {
    sendWUDone();
  }

  // drawCurrentWeaher
  if (weatherReady) {
    if (currentPage == 0) drawCurrentWeather(local_time);
    weatherReady = false;
  }

  // drawAstronomy
  if (astronomyReady) {
    if (currentPage == 0) drawAstronomy(local_time);
    astronomyReady = false;
  }

  // drawTime
  if (currentPage == 0) {
    // drawTime //
    if (booted || m != lastMinute) {
      drawTime(local_time);
      lastMinute = m;
    }
  }

  // syncTime //
  if (m == 15 && s >= 1 && h != lastSyncHour) {
    lastSyncHour = h;
    syncTimeESP32();
  }

  // Auto Backlight
  if (millis() - lastBacklightCheck >= 5000) {
    lastBacklightCheck = millis();
    updateBacklight();
  }

  booted = false;
}


void enterLightSleep() {
  ledcWrite(LEDpin, 0);
  currentBacklightPWM = -1;
  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);

  esp_sleep_enable_ext0_wakeup((gpio_num_t)SLEEP_PIN, 1);

  while (true) {
    esp_light_sleep_start();
    uint32_t stableStart = millis();
    bool confirmed = false;
    while (digitalRead(SLEEP_PIN) == HIGH) {
      if (millis() - stableStart > 2000) {
        confirmed = true;
        break;
      }
      delay(10);
    }
    if (confirmed) break;
  }
  delay(200);
  updateBacklight();
  connectWiFi();
}


void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  uint32_t timeout = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - timeout < 10000)) {
    delay(500);
  }
}


// === OTA CHECK ===
void checkFirmwareUpdate() {
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Check firmware...", 0, 120);

  HTTPClient http;
  client.setInsecure();

  char apiUrl[256];
  snprintf(apiUrl, sizeof(apiUrl), "https://api.github.com/repos/%s/%s/releases/latest", GITHUB_USER, GITHUB_REPO);
  http.begin(client, apiUrl);
  http.addHeader("User-Agent", "ESP32-OTA-Agent");

  int httpCode = http.GET();
  char msgBuf[128];

  if (httpCode == 200) {
    String json = http.getString();
    int tagStart = json.indexOf("\"tag_name\":\"") + 12;
    int tagEnd = json.indexOf("\"", tagStart);

    char latestVersion[32];
    json.substring(tagStart, tagEnd).toCharArray(latestVersion, sizeof(latestVersion));

    snprintf(msgBuf, sizeof(msgBuf), "Firmware current: %s", FIRMWARE_VERSION);
    tft.drawString(msgBuf, 0, 140);
    snprintf(msgBuf, sizeof(msgBuf), "Firmware latest: %s", latestVersion);
    tft.drawString(msgBuf, 0, 160);
    delay(3000);

    if (strcmp(latestVersion, FIRMWARE_VERSION) != 0) {
      tft.fillRect(0, 200, 800, 200, TFT_BLACK);
      tft.drawString("New firmware detected - starting OTA update...", 0, 180);

      char binUrl[256];
      snprintf(binUrl, sizeof(binUrl), "https://github.com/%s/%s/releases/download/%s/%s", GITHUB_USER, GITHUB_REPO, latestVersion, FIRMWARE_FILENAME);

      snprintf(msgBuf, sizeof(msgBuf), "File URL: %s", binUrl);
      tft.drawString(msgBuf, 0, 200);

      HTTPClient https;
      https.begin(client, binUrl);
      https.addHeader("User-Agent", "ESP32-OTA-Agent");
      https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

      int code = https.GET();
      tft.fillRect(0, 200, 240, 120, TFT_BLACK);

      snprintf(msgBuf, sizeof(msgBuf), "HTTP code: %d", code);
      tft.drawString(msgBuf, 0, 220);

      if (code == 200) {
        int len = https.getSize();
        WiFiClient* stream = https.getStreamPtr();

        if (Update.begin(len)) {
          tft.setTextColor(TFT_GREEN, TFT_BLACK);
          tft.drawString("Downloading firmware...", 0, 240);

          uint8_t buff[512];
          int written = 0;
          int lastPercent = -1;

          while (https.connected() && (written < len || len == -1)) {
            size_t sizeAvailable = stream->available();
            if (sizeAvailable) {
              int c = stream->readBytes(buff, ((sizeAvailable > sizeof(buff)) ? sizeof(buff) : sizeAvailable));
              Update.write(buff, c);
              written += c;

              int percent = (len > 0) ? (written * 100 / len) : 0;
              if (percent != lastPercent) {
                lastPercent = percent;
                char pctBuf[16];
                snprintf(pctBuf, sizeof(pctBuf), "%d%%", percent);
                drawProgress(percent, pctBuf);
              }
            }
            delay(1);
          }
          tft.setTextDatum(BL_DATUM);
          if (Update.end()) {
            if (Update.isFinished()) {
              tft.loadFont(AA_FONT_SMALL, LittleFS);
              tft.setTextColor(TFT_CYAN, TFT_BLACK);
              tft.drawString("Update complete -> rebooting...", 0, 260);
              delay(2000);
              ESP.restart();
            } else {
              tft.drawString("Update incomplete...", 0, 260);
              delay(2000);
            }
          } else {
            tft.drawString("Update error...", 0, 260);
            delay(2000);
          }
        } else {
          tft.drawString("Can't update...", 0, 240);
          delay(2000);
        }
      } else {
        snprintf(msgBuf, sizeof(msgBuf), "Fail load file... HTTP %d", code);
        tft.drawString(msgBuf, 0, 220);
        delay(3000);
      }
      https.end();
    } else {
      tft.fillRect(0, 200, 240, 320 - 200, TFT_BLACK);
      tft.drawString("The latest version", 0, 180);
      delay(2000);
    }
  } else {
    snprintf(msgBuf, sizeof(msgBuf), "Can't check firmware: HTTP %d", httpCode);
    tft.drawString(msgBuf, 0, 120);
    delay(2000);
  }
  http.end();
}


void checkButton(time_t local_time) {
  int h = getHour(local_time);
  int m = getMinute(local_time);
  int s = getSecond(local_time);
  // =========================
  // BUTTON
  // =========================
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonState) {
    lastDebounceTime = currentMillis;
  }

  if ((currentMillis - lastDebounceTime) > debounceDelay) {

    if (reading != buttonState) {
      buttonState = reading;

      if (buttonState == LOW) {

        currentPage = !currentPage;

        tft.fillScreen(TFT_BLACK);

        if (currentPage == 0) {
          drawInterfaceSkeleton();
          drawTime(local_time);
          drawData(local_time);
          drawAstronomy(local_time);
          drawCurrentWeather(local_time);
          tft.loadFont(AA_FONT_10, LittleFS);
          GraphWindRain();
          GraphTemp();
          GraphHum();
          debugMemoryTFT(40);

          tft.setTextPadding(0);
          tft.unloadFont();
        } else {
          readSDData(local_time);
          tft.loadFont(AA_FONT_10, LittleFS);
          debugMemoryTFT(0);
          GraphWindRainSD();
          GraphTempSD();
          GraphHumSD();
          espTemp(30);

          tft.setTextPadding(0);
          tft.unloadFont();
        }
      }
    }
  }
  lastButtonState = reading;
}


void GetLoRa() {
  if (e220ttl.available() > 1) {
    tft.loadFont(AA_FONT_10, LittleFS);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_BLACK, TFT_CYAN);
    tft.setTextPadding(tft.textWidth("LORA"));
    tft.drawString("LORA", 624, (currentPage == 0) ? 70 : 0);

    ResponseContainer rc = e220ttl.receiveMessageRSSI();
    rssi = rc.rssi;
    dbm = -(256 - rssi);

    StaticJsonDocument<250> doc;
    DeserializationError error = deserializeJson(doc, rc.data);

    if (!error) {
      float xTempOutDoor = doc["temperature"];
      if (!isnan(xTempOutDoor) && xTempOutDoor > -20.0 && xTempOutDoor < 80.0) {
        TempOutDoor = xTempOutDoor;
      }

      float xHumiOutDoor = doc["humidity"];
      if (!isnan(xHumiOutDoor) && xHumiOutDoor > 0 && xHumiOutDoor <= 100.0) {
        HumiOutDoor = xHumiOutDoor;
      }

      winddy = doc["angle"].as<float>();
      fWindSpeed = doc["speed"].as<float>();
      RainGauge = doc["rain"].as<float>();
      fvalueBat = doc["battery"].as<float>();
      fvalueSol = doc["solar"].as<float>();
      fTimeConnect = doc["timerun"].as<float>();
      Altitude = doc["altitude"].as<float>();
      Pressurebme = doc["pressure"].as<float>();
      Pressurecal = Pressurebme + PRESSURE_OFFSET;

      if (fWindSpeed > maxWindHour) maxWindHour = fWindSpeed;
      mmTotal += RainGauge;
      mmDaily += RainGauge;
      mmGraph += RainGauge;
      mmHourly += RainGauge;

      newLoRaData = true;
    }
    tft.unloadFont();
    tft.fillRect(624, (currentPage == 0) ? 70 : 0, 28, 11, TFT_BLACK);
  }
}


void drawData(time_t local_time) {
  drawAngle();
  drawWindSpeed();
  drawRain();
  drawSignal();
  //  Get Temp/Hum IN
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  float xTempInDoor = temp.temperature;
  if (!isnan(xTempInDoor) && xTempInDoor > 0 && xTempInDoor < 80.0) {
    TempInDoor = xTempInDoor;
  }
  float xHumiInDoor = humidity.relative_humidity;
  if (!isnan(xHumiInDoor) && xHumiInDoor > 0) {
    HumiInDoor = xHumiInDoor;
  }
  drawTemHumOutdoor();
  drawTemHumIndoor();
  espTemp(100);
}


void drawAngle() {
  if (lastArrowHeadX1 != CENTER_X || lastArrowHeadY1 != CENTER_Y) {
    tft.fillTriangle(lastArrowHeadX1, lastArrowHeadY1, lastArrowHeadX2, lastArrowHeadY2, lastArrowTailX, lastArrowTailY, TFT_BLACK);
  }

  float radian = (winddy + 270) * DEG_TO_RAD;
  int arrowTipX = CENTER_X + cos(radian) * (RADIUS - 10);
  int arrowTipY = CENTER_Y + sin(radian) * (RADIUS - 10);
  int arrowTailX = CENTER_X + cos(radian) * (RADIUS + 10);
  int arrowTailY = CENTER_Y + sin(radian) * (RADIUS + 10);

  float arrowAngle = atan2(arrowTailY - arrowTipY, arrowTailX - arrowTipX);
  float arrowHeadAngle1 = arrowAngle + PI / 6;
  float arrowHeadAngle2 = arrowAngle - PI / 6;

  int arrowHeadX1 = arrowTailX + cos(arrowHeadAngle1) * 20;
  int arrowHeadY1 = arrowTailY + sin(arrowHeadAngle1) * 20;
  int arrowHeadX2 = arrowTailX + cos(arrowHeadAngle2) * 20;
  int arrowHeadY2 = arrowTailY + sin(arrowHeadAngle2) * 20;

  for (int r = 15; r <= 20; r++) {
    tft.drawCircle(CENTER_X, CENTER_Y, RADIUS + r, TFT_DARKGREY);
  }

  tft.fillTriangle(arrowTailX, arrowTailY, arrowHeadX1, arrowHeadY1, arrowHeadX2, arrowHeadY2, TFT_RED);

  lastArrowHeadX1 = arrowHeadX1;
  lastArrowHeadY1 = arrowHeadY1;
  lastArrowHeadX2 = arrowHeadX2;
  lastArrowHeadY2 = arrowHeadY2;
  lastArrowTailX = arrowTailX;
  lastArrowTailY = arrowTailY;

  tft.loadFont(AA_FONT_30, LittleFS);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("8888"));

  char windBuf[16];
  snprintf(windBuf, sizeof(windBuf), "%.0f°", winddy);
  tft.drawString(windBuf, CENTER_X, CENTER_Y - 30);

  tft.setTextPadding(0);
  tft.unloadFont();
}


void drawWindSpeed() {
  char windBuf[32];

  tft.loadFont(AA_FONT_LARGE, LittleFS);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("888.8"));

  snprintf(windBuf, sizeof(windBuf), "%.1f", fWindSpeed);
  tft.drawString(windBuf, CENTER_X, CENTER_Y + 3);
  tft.unloadFont();

  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextPadding(tft.textWidth("888.8 max/h "));

  snprintf(windBuf, sizeof(windBuf), "%.1f max/h ", maxWindHour);
  tft.drawString(windBuf, CENTER_X, CENTER_Y + 30);

  tft.setTextPadding(0);
  tft.unloadFont();
}


void drawRain() {
  if (mmGraph > 99.75) mmGraph = 0;

  tft.fillRect(560, 131, 3, 11, TFT_CYAN);

  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  char rainBuf[32];

  // (Hourly)
  tft.setTextPadding(tft.textWidth("888.88mm/d"));
  snprintf(rainBuf, sizeof(rainBuf), "%.2fmm/h", mmHourly);
  tft.drawString(rainBuf, 566, 114);

  // (Daily)
  snprintf(rainBuf, sizeof(rainBuf), "%.2fmm/d", mmDaily);
  tft.drawString(rainBuf, 566, 130);

  // (Total)
  tft.setTextPadding(tft.textWidth("88888.88 mm"));
  snprintf(rainBuf, sizeof(rainBuf), "%.2fmm", mmTotal);
  tft.drawString(rainBuf, 566, 146);

  tft.setTextPadding(0);
  tft.unloadFont();

  tft.fillRect(520, 63, 30, 100, 0x4228);
  tft.fillRect(520, 163 - (int)mmGraph, 30, (int)mmGraph, 0x07FF);

  for (int j = 0; j < 110; j = j + 10) {
    tft.drawLine(520, 163 - j, 549, 163 - j, TFT_BLACK);
  }
}


void drawSignal() {
  WiFiRSSI = WiFi.RSSI();

  if (WiFi.status() != WL_CONNECTED) {
    valueWiFiStrong = 0;
    WiFiRSSI = -120;
  } else {
    valueWiFiStrong = map(WiFiRSSI, -35, -120, 50, 0);
    valueWiFiStrong = constrain(valueWiFiStrong, 0, 50);
  }

  valueLoRaStrong = map(dbm, -35, -120, 50, 0);
  valueLoRaStrong = constrain(valueLoRaStrong, 0, 50);

  // LoRa Station Battery
  gBat = fvalueBat * 1000;
  valueBattery = map(gBat, 2500, 4200, 0, 50);
  valueBattery = constrain(valueBattery, 0, 50);

  // Battery of Display
  float analogVolts = (analogReadMilliVolts(BATTERY_PIN) * 1.556);
  valueBattery1 = map(analogVolts, 2500, 4200, 0, 50);
  valueBattery1 = constrain(valueBattery1, 0, 50);

  // Solar Display
  gSol = fvalueSol * 100;
  valueSolar = map(gSol, 0, 2000, 0, 50);
  valueSolar = constrain(valueSolar, 0, 50);

  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TR_DATUM);

  char sigBuf[32];

  // --- WiFi ---
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("-888dBm"));
  snprintf(sigBuf, sizeof(sigBuf), "%ddBm", WiFiRSSI);
  tft.drawString(sigBuf, 748, 0);

  tft.fillRect(750, 0, 50, 8, 0x4228);
  tft.fillRect(750, 0, valueWiFiStrong, 8, 0xFFFF);

  // --- LoRa ---
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("-888dBm"));
  snprintf(sigBuf, sizeof(sigBuf), "%ddBm", dbm);
  tft.drawString(sigBuf, 748, 10);

  tft.fillRect(750, 10, 50, 8, 0x4228);
  tft.fillRect(750, 10, valueLoRaStrong, 8, 0x07FF);

  // --- Battery ---
  tft.fillRect(750, 20, 50, 8, 0x4228);
  tft.fillRect(750, 20, valueBattery, 4, 0x07E0);
  tft.fillRect(750, 24, valueBattery1, 4, TFT_RED);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("20.00/4.3V"));
  snprintf(sigBuf, sizeof(sigBuf), "%.2f|%.2fv", fvalueBat, (analogVolts / 1000.0));
  tft.drawString(sigBuf, 748, 20);

  // --- Solar ---
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("20.00/20V"));
  snprintf(sigBuf, sizeof(sigBuf), "%.2f/20v", fvalueSol);
  tft.drawString(sigBuf, 748, 30);

  tft.fillRect(750, 30, 50, 8, 0x4228);
  tft.fillRect(750, 30, valueSolar, 8, 0xFFE0);

  // --- (Time Connect) ---
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("4,294,967.295s"));
  snprintf(sigBuf, sizeof(sigBuf), "%.0fs", fTimeConnect);
  tft.drawString(sigBuf, 800, 80);

  tft.setTextPadding(0);
  tft.unloadFont();
}


void drawTemHumOutdoor() {
  if (TempOutDoor < minTempOutDoor) minTempOutDoor = TempOutDoor;
  if (TempOutDoor > maxTempOutDoor) maxTempOutDoor = TempOutDoor;
  if (HumiOutDoor < minHumiOutDoor) minHumiOutDoor = HumiOutDoor;
  if (HumiOutDoor > maxHumiOutDoor) maxHumiOutDoor = HumiOutDoor;

  char thBuf[16];

  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("88.88°"));

  snprintf(thBuf, sizeof(thBuf), "%.2f°", TempOutDoor);
  tft.drawString(thBuf, 455, 186);
  tft.unloadFont();

  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextPadding(tft.textWidth("100.0%"));

  snprintf(thBuf, sizeof(thBuf), "%.1f%%", HumiOutDoor);
  tft.drawString(thBuf, 455, 228);
  tft.unloadFont();

  tft.loadFont(AA_FONT_13, LittleFS);
  tft.setTextDatum(TL_DATUM);

  // ---Min/Max Temperatrue ---
  tft.setTextPadding(tft.textWidth("88.88"));
  snprintf(thBuf, sizeof(thBuf), "%.2f", maxTempOutDoor);
  tft.drawString(thBuf, 585, 188);

  snprintf(thBuf, sizeof(thBuf), "%.2f", minTempOutDoor);
  tft.drawString(thBuf, 585, 204);

  // --- Min/Max Humidity ---
  tft.setTextPadding(tft.textWidth("100.0"));
  snprintf(thBuf, sizeof(thBuf), "%.1f", maxHumiOutDoor);
  tft.drawString(thBuf, 585, 222);

  snprintf(thBuf, sizeof(thBuf), "%.1f", minHumiOutDoor);
  tft.drawString(thBuf, 585, 234);

  tft.setTextPadding(0);
  tft.unloadFont();
}


void drawTemHumIndoor() {
  if (TempInDoor < minTempInDoor) minTempInDoor = TempInDoor;
  if (TempInDoor > maxTempInDoor) maxTempInDoor = TempInDoor;
  if (HumiInDoor < minHumiInDoor) minHumiInDoor = HumiInDoor;
  if (HumiInDoor > maxHumiInDoor) maxHumiInDoor = HumiInDoor;

  char inBuf[16];

  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("88.88°"));
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  snprintf(inBuf, sizeof(inBuf), "%.2f°", TempInDoor);
  tft.drawString(inBuf, 637, 186);
  tft.unloadFont();

  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("100.0%"));

  snprintf(inBuf, sizeof(inBuf), "%.1f%%", HumiInDoor);
  tft.drawString(inBuf, 637, 228);
  tft.unloadFont();

  tft.loadFont(AA_FONT_13, LittleFS);
  tft.setTextDatum(TL_DATUM);

  // --- Min/Max Temperature ---
  tft.setTextPadding(tft.textWidth("88.88"));
  snprintf(inBuf, sizeof(inBuf), "%.2f", maxTempInDoor);
  tft.drawString(inBuf, 767, 188);

  snprintf(inBuf, sizeof(inBuf), "%.2f", minTempInDoor);
  tft.drawString(inBuf, 767, 204);

  // --- Min/Max Humidity ---
  tft.setTextPadding(tft.textWidth("100.0"));
  snprintf(inBuf, sizeof(inBuf), "%.1f", maxHumiInDoor);
  tft.drawString(inBuf, 767, 222);

  snprintf(inBuf, sizeof(inBuf), "%.1f", minHumiInDoor);
  tft.drawString(inBuf, 767, 234);

  // Pressure
  tft.setTextPadding(tft.textWidth("8888.8hPa"));
  snprintf(inBuf, sizeof(inBuf), "%.1fhPa", Pressurecal);
  tft.drawString(inBuf, 542, 29);

  // Altitude
  snprintf(inBuf, sizeof(inBuf), "%.1fm", Altitude);
  tft.drawString(inBuf, 542, 41);

  tft.setTextPadding(0);
  tft.unloadFont();
}


// Esp32_temp //
void espTemp(int baseY) {
  char buf[8];
  snprintf(
    buf,
    sizeof(buf),
    "%d'C",
    (int)round(getESPTemp()));
  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("100'C"));
  tft.drawString(buf, 800, baseY);

  tft.setTextPadding(0);
  tft.unloadFont();
}
/*
  75,5/ 82,5
COLOR CODE ~~~ https://www.computerhope.com/htmcolor.htm
#define BLACK 0x0000
#define NAVY 0x000F
#define DARKGREEN 0x03E0
#define DARKCYAN 0x03EF
#define MAROON 0x7800
#define PURPLE 0x780F
#define OLIVE 0x7BE0
#define LIGHTGREY 0xC618
#define DARKGREY 0x7BEF
#define BLUE 0x001F
#define GREEN 0x07E0
#define CYAN 0x07FF
#define RED 0xF800
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF
#define ORANGE 0xFD20
#define GREENYELLOW 0xAFE5
#define PINK 0xF81F
*/


void ResetValue(time_t local_time) {
  int d = getDay(local_time);
  int h = getHour(local_time);
  int m = getMinute(local_time);
  int s = getSecond(local_time);

  // --- 1. RESET Hour
  static int lastResetHour = -1;
  if (m == 0 && s >= 11 && h != lastResetHour) {
    lastResetHour = h;
    mmHourly = 0;
    maxWindHour = 0;
  }

  // --- 2. RESET Day
  static int lastResetDay = -1;
  if (h == 0 && m == 0 && s >= 12 && d != lastResetDay) {
    lastResetDay = d;

    // Reset Rain
    mmDaily = 0;
    mmGraph = 0;

    // Reset Outdoor
    maxTempOutDoor = 0;
    minTempOutDoor = 99;
    maxHumiOutDoor = 0;
    minHumiOutDoor = 100;

    // Reset Indoor
    maxTempInDoor = 0;
    minTempInDoor = 99;
    maxHumiInDoor = 0;
    minHumiInDoor = 100;
  }
}


void updateGraphWindRain() {
  graphRain = map(mmGraph, 0, 100, 350, 250);
  graphWinddy = map(winddy, 0, 360, 310, 270);
  graphWindMax = maxWindHour;
  if (graphWindMax > 200) graphWindMax = 200;
  graphWindSpeed = map(graphWindMax, 0, 200, 350, 250);
  // Move left
  for (int i = 0; i < 799; i++) {
    b[i] = b[i + 1];
    z[i] = z[i + 1];
    x[i] = x[i + 1];
  }
  // New Point
  b[799] = graphRain;
  z[799] = graphWinddy;
  x[799] = graphWindSpeed;
}


void updateGraphTemp() {
  graphTemOUT = map(TempOutDoor, 0, 50, 410, 360);
  graphTemIN = map(TempInDoor, 0, 50, 410, 360);
  // Move left
  for (int i = 0; i < 799; i++) {
    n[i] = n[i + 1];
    m[i] = m[i + 1];
  }
  // New Point
  n[799] = graphTemOUT;
  m[799] = graphTemIN;
}


void updateGraphHum() {
  graphHumOUT = map(HumiOutDoor, 0, 100, 470, 420);
  graphHumIN = map(HumiInDoor, 0, 100, 470, 420);
  // Move left
  for (int i = 0; i < 799; i++) {
    c[i] = c[i + 1];
    v[i] = v[i + 1];
  }
  // New Point
  c[799] = graphHumOUT;
  v[799] = graphHumIN;
}


void GraphWindRain() {
  tft.fillRect(0, 250, 800, 100, TFT_BLACK);
  //  Graph Line
  for (int m = 0; m < 110; m = m + 10) {
    tft.drawLine(0, 350 - m, 800, 350 - m, 0x3186);  //0x4228
  }
  for (int l = 0; l < 800; l = l + 10) {
    tft.drawLine(0 + l, 250, 0 + l, 350, 0x3186);
  }
  //Red line through 3 Graph
  for (int l = 0; l < 700; l = l + 60) {
    tft.drawLine(80 + l, 250, 80 + l, 350, 0x4228);
  }
  tft.drawLine(20, 270, 800, 270, 0x4228);
  tft.drawLine(20, 310, 800, 310, 0x4228);
  tft.setTextDatum(BL_DATUM);
  tft.setTextPadding(tft.textWidth("8"));
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Wind & Rain", 0, 262);
  tft.drawString("Speed", 95, 262);
  tft.fillRect(80, 254, 10, 3, TFT_RED);
  tft.drawString("Direct", 195, 262);
  tft.fillRect(180, 254, 10, 3, TFT_OLIVE);
  tft.drawString("Rain", 295, 262);
  tft.fillRect(280, 254, 10, 3, TFT_CYAN);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  //  tft.drawString("0", 0, 337);
  tft.drawString("10", 0, 347);
  tft.drawString("20", 0, 337);
  tft.drawString("30", 0, 327);
  tft.drawString("40", 0, 317);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("50", 0, 307);
  tft.drawString("60", 0, 297);
  tft.drawString("70", 0, 287);
  tft.drawString("80", 0, 277);
  //  N E S W N
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_OLIVE, TFT_BLACK);  //#define TFT_DARKRED 0x8800
  tft.drawString("N", 20, 277);
  tft.drawString("W", 20, 287);
  tft.drawString("S", 20, 297);
  tft.drawString("E", 20, 307);
  tft.drawString("N", 20, 317);
  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("180km/h", 800, 267);
  tft.setTextColor(0x4228, TFT_BLACK);
  tft.drawString("160km/h", 800, 277);
  tft.drawString("140km/h", 800, 287);
  tft.drawString("120km/h", 800, 297);
  tft.drawString("100km/h", 800, 307);
  tft.drawString("80km/h", 800, 317);
  tft.drawString("60km/h", 800, 327);
  tft.drawString("40km/h", 800, 337);
  tft.drawString("20km/h", 800, 347);

  for (int k = 799; k >= 0; k--) {
    tft.drawLine(k, b[k], k - 1, b[k - 1], TFT_CYAN);   // Rain gauge/day
    tft.drawLine(k, z[k], k - 1, z[k - 1], TFT_OLIVE);  // Wind Direction
    tft.drawLine(k, x[k], k - 1, x[k - 1], TFT_RED);    // Wind Gust
  }
  tft.fillRect(0, 351, 800, 150, TFT_BLACK);
}


void GraphTemp() {
  tft.fillRect(0, 360, 800, 51, TFT_BLACK);
  //  Graph Line
  for (int m = 0; m < 60; m = m + 10) {
    tft.drawLine(0, 410 - m, 800, 410 - m, 0x3186);  //0x4228
  }
  for (int l = 0; l < 800; l = l + 10) {
    tft.drawLine(0 + l, 360, 0 + l, 410, 0x3186);
  }
  //Red line through 3 Graph
  for (int l = 0; l < 700; l = l + 60) {
    tft.drawLine(80 + l, 360, 80 + l, 410, 0x4228);
  }
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Temperature", 0, 367);
  tft.drawString("Out", 95, 367);
  tft.fillRect(80, 359, 10, 3, TFT_ORANGE);
  tft.drawString("In", 195, 367);
  tft.fillRect(180, 359, 10, 3, TFT_CYAN);
  //  tft.setTextDatum(BL_DATUM);
  tft.setTextPadding(tft.textWidth("8"));
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  //  tft.drawString("0", 0, 407);
  tft.drawString("10", 0, 407);
  tft.drawString("20", 0, 397);
  tft.drawString("30", 0, 387);
  tft.drawString("40", 0, 377);
  //  tft.drawString("50", 0, 357);
  for (int k = 799; k >= 0; k--) {
    tft.drawLine(k, n[k], k - 1, n[k - 1], TFT_ORANGE);  //  humidity outdoor
    tft.drawLine(k, m[k], k - 1, m[k - 1], TFT_CYAN);    //  Temperature outdoor
  }
  tft.fillRect(0, 411, 800, 79, TFT_BLACK);
}


void GraphHum() {
  tft.fillRect(0, 420, 800, 50, TFT_BLACK);
  //  Graph Line
  for (int m = 0; m < 60; m = m + 10) {
    tft.drawLine(0, 470 - m, 800, 470 - m, 0x3186);  //0x4228
  }
  for (int l = 0; l < 800; l = l + 10) {
    tft.drawLine(0 + l, 420, 0 + l, 470, 0x3186);
  }
  //Red line through 3 Graph
  for (int l = 0; l < 700; l = l + 60) {
    tft.drawLine(80 + l, 420, 80 + l, 470, 0x4228);
  }
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Humidity", 0, 427);
  tft.drawString("Out", 95, 427);
  tft.fillRect(80, 419, 10, 3, TFT_ORANGE);
  tft.drawString("In", 195, 427);
  tft.fillRect(180, 419, 10, 3, TFT_CYAN);
  tft.setTextDatum(BL_DATUM);
  tft.setTextPadding(tft.textWidth("8"));
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  //  tft.drawString("0", 0, 477);
  tft.drawString("20", 0, 467);
  tft.drawString("40", 0, 457);
  tft.drawString("60", 0, 447);
  tft.drawString("80", 0, 437);
  //  tft.drawString("100", 0, 427);
  for (int k = 799; k >= 0; k--) {
    tft.drawLine(k, c[k], k - 1, c[k - 1], TFT_ORANGE);  //  humidity outdoor
    tft.drawLine(k, v[k], k - 1, v[k - 1], TFT_CYAN);    //  Temperature outdoor
  }
  tft.fillRect(0, 471, 800, 9, TFT_BLACK);
  tft.setTextDatum(BC_DATUM);
  tft.drawString("72", 80, 482);
  tft.drawString("66", 140, 482);
  tft.drawString("60", 200, 482);
  tft.drawString("54", 260, 482);
  tft.drawString("48", 320, 482);
  tft.drawString("42", 380, 482);
  tft.drawString("36", 440, 482);
  tft.drawString("30", 500, 482);
  tft.drawString("24", 560, 482);
  tft.drawString("18", 620, 482);
  tft.drawString("12", 680, 482);
  tft.drawString("6", 740, 482);
  tft.drawString("0", 800, 482);
}


void updateBlynk() {
  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_BLACK, TFT_GREENYELLOW);
  tft.setTextPadding(tft.textWidth("BLYNK"));
  tft.drawString("BLYNK", 654, (currentPage == 0) ? 70 : 0);
  tft.unloadFont();
  Blynk.virtualWrite(V8, TempInDoor);
  Blynk.virtualWrite(V9, HumiInDoor);
  Blynk.virtualWrite(V10, TempOutDoor);
  Blynk.virtualWrite(V11, HumiOutDoor);
  Blynk.virtualWrite(V12, winddy);
  Blynk.virtualWrite(V13, fWindSpeed);
  Blynk.virtualWrite(V14, mmDaily);
  Blynk.virtualWrite(V15, maxWindHour);
  Blynk.virtualWrite(V16, fvalueBat);
  Blynk.virtualWrite(V17, fvalueSol);
  tft.fillRect(654, (currentPage == 0) ? 70 : 0, 36, 11, TFT_BLACK);
}


void sendWindy() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (windyStationID[0] == '\0') return;
  if (windyPassword[0] == '\0') return;

  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;

  char url[320];

  snprintf(
    url,
    sizeof(url),

    "https://stations.windy.com/api/v2/observation/update"
    "?id=%s"
    "&PASSWORD=%s"
    "&temp=%.1f"
    "&humidity=%.0f"
    "&mbar=%.1f"
    "&wind=%.2f"
    "&gust=%.2f"
    "&winddir=%.0f"
    "&precip=%.2f"
    "&softwaretype=ESP32-S3"
    "&stationtype=DIY",

    windyStationID,
    windyPassword,

    TempOutDoor,
    HumiOutDoor,
    Pressurecal,

    (fWindSpeed / 3.6f),   // km/h -> m/s
    (maxWindHour / 3.6f),  // km/h -> m/s

    winddy,

    mmHourly);

  http.setConnectTimeout(3000);
  http.setTimeout(5000);

  if (!http.begin(client, url)) {
    //Serial.println(F("[Windy] Begin Failed"));
    return;
  }
  int httpCode = http.GET();

  windyLastHttpCode = httpCode;
  windyResultPending = true;

  http.end();
}


void sendWindyDone() {
  windyResultPending = false;
  int code = windyLastHttpCode;

  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("WD"));
  const int yLocal = (currentPage == 0) ? 70 : 30;
  const int xLocal = (currentPage == 0) ? 690 : 672;

  switch (code) {
    case HTTP_CODE_OK:
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
      tft.drawString("WD", xLocal, yLocal);
      break;

    case 400:
    case 401:
    case 429:
      tft.setTextColor(TFT_BLACK, TFT_RED);
      tft.drawString("WD", xLocal, yLocal);
      break;

    case 409:
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
      tft.drawString("WD", xLocal, yLocal);
      break;

    default:
      char codeBuf[8];
      snprintf(codeBuf, sizeof(codeBuf), "%d", code);
      tft.setTextColor(TFT_BLACK, TFT_RED);
      tft.drawString(codeBuf, xLocal, yLocal);
      break;
  }

  tft.setTextPadding(0);
  tft.unloadFont();
}


void sendWU() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (wuStationID[0] == '\0') {
    return;
  }

  if (wuStationKey[0] == '\0') {
    return;
  }

  WiFiClientSecure client;  // <-- đổi từ WiFiClient
  client.setInsecure();     // <-- bỏ qua verify cert, đơn giản & đủ dùng

  HTTPClient http;

  char url[420];

  snprintf(
    url,
    sizeof(url),
    "https://weatherstation.wunderground.com/weatherstation/updateweatherstation.php"
    "?ID=%s"
    "&PASSWORD=%s"
    "&dateutc=now"
    "&winddir=%.0f"
    "&windspeedmph=%.2f"
    "&windgustmph=%.2f"
    "&humidity=%.0f"
    "&tempf=%.2f"
    "&baromin=%.3f"
    "&rainin=%.3f"
    "&dailyrainin=%.3f"
    "&softwaretype=ESP32-S3"
    "&action=updateraw",
    wuStationID,
    wuStationKey,
    winddy,
    fWindSpeed * 0.621371f,
    maxWindHour * 0.621371f,
    HumiOutDoor,
    TempOutDoor * 9.0f / 5.0f + 32.0f,
    Pressurecal * 0.029529983f,
    mmHourly * 0.0393701f,
    mmDaily * 0.0393701f);

  http.setConnectTimeout(3000);
  http.setTimeout(5000);

  if (!http.begin(client, url)) {
    return;
  }

  int httpCode = http.GET();

  WULastHttpCode = httpCode;

  if (httpCode > 0) {
    WUResponse = http.getString();
    WUResponse.trim();
  } else {
    WUResponse = "";
  }

  WUResultPending = true;

  http.end();
}


void sendWUDone() {
  WUResultPending = false;
  int code = WULastHttpCode;

  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("WU"));

  const int yLocal = (currentPage == 0) ? 70 : 30;
  const int xLocal = (currentPage == 0) ? 709 : 691;

  if (code == HTTP_CODE_OK) {

    if (WUResponse.startsWith("success")) {
      tft.setTextColor(TFT_BLACK, TFT_GREEN);
      tft.drawString("WU", xLocal, yLocal);
      //Serial.println(F("[WU] Upload OK"));
    } else {
      tft.setTextColor(TFT_BLACK, TFT_RED);
      tft.drawString("WU", xLocal, yLocal);
      //Serial.printf("[WU] %s\n", WUResponse.c_str());
    }

  } else {

    char codeBuf[8];
    snprintf(codeBuf, sizeof(codeBuf), "%d", code);

    tft.setTextColor(TFT_BLACK, TFT_RED);
    tft.drawString(codeBuf, xLocal, yLocal);
    //Serial.printf("[WU] HTTP %d\n", code);
  }

  tft.setTextPadding(0);
  tft.unloadFont();
}


void saveSDData(time_t local_time) {
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth(" SD "));
  const int yStatus = (currentPage == 0) ? 100 : 30;

  if (!sdReady) {
    SD.end();     // 🔥 reset SD stack
    spiSD.end();  // 🔥 reset SPI
    delay(50);

    spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sdReady = SD.begin(SD_CS, spiSD);
    if (!sdReady) {
      tft.setTextColor(TFT_BLACK, TFT_RED);
      tft.drawString(" SD ", 770, yStatus);
      return;
    }
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.drawString(" SD ", 770, yStatus);
  }

  char fileName[32];
  snprintf(fileName, sizeof(fileName), "/log_%04d_%02d.csv", getYear(local_time), getMonth(local_time));
  //
  if (!SD.exists(fileName)) {

    File nf = SD.open(fileName, FILE_WRITE);

    if (nf) {

      nf.println(
        "DateTime,"
        "TempOutMax,TempOutMin,TempOut,"
        "TempInMax,TempInMin,TempIn,"
        "HumOutMax,HumOutMin,HumOut,"
        "HumInMax,HumInMin,HumIn,"
        "WindSpeed,WindGustHour,WindDirection,"
        "Rain,RainHour,RainDay,Solar_V");

      nf.close();
    }
  }
  //
  File f = SD.open(fileName, FILE_APPEND);
  if (!f) {
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    tft.drawString(" SD ", 770, yStatus);
    SD.end();
    spiSD.end();
    sdReady = false;
    digitalWrite(SD_CS, HIGH);
    return;
  }

  // --- Format "YYYY-MM-DD HH:MM" ---
  char datetime[20];
  sprintf(datetime, "%04d-%02d-%02d %02d:%02d",
          getYear(local_time), getMonth(local_time), getDay(local_time),
          getHour(local_time), getMinute(local_time));

  // --- Write data in FILE ---
  f.printf(
    "%s,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f,%.2f,%.2f,"
    "%.2f\n",
    datetime,
    maxTempOutDoor, minTempOutDoor, TempOutDoor,
    maxTempInDoor, minTempInDoor, TempInDoor,
    maxHumiOutDoor, minHumiOutDoor, HumiOutDoor,
    maxHumiInDoor, minHumiInDoor, HumiInDoor,
    fWindSpeed, maxWindHour, winddy,
    mmTotal, mmHourly, mmDaily, fvalueSol);

  f.close();
  tft.setTextColor(TFT_BLACK, TFT_GREEN);
  tft.drawString(" SD ", 770, yStatus);
/*
  logCount++;

  char logBuf[8];
  snprintf(logBuf, sizeof(logBuf), "%d", logCount);

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("99999"));
  tft.drawString(logBuf, 732, yStatus);
*/
  debugMemoryTFT(currentPage == 0 ? 40 : 0);
}


void readSDData(time_t local_time) {

  // =====================================================
  // CLEAR BUFFER
  // =====================================================
  for (int i = 0; i < MAX_SD_POINTS; i++) {
    sdTempOut[i] = 410;
    sdTempIn[i] = 410;
    sdHumOut[i] = 470;
    sdHumIn[i] = 470;
    sdWindGust[i] = 350;
    sdWindDir[i] = 310;
    sdRain[i] = 350;
  }
  sdValidPoints = 0;

  // =====================================================
  // RESET SD IF FAIL
  // =====================================================
  if (!sdReady) {
    SD.end();
    spiSD.end();
    delay(50);
    spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sdReady = SD.begin(SD_CS, spiSD);
    if (!sdReady) {
      digitalWrite(SD_CS, HIGH);
      return;
    }
  }

  // =====================================================
  // CIRCULAR BUFFER — tránh memmove
  // =====================================================
  // Dùng ring buffer: head trỏ vị trí ghi tiếp theo
  int head = 0;         // vị trí ghi tiếp theo trong vòng tròn
  int totalStored = 0;  // tổng điểm đã lưu (tối đa MAX_SD_POINTS)

  // =====================================================
  // Chunk read buffer — đọc SD theo block lớn
  // =====================================================
  static char chunkBuf[1024];  // static → không tốn stack
  static char line[MAX_LINE_LENGTH];

  for (int monthOffset = 2; monthOffset >= 0; monthOffset--) {

    int fileYear = getYear(local_time);
    int fileMonth = getMonth(local_time) - monthOffset;
    if (fileMonth <= 0) {
      fileMonth += 12;
      fileYear--;
    }

    char fileName[32];
    snprintf(fileName, sizeof(fileName),
             "/log_%04d_%02d.csv", fileYear, fileMonth);

    File dataFile = SD.open(fileName, FILE_READ);
    if (!dataFile) continue;

    // Skip header
    if (dataFile.available())
      dataFile.readBytesUntil('\n', line, sizeof(line));

    // -------------------------------------------------
    // Đọc chunk → parse trong RAM
    // -------------------------------------------------
    int linePos = 0;
    bool firstChunk = true;

    while (dataFile.available()) {

      // Đọc chunk
      int bytesRead = dataFile.readBytes(chunkBuf, sizeof(chunkBuf) - 1);
      if (bytesRead <= 0) break;
      chunkBuf[bytesRead] = '\0';

      // Parse từng ký tự trong chunk
      for (int ci = 0; ci < bytesRead; ci++) {

        char c = chunkBuf[ci];

        if (c == '\n' || c == '\r') {

          if (linePos > 0) {
            line[linePos] = '\0';
            linePos = 0;

            // ----------------------------------------
            // PARSE LINE
            // ----------------------------------------
            if (strlen(line) < 10) continue;

            int fYear, fMonth, fDay, fHour, fMinute;
            float maxTempOut, minTempOut, tempOutVal;
            float maxTempIn, minTempIn, tempIn;
            float maxHumOut, minHumOut, humOut;
            float maxHumIn, minHumIn, humIn;
            float windSpeed, windGust, windDir;
            float rain, rainHourly, rainDailyVal;
            float solar;

            int parsedFields = sscanf(line,
                                      "%d-%d-%d %d:%d,"
                                      "%f,%f,%f,%f,%f,%f,"
                                      "%f,%f,%f,%f,%f,%f,"
                                      "%f,%f,%f,"
                                      "%f,%f,%f,%f",
                                      &fYear, &fMonth, &fDay, &fHour, &fMinute,
                                      &maxTempOut, &minTempOut, &tempOutVal,
                                      &maxTempIn, &minTempIn, &tempIn,
                                      &maxHumOut, &minHumOut, &humOut,
                                      &maxHumIn, &minHumIn, &humIn,
                                      &windSpeed, &windGust, &windDir,
                                      &rain, &rainHourly, &rainDailyVal,
                                      &solar);

            if (parsedFields != 24) continue;

            // Filter
            if (tempOutVal < -20 || tempOutVal > 60) continue;
            if (tempIn < -20 || tempIn > 60) continue;
            if (humOut < 0 || humOut > 100) continue;
            if (humIn < 0 || humIn > 100) continue;
            if (windGust < 0 || windGust > 250) continue;
            if (windDir < 0 || windDir > 360) continue;
            if (rainDailyVal < 0 || rainDailyVal > 500) continue;
            if (isnan(tempOutVal) || isnan(tempIn) || isnan(humOut) || isnan(humIn) || isnan(windGust) || isnan(windDir) || isnan(rainDailyVal)) continue;

            // Chỉ lấy mốc xx:00
            if (fMinute != 0) continue;

            // ----------------------------------------
            // GHI VÀO CIRCULAR BUFFER — không memmove
            // ----------------------------------------
            tempOutBuffer[head] = (int16_t)tempOutVal;
            tempInBuffer[head] = (int16_t)tempIn;
            humOutBuffer[head] = (int16_t)humOut;
            humInBuffer[head] = (int16_t)humIn;
            windGustBuffer[head] = (int16_t)windGust;
            windDirBuffer[head] = (int16_t)windDir;
            rainBuffer[head] = (int16_t)rainDailyVal;

            head = (head + 1) % MAX_SD_POINTS;  // vòng tròn
            if (totalStored < MAX_SD_POINTS) totalStored++;
          }

        } else {
          // Tích lũy ký tự vào line buffer
          if (linePos < (int)sizeof(line) - 1)
            line[linePos++] = c;
        }
      }
    }

    dataFile.close();
  }

  // =====================================================
  // MAP DATA — resolve circular buffer ra sdXxx[]
  // =====================================================
  sdValidPoints = totalStored;

  int startX = MAX_SD_POINTS - totalStored;

  // oldest entry trong ring buffer nằm tại:
  // nếu buffer chưa đầy → index 0
  // nếu buffer đầy      → index head (đây là ô bị overwrite tiếp theo = oldest)
  int oldest = (totalStored < MAX_SD_POINTS) ? 0 : head;

  for (int i = 0; i < totalStored; i++) {

    int src = (oldest + i) % MAX_SD_POINTS;  // đọc theo thứ tự thời gian
    int x = startX + i;

    sdTempOut[x] = map(constrain(tempOutBuffer[src], 0, 50), 0, 50, 360, 260);
    sdTempIn[x] = map(constrain(tempInBuffer[src], 0, 50), 0, 50, 360, 260);
    sdHumOut[x] = map(constrain(humOutBuffer[src], 0, 100), 0, 100, 470, 370);
    sdHumIn[x] = map(constrain(humInBuffer[src], 0, 100), 0, 100, 470, 370);
    sdWindGust[x] = map(constrain(windGustBuffer[src], 0, 200), 0, 200, 250, 50);
    sdWindDir[x] = map(constrain(windDirBuffer[src], 0, 360), 0, 360, 170, 90);
    sdRain[x] = map(constrain(rainBuffer[src], 0, 100), 0, 100, 250, 50);
  }

  digitalWrite(SD_CS, HIGH);
  delay(10);
}


// =====================================================
// GRAPH WIND + RAIN
// =====================================================
void GraphWindRainSD() {
  //  Graph Line
  for (int m = 0; m < 210; m = m + 20) {
    tft.drawLine(0, 250 - m, 800, 250 - m, 0x3186);  //0x4228
  }
  for (int l = 80; l <= 800; l = l += 24) {
    tft.drawLine(l, 50, l, 250, 0x3186);
  }
  //Red line through 3 Graph
  for (int x = 80; x <= 780; x += 48) {
    tft.drawLine(x, 50, x, 250, 0x4228);
  }
  tft.drawLine(20, 90, 800, 90, 0x4228);
  tft.drawLine(20, 170, 800, 170, 0x4228);
  tft.setTextDatum(BL_DATUM);
  tft.setTextPadding(tft.textWidth("8"));
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Wind & Rain", 0, 57);
  tft.drawString("Speed", 95, 57);
  tft.fillRect(80, 49, 10, 3, TFT_RED);
  tft.drawString("Direct", 195, 57);
  tft.fillRect(180, 49, 10, 3, TFT_OLIVE);
  tft.drawString("Rain", 295, 57);
  tft.fillRect(280, 49, 10, 3, TFT_CYAN);

  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("180km/h", 800, 77);
  tft.setTextColor(0x4228, TFT_BLACK);
  tft.drawString("160km/h", 800, 97);
  tft.drawString("140km/h", 800, 117);
  tft.drawString("120km/h", 800, 137);
  tft.drawString("100km/h", 800, 157);
  tft.drawString("80km/h", 800, 177);
  tft.drawString("60km/h", 800, 197);
  tft.drawString("40km/h", 800, 217);
  tft.drawString("20km/h", 800, 237);

  for (int k = 799; k > 0; k--) {

    if (k < (800 - sdValidPoints)) continue;

    tft.drawLine(k,
                 sdRain[k],
                 k - 1,
                 sdRain[k - 1],
                 TFT_CYAN);

    tft.drawLine(k,
                 sdWindDir[k],
                 k - 1,
                 sdWindDir[k - 1],
                 TFT_OLIVE);

    tft.drawLine(k,
                 sdWindGust[k],
                 k - 1,
                 sdWindGust[k - 1],
                 TFT_RED);
  }

  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  //  tft.drawString("0", 0, 337);
  tft.drawString("10", 0, 237);
  tft.drawString("20", 0, 217);
  tft.drawString("30", 0, 197);
  tft.drawString("40", 0, 177);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("50", 0, 157);
  tft.drawString("60", 0, 137);
  tft.drawString("70", 0, 117);
  tft.drawString("80", 0, 97);
  tft.drawString("90", 0, 77);
  //  N E S W N
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_OLIVE, TFT_BLACK);  //#define TFT_DARKRED 0x8800
  tft.drawString("N", 20, 97);
  tft.drawString("W", 20, 117);
  tft.drawString("S", 20, 137);
  tft.drawString("E", 20, 157);
  tft.drawString("N", 20, 177);
}


// =====================================================
// GRAPH TEMP
// =====================================================
void GraphTempSD() {
  //  Graph Line
  for (int m = 0; m < 110; m = m + 20) {
    tft.drawLine(0, 360 - m, 800, 360 - m, 0x3186);  //0x4228
  }
  for (int l = 80; l <= 800; l = l += 24) {
    tft.drawLine(l, 260, l, 360, 0x3186);
  }
  //Red line through 3 Graph
  for (int x = 80; x <= 780; x += 48) {
    tft.drawLine(x, 260, x, 360, 0x4228);
  }
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Temperature", 0, 267);
  tft.drawString("Out", 95, 267);
  tft.fillRect(80, 259, 10, 3, TFT_ORANGE);
  tft.drawString("In", 195, 267);
  tft.fillRect(180, 259, 10, 3, TFT_CYAN);

  for (int k = 799; k > 0; k--) {

    if (k < (800 - sdValidPoints)) continue;

    tft.drawLine(k,
                 sdTempOut[k],
                 k - 1,
                 sdTempOut[k - 1],
                 TFT_ORANGE);

    tft.drawLine(k,
                 sdTempIn[k],
                 k - 1,
                 sdTempIn[k - 1],
                 TFT_CYAN);
  }

  tft.setTextPadding(tft.textWidth("8"));
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  //  tft.drawString("0", 0, 407);
  tft.drawString("10", 0, 347);
  tft.drawString("20", 0, 327);
  tft.drawString("30", 0, 307);
  tft.drawString("40", 0, 287);
}


// =====================================================
// GRAPH HUM
// =====================================================
void GraphHumSD() {
  //  Graph Line
  for (int m = 0; m < 110; m = m + 20) {
    tft.drawLine(0, 470 - m, 800, 470 - m, 0x3186);  //0x4228
  }
  for (int l = 80; l <= 800; l = l += 24) {
    tft.drawLine(l, 370, l, 470, 0x3186);
  }
  //Red line through 3 Graph
  for (int x = 80; x <= 780; x += 48) {
    tft.drawLine(x, 370, x, 470, 0x4228);
  }
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Humidity", 0, 377);
  tft.drawString("Out", 95, 377);
  tft.fillRect(80, 369, 10, 3, TFT_ORANGE);
  tft.drawString("In", 195, 377);
  tft.fillRect(180, 369, 10, 3, TFT_CYAN);

  for (int k = 799; k > 0; k--) {

    if (k < (800 - sdValidPoints)) continue;

    tft.drawLine(k,
                 sdHumOut[k],
                 k - 1,
                 sdHumOut[k - 1],
                 TFT_ORANGE);

    tft.drawLine(k,
                 sdHumIn[k],
                 k - 1,
                 sdHumIn[k - 1],
                 TFT_CYAN);
  }

  tft.setTextDatum(BL_DATUM);
  tft.setTextPadding(tft.textWidth("8"));
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  //  tft.drawString("0", 0, 477);
  tft.drawString("20", 0, 457);
  tft.drawString("40", 0, 437);
  tft.drawString("60", 0, 417);
  tft.drawString("80", 0, 397);

  tft.setTextDatum(BC_DATUM);
  tft.drawString("2", 752, 482);
  tft.drawString("4", 704, 482);
  tft.drawString("6", 656, 482);
  tft.drawString("8", 608, 482);
  tft.drawString("10", 560, 482);
  tft.drawString("12", 512, 482);
  tft.drawString("14", 464, 482);
  tft.drawString("16", 416, 482);
  tft.drawString("18", 368, 482);
  tft.drawString("20", 320, 482);
  tft.drawString("22", 272, 482);
  tft.drawString("24", 224, 482);
  tft.drawString("26", 176, 482);
  tft.drawString("28", 128, 482);
  tft.drawString("30", 80, 482);
}


/***************************************************************************************
**                          Draw the clock digits
***************************************************************************************/
void drawTime(time_t local_time) {
  sprTime.fillSprite(TFT_BLACK);
  sprTime.setTextDatum(BC_DATUM);
  sprTime.setTextColor(TFT_GREENYELLOW, TFT_BLACK);

  char timeBuf[32];

  sprTime.loadFont(AA_FONT_20, LittleFS);
  sprTime.setTextPadding(sprTime.textWidth("WEDNESDAY"));

  sprTime.drawString(dayStrESP32(local_time), 70, 30);

  snprintf(timeBuf, sizeof(timeBuf), "%d/%d/%d",
           getDay(local_time),
           getMonth(local_time),
           getYear(local_time));
  sprTime.drawString(timeBuf, 70, 56);
  sprTime.unloadFont();

  sprTime.loadFont(AA_FONT_70, LittleFS);

  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d",
           getHour(local_time),
           getMinute(local_time));

  sprTime.setTextPadding(sprTime.textWidth("44:44"));
  sprTime.drawString(timeBuf, 222, 68);
  sprTime.unloadFont();

  sprTime.pushSprite(0, 0);
  sprTime.setTextPadding(0);
}


/***************************************************************************************
**                          Fetch the weather data  and update screen
***************************************************************************************/
// Update the Internet based information and update screen
void updateWeatherData(time_t local_time) {
  int h = getHour(local_time);
  HTTPClient http;
  String url1h = "https://weather.visualcrossing.com/VisualCrossingWebServices/rest/services/timeline/";
  url1h += locationKey;
  url1h += "/today?unitGroup=metric&include=current&key=" + String(apiKey);

  bool validData = false;
  int retry = 0;

  while (!validData && retry < 3) {
    http.setConnectTimeout(4000);
    http.setTimeout(4000);
    http.begin(url1h);

    int httpResponseCode = http.GET();

    if (httpResponseCode == HTTP_CODE_OK) {
      DynamicJsonDocument doc(1024);
      DeserializationError error =
        deserializeJson(
          doc,
          http.getStream(),
          DeserializationOption::Filter(weatherFilter));

      if (!error) {
        JsonObject current = doc["currentConditions"];
        if (!current.isNull()) {
          strlcpy(weatherData.timeupdate, current["datetime"] | "", sizeof(weatherData.timeupdate));
          strlcpy(weatherData.weatherIcon, current["icon"] | "", sizeof(weatherData.weatherIcon));
          weatherData.temperature = current["feelslike"];
          weatherData.humidity = current["humidity"];
          weatherData.windDirectionDegrees = current["winddir"];
          weatherData.windSpeedKmph = current["windgust"];
          weatherData.rainmm = current["precip"];
          weatherData.UVindex = current["uvindex"];
          weatherData.Cloudcover = current["cloudcover"];
          weatherData.isday = (h >= 6 && h < 18) ? 1 : 0;

          validData = true;
        }
      }
    }

    http.end();
    if (validData) break;
    retry++;
    delay(500);
  }

  if (validData) {
    //    GetWeatherData = true;
    weatherReady = true;
  }
}

/***************************************************************************************
**                          Draw the current weather
***************************************************************************************/
void drawCurrentWeather(time_t local_time) {
  int h = getHour(local_time);
  int m = getMinute(local_time);

  char buf[32];

  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("12:30/12:00"));

  snprintf(buf, sizeof(buf), "|%02d:%02d/%.5s", h, m, weatherData.timeupdate);
  tft.drawString(buf, 799, 70);
  tft.unloadFont();

  // --- ICON METEOCON ---
  String weathertest = getMeteoconIcon(weatherData.weatherIcon, weatherData.Cloudcover, weatherData.isday);
  snprintf(buf, sizeof(buf), "/icon/%s.bmp", weathertest.c_str());
  ui.drawBmp(buf, 0, 63);

  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Cloudcover
  snprintf(buf, sizeof(buf), "%d%%", weatherData.Cloudcover);
  tft.setTextPadding(tft.textWidth(" 100%"));
  tft.drawString(buf, 195, 94);

  // UV
  snprintf(buf, sizeof(buf), "%d", weatherData.UVindex);
  tft.setTextPadding(tft.textWidth("88"));
  tft.drawString(buf, 273, 94);

  // Wind
  snprintf(buf, sizeof(buf), "%.1fkm/h", weatherData.windSpeedKmph);
  tft.setTextPadding(tft.textWidth("88,88km/h"));
  tft.drawString(buf, 195, 146);

  // Rain
  snprintf(buf, sizeof(buf), "%.2fmm", weatherData.rainmm);
  tft.setTextPadding(tft.textWidth("88,88mm"));
  tft.drawString(buf, 273, 146);
  tft.unloadFont();

  // Direct
  int windAngle = (int)((weatherData.windDirectionDegrees + 22.5) / 45.0);
  if (windAngle > 7 || windAngle < 0) windAngle = 0;

  const char* windDir[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
  snprintf(buf, sizeof(buf), "/wind/%s.bmp", windDir[windAngle]);
  ui.drawBmp(buf, 114, 86);

  // Temperature
  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("88,8°"));

  snprintf(buf, sizeof(buf), "%.1f°", weatherData.temperature);
  tft.drawString(buf, 322, 186);
  tft.unloadFont();

  // Humidity
  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("100%"));

  snprintf(buf, sizeof(buf), "%d%%", weatherData.humidity);
  tft.drawString(buf, 322, 228);
  tft.unloadFont();

  //
  tft.loadFont(AA_FONT_10, LittleFS);
  debugMemoryTFT(40);

  tft.setTextPadding(0);
  tft.unloadFont();
}


void updateAstronomy(time_t local_time) {
  HTTPClient http;

  String url1d = "https://weather.visualcrossing.com/VisualCrossingWebServices/rest/services/timeline/";
  url1d += locationKey;
  url1d += "/today?unitGroup=metric&include=current&elements=moonrise,moonset,moonphase,sunrise,sunset&key=" + String(apiKey);

  bool validData = false;
  int retry = 0;

  while (!validData && retry < 3) {
    http.setConnectTimeout(4000);
    http.setTimeout(4000);
    http.begin(url1d);

    int httpResponseCode = http.GET();

    if (httpResponseCode == HTTP_CODE_OK) {

      DynamicJsonDocument doc(512);

      DeserializationError error =
        deserializeJson(
          doc,
          http.getStream(),
          DeserializationOption::Filter(astronomyFilter));

      if (!error) {
        JsonObject day0 = doc["days"][0];
        if (!day0.isNull() && day0.containsKey("moonphase")) {
          strlcpy(astronomyData.sunrise, day0["sunrise"] | "", sizeof(astronomyData.sunrise));
          strlcpy(astronomyData.sunset, day0["sunset"] | "", sizeof(astronomyData.sunset));
          strlcpy(astronomyData.moonrise, day0["moonrise"] | "", sizeof(astronomyData.moonrise));
          strlcpy(astronomyData.moonset, day0["moonset"] | "", sizeof(astronomyData.moonset));

          astronomyData.moonage = day0["moonphase"].as<float>();

          validData = true;
        }
      }
    }

    http.end();
    if (validData) break;
    retry++;
    delay(500);
  }

  if (validData) {
    //    GetAstronomy = true;
    astronomyReady = true;
  }
}


// Backlight //
void updateBacklight() {
  int lux = lightMeter.readLightLevel();

  int pwm;

  if (lux < 20)
    pwm = 50;
  else if (lux < 40)
    pwm = 150;
  else if (lux < 80)
    pwm = 200;
  else
    pwm = 255;

  if (pwm != currentBacklightPWM) {
    ledcWrite(LEDpin, pwm);
    currentBacklightPWM = pwm;
  }
}


/***************************************************************************************
**                          Draw Sun rise/set, Moon, cloud cover and humidity
***************************************************************************************/
void drawAstronomy(time_t local_time) {
  int d = getDay(local_time);
  int m = getMonth(local_time);
  int y = getYear(local_time);

  LunarDate lunar = convertSolar2Lunar(d, m, y);

  char buf[32];

  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("30/12"));

  snprintf(buf, sizeof(buf), "%d/%d", lunar.day, lunar.month);
  tft.drawString(buf, 279, 207);

  snprintf(buf, sizeof(buf), "/moon/%d.bmp", lunar.day);
  ui.drawBmp(buf, 90, 167);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("88 : 88"));
  tft.setTextDatum(BC_DATUM);

  char sunriseRaw[9];
  snprintf(sunriseRaw, sizeof(sunriseRaw), "%.8s", astronomyData.sunrise);
  String sunrise24 = convertTo24HourFormat(sunriseRaw);
  tft.drawString(sunrise24.c_str(), 40, 207);

  char sunsetRaw[9];
  snprintf(sunsetRaw, sizeof(sunsetRaw), "%.8s", astronomyData.sunset);
  String sunset24 = convertTo24HourFormat(sunsetRaw);
  tft.drawString(sunset24.c_str(), 40, 248);

  char moonriseRaw[9];
  snprintf(moonriseRaw, sizeof(moonriseRaw), "%.8s", astronomyData.moonrise);
  String moonrise24 = convertTo24HourFormat(moonriseRaw);
  tft.drawString(moonrise24.c_str(), 219, 207);

  char moonsetRaw[9];
  snprintf(moonsetRaw, sizeof(moonsetRaw), "%.8s", astronomyData.moonset);
  String moonset24 = convertTo24HourFormat(moonsetRaw);
  tft.drawString(moonset24.c_str(), 219, 248);

  tft.setTextPadding(0);
  tft.unloadFont();
}


/***************************************************************************************
**                          Get the icon file name from the index number
***************************************************************************************/
const char* getMeteoconIcon(const char* weatherIcon, int Cloudcover, int isday) {
  if (isday == 1) {
    if (strcmp(weatherIcon, "clear-day") == 0 && Cloudcover <= 10) return "Clear";
    if (strcmp(weatherIcon, "clear-night") == 0 && Cloudcover <= 10) return "Clear";

    if (strcmp(weatherIcon, "clear-day") == 0 && Cloudcover > 10) return "Mostly Clear";
    if (strcmp(weatherIcon, "clear-night") == 0 && Cloudcover > 10) return "Mostly Clear";

    if (strcmp(weatherIcon, "partly-cloudy-day") == 0 && Cloudcover <= 50) return "Partly Cloudy";
    if (strcmp(weatherIcon, "partly-cloudy-night") == 0 && Cloudcover <= 50) return "Partly Cloudy";

    if (strcmp(weatherIcon, "partly-cloudy-day") == 0 && Cloudcover > 50) return "Mostly Cloudy";
    if (strcmp(weatherIcon, "partly-cloudy-night") == 0 && Cloudcover > 50) return "Mostly Cloudy";

    if (strcmp(weatherIcon, "cloudy") == 0) return "Cloudy";
    if (strcmp(weatherIcon, "fog") == 0) return "Fog";
    if (strcmp(weatherIcon, "wind") == 0) return "Wind";
    if (strcmp(weatherIcon, "rain") == 0) return "Rain";
    if (strcmp(weatherIcon, "showers-day") == 0) return "Light Rain";
    if (strcmp(weatherIcon, "thunder-rain") == 0 || strcmp(weatherIcon, "thunder-showers-day") == 0) return "Thunderstorm";
  } else {
    if (strcmp(weatherIcon, "clear-night") == 0 && Cloudcover <= 10) return "nt_Clear";
    if (strcmp(weatherIcon, "clear-day") == 0 && Cloudcover <= 10) return "nt_Clear";

    if (strcmp(weatherIcon, "clear-night") == 0 && Cloudcover > 10) return "nt_Mostly Clear";
    if (strcmp(weatherIcon, "clear-day") == 0 && Cloudcover > 10) return "nt_Mostly Clear";

    if (strcmp(weatherIcon, "partly-cloudy-night") == 0 && Cloudcover <= 50) return "nt_Partly Cloudy";
    if (strcmp(weatherIcon, "partly-cloudy-day") == 0 && Cloudcover <= 50) return "nt_Partly Cloudy";

    if (strcmp(weatherIcon, "partly-cloudy-night") == 0 && Cloudcover > 50) return "nt_Mostly Cloudy";
    if (strcmp(weatherIcon, "partly-cloudy-day") == 0 && Cloudcover > 50) return "nt_Mostly Cloudy";

    if (strcmp(weatherIcon, "cloudy") == 0) return "Cloudy";
    if (strcmp(weatherIcon, "fog") == 0) return "Fog";
    if (strcmp(weatherIcon, "wind") == 0) return "Wind";
    if (strcmp(weatherIcon, "rain") == 0) return "Rain";
    if (strcmp(weatherIcon, "showers-night") == 0) return "Light Rain";
    if (strcmp(weatherIcon, "thunder-rain") == 0 || strcmp(weatherIcon, "thunder-showers-night") == 0) return "Thunderstorm";
  }
  return "Unknown";
}


/***************************************************************************************
**                          Update progress bar
***************************************************************************************/
void drawProgress(uint8_t percentage, const char* text) {
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.setTextPadding(800);

  tft.drawString(text, 400, 260);

  ui.drawProgressBar(0, 269, 800, 15, percentage, TFT_DARKGREY, TFT_CYAN);

  tft.setTextPadding(0);
  tft.unloadFont();
}

/***************************************************************************************
**                          Determine place to split a line line
***************************************************************************************/
// determine the "space" split point in a long string
int splitIndex(String text) {
  uint16_t index = 0;
  while ((text.indexOf(' ', index) >= 0) && (index <= text.length() / 2)) {
    index = text.indexOf(' ', index) + 1;
  }
  if (index) index--;
  return index;
}

/***************************************************************************************
**                          Right side offset to a character
***************************************************************************************/
// Calculate coord delta from end of text String to start of sub String contained within that text
// Can be used to vertically right align text so for example a colon ":" in the time value is always
// plotted at same point on the screen irrespective of different proportional character widths,
// could also be used to align decimal points for neat formatting
int rightOffset(String text, String sub) {
  int index = text.indexOf(sub);
  return tft.textWidth(text.substring(index));
}

/***************************************************************************************
**                          Left side offset to a character
***************************************************************************************/
// Calculate coord delta from start of text String to start of sub String contained within that text
// Can be used to vertically left align text so for example a colon ":" in the time value is always
// plotted at same point on the screen irrespective of different proportional character widths,
// could also be used to align decimal points for neat formatting
int leftOffset(String text, String sub) {
  int index = text.indexOf(sub);
  return tft.textWidth(text.substring(0, index));
}

/***************************************************************************************
**                          Draw circle segment
***************************************************************************************/
// Draw a segment of a circle, centred on x,y with defined start_angle and subtended sub_angle
// Angles are defined in a clockwise direction with 0 at top
// Segment has radius r and it is plotted in defined colour
// Can be used for pie charts etc, in this sketch it is used for wind direction
#define DEG2RAD 0.0174532925  // Degrees to Radians conversion factor
#define INC 2                 // Minimum segment subtended angle and plotting angle increment (in degrees)
void fillSegment(int x, int y, int start_angle, int sub_angle, int r, unsigned int colour) {
  // Calculate first pair of coordinates for segment start
  float sx = cos((start_angle - 90) * DEG2RAD);
  float sy = sin((start_angle - 90) * DEG2RAD);
  uint16_t x1 = sx * r + x;
  uint16_t y1 = sy * r + y;

  // Draw colour blocks every INC degrees
  for (int i = start_angle; i < start_angle + sub_angle; i += INC) {

    // Calculate pair of coordinates for segment end
    int x2 = cos((i + 1 - 90) * DEG2RAD) * r + x;
    int y2 = sin((i + 1 - 90) * DEG2RAD) * r + y;

    tft.fillTriangle(x1, y1, x2, y2, x, y, colour);

    // Copy segment end to segment start for next segment
    x1 = x2;
    y1 = y2;
  }
}

String convertTo24HourFormat(String hour12) {
  String hourStr = hour12.substring(0, 2);    // "05"
  String minuteStr = hour12.substring(3, 5);  // "12"
  String period = hour12.substring(6);        // "PM" hoặc "AM"

  int hour = hourStr.toInt();
  int minute = minuteStr.toInt();

  if (period == "PM" && hour != 12) {
    hour += 12;
  } else if (period == "AM" && hour == 12) {
    hour = 0;
  }

  // Định dạng lại theo chuẩn 24h
  String result = (hour < 10 ? "0" : "") + String(hour) + ":" + (minute < 10 ? "0" : "") + String(minute);
  return result;
}
