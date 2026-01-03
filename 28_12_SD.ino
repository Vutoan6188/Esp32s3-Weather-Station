#define AA_FONT_SMALL "fonts/NotoSansBold15"  // 15 point sans serif bold
#define AA_FONT_LARGE "fonts/NotoSansBold36"  // 36 point sans serif bold
#define AA_FONT_10 "fonts/NotoSans-Bold10"
#define AA_FONT_13 "fonts/NotoSans-Bold13"
#define AA_FONT_20 "fonts/NotoSans-Bold20"
#define AA_FONT_30 "fonts/NotoSans-Bold30"
#define AA_FONT_40 "fonts/NotoSans-Bold40"
#define AA_FONT_70 "fonts/NotoSans-Bold70"
#define FIRMWARE_VERSION "0.0.1"
/**                          Load the libraries and settings
***************************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h> // https://github.com/Bodmer/TFT_eSPI
#include <TJpg_Decoder.h>
#include <FS.h>
#include <LittleFS.h>
#include "GfxUi.h"          // Attached to this sketch
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WiFiManager.h>          // https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
#include <BlynkSimpleEsp32.h>
#include <Update.h>
#include "All_Settings.h"
#include <JSON_Decoder.h> // https://github.com/Bodmer/JSON_Decoder
#include "NTP_Time.h"     // Attached to this sketch, see that tab for library needs
/***************************************************************************************
**                          Define the globals and class instances
***************************************************************************************/
TFT_eSPI tft = TFT_eSPI(); 
TFT_eSprite sprTime = TFT_eSprite(&tft);
TFT_eSprite sprJson = TFT_eSprite(&tft); 
boolean booted = false;
bool syncDone = false;
bool newLoRaData = false;
bool newBlynkData = false;
bool GetWeatherData = false;
bool GetAstronomy = false;
bool hourlyReset = false;
bool dailyReset = false;
WiFiManager wifiManager;
GfxUi ui = GfxUi(&tft); // Jpeg and bmpDraw functions TODO: pull outside of a class
long lastDownloadUpdate = millis();
long last_GRAPH = millis();
/***************************************************************************************
**                          Declare prototypes
***************************************************************************************/
void updateWeatherData();
void updateAstronomy();
void drawProgress(uint8_t percentage, String text);
void drawTime();
void updateBlynk();
void enterLightSleep();
void GetLoRa();
void drawTemHumIndoor();
void drawTemHumOutdoor();
void drawAngle();
void drawWindSpeed();
void drawRain();
void drawSignal();
void ResetValue();
void connectWiFi();
void drawData();
void drawGraphThrough();
void GraphWindRain();
void GraphTemp();
void GraphHum();
void drawCurrentWeather();
void saveData();
const char* getMeteoconIcon(String weatherIcon, int Cloudcover, int isday);
void drawAstronomy();
void fillSegment(int x, int y, int start_angle, int sub_angle, int r, unsigned int colour);
int leftOffset(String text, String sub);
int rightOffset(String text, String sub);
int splitIndex(String text);
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap)
{
   // Stop further decoding as image is running off bottom of screen
  if ( y >= tft.height() ) return 0;
  // This function will clip the image block rendering automatically at the TFT boundaries
  tft.pushImage(x, y, w, h, bitmap);
  // Return 1 to decode next block
  return 1;
}

struct CurrentWeather {
  float temperature;
  float pressure;
  float windSpeedKmph;
  int humidity;
  int windDirectionDegrees;
  int UVindex;
  int Cloudcover;
  String weatherIcon;
  String weatherStat;
  String timeupdate;
  float rainmm;
  int isday;  
};
CurrentWeather *currentWeather;
struct DailyWeather {
  float moonage;
  String sunrise;
  String sunset;
  String moonrise;
  String moonset;
};
DailyWeather* dailyWeather;

int LEDpin = 3;
#include "VietnameseLunar.h"
#include <ArduinoJson.h>
#include "Wire.h"
/*/BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C
*/
//AHT30
#include <Adafruit_AHTX0.h>
Adafruit_AHTX0 aht;
//BH1750
#include <BH1750.h>
BH1750 lightMeter;
// E220
int rssi;
int dbm;
int valueLoRaStrongE220;
// LoRa Received Signal Strength Indicator
int valueLoRaStrong;
// WiFi Received Signal Strength Indicator
int valueWiFiStrong;
int WiFiRSSI;
//  Battery of Station
float fvalueBat;
float valueBattery;
float valueBat;
float gBat;
//  Battery of Display
float valueBattery1;
//  Solar Strength Indicator
float fvalueSol;
float valueSolar;
float valueSol;
float gSol;
// Value TemHum OUTDOOR
float TempOutDoor;
float HumiOutDoor;
float maxTempOutDoor;
float minTempOutDoor;
float maxHumiOutDoor;
float minHumiOutDoor;
// Value TemHum INDOOR
float TempInDoor;
float HumiInDoor;
float maxTempInDoor;
float minTempInDoor;
float maxHumiInDoor;
float minHumiInDoor;
// Value WindSpeed
float fWindSpeed;
float maxWindHour;
//float minWindHour;
//float avgWind;
// Value WindDirect
float winddy;
// Value Rain
float RainGauge;
float mmHourly;
float mmDaily;
float mmGraph;
float mmTotal = 0;
float prevTotal = 0;
// TimeConnect
float fTimeConnect;
//GraphWindRain
float graphWindSpeed;
float graphWinddy;
int x[800];
int y[800];
int z[800];
int a[800];
float graphRain;
int b[800];
int g[800];
//GraphTemp
float graphHumOUT;
float graphTemOUT;
int c[800];
int d[800];
int v[800];
int f[800];
//GraphHum
float graphHumIN;
float graphTemIN;
int n[800];
int h[800];
int m[800];
int j[800];

//float Altitude;
//float Pressure;
//  Ebyte E220
#define LORA_RX_PIN 4
#define LORA_TX_PIN 5
//#include <HardwareSerial.h> // theo GPT thì không cần khai báo vì trong thư viện LoRa_E220.h đã kèm theo HardwareSerial.h, thêm vào trùng lặp sẽ hay bị treo
#define ENABLE_RSSI true
#include "LoRa_E220.h"
LoRa_E220 e220ttl(LORA_TX_PIN, LORA_RX_PIN, &Serial2, UART_BPS_RATE_9600, SERIAL_8N1);

//float bmetemperature, bmehumidity, bmePressure, bmeAltitude;
float Pressure, Altitude;
#define SEALEVELPRESSURE_HPA (1013.25)
unsigned long currentMillis = 0;
unsigned long connectMillis = 0;
unsigned long LoRaMillis = 0;
unsigned long lastCheckPower = 0;
//  Graph
const int UPDATE_GRAPH_SECS = 6 * 60UL;    //6 * 60UL;  // 60UL = 1minute => 6 minute
// Toạ độ tâm vòng tròn và bán kính
#define CENTER_X 540
#define CENTER_Y 82
#define RADIUS 50
int lastArrowHeadX1 = CENTER_X;
int lastArrowHeadY1 = CENTER_Y;
int lastArrowHeadX2 = CENTER_X;
int lastArrowHeadY2 = CENTER_Y;
int lastArrowTipX = CENTER_X;
int lastArrowTipY = CENTER_Y;
/***************************************************************************************
**                          Variables for WiFiManager
***************************************************************************************/
char locationKey[40] = "";
char apiKey[48] = "";
char authKey[40] = "";
char shortKey[11];

// 🧰 Thông tin GitHub
const char* GITHUB_USER = "Vutoan6188";
const char* GITHUB_REPO = "Esp32s3-Weather-Station";
const char* FIRMWARE_FILENAME = "Esp32s3_Weather_Station.ino.bin";  // Tên file .bin trong phần release

WiFiClientSecure client;

#define BUTTON_PIN 0 // Pin 0 -> BOOT Button
#define SLEEP_PIN 2
#define BATTERY_PIN 1

#define SD_CS 21
#define SD_SCK 36
#define SD_MOSI 37 //SD_DIN
#define SD_MISO 38 //SD_DO

SPIClass spiSD(FSPI);
uint32_t logCount = 0;
bool sdReady = false;
/***************************************************************************************
**                          Setup
***************************************************************************************/
void setup() {
//  Serial.begin(115200);
  // Tắt hoàn toàn Bluetooth để tiết kiệm pin
  btStop();
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Kích hoạt pull-up nội bộ cho GPIO09
  pinMode(SLEEP_PIN, INPUT_PULLDOWN);  // Tránh floating khi mất nguồn
  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
  //DIM
  ledcAttach(LEDpin, 1000, 8);
  ledcWrite(LEDpin, 255);
  tft.setRotation(3);
  tft.begin();
  sprJson.createSprite(156, 11); 
  sprTime.createSprite(323, 56);
  tft.fillScreen(TFT_BLACK);
  if (!LittleFS.begin()) {
    //Serial.println("Flash FS initialisation failed!");
    while (1) yield(); // Stay here twiddling thumbs waiting
  }
  //Serial.println("\nFlash FS available!");

  EEPROM.begin(512); // Khởi tạo EEPROM

  // Kiểm tra trạng thái nút nhấn khi khởi động
  if (digitalRead(BUTTON_PIN) == LOW) { // Nếu nút nhấn được nhấn
    //Serial.println("Nút nhấn được nhấn, xóa cấu hình cũ...");
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(BC_DATUM); // Bottom Centre datum
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextPadding(240); // Pad next drawString() text to full width to over-write old text
  tft.drawString(" ", 400, 220); 
  tft.drawString("Clearing the old configuration", 400, 240);
    // Xóa cấu hình WiFi và thông số
    WiFiManager wifiManager;
    wifiManager.resetSettings(); // Xóa cấu hình WiFi đã lưu
    EEPROM.write(0, 0); // Xóa locationKey, apiKey và authKey trong EEPROM
    EEPROM.commit();

  } else {
    // Nếu không nhấn nút, đọc locationKey và apiKey từ EEPROM
    EEPROM.get(0, locationKey);
    EEPROM.get(40, apiKey);
    EEPROM.get(88, authKey);

    //Serial.print("Location Key từ EEPROM: ");
    //Serial.println(locationKey);
    //Serial.print("API Key từ EEPROM: ");
    //Serial.println(apiKey);
    //Serial.print("Auth Key từ EEPROM: ");
    //Serial.println(authKey);
  }
  // Thêm custom parameters cho WiFiManager
  WiFiManagerParameter custom_locationKey("locationKey", "Location", locationKey, 40);
  WiFiManagerParameter custom_apiKey("apiKey", "API_key", apiKey, 48);
  WiFiManagerParameter custom_authKey("authKey", "BLYNK_AUTH_TOKEN", authKey, 40);
  
  wifiManager.addParameter(&custom_locationKey);
  wifiManager.addParameter(&custom_apiKey);
  wifiManager.addParameter(&custom_authKey);
  // Tự động kết nối WiFi
  if (!wifiManager.autoConnect("Weather_Station_Config")) {
    //Serial.println("Không thể kết nối WiFi");
    ESP.restart(); // Khởi động lại nếu kết nối không thành công
  }
  // Clear bottom section of screen
  tft.fillRect(0, 100, 800, 200, TFT_BLACK);
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(BL_DATUM); // Bottom Centre datum
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.drawString("Connecting to WiFi SSID: " + WiFi.SSID(), 0, 100);
  checkFirmwareUpdate();
  // Lưu thông số mới vào EEPROM
  strcpy(locationKey, custom_locationKey.getValue());
  strcpy(apiKey, custom_apiKey.getValue());
  strcpy(authKey, custom_authKey.getValue());

  EEPROM.put(0, locationKey);
  EEPROM.put(40, apiKey);
  EEPROM.put(88, authKey);
  EEPROM.commit();
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(BL_DATUM); // Bottom Centre datum
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Location: " + String(locationKey), 0, 370);
  tft.drawString("Api Key: " + String(apiKey), 0, 390);
  tft.drawString("Blynk Auth Token: " + String(authKey), 0, 410);
  //Serial.print("Location Key: ");
  //Serial.println(locationKey);
  //Serial.print("API Key: ");
  //Serial.println(apiKey);
  //Serial.print("Auth Key: ");
  //Serial.println(authKey);

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, spiSD)) {
    Serial.println("❌ SD init failed");
    tft.drawString("SD init failed", 0, 60);
  } else {
    Serial.println("✅ SD ready");
    tft.drawString("SD ready", 0, 60);
    // 👉 Chỉ tạo file + header nếu chưa có
    if (!SD.exists("/weather_log.csv")) {
      File f = SD.open("/weather_log.csv", FILE_WRITE);
      f.println("Date_Time,Wind_Gust/h,Rain_Daily,TempOut,HumiOut,TempIn,HumiIn,Direct");
      f.close();
      Serial.println("📄 Created weather_log.csv + header");
      tft.drawString("Created weather_log.csv + header", 0, 80);
    }
  }

  Wire.begin(6,7);
  //bme.begin(0x76); 
  aht.begin();
  lightMeter.begin();
  e220ttl.begin();

  delay(3000);
  tft.fillScreen(TFT_BLACK);
  
  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);
  TJpgDec.setSwapBytes(true); // May need to swap the jpg colour bytes (endianess)

  // Draw splash screen
  if (LittleFS.exists("/splash/weather_icon.jpg")   == true) { 
    TJpgDec.drawFsJpg(160, 0, "/splash/weather_icon.jpg", LittleFS);
  }
  delay(3000);

  // Clear bottom section of screen
  tft.fillRect(0, 200, 800, 400, TFT_BLACK);

  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(BC_DATUM); // Bottom Centre datum
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  tft.drawString("Original by: blog.squix.org", 400, 260);
  tft.drawString("Adapted by: Bodmer", 400, 280);
//  delay(2000);
//  delay(180000); //3 minute

  tft.fillRect(0, 200, 800, 400, TFT_BLACK);
// Chú ý: mấy lần trước sử dụng TOKEN giả khỏi nó đăng nhập Blynk. ai ngờ là nguyên nhân gây ra treo hệ thống lúc khởi động
//      nó cứ đứng ở đoạn đang kết nối WIFI hoài không chịu đi tiếp
//        và lúc đã vào được giao diện chính thì phần hiển thị thời gian cập nhật ban đầu sẽ là 7:00 cho updateWeatherData và updeDataMoon
//  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
  Blynk.config(authKey);
  tft.setTextDatum(BC_DATUM);
  tft.drawString("Fetching weather data...", 400, 240);

  // Fetch the time
  drawProgress(20, "Updating time...");
  udp.begin(localPort);
  syncTime();
  drawProgress(50, "Update conditions...");  
  tft.unloadFont();
//  Default value INDOOR
  maxTempInDoor = 0;
  minTempInDoor = 99;
  maxHumiInDoor = 0;
  minHumiInDoor = 99;
//  Default value OUTDOOR
  maxTempOutDoor = 0;
  minTempOutDoor = 99;
  maxHumiOutDoor = 0;
  minHumiOutDoor = 99;
//  Default value Rain
  mmHourly = 0;
  mmDaily = 0;
  mmGraph = 0;
//  Default value WindHour
  maxWindHour = 0;
//  minWindHour = 99;
//  avgWind = 0;
//  Default value Graph Bar
 valueLoRaStrong = 0;
 valueWiFiStrong = 0;
 valueBattery = 0;
 valueSolar = 0;
//  Graph Range
  for (int i = 800; i >= 0; i--) {
  //Graph
    x[i] = 9999;
    y[i] = 9999;
    z[i] = 9999;
    a[i] = 9999;
    b[i] = 9999;
    g[i] = 9999;
  //Graph1
    c[i] = 9999;
    d[i] = 9999;
    v[i] = 9999;
    f[i] = 9999;
  //Graph2
    n[i] = 9999;
    h[i] = 9999;
    m[i] = 9999;
    j[i] = 9999;      
  }
  drawProgress(100, "Done...");  
  tft.fillScreen(TFT_BLACK);

  tft.drawLine(10, 163, 680, 163, 0x4228);
  tft.drawLine(310, 59, 310, 250, 0x4228);
  tft.drawLine(10, 250, 790, 250, 0x4228);
  tft.drawLine(10, 57, 460, 57, 0x4228);
  tft.drawLine(440, 163, 440, 250, 0x4228);
  tft.drawLine(622, 163, 622, 250, 0x4228);

  ui.drawBmp("/icon50/Altitude.bmp", 335, 4);
  ui.drawBmp("/icon50/cushin.bmp", 704, 72);
  ui.drawBmp("/icon50/calendar.bmp", 255, 178);

  ui.drawBmp("/icon50/sunrise.bmp", 22, 167);
  ui.drawBmp("/icon50/sunset.bmp", 22, 207);
  ui.drawBmp("/icon50/moonrise.bmp", 207, 169);
  ui.drawBmp("/icon50/moonset.bmp", 207, 209);  

  ui.drawBmp("/icon50/cloud.bmp", 178, 69);
  ui.drawBmp("/icon50/uv.bmp", 259, 63);
  ui.drawBmp("/icon50/wind.bmp", 184, 120);
  ui.drawBmp("/icon50/rain.bmp", 266, 119);

  ui.drawBmp("/icon50/rain.bmp", 381, 68);  

  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TR_DATUM);  
  tft.drawString("Version " + String(FIRMWARE_VERSION), 635, 0);

  tft.setTextDatum(BR_DATUM);
// Rain Graph
  tft.setTextPadding(tft.textWidth("88"));
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("20", 333, 150);
  tft.drawString("40", 333, 130);
  tft.setTextColor(TFT_RED, TFT_BLACK);  
  tft.drawString("60", 333, 110);
  tft.drawString("80", 333, 90);
  tft.drawString("100", 333, 70);
  tft.drawLine(335, 113, 335, 163, 0x4228);
    tft.drawLine(335, 63, 335, 113, TFT_RED);  
  tft.drawLine(368, 113, 368, 163, 0x4228);
    tft.drawLine(368, 63, 368, 113, TFT_RED);  
// Signal
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Wifi"));
  tft.drawString("WiFi", 644, 0);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("LoRa"));
  tft.drawString("LoRa", 644, 10);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Battery:"));
  tft.drawString("Battery", 644, 20); 
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Solar"));
  tft.drawString("Solar", 644, 30);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Station Runtime"));
  tft.drawString("Station runtime", 644, 50);  
  //  ForeCasts
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("Forecasts"));
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.drawString("FORECAST", 321, 169);
  //  OUTDOOR
  tft.setTextPadding(tft.textWidth("OUTDOOR"));
  tft.drawString("OUTDOOR", 454, 169);
  //  INDOOR
  tft.setTextPadding(tft.textWidth("INDOOR"));
  tft.drawString("INDOOR", 636, 169);

  tft.unloadFont();
  booted = true;
}
/***************************************************************************************
**                          Loop
***************************************************************************************/
void loop() {
  currentMillis = millis();
  time_t local_time = TIMEZONE.toLocal(now(), &tz1_Code);
  int h = hour(local_time);
  int m = minute(local_time);
  int s = second(local_time);  
  if (digitalRead(SLEEP_PIN) == HIGH) {
    Blynk.run();
  }
  // Kiểm tra trạng thái nguồn mỗi 1 giây
  if (currentMillis - lastCheckPower >= 1000) {
    lastCheckPower = currentMillis;
    // Kiểm tra điều kiện vào Light Sleep
    if (digitalRead(SLEEP_PIN) == LOW) {
    enterLightSleep();
    }
  }
  // 🔄 Kiểm tra WiFi mỗi 10 phút
  if ((currentMillis - connectMillis >= 600000) && (WiFi.status() != WL_CONNECTED)) {
    connectMillis = currentMillis;
    //Serial.println("🔄 WiFi lost connect, trying again...");
    WiFi.disconnect();
    WiFi.reconnect();
  } 
  //  loraE220();
  if (currentMillis - LoRaMillis >= 50) {
    LoRaMillis = currentMillis;
    GetLoRa();
  }
  if (newLoRaData) {
    drawData(local_time);
    newBlynkData = true;
    newLoRaData = false;
  }
  if (newBlynkData) {
    updateBlynk();
    newBlynkData = false;
  }
  // If minute has changed then request new time from NTP server
  if (booted || m != lastMinute)
  {
    // Update displayed time first as we may have to wait for a response
    drawTime(local_time);
    lastMinute = m;
  }
  if (booted || (h == 0 && m == 1 && !GetAstronomy)) {
    updateAstronomy(local_time);
  }
  if (h != 0) {
    GetAstronomy = false;
  }
  if (booted || ((m == 0 || m == 30) && s > 5 && !GetWeatherData)) {
    updateWeatherData(h, m);
  }
  if (m != 0 && m != 30) {
    GetWeatherData = false;
  }
  if (booted || ((m == 15 || m == 45) && s > 5 && !syncDone)) {
    // Request and synchronise the local clock
    syncTime();
    syncDone = true;
  }
  if ( s > 10) syncDone = false;
  booted = false;
}

void enterLightSleep() {
//  Serial.println("🌙 Entering Light Sleep...");
  ledcWrite(LEDpin, 0);
  WiFi.disconnect(false);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)SLEEP_PIN, 1);  
  esp_light_sleep_start();
//  Serial.println("🌞 Waking up from Light Sleep...");
  delay(1000);
  connectWiFi();
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin();
    //Serial.print(".");
    delay(1000);
  }
//  Serial.println("\n✅ WiFi connected!");
}

// === OTA CHECK ===
void checkFirmwareUpdate() {
  tft.setTextDatum(BL_DATUM); // Bottom Centre datum
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  //Serial.println("\n🔍 Check firmware on GitHub...");
  tft.drawString("Check firmware...", 0, 120);

  HTTPClient http;
  client.setInsecure();  // Bỏ xác minh chứng chỉ SSL
  http.begin(client, String("https://api.github.com/repos/") + GITHUB_USER + "/" + GITHUB_REPO + "/releases/latest");
  http.addHeader("User-Agent", "ESP32-OTA-Agent");

  int httpCode = http.GET();
  if (httpCode == 200) {
    String json = http.getString();
    int tagStart = json.indexOf("\"tag_name\":\"") + 12;
    int tagEnd = json.indexOf("\"", tagStart);
    String latestVersion = json.substring(tagStart, tagEnd);

    //Serial.printf("🔹 Firmware latest: %s\n", latestVersion.c_str());
    //Serial.printf("🔹 Firmware current: %s\n", FIRMWARE_VERSION);
    tft.drawString("Firmware current: " + String(FIRMWARE_VERSION), 0, 140);
    tft.drawString("Firmware latest: " + String(latestVersion.c_str()), 0, 160);
    delay(3000);
    if (latestVersion != FIRMWARE_VERSION) {
      tft.fillRect(0, 200, 800, 200, TFT_BLACK);
      //Serial.println("⚡ New firmware detected — starting OTA update...");
      tft.drawString("New firmware detected - starting OTA update...", 0, 180);
      String binUrl = "https://github.com/" + String(GITHUB_USER) + "/" + String(GITHUB_REPO) +
                      "/releases/download/" + latestVersion + "/" + FIRMWARE_FILENAME;

      //Serial.println("📦 File URL: " + binUrl);
      tft.drawString("File URL: " + String(binUrl), 0, 200);
      HTTPClient https;
      https.begin(client, binUrl);
      https.addHeader("User-Agent", "ESP32-OTA-Agent");
      https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // 🟢 Theo dõi redirect 302

      int code = https.GET();
      //Serial.printf("🔁 HTTP Code: %d\n", code);
      tft.fillRect(0, 200, 240, 120, TFT_BLACK);
      tft.drawString("HTTP code: " + String(code), 0, 220);
      if (code == 200) {
        int len = https.getSize();
        WiFiClient *stream = https.getStreamPtr();

      if (Update.begin(len)) {
        //Serial.println("📥 Đang tải firmware...");
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
            //Serial.printf("Progress: %d%%\r", percent);
            drawProgress(percent, String(percent) + "%");
      //      int filledWidth = map(percent, 0, 100, 0, barWidth);
            }
          }
          delay(1);
        }
        tft.setTextDatum(BL_DATUM);
        if (Update.end()) {
        if (Update.isFinished()) {
          //Serial.println("\n🎉 Update complete — rebooting...");
          tft.loadFont(AA_FONT_SMALL, LittleFS); 
          tft.setTextColor(TFT_CYAN, TFT_BLACK);
          tft.drawString("Update complete -> rebooting...", 0, 260);
          delay(2000);
          ESP.restart();
          } else {
            //Serial.println("❌ Update chưa hoàn tất!");
            tft.drawString("Update incomplete...", 0, 260);
            delay(2000);
            }
          } else {
            //Serial.printf("❌ Update error: %s\n", Update.errorString());
            tft.drawString("Update error...", 0, 260);
            delay(2000);
            }
      } else {
          //Serial.println("❌ Không thể bắt đầu update!");
          tft.drawString("Can't update...", 0, 240);
          delay(2000);
        }
      } else {
        //Serial.printf("❌ Tải file thất bại! HTTP code: %d\n", code);
        tft.drawString("Fail load file..." + String(code), 0, 220);
        delay(3000);
      }
      https.end();
    } else {
      tft.fillRect(0, 200, 240, 320 - 200, TFT_BLACK);
      //Serial.println("✅ The latest version");
      tft.drawString("The latest version", 0, 180);
      delay(2000);
    }
  } else {
    //Serial.printf("❌ Không thể kiểm tra phiên bản! HTTP code: %d\n", httpCode);
    tft.drawString("Can't check firmware" + String(httpCode), 0, 120);
    delay(2000);
  }
  http.end();
  tft.unloadFont();
}

void GetLoRa() {   
  if (e220ttl.available() > 1) {
    tft.loadFont(AA_FONT_10, LittleFS);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_BLACK, TFT_CYAN);
    tft.setTextPadding(tft.textWidth("LORA"));
    tft.drawString("LORA", 644, 40); 
    ResponseContainer rc = e220ttl.receiveMessageRSSI();  
    rssi = rc.rssi, DEC;
    dbm = - (256 - rssi);
    StaticJsonDocument<250> doc;
    DeserializationError error = deserializeJson(doc, rc.data);      
  //  Get Temp/Hum OUT
    float xTempOutDoor = doc["temperature"];
    if (!isnan(xTempOutDoor) && xTempOutDoor > 0 && xTempOutDoor < 80.0) {
      TempOutDoor = xTempOutDoor;
    }
    float xHumiOutDoor = doc["humidity"];
    if (!isnan(xHumiOutDoor) && xHumiOutDoor > 0) {
      HumiOutDoor = xHumiOutDoor;
    }
  //  Get WindDirect
    winddy = doc["angle"]; //= random(0, 360);//
  //  Get WindSpeed
    fWindSpeed = doc["speed"];
  //  Get Rain
    RainGauge = doc["rain"];
  // Get Battery
    fvalueBat = doc["battery"];
  // Get Solar
    fvalueSol = doc["solar"];
  //  Get TimeConnect
    fTimeConnect = doc["timerun"];
  //  Altitude
    Altitude = doc["altitude"];      
  //  Pressure
    Pressure = doc["pressure"];

    if (fWindSpeed > maxWindHour) maxWindHour = fWindSpeed;    
    mmTotal += RainGauge;
    mmDaily += RainGauge;
    mmGraph += RainGauge;
    mmHourly += RainGauge;

    newLoRaData = true;
    tft.fillRect(644, 40, 28, 11, TFT_BLACK);    
  }
}

void drawData(time_t local_time) {
  if (currentMillis > 30000)  {
    drawAngle();
    drawWindSpeed();
    drawRain();
    drawSignal();
    //  Get Temp/Hum IN
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    float xTempInDoor = temp.temperature; // Đọc nhiệt độ
    if (!isnan(xTempInDoor) && xTempInDoor > 0 && xTempInDoor < 80.0) {
      TempInDoor = xTempInDoor;
    }
    float xHumiInDoor = humidity.relative_humidity; // Đọc độ ẩm
    if (!isnan(xHumiInDoor) && xHumiInDoor > 0) {
      HumiInDoor = xHumiInDoor;
    }
/*      float xTempInDoor = bme.readTemperature(); // Đọc nhiệt độ
      if (!isnan(xTempInDoor) && xTempInDoor > 0 && xTempInDoor < 80.0) {
      TempInDoor = xTempInDoor;
      }
      float xHumiInDoor = bme.readHumidity(); // Đọc độ ẩm
      if (!isnan(xHumiInDoor) && xHumiInDoor > 0) {
      HumiInDoor = xHumiInDoor;
      }
      bmePressure = bme.readPressure() / 100.00 + 62.9; //tham khảo calibration tại link  https://www.youtube.com/watch?v=Wq-Kb7D8eQ4 hoặc đơn giản lấy giá trị áp suất khí quyển theo các web thời tiết - giá trị chuẩn theo code mẫu bme280 là ra con số 55.84 cộng thêm
      bmeAltitude = bme.readAltitude(SEALEVELPRESSURE_HPA) + 14;  //lấy độ cao thực tế của Tp Buôn ma thuột - độ cao chưa cộng với 14 là ra số 14 để cộng vào cho chuẩn với cao độ của tp bmt là 536m
*/
    drawTemHumOutdoor();
    drawTemHumIndoor();
  }
  ResetValue(local_time);
  tft.setTextPadding(0);
  tft.unloadFont();

  if (millis() - last_GRAPH > 1000UL * UPDATE_GRAPH_SECS ) {
    last_GRAPH = millis();
    tft.loadFont(AA_FONT_10, LittleFS);
    GraphWindRain();
    GraphTemp();
    GraphHum();
    saveData(local_time);
    tft.setTextPadding(0);
    tft.unloadFont();
  }
}

void drawAngle() {
  if (lastArrowHeadX1 != CENTER_X || lastArrowHeadY1 != CENTER_Y) {
    tft.fillTriangle(lastArrowHeadX1, lastArrowHeadY1, lastArrowHeadX2, lastArrowHeadY2, lastArrowTipX, lastArrowTipY, TFT_BLACK);
  }
  // Tính toạ độ đầu và đuôi mũi tên mới
  float radian = (winddy + 270) * 0.0174533;                         // Chuyển đổi độ sang radian
  int arrowTipX = CENTER_X + cos(radian) * (RADIUS - 10);   // Đầu mũi tên gần tâm hơn
  int arrowTipY = CENTER_Y + sin(radian) * (RADIUS - 10);   // Đầu mũi tên gần tâm hơn
  int arrowTailX = CENTER_X + cos(radian) * (RADIUS + 10);  // Tăng số mũi tên xa tâm hơn
  int arrowTailY = CENTER_Y + sin(radian) * (RADIUS + 10);  // Tăng số mũi tên xa tâm hơn
  // Vẽ đầu mũi tên (tam giác)
  float arrowAngle = atan2(arrowTailY - arrowTipY, arrowTailX - arrowTipX);
  float arrowHeadAngle1 = arrowAngle + PI / 6;
  float arrowHeadAngle2 = arrowAngle - PI / 6;

  int arrowHeadX1 = arrowTailX + cos(arrowHeadAngle1) * 20; // 4 dòng code ở đây tăng số để mũi tên tăng kích
  int arrowHeadY1 = arrowTailY + sin(arrowHeadAngle1) * 20;
  int arrowHeadX2 = arrowTailX + cos(arrowHeadAngle2) * 20;
  int arrowHeadY2 = arrowTailY + sin(arrowHeadAngle2) * 20;

  tft.drawCircle(CENTER_X, CENTER_Y, RADIUS + 15, TFT_DARKGREY); // Vòng tròn hướng gió
    tft.drawCircle(CENTER_X, CENTER_Y, RADIUS + 16, TFT_DARKGREY);
      tft.drawCircle(CENTER_X, CENTER_Y, RADIUS + 17, TFT_DARKGREY);
        tft.drawCircle(CENTER_X, CENTER_Y, RADIUS + 18, TFT_DARKGREY);
          tft.drawCircle(CENTER_X, CENTER_Y, RADIUS + 19, TFT_DARKGREY);
            tft.drawCircle(CENTER_X, CENTER_Y, RADIUS +20, TFT_DARKGREY);
  tft.fillTriangle(arrowTailX, arrowTailY, arrowHeadX1, arrowHeadY1, arrowHeadX2, arrowHeadY2, TFT_RED);
  // Cập nhật tọa độ mũi tên cũ
  lastArrowHeadX1 = arrowHeadX1;
  lastArrowHeadY1 = arrowHeadY1;
  lastArrowHeadX2 = arrowHeadX2;
  lastArrowHeadY2 = arrowHeadY2;
  lastArrowTipX = arrowTailX;
  lastArrowTipY = arrowTailY;
  //CompassRose
  tft.loadFont(AA_FONT_30, LittleFS);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("8888"));
  tft.drawString(String(winddy, 0) + "°", CENTER_X, CENTER_Y - 30);
  tft.unloadFont();
}

void drawWindSpeed() {
  tft.loadFont(AA_FONT_LARGE, LittleFS);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("888.8"));
  tft.drawString(String(fWindSpeed, 1), CENTER_X, CENTER_Y + 3);
  tft.unloadFont();
//      if (fWindSpeed < minWindHour)
//        minWindHour = fWindSpeed;
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextPadding(tft.textWidth("888.8  max/h"));
//      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.drawString(String(maxWindHour, 1) + " max/h ", CENTER_X, CENTER_Y + 30);
  tft.unloadFont();
//      tft.fillRect(711, 136, 3, 12, TFT_ORANGE);
  tft.setTextPadding(tft.textWidth("km/h"));
  tft.loadFont(AA_FONT_13, LittleFS);
  tft.drawString("km/h", CENTER_X, CENTER_Y + 50);
  tft.unloadFont();
}

void drawRain() {
  // Rain Counts
  if (mmGraph > 99.75) mmGraph = 0;
  //  Rain Hourly
  tft.fillRect(374, 131, 3, 11, TFT_CYAN);  // for mmDaily data row
  tft.loadFont(AA_FONT_SMALL, LittleFS);  
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("888.88mm/d"));
  //  Rain Hourly
  tft.drawString(String(mmHourly, 2) + "mm/h", 380, 114);
  //  Rain Daily
  tft.drawString(String(mmDaily, 2) + "mm/d", 380, 130);
  //  Rain Total TFT
  tft.setTextPadding(tft.textWidth("88888.88 mm"));
  tft.drawString(String(mmTotal, 2) + "mm", 380, 146);
  tft.unloadFont();
  // RainGauge
  tft.fillRect(337, 63, 30, 100, 0x4228);                  //background GREY of bar graph
  tft.fillRect(337, 163 - mmGraph, 30, mmGraph, 0x07FF);  //bar graph CYAN
  for (int j = 0; j < 110; j = j + 10) {
    tft.drawLine(337, 163 - j, 366, 163 - j, TFT_BLACK);  // Line of RainGauge 0x07E00 == GREEN
  }
}

void drawSignal () {
  WiFiRSSI = WiFi.RSSI();
  valueLoRaStrong = map(dbm, -35, -120, 50, 0);  // trước là -30  code chuẩn là 256,160.50.0. nhưng khi 2 mạch cách nhau 1m mà sóng chỉ được -22dbm. nên sửa lại 234,160,50,0
  valueWiFiStrong = map(WiFi.RSSI(), -35, -120, 50, 0);  //thử bỏ mạch phát wifi cách 20cm thì chỉ lên được -35dbm
  if (WiFi.status() != WL_CONNECTED) {
    valueWiFiStrong = 0;
    WiFiRSSI = -120;
  }
// Signal Strong LoRa & WiFi
  tft.loadFont(AA_FONT_10, LittleFS);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth("-888dBm"));
  tft.drawString(String(WiFiRSSI) + "dBm", 748, 0);
  tft.fillRect(750, 1, 50, 8, 0x4228);
  tft.fillRect(750, 2, valueWiFiStrong, 6, 0xFFFF);  
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth("-888dBm"));      
  tft.drawString(String(dbm) + "dBm", 748, 10);
  tft.fillRect(750, 11, 50, 8, 0x4228);
  tft.fillRect(750, 12, valueLoRaStrong, 6, 0x07FF);
// LoRa Staion Battery
  gBat = fvalueBat*1000;
  valueBattery = map(gBat, 2500, 4200, 0, 50);  
  tft.fillRect(750, 21, 50, 8, 0x4228);
  tft.fillRect(750, 22, valueBattery, 3, 0x07E0);
// Battery of Display
  float analogVolts = (analogReadMilliVolts(BATTERY_PIN) * 1.556);
  valueBattery1 = map(analogVolts, 2500, 4200, 0, 50);  
  tft.fillRect(750, 26, valueBattery1, 3, TFT_RED);  
  tft.setTextColor(TFT_GREEN, TFT_BLACK);  
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth("20.00/4.3V"));
  tft.drawString(String(fvalueBat) + "|" + String((analogVolts/1000), 2)+ "v", 748, 20);  
//  Serial.printf("ADC millivolts value = %d\n", analogVolts);  
// Solar display
  gSol = fvalueSol * 100;
  valueSolar = map(gSol, 0, 2000, 0, 50);  
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);  
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth("20.00/20V"));
  tft.drawString(String(fvalueSol) + "/20v", 748, 30);
  tft.fillRect(750, 31, 50, 8, 0x4228);
  tft.fillRect(750, 32, valueSolar, 6, 0xFFE0);
// Time Connect
  tft.setTextColor(TFT_WHITE, TFT_BLACK);  
  tft.setTextDatum(TR_DATUM);
  tft.setTextPadding(tft.textWidth("4,294,967.295s"));
  tft.drawString(String(fTimeConnect, 0) + "s", 800, 50);
  tft.unloadFont();
}

void drawTemHumOutdoor() {
  if (TempOutDoor < minTempOutDoor) minTempOutDoor = TempOutDoor;
  if (TempOutDoor > maxTempOutDoor) maxTempOutDoor = TempOutDoor;
  if (HumiOutDoor < minHumiOutDoor) minHumiOutDoor = HumiOutDoor;
  if (HumiOutDoor > maxHumiOutDoor) maxHumiOutDoor = HumiOutDoor;
  //  Temp OUTDOOR
//  tft.fillRect(292, 178, 3, 27, TFT_GREEN);
  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("88,88°"));
  tft.drawString(String(TempOutDoor, 2) + "°", 455, 186);
  tft.unloadFont();
  //  Humi OUTDOOR
  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextPadding(tft.textWidth("100.0%"));
  tft.drawString(String(HumiOutDoor, 1) + "%", 455, 228);
  tft.unloadFont();
  //  min/maxTemp
  tft.loadFont(AA_FONT_13, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("88.88"));
  tft.drawString(String(maxTempOutDoor, 2), 585, 188);
  tft.drawString(String(minTempOutDoor, 2), 585, 204);
  //  min/maxHumi
  tft.setTextPadding(tft.textWidth("100.0"));
  tft.drawString(String(maxHumiOutDoor, 1), 585, 222);
  tft.drawString(String(minHumiOutDoor, 1), 585, 234);
  tft.unloadFont();
/*//  Altitude && Pressure  
  tft.loadFont(AA_FONT_20);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("8888hpa"));
  tft.drawString(String(Altitude, 1) + "m", 700, 218);
  tft.drawString(String(Pressure, 4), 700, 188);   */
}

void drawTemHumIndoor() {
  if (TempInDoor < minTempInDoor) minTempInDoor = TempInDoor;
  if (TempInDoor > maxTempInDoor) maxTempInDoor = TempInDoor;
  if (HumiInDoor < minHumiInDoor) minHumiInDoor = HumiInDoor; 
  if (HumiInDoor > maxHumiInDoor) maxHumiInDoor = HumiInDoor;
  //  INDOOR TEMP
//  tft.fillRect(662, 174, 3, 27, TFT_LIGHTGREY);
  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("88.88°"));
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(String(TempInDoor, 2) + "°", 637, 186);
  tft.unloadFont();
  //  INDOOR HUMI
//  tft.fillRect(662, 219, 3, 12, TFT_DARKGREY);
  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("100.0%"));
  tft.drawString(String(HumiInDoor, 1) + "%", 637, 228);
  tft.unloadFont();
  //  minmax Temp
  tft.loadFont(AA_FONT_13, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("88.88"));
  tft.drawString(String(maxTempInDoor, 2), 767, 188);
  tft.drawString(String(minTempInDoor, 2), 767, 204);
  //  minmax Humi
  tft.setTextPadding(tft.textWidth("100.0"));
  tft.drawString(String(maxHumiInDoor, 1), 767, 222);
  tft.drawString(String(minHumiInDoor, 1), 767, 234);
  //  Press && Altitude
  tft.setTextPadding(tft.textWidth("8888.8hPa"));
  tft.drawString(String(Pressure + 6.8, 1) + "hPa", 407, 29);
  tft.drawString(String(Altitude, 1) + "m", 407, 41);
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
  int h = hour(local_time);
  int m = minute(local_time);
  int s = second(local_time); 
//  HOURLY
  if (m == 59 && s > 55 && !hourlyReset) {
  //  Rain
    mmHourly = 0;
  //  Wind
    maxWindHour = 0;    
//    minWindHour = 99;
    hourlyReset = true;
  }
  if (m != 0) {
    hourlyReset = false;
  }
//  DAILY  
  if (h == 23 && m == 59 && s > 55 && !dailyReset) {
  //  Rain
  mmDaily = 0;
  mmGraph = 0;
//  Outdoor
  maxTempOutDoor = 0;
  minTempOutDoor = 99;
  maxHumiOutDoor = 0;
  minHumiOutDoor = 99;
//  Indoor
  maxTempInDoor = 0;
  minTempInDoor = 99;
  maxHumiInDoor = 0;
  minHumiInDoor = 99;

  dailyReset = true;
  }
  if (!(h == 0 && m == 0)) {
    dailyReset = false;
  }
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

  graphRain = map(mmGraph, 0, 100, 350, 250);
    b[800] = graphRain;
  graphWinddy = map(winddy, 0, 360, 310, 270);
  if (maxWindHour > 200) maxWindHour = 200;
  graphWindSpeed = map(maxWindHour, 0, 200, 350, 250);
    z[800] = graphWinddy;
    x[800] = graphWindSpeed;
  for (int k = 800; k >= 0; k--) {
    tft.drawLine(k, b[k], k - 1, b[k - 1], TFT_CYAN);  // Rain gauge/day
    g[k - 1] = b[k];
    tft.drawLine(k, z[k], k - 1, z[k - 1], TFT_OLIVE); // Wind Direction
    tft.drawLine(k, x[k], k - 1, x[k - 1], TFT_RED);   // Wind Gust
    a[k - 1] = z[k];
    y[k - 1] = x[k];
  }
  for (int i = 800; i >= 0; i--) {
    z[i] = a[i];
    x[i] = y[i];  //send the shifted data back to the variable x
    b[i] = g[i];  //send the shifted data back to the variable c
  }
  tft.fillRect(0, 351, 800, 150, TFT_BLACK);
}

void GraphTemp() {
//  tft.loadFont(AA_FONT_10, LittleFS);
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
  graphTemOUT = map(TempOutDoor, 0, 50, 410, 360);
  graphTemIN = map(TempInDoor, 0, 50, 410, 360);
    n[800] = graphTemOUT;
    m[800] = graphTemIN;
  for (int k = 800; k >= 0; k--) {
    tft.drawLine(k, n[k], k - 1, n[k - 1], TFT_ORANGE); //  humidity outdoor
    tft.drawLine(k, m[k], k - 1, m[k - 1], TFT_CYAN);  //  Temperature outdoor
    h[k - 1] = n[k];                    //saves the information shifted one position temporarily in y
    j[k - 1] = m[k];
  }
  for (int k = 800; k >= 0; k--) {
    n[k] = h[k];  //send the shifted data back to the variable c
    m[k] = j[k];
  }
  tft.fillRect(0, 411, 800, 79, TFT_BLACK);
}

void GraphHum() {
//  tft.loadFont(AA_FONT_10, LittleFS);
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
  graphHumOUT = map(HumiOutDoor, 0, 100, 470, 420);
  graphHumIN = map(HumiInDoor, 0, 100, 470, 420);
    c[800] = graphHumOUT;
    v[800] = graphHumIN;
  for (int k = 800; k >= 0; k--) {
    tft.drawLine(k, c[k], k - 1, c[k - 1], TFT_ORANGE); //  humidity outdoor
    tft.drawLine(k, v[k], k - 1, v[k - 1], TFT_CYAN);  //  Temperature outdoor
    d[k - 1] = c[k];                    //saves the information shifted one position temporarily in y
    f[k - 1] = v[k];
  }
  for (int k = 800; k >= 0; k--) {
    c[k] = d[k];  //send the shifted data back to the variable c
    v[k] = f[k];
  }
  tft.fillRect(0, 471, 800, 9, TFT_BLACK);
  tft.drawString("72", 75, 482);
  tft.drawString("66", 135, 482);
  tft.drawString("60", 195, 482);
  tft.drawString("54", 255, 482);
  tft.drawString("48", 315, 482);
  tft.drawString("42", 375, 482);
  tft.drawString("36", 435, 482);
  tft.drawString("30", 495, 482);
  tft.drawString("24", 555, 482);
  tft.drawString("18", 614, 482);
  tft.drawString("12", 674, 482);
  tft.drawString("6", 737, 482);
  tft.drawString("0", 797, 482);
}

void updateBlynk() {
  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_BLACK, TFT_RED);
  tft.setTextPadding(tft.textWidth("BLYNK"));
//  tft.fillRect(644, 40, 36, 10, TFT_RED); 
  tft.drawString("BLYNK", 674, 40);
  tft.unloadFont();
  Blynk.virtualWrite(V8,TempInDoor);
  Blynk.virtualWrite(V9,HumiInDoor);
//  Blynk.virtualWrite(V9,(currentMillis/1000));
//  Serial.println(currentMillis/1000);
  Blynk.virtualWrite(V10,TempOutDoor);
  Blynk.virtualWrite(V11,HumiOutDoor);
  Blynk.virtualWrite(V12,winddy);
  Blynk.virtualWrite(V13,fWindSpeed);
  Blynk.virtualWrite(V14,mmDaily);
  Blynk.virtualWrite(V15,maxWindHour);
  Blynk.virtualWrite(V16,fvalueBat);
  Blynk.virtualWrite(V17,fvalueSol);  
  tft.fillRect(674, 40, 36, 11, TFT_BLACK);  
}

void saveData(time_t local_time) {
  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TL_DATUM);  
  if (!sdReady) {
    sdReady = SD.begin(SD_CS, spiSD);
    if (!sdReady) {
      Serial.println("⚠️ SD missing → skip");
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.drawString("SD", 593, 10);
      return;
    }
    Serial.println("🔁 SD mounted");
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("SD", 593, 10);
  }

  File f = SD.open("/weather_log.csv", FILE_APPEND);
  if (!f) {
    Serial.println("❌ File open fail → mark SD lost");
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("SD", 593, 10);
    sdReady = false;      // lần sau sẽ thử mount lại
    return;
  }
  // ---- Định dạng thời gian YYYY-MM-DD HH:MM ----
  char datetime[20];
  sprintf(datetime, "%04d-%02d-%02d %02d:%02d",
          year(local_time),
          month(local_time),
          day(local_time),
          hour(local_time),
          minute(local_time));

  f.printf("%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
           datetime,
           maxWindHour,
           mmDaily,
           TempOutDoor,
           HumiOutDoor,
           TempInDoor,
           HumiInDoor,
           winddy);

  f.close();
  logCount++;
  tft.setTextDatum(TR_DATUM);    
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("99999"));  
  tft.drawString(String(logCount), 635, 10);  
}

/***************************************************************************************
**                          Draw the clock digits
***************************************************************************************/
void drawTime(time_t local_time) {
  sprTime.fillSprite(TFT_BLACK);
  sprTime.setTextDatum(BC_DATUM);
  sprTime.setTextColor(TFT_GREENYELLOW, TFT_BLACK);

  sprTime.loadFont(AA_FONT_20, LittleFS);
  sprTime.setTextPadding(tft.textWidth("WEDNESDAY"));
  sprTime.drawString(dayStr(weekday(local_time)), 70, 30);
  String dateNow = "";
  dateNow += day(local_time);
  dateNow += "/";
  dateNow += month(local_time);
  dateNow += "/";
  dateNow += year(local_time);
  sprTime.drawString(dateNow, 70, 56);
  tft.unloadFont();

  sprTime.loadFont(AA_FONT_70, LittleFS);
  String timeNow = "";
  if (hour(local_time) < 10) timeNow += "0";
  timeNow += hour(local_time);
  timeNow += ":";
  if (minute(local_time) < 10) timeNow += "0";
  timeNow += minute(local_time);
  sprTime.setTextPadding(tft.textWidth("44:44"));  // String width + margin
  sprTime.drawString(timeNow, 222, 68);

  sprTime.pushSprite(0, 0);
  tft.unloadFont();
  // AUTO DIM BACKLIGHT
  int lux = lightMeter.readLightLevel();
  if (lux >= 0 && lux < 20)   ledcWrite(LEDpin, 50);
  else if (lux >= 20 && lux < 40)   ledcWrite(LEDpin, 150);
  else if (lux >= 40 && lux < 80)   ledcWrite(LEDpin, 200);
  else if (lux >= 80)   ledcWrite(LEDpin, 255);
/*
if (hour(local_time) >= 21 || hour(local_time) < 9)
  ledcWrite(0, 50);
/*else if (hour(local_time) == 6 && minute(local_time) < 30)
  ledcWrite(pwmChannel, 90);
else if (hour(local_time) == 7 && minute(local_time) > 15)
  ledcWrite(pwmChannel, 20);  
else if (hour(local_time) > 7 && hour(local_time) < 11)
  ledcWrite(pwmChannel, 20);
else if (hour(local_time) == 11 && minute(local_time) < 40)
  ledcWrite(pwmChannel, 20);
else if (hour(local_time) > 12 && hour(local_time) < 17)
  ledcWrite(pwmChannel, 20);
else if (hour(local_time) == 17 && minute(local_time) < 10)
  ledcWrite(pwmChannel, 20);
* 
else
  ledcWrite(0, 255);
*/
}

/***************************************************************************************
**                          Fetch the weather data  and update screen
***************************************************************************************/
// Update the Internet based information and update screen
void updateWeatherData(int h, int m) {
  currentWeather = new CurrentWeather();
  HTTPClient http;
  // CurrentWeather Get JSON
  // API endpoint for hourly weather
  String url1h = "https://weather.visualcrossing.com/VisualCrossingWebServices/rest/services/timeline/";
  url1h += locationKey;
  url1h += "/today?unitGroup=metric&include=current&key=";
  url1h += apiKey;
  
  bool validData = false;
  int retry = 0;
  
  while (!validData && retry < 5) {
    http.setConnectTimeout(4000);
    http.setTimeout(4000);
    http.begin(url1h);
    int httpResponseCode1h = http.GET();
    if (httpResponseCode1h <= 0)  {
      http.end();
      retry++;
      delay(200);
      continue;
    }
//  if (httpResponseCode1h > 0) {
    String jsonResponse = http.getString();
//    tft.loadFont(AA_FONT_10, LittleFS);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextPadding(tft.textWidth("Request did not match an endpoint."));  // String width + margin
    //tft.drawString(timeup + "/" + updatetime, 710, 40);
     // Check error
    if (jsonResponse.startsWith("Maximum daily cost exceeded")) {
      //Serial.println("❌ VisualCrossing: Vuot 1000 records/ngay!");
      tft.drawString("Maximum daily cost exceeded", 710, 40);
      validData = false;
      http.end();
      break;
    }
    if (jsonResponse.startsWith("Request did not match an endpoint.")) {
      tft.drawString("Request did not match an endpoint.", 710, 40);
      validData = false;
      http.end();
      break;
    }
    if (jsonResponse.startsWith("No session info found. ")) {
      tft.drawString("No session info found.  ", 710, 40);
      validData = false;
      http.end();
      break;
    }
    const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(31) + 1024;
    DynamicJsonDocument doc(capacity);
    DeserializationError error = deserializeJson(doc, jsonResponse);

    if (!error) {
      JsonObject weatherData = doc["currentConditions"];
      if (weatherData["feelslike"].is<float>()) {
        validData = true;
        currentWeather->timeupdate = weatherData["datetime"].as<String>();
//        jsonglobal = jsonResponse.substring(15, 34) + "/Api: " + shortKey;
        currentWeather->weatherIcon = weatherData["icon"].as<String>();
//        String Icon = String(currentWeather->weatherIcon).substring(0, 24);
//        int weathercon = currentWeather->weatherIcon;
        currentWeather->temperature = weatherData["feelslike"];
        currentWeather->humidity = weatherData["humidity"];
        currentWeather->windDirectionDegrees = weatherData["winddir"];
        currentWeather->windSpeedKmph = weatherData["windgust"];
        currentWeather->rainmm = weatherData["precip"];
        currentWeather->UVindex = weatherData["uvindex"];
        currentWeather->Cloudcover = weatherData["cloudcover"];
        int is_day = (h >= 6 && h < 18) ? 1 : 0;
        currentWeather->isday = is_day;        
      }
    }
  http.end();  
  }

  if (!validData) {
    delete currentWeather;  
    return;
  }

  GetWeatherData = true;
  fillSegment(6, 27, 0, 360, 5, TFT_RED);
  fillSegment(6, 27, 0, 360, 6, TFT_BLACK);
  drawCurrentWeather(h, m);
  tft.unloadFont();
  delete currentWeather;
}

/***************************************************************************************
**                          Draw the current weather
***************************************************************************************/
void drawCurrentWeather(int h, int m) {
  String timeup = "";
  if (h < 10) timeup += "0";
  timeup += h;
  timeup += ":";
  if (m < 10) timeup += "0";
  timeup += m;
  //  String updatetime = String(urrentWeather->timeupdate).substring(11, 16);
  String updatetime = "";
  updatetime += String(currentWeather->timeupdate).substring(0, 5);
  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Request did not match an endpoint."));  // String width + margin
  tft.drawString(timeup + "/" + updatetime, 710, 40);
//  String code = "";
//  code += String(currentWeather->weatherIcon).substring(0, 4);
//  tft.drawString("/" + code, 800, 40);
//  tft.drawString("20:20/20:00", 769, 60);
  String weathertest = "";
  weathertest = getMeteoconIcon(currentWeather->weatherIcon, currentWeather->Cloudcover, currentWeather->isday);
  ui.drawBmp("/icon/" + weathertest + ".bmp", 0, 63); 
  String weatherText = "";
  // Weather Text
  strncpy(shortKey, apiKey, 5);
  shortKey[5] = '\0';  // Đảm bảo kết thúc chuỗi
  weatherText = "Api:" + String(shortKey) + "|location:" + locationKey;
//  tft.unloadFont();
  sprJson.fillSprite(TFT_BLACK);
  sprJson.loadFont(AA_FONT_10, LittleFS);
  sprJson.setTextColor(TFT_RED, TFT_BLACK);
  sprJson.setTextDatum(TL_DATUM);
  sprJson.setTextPadding(tft.textWidth("The allowed number|Api:888888"));
  sprJson.drawString(weatherText, 0, 1);
  sprJson.pushSprite(644, 60);
  tft.unloadFont();
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TC_DATUM);
  tft.setTextPadding(0);
  String cloudCover = "";
  cloudCover += currentWeather->Cloudcover;
  cloudCover += "%";
  tft.setTextPadding(tft.textWidth(" 100%"));
  tft.drawString(cloudCover, 195, 94);

  int UVindex = currentWeather->UVindex;
  tft.setTextPadding(0);  
  tft.setTextPadding(tft.textWidth("88"));  // Max string length?
  tft.drawString(String(UVindex), 273, 94);

  tft.setTextPadding(0);
  weatherText = String(currentWeather->windSpeedKmph, 1);
  weatherText += "km/h"; 
  tft.setTextPadding(tft.textWidth("88,88km/h"));  // Max string length?
  tft.drawString(weatherText, 195, 146);

  tft.setTextPadding(0);
  weatherText = String(currentWeather->rainmm, 2);
  weatherText += "mm";
  tft.setTextPadding(tft.textWidth("88,88mm"));
  tft.drawString(weatherText, 273, 146);  //"Rain":{"Value":1.9,"Unit":"mm",
  tft.unloadFont();
  int windAngle = (currentWeather->windDirectionDegrees + 22.5) / 45;
  if (windAngle > 7) windAngle = 0;
  String wind[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
  ui.drawBmp("/wind/" + wind[windAngle] + ".bmp", 114, 86);

  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  // Font ASCII code 0xB0 is a degree symbol, but o used instead in small font
  tft.setTextPadding(tft.textWidth("88,8°"));  // Max width of values
//  String weatherText = "";
  weatherText = String(currentWeather->temperature, 1);  // Make it integer temperature
  weatherText += "°";
  //Serial.print("Temperature: ");
  //Serial.print(currentWeather->temperature);
  tft.drawString(weatherText, 322, 186);
  tft.unloadFont();
  String humidity = "";
  humidity += currentWeather->humidity;
  humidity += "%";
  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextDatum(TL_DATUM);
//  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("100%"));
  tft.drawString(humidity, 322, 228);
}

void updateAstronomy(time_t local_time) {
  dailyWeather = new DailyWeather();
  HTTPClient http;
  // DailyWeather Get JSON
  // API endpoint for ForeCasts 1 Day
  String url1d = "https://weather.visualcrossing.com/VisualCrossingWebServices/rest/services/timeline/";
  url1d += locationKey;
  url1d += "/today?unitGroup=metric&include=current&elements=moonrise,moonset,moonphase,sunrise,sunset&key=";
  url1d += apiKey;
  bool validData1 = false;
  int retryM = 0;
  
  while (!validData1 && retryM < 5) { 
    http.setConnectTimeout(4000);
    http.setTimeout(4000); 
    http.begin(url1d);
    int httpResponseCode1d = http.GET();
    if (httpResponseCode1d <= 0)  {
      http.end();
      retryM++;
      delay(200);
      continue;
    }
//    if (httpResponseCode1d > 0) {
    String jsonResponse1d = http.getString();
    //    Serial.println(jsonResponse1d);
    const size_t capacity = JSON_ARRAY_SIZE(1) + JSON_OBJECT_SIZE(31) + 1024;
    DynamicJsonDocument doc(capacity);
    DeserializationError error = deserializeJson(doc, jsonResponse1d);
    if (!error) {
      JsonObject astronomy = doc["days"][0];
      if (astronomy["moonphase"].is<float>()) {
        validData1 = true;     
        dailyWeather->sunrise = astronomy["sunrise"].as<String>();
        dailyWeather->sunset = astronomy["sunset"].as<String>();
        dailyWeather->moonrise = astronomy["moonrise"].as<String>();
        dailyWeather->moonset = astronomy["moonset"].as<String>();
        dailyWeather->moonage = astronomy["moonphase"].as<float>();
   /* Serial.println("🌅 Mặt trời mọc: " + dailyWeather->sunrise);
    Serial.println("🌇 Mặt trời lặn: " + dailyWeather->sunset);
    Serial.println("🌙 Mặt trăng mọc: " + dailyWeather->moonrise);
    Serial.println("🌘 Mặt trăng lặn: " + dailyWeather->moonset); */ 
      }
    }
  http.end();  
  }
  if (!validData1) {
    delete dailyWeather;
    return;
  }
  GetAstronomy = true;
  drawAstronomy(local_time);
  tft.unloadFont();
  delete dailyWeather;
}

/***************************************************************************************
**                          Draw Sun rise/set, Moon, cloud cover and humidity
***************************************************************************************/
void drawAstronomy(time_t local_time) {
  int d = day(local_time);
  int m = month(local_time);
  int y = year(local_time);
  LunarDate lunar = convertSolar2Lunar(d, m, y);

  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("30/12"));  
  tft.drawString(String(lunar.day) + "/" + String(lunar.month), 279, 207);
  ui.drawBmp("/moon/" + String(lunar.day) + ".bmp", 90, 167);
  String sunrise12 = String(dailyWeather->sunrise).substring(0, 8);
  String sunrise24 = convertTo24HourFormat(sunrise12);
  String sunset12  = String(dailyWeather->sunset).substring(0, 8);
  String sunset24 = convertTo24HourFormat(sunset12);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("88 : 88"));
  tft.setTextDatum(BC_DATUM);
  tft.drawString(String(sunrise24), 40, 207);
  tft.drawString(String(sunset24), 40, 248);
  String moonrise12 = String(dailyWeather->moonrise).substring(0, 8);
  String moonrise24 = convertTo24HourFormat(moonrise12);
  String moonset12  = String(dailyWeather->moonset).substring(0, 8);
  String moonset24 = convertTo24HourFormat(moonset12);
  tft.drawString(String(moonrise24), 219, 207);
  tft.drawString(String(moonset24), 219, 248); 
}


/***************************************************************************************
**                          Get the icon file name from the index number
***************************************************************************************/
const char* getMeteoconIcon(String weatherIcon, int Cloudcover, int isday)
{
  if (isday == 1) {
    if (weatherIcon == "clear-day" && Cloudcover <= 10) return "Clear";
    if (weatherIcon == "clear-night" && Cloudcover <= 10) return "Clear";

    if (weatherIcon == "clear-day" && Cloudcover > 10) return "Mostly Clear";
    if (weatherIcon == "clear-night" && Cloudcover > 10) return "Mostly Clear";

    if (weatherIcon == "partly-cloudy-day" && Cloudcover <= 50) return "Partly Cloudy";
    if (weatherIcon == "partly-cloudy-night" && Cloudcover <= 50) return "Partly Cloudy";

    if (weatherIcon == "partly-cloudy-day" && Cloudcover > 50) return "Mostly Cloudy";
    if (weatherIcon == "partly-cloudy-night" && Cloudcover > 50) return "Mostly Cloudy";

    if (weatherIcon == "cloudy") return "Cloudy";
    if (weatherIcon == "fog") return "Fog";
    if (weatherIcon == "wind") return "Wind";
    if (weatherIcon == "rain") return "Rain";
    if (weatherIcon == "showers-day") return "Light Rain";
    if (weatherIcon == "thunder-rain" || weatherIcon == "thunder-showers-day") return "Thunderstorm";
  } else {
    if (weatherIcon == "clear-night" && Cloudcover <= 10) return "nt_Clear";
    if (weatherIcon == "clear-night" && Cloudcover > 10) return "nt_Mostly Clear";
    if (weatherIcon == "partly-cloudy-night" && Cloudcover <= 50) return "nt_Partly Cloudy";
    if (weatherIcon == "partly-cloudy-night" && Cloudcover > 50) return "nt_Mostly Cloudy";
    if (weatherIcon == "cloudy") return "Cloudy";
    if (weatherIcon == "fog") return "Fog";
    if (weatherIcon == "wind") return "Wind";
    if (weatherIcon == "rain") return "Rain";
    if (weatherIcon == "showers-night") return "Light Rain";
    if (weatherIcon == "thunder-rain" || weatherIcon == "thunder-showers-night") return "Thunderstorm";
  }
  return "Unknown";
}
/*
{
  if (weatherIcon == "clear-day" && Cloudcover <= 10) return "Clear";
  if (weatherIcon == "clear-day" && Cloudcover > 10) return "Mostly Clear";
  if (weatherIcon == "partly-cloudy-day" && Cloudcover <= 50) return "Partly Cloudy";
  if (weatherIcon == "partly-cloudy-day" && Cloudcover > 50) return "Mostly Cloudy";
  if (weatherIcon == "cloudy") return "Cloudy";
  if (weatherIcon == "fog") return "Fog";
  if (weatherIcon == "wind") return "Wind";
  if (weatherIcon == "rain") return "Rain";
  if (weatherIcon == "showers-day" || weatherIcon == "showers-night") return "Light Rain";
  if (weatherIcon == "thunder-rain" || weatherIcon == "thunder-showers-day" || weatherIcon == "thunder-showers-night") return "Thunderstorm";

  if (weatherIcon == "clear-night" && Cloudcover <= 10) return "nt_Clear";
  if (weatherIcon == "clear-night" && Cloudcover > 10) return "nt_Mostly Clear";
  if (weatherIcon == "partly-cloudy-night" && Cloudcover <= 50) return "nt_Partly Cloudy";
  if (weatherIcon == "partly-cloudy-night" && Cloudcover > 50) return "nt_Mostly Cloudy";

  return "Unknown";
}
*/
/***************************************************************************************
**                          Update progress bar
***************************************************************************************/
void drawProgress(uint8_t percentage, String text) {
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
  String hourStr = hour12.substring(0, 2);     // "05"
  String minuteStr = hour12.substring(3, 5);   // "12"
  String period = hour12.substring(6);         // "PM" hoặc "AM"

  int hour = hourStr.toInt();
  int minute = minuteStr.toInt();

  if (period == "PM" && hour != 12) {
    hour += 12;
  } else if (period == "AM" && hour == 12) {
    hour = 0;
  }

  // Định dạng lại theo chuẩn 24h
  String result = (hour < 10 ? "0" : "") + String(hour) + ":" +
                  (minute < 10 ? "0" : "") + String(minute);
  return result;
}