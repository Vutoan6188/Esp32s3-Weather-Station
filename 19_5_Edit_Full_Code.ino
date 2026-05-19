#define AA_FONT_SMALL "fonts/NotoSansBold15"  // 15 point sans serif bold
#define AA_FONT_LARGE "fonts/NotoSansBold36"  // 36 point sans serif bold
#define AA_FONT_10 "fonts/NotoSans-Bold10"
#define AA_FONT_13 "fonts/NotoSans-Bold13"
#define AA_FONT_20 "fonts/NotoSans-Bold20"
#define AA_FONT_30 "fonts/NotoSans-Bold30"
#define AA_FONT_40 "fonts/NotoSans-Bold40"
#define AA_FONT_70 "fonts/NotoSans-Bold70"
#define FIRMWARE_VERSION "0.1.0"

/**                         Load the libraries and settings
***************************************************************************************/
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <TFT_eSPI.h>  // https://github.com/Bodmer/TFT_eSPI
#include <TJpg_Decoder.h>
#include <FS.h>
#include <LittleFS.h>
#include "GfxUi.h"  // Attached to this sketch
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <EEPROM.h>
#include <BlynkSimpleEsp32.h>
#include <Update.h>
#include "All_Settings.h"
#include <ArduinoJson.h>  // Đưa ArduinoJson lên đây, bỏ JSON_Decoder (không dùng nữa)
#include "NTP_Time.h"     // Attached to this sketch, see that tab for library needs
#include "VietnameseLunar.h"
#include "Wire.h"

//AHT30
#include <Adafruit_AHTX0.h>
//BH1750
#include <BH1750.h>
// Ebyte E220
#include "LoRa_E220.h"

/***************************************************************************************
**                          Define the globals and class instances
***************************************************************************************/
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprTime = TFT_eSprite(&tft);

// Tối ưu: Gộp các cờ (flags) để dễ quản lý, boolean và bool là một, nên dùng bool cho chuẩn C++
bool booted = false;
bool syncDone = false;
bool newLoRaData = false;
bool newBlynkData = false;
bool BlynkErrorDrawn = false;
bool GetWeatherData = false;
bool GetAstronomy = false;

WiFiManager wifiManager;
GfxUi ui = GfxUi(&tft);

// Tối ưu: Đổi long sang uint32_t (tương đương unsigned long) cho các biến dùng với millis() để tránh lỗi tràn số (rollover)
uint32_t lastDownloadUpdate = millis();
int last_GRAPH = -1;

// Khai báo ngoại vi
Adafruit_AHTX0 aht;
BH1750 lightMeter;

#define LORA_RX_PIN 4
#define LORA_TX_PIN 5
#define ENABLE_RSSI true
LoRa_E220 e220ttl(LORA_TX_PIN, LORA_RX_PIN, &Serial2, UART_BPS_RATE_9600, SERIAL_8N1);

WiFiClientSecure client;

/***************************************************************************************
**                          Declare prototypes
***************************************************************************************/
void updateWeatherData();
void updateAstronomy();
void drawProgress(uint8_t percentage, const char* text);  // Tối ưu: Đổi String text sang const char*
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
void checkButton();
void saveSDData();
void readSDData();
const char* getMeteoconIcon(const char* weatherIcon, int Cloudcover, int isday);  // Tối ưu: Đổi String sang const char*
void drawAstronomy();
void fillSegment(int x, int y, int start_angle, int sub_angle, int r, unsigned int colour);

// Tối ưu: Đổi String sang const char* trong các hàm xử lý chuỗi
int leftOffset(const char* text, const char* sub);
int rightOffset(const char* text, const char* sub);
int splitIndex(const char* text);

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (y >= tft.height()) return 0;
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

/***************************************************************************************
**                          Structs & Data
***************************************************************************************/
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
int LEDpin = 3;
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
float Pressure, Altitude;
#define SEALEVELPRESSURE_HPA (1013.25)

/***************************************************************************************
**                          Graph Buffers & Timers
***************************************************************************************/
float graphWindSpeed, graphWindMax, graphWinddy, graphRain;
float graphHumOUT, graphTemOUT, graphHumIN, graphTemIN;

// Khai báo 2 mảng lưu lịch sử 24 giờ (từ 0h đến 23h)
float history_TempOut[24] = {0.0};
float history_RainDaily[24] = {0.0};

// CẢNH BÁO: Mảng đồ thị của ca tiêu tốn khá nhiều RAM (800 int = 1.6KB/mảng)
// Tổng cộng: 14 mảng x 800 int = Hơn 22KB RAM chỉ dành cho đồ thị!
// Nếu không cần vẽ tới 800 điểm ảnh cùng lúc, ca nên xem xét giảm số 800 này xuống (ví dụ 480 là đủ quét ngang màn hình).
int x[800], y[800], z[800], a[800], b[800], g[800];
int c[800], d[800], v[800], f[800];
int n[800], h[800], m[800], j[800];
int currentPage = 0;  // Trang 0: Màn hình chính, Trang 1: Màn hình lịch sử SD
// Timers (Dùng uint32_t thay vì unsigned long để code ngắn gọn, bản chất là như nhau)
uint32_t currentMillis = 0;
uint32_t connectMillis = 0;
uint32_t LoRaMillis = 0;
uint32_t lastCheckPower = 0;
uint32_t logCount = 0;

/***************************************************************************************
**                          Compass Math
***************************************************************************************/
#define CENTER_X 398
#define CENTER_Y 82
#define RADIUS 50
// Tối ưu: Đổi tên biến đuôi la bàn để sau này chạy hàm xóa sạch sẽ
int lastArrowHeadX1 = CENTER_X;
int lastArrowHeadY1 = CENTER_Y;
int lastArrowHeadX2 = CENTER_X;
int lastArrowHeadY2 = CENTER_Y;
int lastArrowTailX = CENTER_X;  // Đổi từ Tip thành Tail
int lastArrowTailY = CENTER_Y;  // Đổi từ Tip thành Tail

/***************************************************************************************
**                          Config Data (WiFi/GitHub)
***************************************************************************************/
char locationKey[40] = "";
char apiKey[48] = "";
char authKey[40] = "";

const char* GITHUB_USER = "Vutoan6188";
const char* GITHUB_REPO = "Esp32s3-Weather-Station";
const char* FIRMWARE_FILENAME = "Esp32s3_Weather_Station.ino.bin";

bool sdReady = false;
/***************************************************************************************
**                          Setup
***************************************************************************************/
void setup() {
  // Tắt hoàn toàn Bluetooth để tiết kiệm pin
  btStop();
  pinMode(BUTTON_PIN, INPUT_PULLUP);   // Kích hoạt pull-up nội bộ cho GPIO09
  pinMode(SLEEP_PIN, INPUT_PULLDOWN);  // Tránh floating khi mất nguồn

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

  // Kiểm tra trạng thái nút nhấn khi khởi động
  if (digitalRead(BUTTON_PIN) == LOW) {
    tft.loadFont(AA_FONT_SMALL, LittleFS);
    tft.setTextDatum(BC_DATUM);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextPadding(240);
    tft.drawString(" ", 400, 220);
    tft.drawString("Clearing the old configuration", 400, 240);

    // Xóa cấu hình WiFi và thông số
    WiFiManager wifiManager;
    wifiManager.resetSettings();  // Xóa cấu hình WiFi đã lưu
    EEPROM.write(0, 0);           // Xóa locationKey, apiKey và authKey trong EEPROM
    EEPROM.commit();
  } else {
    // Nếu không nhấn nút, đọc từ EEPROM
    EEPROM.get(0, locationKey);
    EEPROM.get(40, apiKey);
    EEPROM.get(88, authKey);
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
    ESP.restart();  // Khởi động lại nếu kết nối không thành công
  }

  // Clear bottom section of screen
  tft.fillRect(0, 100, 800, 200, TFT_BLACK);
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spiSD)) {
    tft.drawString("SD init failed", 0, 80);
  } else {
    tft.drawString("SD ready", 0, 80);
  }

  // Tối ưu: Dùng snprintf thay thế phép cộng chuỗi WiFi.SSID()
  char wifiBuf[64];
  snprintf(wifiBuf, sizeof(wifiBuf), "Connecting to WiFi SSID: %s", WiFi.SSID().c_str());
  tft.drawString(wifiBuf, 0, 100);

  checkFirmwareUpdate();

  // Lưu thông số mới vào EEPROM
  strcpy(locationKey, custom_locationKey.getValue());
  strcpy(apiKey, custom_apiKey.getValue());
  strcpy(authKey, custom_authKey.getValue());

  EEPROM.put(0, locationKey);
  EEPROM.put(40, apiKey);
  EEPROM.put(88, authKey);
  EEPROM.commit();

  // Tối ưu: Dùng snprintf in thông tin cấu hình ra màn hình (Sạch bóng String)
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

  delay(3000);
  tft.fillScreen(TFT_BLACK);

  TJpgDec.setJpgScale(1);
  TJpgDec.setCallback(tft_output);
  TJpgDec.setSwapBytes(true);

  // Draw splash screen
  if (LittleFS.exists("/splash/weather_icon.jpg") == true) {
    TJpgDec.drawFsJpg(160, 0, "/splash/weather_icon.jpg", LittleFS);
  }
  delay(3000);

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
  udp.begin(localPort);
  syncTime();  // Chip lấy giờ chuẩn từ internet/vệ tinh tại đây

  // --- ĐOẠN ĐÚC FILE THEO THÁNG SAU KHI ĐÃ CÓ GIỜ CHUẨN ---
  if (sdReady)  // Chỉ chạy nếu phần cứng thẻ SD ở trên đã khởi động thành công
  {
    time_t now_time = now();
    char bootFileName[32];

    // Đúc tên file động theo tháng. Ví dụ kết quả: "/log_2026_05.csv"
    snprintf(bootFileName, sizeof(bootFileName), "/log_%04d_%02d.csv",
             year(now_time),
             month(now_time));

    // Nếu file của tháng này chưa tồn tại thì tiến hành tạo mới và nạp Header
    if (!SD.exists(bootFileName)) {
      File f = SD.open(bootFileName, FILE_WRITE);
      if (f) {
        // Ghi dòng tiêu đề khớp 100% với các cột cũ của ca
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
        Serial.print(">>>> Da tao file thang moi: ");
        Serial.println(bootFileName);
      }
    } else {
      Serial.print(">>>> File thang nay da co san: ");
      Serial.println(bootFileName);
    }
  }
  // --- KẾT THÚC ĐOẠN ĐÚC FILE THEO THÁNG ---

  drawProgress(50, "Update conditions...");
  tft.unloadFont();

  // Default value INDOOR / OUTDOOR / Rain / Wind
  maxTempInDoor = 0;
  minTempInDoor = 99;
  maxHumiInDoor = 0;
  minHumiInDoor = 99;
  maxTempOutDoor = 0;
  minTempOutDoor = 99;
  maxHumiOutDoor = 0;
  minHumiOutDoor = 99;
  mmHourly = 0;
  mmDaily = 0;
  mmGraph = 0;
  maxWindHour = 0;
  valueLoRaStrong = 0;
  valueWiFiStrong = 0;
  valueBattery = 0;
  valueSolar = 0;

  // --- SỬA LỖI TRÀN MẢNG ĐỒ THỊ ---
  // Mảng có 800 phần tử thì index chạy từ 799 về 0. Để i = 800 sẽ bị sập RAM đột ngột!
  for (int i = 799; i >= 0; i--) {
    x[i] = 9999;
    y[i] = 9999;
    z[i] = 9999;
    a[i] = 9999;
    b[i] = 9999;
    g[i] = 9999;  // Graph
    c[i] = 9999;
    d[i] = 9999;
    v[i] = 9999;
    f[i] = 9999;  // Graph1
    n[i] = 9999;
    h[i] = 9999;
    m[i] = 9999;
    j[i] = 9999;  // Graph2
  }

  drawProgress(100, "Done...");
  tft.fillScreen(TFT_BLACK);

  drawInterfaceSkeleton();

  booted = true;
}


void drawInterfaceSkeleton() {
  // Vẽ các đường Grid lines giao diện
  tft.drawLine(10, 163, 680, 163, 0x4228);
  tft.drawLine(310, 58, 310, 250, 0x4228);
  tft.drawLine(10, 250, 790, 250, 0x4228);
  tft.drawLine(10, 57, 320, 57, 0x4228);
  tft.drawLine(474, 57, 614, 57, 0x4228);
  tft.drawLine(440, 163, 440, 250, 0x4228);
  tft.drawLine(622, 163, 622, 250, 0x4228);
  tft.drawLine(639, 0, 639, 70, 0x4228);

  // Vẽ icon tĩnh từ thẻ nhớ
  ui.drawBmp("/icon50/Altitude.bmp", 472, 4);
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
  ui.drawBmp("/icon50/rain.bmp", 565, 68);

  // --- TỐI ƯU TUYỆT ĐỐI KHU VỰC IN THÔNG TIN FW (TRẢM STRING) ---
  char weatherBuf[64];
  char shortKey[6];
  char shortLocation[6];

  // Trích 5 ký tự an toàn không sợ lỗi chuỗi
  snprintf(shortKey, sizeof(shortKey), "%.5s", apiKey);
  snprintf(shortLocation, sizeof(shortLocation), "%.5s", locationKey);

  // Gom tất cả vào khuôn đúc duy nhất
  snprintf(weatherBuf, sizeof(weatherBuf), "Api:%s|location:%s|fw:%s", shortKey, shortLocation, FIRMWARE_VERSION);

  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("API:EN9AS|location:12.78|fw:0.0.2"));
  tft.drawString(weatherBuf, 624, 60);

  // In các nhãn đồ thị mưa
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

  // In các nhãn hệ thống
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
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Station Runtime"));
  tft.drawString("Station runtime", 624, 40);

  // In nhãn Khu vực
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("Forecasts"));
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.drawString("FORECAST", 321, 169);
  tft.setTextPadding(tft.textWidth("OUTDOOR"));
  tft.drawString("OUTDOOR", 454, 169);
  tft.setTextPadding(tft.textWidth("INDOOR"));
  tft.drawString("INDOOR", 636, 169);

  // VẼ ĐƠN VỊ km/h
  tft.loadFont(AA_FONT_13, LittleFS);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(tft.textWidth("km/h"));  // TỐI ƯU: Đưa xuống sau loadFont để tính chiều rộng chuẩn xác 100%
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("km/h", CENTER_X, CENTER_Y + 50);
  tft.unloadFont();

  tft.setTextPadding(0);  // Reset padding về 0 an toàn cho các hàm vẽ sau
  //  Serial.println("Đã vẽ lại khung xương giao diện Trang 0!");
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

  // =========================================================
  // TÁC VỤ 1: CÁC TÁC VỤ CHẠY NGẦM (DÙ Ở TRANG NÀO CŨNG PHẢI CHẠY)
  // =========================================================

  // 1. Giữ kết nối ngầm Blynk khi máy thức và có mạng
  if (digitalRead(SLEEP_PIN) == HIGH && WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }

  // 2. Liên tục check nút bấm để đảo trang (0 <-> 1)
  checkButton();

  // 3. Quét dữ liệu LoRa từ trạm cảm biến ngoại vi (Mỗi 50ms)
  if (currentMillis - LoRaMillis >= 50) {
    LoRaMillis = currentMillis;
    GetLoRa();
  }

  // 4. Nếu có dữ liệu LoRa mới -> Đẩy lên Blynk ngầm (Chỉ vẽ data live nếu đang ở Trang 0)
  if (newLoRaData) {
    if (currentPage == 0) {
      drawData(local_time);  // Chỉ vẽ lên màn hình nếu đang ở trang 0
    }
    if (Blynk.connected()) {
      updateBlynk();  // Luôn luôn cập nhật lên Blynk ngầm
    }
    newLoRaData = false;
  }

  // 5. Kiểm tra chế độ tiết kiệm năng lượng (Light Sleep)
  if (currentMillis - lastCheckPower >= 1000) {
    lastCheckPower = currentMillis;
    if (digitalRead(SLEEP_PIN) == LOW) {
      enterLightSleep();
    }
  }

  // 6. Tự động kết nối lại WiFi nếu mất mạng quá 10 phút
  if ((currentMillis - connectMillis >= 600000) && (WiFi.status() != WL_CONNECTED)) {
    connectMillis = currentMillis;
    WiFi.disconnect();
    WiFi.reconnect();
  }

  // 7. Cập nhật dữ liệu thiên văn lúc 00:01 sáng ngầm
  if (booted || (h == 0 && m == 1 && !GetAstronomy)) {
    updateAstronomy(local_time);
    GetAstronomy = true;
  }
  if (h != 0) GetAstronomy = false;

  // 8. Cập nhật Weather API đầu mỗi giờ ngầm (Phút 00, giây > 5)
  if (booted || (m == 0 && s > 5 && !GetWeatherData)) {
    updateWeatherData(h, m);
    GetWeatherData = true;
  }
  if (m != 0) GetWeatherData = false;

  // 9. Đồng bộ thời gian NTP vào phút 15 và 45 ngầm
  if (booted || ((m == 15 || m == 45) && s > 5 && !syncDone)) {
    syncTime();
    syncDone = true;
  }
  if (s > 10) syncDone = false;


  // =========================================================
  // TÁC VỤ 2: PHÂN LUỒNG HIỂN THỊ (CHỈ QUYẾT ĐỊNH VIỆC VẼ MÀN HÌNH)
  // =========================================================
  switch (currentPage) {
    case 0:
      {
        // Giám sát kết nối Blynk và in chữ cảnh báo lỗi lên màn hình chính
        static unsigned long lastCheckBlynk = 0;
        if (currentMillis - lastCheckBlynk >= 2000) {
          lastCheckBlynk = currentMillis;
          if (!Blynk.connected()) {
            if (!BlynkErrorDrawn) {
              BlynkErrorDrawn = true;
              tft.loadFont(AA_FONT_10, LittleFS);
              tft.setTextDatum(TL_DATUM);
              tft.setTextColor(TFT_BLACK, TFT_RED);
              tft.setTextPadding(tft.textWidth("BLYNK"));
              tft.drawString("BLYNK", 654, 50);
              tft.unloadFont();
            }
          } else {
            BlynkErrorDrawn = false;
          }
        }

        // Cập nhật đồng hồ lên màn hình theo mỗi phút
        if (booted || m != lastMinute) {
          drawTime(local_time);
          lastMinute = m;
        }

        break;
      }

    case 1:
      {
        // Màn hình lịch sử: Để trống hoàn toàn vì tất cả tác vụ vẽ đồ thị tĩnh
        // đã được hàm checkButton() gọi xử lý duy nhất 1 lần lúc bấm nút rồi!
        break;
      }
  }

  // Sau khi hết vòng loop đầu tiên thì hạ cờ booted xuống
  booted = false;
}

void enterLightSleep() {
  ledcWrite(LEDpin, 0);
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
  tft.setTextDatum(BL_DATUM);  // Bottom Left datum
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.drawString("Check firmware...", 0, 120);

  HTTPClient http;
  client.setInsecure();  // Bỏ xác minh chứng chỉ SSL

  // Tối ưu 1: Đúc khuôn URL kiểm tra phiên bản (Không dùng String)
  char apiUrl[256];
  snprintf(apiUrl, sizeof(apiUrl), "https://api.github.com/repos/%s/%s/releases/latest", GITHUB_USER, GITHUB_REPO);
  http.begin(client, apiUrl);
  http.addHeader("User-Agent", "ESP32-OTA-Agent");

  int httpCode = http.GET();
  char msgBuf[128];  // Tối ưu 2: Dùng chung 1 buffer cho tất cả các thông báo màn hình

  if (httpCode == 200) {
    String json = http.getString();  // Khối JSON nhỏ, an toàn để dùng String tạm thời
    int tagStart = json.indexOf("\"tag_name\":\"") + 12;
    int tagEnd = json.indexOf("\"", tagStart);

    // Tối ưu 3: Trích xuất phiên bản sang mảng char thay vì biến String
    char latestVersion[32];
    json.substring(tagStart, tagEnd).toCharArray(latestVersion, sizeof(latestVersion));

    snprintf(msgBuf, sizeof(msgBuf), "Firmware current: %s", FIRMWARE_VERSION);
    tft.drawString(msgBuf, 0, 140);
    snprintf(msgBuf, sizeof(msgBuf), "Firmware latest: %s", latestVersion);
    tft.drawString(msgBuf, 0, 160);
    delay(3000);

    // Dùng strcmp để so sánh mảng ký tự
    if (strcmp(latestVersion, FIRMWARE_VERSION) != 0) {
      tft.fillRect(0, 200, 800, 200, TFT_BLACK);
      tft.drawString("New firmware detected - starting OTA update...", 0, 180);

      // Tối ưu 4: Đúc khuôn URL tải file bin trực tiếp
      char binUrl[256];
      snprintf(binUrl, sizeof(binUrl), "https://github.com/%s/%s/releases/download/%s/%s", GITHUB_USER, GITHUB_REPO, latestVersion, FIRMWARE_FILENAME);

      snprintf(msgBuf, sizeof(msgBuf), "File URL: %s", binUrl);
      tft.drawString(msgBuf, 0, 200);

      HTTPClient https;
      https.begin(client, binUrl);
      https.addHeader("User-Agent", "ESP32-OTA-Agent");
      https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);  // Theo dõi redirect 302

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
                // Tối ưu 5: Đúc khuôn phần trăm tiến trình
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


void checkButton() {
  // Kiểm tra xem nút có được bấm hay không (Giả sử nút bấm tích cực mức LOW - nối đất)
  // Nếu nút của ca kích hoạt mức HIGH, ca sửa thành: digitalRead(BUTTON_PIN) == HIGH
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);  // Chờ 50ms để chống dội phím (Debounce)

    // Xác nhận lại lần nữa xem có đúng là ca đang bấm thật không
    if (digitalRead(BUTTON_PIN) == LOW) {

      tft.fillScreen(TFT_BLACK);  // Xóa sạch màn hình cũ trước khi chuyển trang

      // Đảo trạng thái trang (Từ 0 thành 1, từ 1 về 0)
      if (currentPage == 0) {
        currentPage = 1;
        Serial.println("Chuyển sang: MÀN HÌNH LỊCH SỬ SD");

        // 1. Lấy thời gian thực tại thời điểm bấm nút
        time_t c_time = TIMEZONE.toLocal(now(), &tz1_Code);

        // 2. Đúc tên file của tháng hiện tại để mở
        char targetFile[32];
        snprintf(targetFile, sizeof(targetFile), "/log_%04d_%02d.csv", year(c_time), month(c_time));

      } else {
        currentPage = 0;
        Serial.println("Quay về: MÀN HÌNH CHÍNH");

        // 1. Vẽ lại toàn bộ đường kẻ, khung viền, icon tĩnh
        drawInterfaceSkeleton();

        // 2. Ép cờ booted lên true để kích hoạt vẽ lại số liệu lập tức
        booted = true;
      }

      // Chờ cho đến khi ca nhả tay ra khỏi nút bấm mới chạy tiếp
      // Để tránh việc giữ tay lâu làm màn hình nhảy trang liên tục
      while (digitalRead(BUTTON_PIN) == LOW) {
        delay(10);
      }
    }
  }
}


void GetLoRa() {
  if (e220ttl.available() > 1) {
    // --- 1. Hiển thị trạng thái đang nhận ---
    tft.loadFont(AA_FONT_10, LittleFS);
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_BLACK, TFT_CYAN);
    tft.setTextPadding(tft.textWidth("LORA"));
    tft.drawString("LORA", 624, 50);

    ResponseContainer rc = e220ttl.receiveMessageRSSI();

    // --- 2. Xử lý sóng (Đã dọn dẹp cú pháp dư thừa) ---
    rssi = rc.rssi;
    dbm = -(256 - rssi);

    // --- 3. Xử lý JSON an toàn tuyệt đối ---
    StaticJsonDocument<250> doc;
    DeserializationError error = deserializeJson(doc, rc.data);

    // CHỈ lấy dữ liệu khi chuỗi JSON hợp lệ, tránh crash máy khi nhiễu sóng
    if (!error) {
      // Get Temp OUT (Nới lỏng giới hạn nếu có nhiệt độ âm)
      float xTempOutDoor = doc["temperature"];
      if (!isnan(xTempOutDoor) && xTempOutDoor > -20.0 && xTempOutDoor < 80.0) {
        TempOutDoor = xTempOutDoor;
      }

      // Get Hum OUT
      float xHumiOutDoor = doc["humidity"];
      if (!isnan(xHumiOutDoor) && xHumiOutDoor >= 0 && xHumiOutDoor <= 100.0) {
        HumiOutDoor = xHumiOutDoor;
      }

      // Ép kiểu rõ ràng (.as<float>()) để thư viện ArduinoJson biên dịch tối ưu nhất
      winddy = doc["angle"].as<float>();
      fWindSpeed = doc["speed"].as<float>();
      RainGauge = doc["rain"].as<float>();
      fvalueBat = doc["battery"].as<float>();
      fvalueSol = doc["solar"].as<float>();
      fTimeConnect = doc["timerun"].as<float>();
      Altitude = doc["altitude"].as<float>();
      Pressure = doc["pressure"].as<float>();

      // Xử lý cộng dồn
      if (fWindSpeed > maxWindHour) maxWindHour = fWindSpeed;
      mmTotal += RainGauge;
      mmDaily += RainGauge;
      mmGraph += RainGauge;
      mmHourly += RainGauge;

      newLoRaData = true;
    }

    // --- 4. Dọn dẹp màn hình và bộ nhớ ---
    tft.unloadFont();  // Cực kỳ quan trọng để không hao hụt RAM của TFT
    tft.fillRect(624, 50, 28, 11, TFT_BLACK);
  }
}

void drawData(time_t local_time) {
  int m = minute(local_time);
  int s = second(local_time);
  if (currentMillis > 30000) {
    drawAngle();
    drawWindSpeed();
    drawRain();
    drawSignal();
    //  Get Temp/Hum IN
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    float xTempInDoor = temp.temperature;  // Đọc nhiệt độ
    if (!isnan(xTempInDoor) && xTempInDoor > 0 && xTempInDoor < 80.0) {
      TempInDoor = xTempInDoor;
    }
    float xHumiInDoor = humidity.relative_humidity;  // Đọc độ ẩm
    if (!isnan(xHumiInDoor) && xHumiInDoor > 0) {
      HumiInDoor = xHumiInDoor;
    }
    drawTemHumOutdoor();
    drawTemHumIndoor();
  }
  ResetValue(local_time);
  tft.setTextPadding(0);
  tft.unloadFont();

  if ((m % 6 == 0 && s > 29) && (m != last_GRAPH)) {
    last_GRAPH = m;
    tft.loadFont(AA_FONT_10, LittleFS);
    GraphWindRain();
    GraphTemp();
    GraphHum();
    saveSDData(local_time);
    tft.setTextPadding(0);
    tft.unloadFont();
  }
}


void drawAngle() {
  // 1. XÓA MŨI TÊN CŨ (Sử dụng tên biến mới lastArrowTailX/Y của ca)
  if (lastArrowHeadX1 != CENTER_X || lastArrowHeadY1 != CENTER_Y) {
    tft.fillTriangle(lastArrowHeadX1, lastArrowHeadY1, lastArrowHeadX2, lastArrowHeadY2, lastArrowTailX, lastArrowTailY, TFT_BLACK);
  }

  // 2. TÍNH TOÁN TOẠ ĐỘ MŨI TÊN MỚI
  float radian = (winddy + 270) * DEG_TO_RAD;
  int arrowTipX = CENTER_X + cos(radian) * (RADIUS - 10);
  int arrowTipY = CENTER_Y + sin(radian) * (RADIUS - 10);
  int arrowTailX = CENTER_X + cos(radian) * (RADIUS + 10);
  int arrowTailY = CENTER_Y + sin(radian) * (RADIUS + 10);

  // Tính góc tam giác đuôi mũi tên
  float arrowAngle = atan2(arrowTailY - arrowTipY, arrowTailX - arrowTipX);
  float arrowHeadAngle1 = arrowAngle + PI / 6;
  float arrowHeadAngle2 = arrowAngle - PI / 6;

  int arrowHeadX1 = arrowTailX + cos(arrowHeadAngle1) * 20;
  int arrowHeadY1 = arrowTailY + sin(arrowHeadAngle1) * 20;
  int arrowHeadX2 = arrowTailX + cos(arrowHeadAngle2) * 20;
  int arrowHeadY2 = arrowTailY + sin(arrowHeadAngle2) * 20;

  // 3. VẼ VÒNG TRÒN LA BÀN
  for (int r = 15; r <= 20; r++) {
    tft.drawCircle(CENTER_X, CENTER_Y, RADIUS + r, TFT_DARKGREY);
  }

  // Vẽ mũi tên màu đỏ hướng gió mới
  tft.fillTriangle(arrowTailX, arrowTailY, arrowHeadX1, arrowHeadY1, arrowHeadX2, arrowHeadY2, TFT_RED);

  // 4. CẬP NHẬT TỌA ĐỘ MŨI TÊN MỚI THÀNH CŨ ĐỂ LƯỢT SAU XÓA
  lastArrowHeadX1 = arrowHeadX1;
  lastArrowHeadY1 = arrowHeadY1;
  lastArrowHeadX2 = arrowHeadX2;
  lastArrowHeadY2 = arrowHeadY2;
  lastArrowTailX = arrowTailX;  // Cập nhật đúng biến Đuôi
  lastArrowTailY = arrowTailY;  // Cập nhật đúng biến Đuôi

  // 5. IN CHỮ SỐ ĐỘ HƯỚNG GIÓ (Đã trảm String)
  tft.loadFont(AA_FONT_30, LittleFS);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("8888"));

  char windBuf[16];
  snprintf(windBuf, sizeof(windBuf), "%.0f°", winddy);
  tft.drawString(windBuf, CENTER_X, CENTER_Y - 30);

  tft.unloadFont();
}


void drawWindSpeed() {
  char windBuf[32];  // Buffer dùng chung để đúc chuỗi, chạy cực nhẹ trên Stack

  // 1. VẼ TỐC ĐỘ GIÓ HIỆN TẠI (Chữ to trung tâm)
  tft.loadFont(AA_FONT_LARGE, LittleFS);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("888.8"));

  snprintf(windBuf, sizeof(windBuf), "%.1f", fWindSpeed);
  tft.drawString(windBuf, CENTER_X, CENTER_Y + 3);
  tft.unloadFont();

  // 2. VẼ TỐC ĐỘ GIÓ CỰC ĐẠI TRONG GIỜ (Chữ nhỏ bên dưới)
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextPadding(tft.textWidth("888.8 max/h "));  // Padding tính theo font SMALL

  snprintf(windBuf, sizeof(windBuf), "%.1f max/h ", maxWindHour);
  tft.drawString(windBuf, CENTER_X, CENTER_Y + 30);
  tft.unloadFont();

  tft.setTextPadding(0);  // Reset padding về 0 an toàn cho các hàm vẽ sau
}


void drawRain() {
  // 1. Giới hạn chiều cao đồ thị cột mưa (Thước đo cao tối đa 100 pixel)
  if (mmGraph > 99.75) mmGraph = 0;

  // Vẽ chỉ thị màu nhỏ dòng dữ liệu Daily
  tft.fillRect(560, 131, 3, 11, TFT_CYAN);

  // 2. IN THÔNG SỐ CHỮ (TRẢM STRING)
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  char rainBuf[32];  // Buffer tĩnh chạy cực nhẹ trên Stack thay thế hoàn toàn String

  // Lượng mưa mỗi giờ (Hourly)
  tft.setTextPadding(tft.textWidth("888.88mm/d"));  // Đệm rộng cố định cho định dạng mm/h hoặc mm/d
  snprintf(rainBuf, sizeof(rainBuf), "%.2fmm/h", mmHourly);
  tft.drawString(rainBuf, 566, 114);

  // Lượng mưa trong ngày (Daily)
  snprintf(rainBuf, sizeof(rainBuf), "%.2fmm/d", mmDaily);
  tft.drawString(rainBuf, 566, 130);

  // Tổng lượng mưa (Total)
  tft.setTextPadding(tft.textWidth("88888.88 mm"));  // Tăng đệm rộng hơn vì số tổng tích lũy có thể rất lớn
  snprintf(rainBuf, sizeof(rainBuf), "%.2fmm", mmTotal);
  tft.drawString(rainBuf, 566, 146);

  tft.unloadFont();
  tft.setTextPadding(0);  // Tối ưu: Reset padding về 0 để bảo vệ các hàm vẽ sau không bị lỗi nền

  // 3. VẼ ĐỒ THỊ ỐNG NGHIỆM ĐO MƯA (RainGauge)
  tft.fillRect(520, 63, 30, 100, 0x4228);                           // Nền xám của thanh đồ thị
  tft.fillRect(520, 163 - (int)mmGraph, 30, (int)mmGraph, 0x07FF);  // Cột nước mưa màu Cyan (Xanh băng)

  // Vẽ các vạch chia độ (Tick marks) cách nhau mỗi 10 pixel
  for (int j = 0; j < 110; j = j + 10) {
    tft.drawLine(520, 163 - j, 549, 163 - j, TFT_BLACK);
  }
}


void drawSignal() {
  WiFiRSSI = WiFi.RSSI();

  // 1. TÍNH TOÁN VÀ CHỐNG TRÀN KHUNG HÌNH (Sử dụng thêm hàm constrain)
  if (WiFi.status() != WL_CONNECTED) {
    valueWiFiStrong = 0;
    WiFiRSSI = -120;
  } else {
    valueWiFiStrong = map(WiFiRSSI, -35, -120, 50, 0);
    valueWiFiStrong = constrain(valueWiFiStrong, 0, 50);  // Bảo hiểm không tràn quá 50px
  }

  valueLoRaStrong = map(dbm, -35, -120, 50, 0);
  valueLoRaStrong = constrain(valueLoRaStrong, 0, 50);  // Bảo hiểm sóng LoRa lúc ở gần (-22dBm) không phá khung

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

  // 2. BẮT ĐẦU VẼ GIAO DIỆN (TRẢM HOÀN TOÀN STRING)
  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TR_DATUM);

  char sigBuf[32];  // Buffer tĩnh đa năng, chạy cực nhẹ trên Stack

  // --- Sóng WiFi ---
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("-888dBm"));
  snprintf(sigBuf, sizeof(sigBuf), "%ddBm", WiFiRSSI);
  tft.drawString(sigBuf, 748, 0);

  tft.fillRect(750, 1, 50, 8, 0x4228);
  tft.fillRect(750, 2, valueWiFiStrong, 6, 0xFFFF);

  // --- Sóng LoRa ---
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("-888dBm"));
  snprintf(sigBuf, sizeof(sigBuf), "%ddBm", dbm);
  tft.drawString(sigBuf, 748, 10);

  tft.fillRect(750, 11, 50, 8, 0x4228);
  tft.fillRect(750, 12, valueLoRaStrong, 6, 0x07FF);

  // --- Khối Quản Lý Nguồn (Pin Trạm & Pin Màn Hình) ---
  tft.fillRect(750, 21, 50, 8, 0x4228);              // Nền xám chung cực thông minh của ca
  tft.fillRect(750, 22, valueBattery, 3, 0x07E0);    // Thanh pin LoRa (Xanh lá)
  tft.fillRect(750, 26, valueBattery1, 3, TFT_RED);  // Thanh pin Màn hình (Đỏ)

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("20.00/4.3V"));
  snprintf(sigBuf, sizeof(sigBuf), "%.2f|%.2fv", fvalueBat, (analogVolts / 1000.0));
  tft.drawString(sigBuf, 748, 20);

  // --- Điện Áp Solar ---
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("20.00/20V"));
  snprintf(sigBuf, sizeof(sigBuf), "%.2f/20v", fvalueSol);
  tft.drawString(sigBuf, 748, 30);

  tft.fillRect(750, 31, 50, 8, 0x4228);
  tft.fillRect(750, 32, valueSolar, 6, 0xFFE0);

  // --- Thời Gian Kết Nối (Time Connect) ---
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("4,294,967.295s"));
  snprintf(sigBuf, sizeof(sigBuf), "%.0fs", fTimeConnect);
  tft.drawString(sigBuf, 800, 40);

  tft.unloadFont();
  tft.setTextPadding(0);  // Dọn dẹp vùng đệm an toàn
}


void drawTemHumOutdoor() {
  // 1. CẬP NHẬT ĐỈNH MIN/MAX (Giữ nguyên logic chuẩn của ca)
  if (TempOutDoor < minTempOutDoor) minTempOutDoor = TempOutDoor;
  if (TempOutDoor > maxTempOutDoor) maxTempOutDoor = TempOutDoor;
  if (HumiOutDoor < minHumiOutDoor) minHumiOutDoor = HumiOutDoor;
  if (HumiOutDoor > maxHumiOutDoor) maxHumiOutDoor = HumiOutDoor;

  char thBuf[16];  // Buffer tĩnh đa năng dùng chung cho toàn hàm, cực kỳ nhẹ

  // 2. IN NHIỆT ĐỘ NGOÀI TRỜI (Font 40 to rõ)
  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("88.88°"));

  snprintf(thBuf, sizeof(thBuf), "%.2f°", TempOutDoor);
  tft.drawString(thBuf, 455, 186);
  tft.unloadFont();

  // 3. IN ĐỘ ẨM NGOÀI TRỜI (Font 20)
  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextPadding(tft.textWidth("100.0%"));

  // Lưu ý: Trong snprintf, để in ra dấu % ca phải viết là %% nhé
  snprintf(thBuf, sizeof(thBuf), "%.1f%%", HumiOutDoor);
  tft.drawString(thBuf, 455, 228);
  tft.unloadFont();

  // 4. IN CÁC THÔNG SỐ MIN/MAX (Font 13)
  tft.loadFont(AA_FONT_13, LittleFS);
  tft.setTextDatum(TL_DATUM);

  // --- Vẽ Min/Max Nhiệt độ ---
  tft.setTextPadding(tft.textWidth("88.88"));
  snprintf(thBuf, sizeof(thBuf), "%.2f", maxTempOutDoor);
  tft.drawString(thBuf, 585, 188);

  snprintf(thBuf, sizeof(thBuf), "%.2f", minTempOutDoor);
  tft.drawString(thBuf, 585, 204);

  // --- Vẽ Min/Max Độ ẩm ---
  tft.setTextPadding(tft.textWidth("100.0"));
  snprintf(thBuf, sizeof(thBuf), "%.1f", maxHumiOutDoor);
  tft.drawString(thBuf, 585, 222);

  snprintf(thBuf, sizeof(thBuf), "%.1f", minHumiOutDoor);
  tft.drawString(thBuf, 585, 234);

  tft.unloadFont();
  tft.setTextPadding(0);  // Reset padding an toàn
}


void drawTemHumIndoor() {
  // 1. CẬP NHẬT ĐỈNH MIN/MAX TRONG NHÀ
  if (TempInDoor < minTempInDoor) minTempInDoor = TempInDoor;
  if (TempInDoor > maxTempInDoor) maxTempInDoor = TempInDoor;
  if (HumiInDoor < minHumiInDoor) minHumiInDoor = HumiInDoor;
  if (HumiInDoor > maxHumiInDoor) maxHumiInDoor = HumiInDoor;

  char inBuf[16];  // Mảng ký tự tĩnh đa năng chạy trên Stack, cực kỳ nhẹ

  // 2. IN NHIỆT ĐỘ TRONG NHÀ (Font 40)
  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("88.88°"));
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  snprintf(inBuf, sizeof(inBuf), "%.2f°", TempInDoor);
  tft.drawString(inBuf, 637, 186);
  tft.unloadFont();

  // 3. IN ĐỘ ẨM TRONG NHÀ (Font 20)
  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("100.0%"));

  snprintf(inBuf, sizeof(inBuf), "%.1f%%", HumiInDoor);  // Double %% để in ra ký tự phần trăm
  tft.drawString(inBuf, 637, 228);
  tft.unloadFont();

  // 4. IN CÁC THÔNG SỐ MIN/MAX (Font 13)
  tft.loadFont(AA_FONT_13, LittleFS);
  tft.setTextDatum(TL_DATUM);

  // --- Vẽ Min/Max Nhiệt độ Indoor ---
  tft.setTextPadding(tft.textWidth("88.88"));
  snprintf(inBuf, sizeof(inBuf), "%.2f", maxTempInDoor);
  tft.drawString(inBuf, 767, 188);

  snprintf(inBuf, sizeof(inBuf), "%.2f", minTempInDoor);
  tft.drawString(inBuf, 767, 204);

  // --- Vẽ Min/Max Độ ẩm Indoor ---
  tft.setTextPadding(tft.textWidth("100.0"));
  snprintf(inBuf, sizeof(inBuf), "%.1f", maxHumiInDoor);
  tft.drawString(inBuf, 767, 222);

  snprintf(inBuf, sizeof(inBuf), "%.1f", minHumiInDoor);
  tft.drawString(inBuf, 767, 234);

  // 5. IN ÁP SUẤT & ĐỘ CAO (Font 13)
  tft.setTextPadding(tft.textWidth("8888.8hPa"));

  // Áp suất (Đã cộng bù 6.8 hPa và làm tròn 1 chữ số thập phân)
  snprintf(inBuf, sizeof(inBuf), "%.1fhPa", Pressure + 6.8);
  tft.drawString(inBuf, 542, 29);

  // Độ cao (Lấy 1 chữ số thập phân kèm ký tự m)
  snprintf(inBuf, sizeof(inBuf), "%.1fm", Altitude);
  tft.drawString(inBuf, 542, 41);

  tft.unloadFont();
  tft.setTextPadding(0);  // Khóa đuôi an toàn, trả mặt bằng sạch cho các hàm sau
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
  int d = day(local_time);
  int h = hour(local_time);
  int m = minute(local_time);

  // --- 1. RESET MỖI GIỜ (Vào phút 59) ---
  static int lastResetHour = -1;
  if (m == 59 && h != lastResetHour) {
    lastResetHour = h;
    mmHourly = 0;     // Reset lượng mưa trong giờ
    maxWindHour = 0;  // Reset tốc độ gió cực đại trong giờ
  }

  // --- 2. RESET MỖI NGÀY (Vào lúc 23h59) ---
  static int lastResetDay = -1;
  if (h == 23 && m == 59 && d != lastResetDay) {
    lastResetDay = d;

    // Reset Mưa dầm
    mmDaily = 0;
    mmGraph = 0;

    // Reset Outdoor (Đã nâng minHumi lên 100)
    maxTempOutDoor = 0;
    minTempOutDoor = 99;
    maxHumiOutDoor = 0;
    minHumiOutDoor = 100;  // Chuẩn hóa: Bắt được ngay cả khi độ ẩm ngày mới đạt max 100%

    // Reset Indoor (Đã nâng minHumi lên 100)
    maxTempInDoor = 0;
    minTempInDoor = 99;
    maxHumiInDoor = 0;
    minHumiInDoor = 100;  // Chuẩn hóa: Đảm bảo logic so sánh luôn đúng
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
  b[799] = graphRain;
  graphWinddy = map(winddy, 0, 360, 310, 270);
  graphWindMax = maxWindHour;
  if (graphWindMax > 200) graphWindMax = 200;
  graphWindSpeed = map(graphWindMax, 0, 200, 350, 250);
  z[799] = graphWinddy;
  x[799] = graphWindSpeed;
  for (int k = 799; k >= 0; k--) {
    tft.drawLine(k, b[k], k - 1, b[k - 1], TFT_CYAN);  // Rain gauge/day
    g[k - 1] = b[k];
    tft.drawLine(k, z[k], k - 1, z[k - 1], TFT_OLIVE);  // Wind Direction
    tft.drawLine(k, x[k], k - 1, x[k - 1], TFT_RED);    // Wind Gust
    a[k - 1] = z[k];
    y[k - 1] = x[k];
  }
  for (int i = 799; i >= 0; i--) {
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
  n[799] = graphTemOUT;
  m[799] = graphTemIN;
  for (int k = 799; k >= 0; k--) {
    tft.drawLine(k, n[k], k - 1, n[k - 1], TFT_ORANGE);  //  humidity outdoor
    tft.drawLine(k, m[k], k - 1, m[k - 1], TFT_CYAN);    //  Temperature outdoor
    h[k - 1] = n[k];                                     //saves the information shifted one position temporarily in y
    j[k - 1] = m[k];
  }
  for (int k = 799; k >= 0; k--) {
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
  c[799] = graphHumOUT;
  v[799] = graphHumIN;
  for (int k = 799; k >= 0; k--) {
    tft.drawLine(k, c[k], k - 1, c[k - 1], TFT_ORANGE);  //  humidity outdoor
    tft.drawLine(k, v[k], k - 1, v[k - 1], TFT_CYAN);    //  Temperature outdoor
    d[k - 1] = c[k];                                     //saves the information shifted one position temporarily in y
    f[k - 1] = v[k];
  }
  for (int k = 799; k >= 0; k--) {
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
  tft.setTextColor(TFT_BLACK, TFT_GREENYELLOW);
  tft.setTextPadding(tft.textWidth("BLYNK"));
  //  tft.fillRect(644, 40, 36, 10, TFT_RED);
  tft.drawString("BLYNK", 654, 50);
  tft.unloadFont();
  Blynk.virtualWrite(V8, TempInDoor);
  Blynk.virtualWrite(V9, HumiInDoor);
  //  Blynk.virtualWrite(V9,(currentMillis/1000));
  //  Serial.println(currentMillis/1000);
  Blynk.virtualWrite(V10, TempOutDoor);
  Blynk.virtualWrite(V11, HumiOutDoor);
  Blynk.virtualWrite(V12, winddy);
  Blynk.virtualWrite(V13, fWindSpeed);
  Blynk.virtualWrite(V14, mmDaily);
  Blynk.virtualWrite(V15, maxWindHour);
  Blynk.virtualWrite(V16, fvalueBat);
  Blynk.virtualWrite(V17, fvalueSol);
  tft.fillRect(654, 50, 36, 11, TFT_BLACK);
}


void saveSDData(time_t local_time) {
  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth(" SD "));

  // --- 1. TỰ ĐỘNG PHỤC HỒI KHI THẺ SD BỊ MẤT KẾT NỐI ---
  if (!sdReady) {
    SD.end();     // 🔥 reset SD stack
    spiSD.end();  // 🔥 reset SPI
    delay(50);

    spiSD.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    sdReady = SD.begin(SD_CS, spiSD);
    if (!sdReady) {
      tft.setTextColor(TFT_BLACK, TFT_RED);
      tft.drawString(" SD ", 690, 50);
      return;
    }
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.drawString(" SD ", 690, 50);
  }

  // --- 2. ĐÚC TÊN FILE ĐỘNG THEO THÁNG NĂM TẠI THỜI ĐIỂM GHI LOG ---
  char fileName[32];
  // Kết quả đúc ra chuỗi dạng: "/log_2026_05.csv"
  snprintf(fileName, sizeof(fileName), "/log_%04d_%02d.csv",
           year(local_time),
           month(local_time));

  // --- 3. MỞ FILE ĐỘNG THEO THÁNG VỪA ĐÚC ---
  File f = SD.open(fileName, FILE_APPEND);
  if (!f) {
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    tft.drawString(" SD ", 690, 50);
    SD.end();
    spiSD.end();
    sdReady = false;  // Lần sau vào loop sẽ thử mount lại
    return;
  }

  // --- 4. ĐỊNH DẠNG CHUỖI THỜI GIAN "YYYY-MM-DD HH:MM" ---
  char datetime[20];
  sprintf(datetime, "%04d-%02d-%02d %02d:%02d",
          year(local_time),
          month(local_time),
          day(local_time),
          hour(local_time),
          minute(local_time));

  // --- 5. GHI DỮ LIỆU VÀO FILE ---
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

  logCount++;

  // --- 6. IN SỐ LƯỢT GHI LOG CẠNH CHỮ SD (DÙNG CHUỖI TĨNH) ---
  char logBuf[8];
  snprintf(logBuf, sizeof(logBuf), "%d", logCount);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("9999"));
  tft.drawString(logBuf, 712, 50);

  tft.unloadFont();
  tft.setTextPadding(0);  // Reset padding an toàn
}


// Hàm đọc file log từ thẻ SD và bóc tách dữ liệu lịch sử 24 giờ
void readSDData(time_t local_time) {
  // 1. Tạo đường dẫn file theo ngày hiện tại (Ví dụ: /2026/05/19.txt)
  char fileName[32];
  sprintf(fileName, "/%04d/%02d/%02d.txt", year(local_time), month(local_time), day(local_time));
  
  Serial.print("--- ĐANG ĐỌC LỊCH SỬ TỪ THẺ SD: ");
  Serial.println(fileName);

  // 2. Khởi tạo lại 2 mảng về 0.0 trước khi nạp dữ liệu mới
  for (int i = 0; i < 24; i++) {
    history_TempOut[i] = 0.0;
    history_RainDaily[i] = 0.0;
  }

  // 3. Mở file trên thẻ SD
  File dataFile = SD.open(fileName, FILE_READ);
  if (!dataFile) {
    Serial.println("❌ Lỗi: Không tìm thấy file log ngày hôm nay trên thẻ SD!");
    return; // Thoát nếu không có file
  }

  // 4. Đọc từng dòng trong file cho đến hết
  while (dataFile.available()) {
    String line = dataFile.readStringUntil('\n');
    line.trim(); // Xóa các ký tự xuống dòng thừa (\r, \n)
    
    if (line.length() == 0) continue; // Dòng trống thì bỏ qua

    // Khởi tạo các biến để bóc tách chuỗi CSV
    int commaIndex = 0;
    int columnCount = 0;
    String timeStr = "";
    float tempOutVal = 0.0;
    float rainDailyVal = 0.0;

    // Vòng lặp cắt chuỗi theo dấu phẩy ','
    while (line.length() > 0) {
      commaIndex = line.indexOf(',');
      String field = "";
      
      if (commaIndex != -1) {
        field = line.substring(0, commaIndex);
        line = line.substring(commaIndex + 1); // Cắt bỏ phần đã xử lý
      } else {
        field = line; // Trường cuối cùng trong dòng
        line = "";
      }
      field.trim();

      // Bóc tách dữ liệu theo số thứ tự cột (Bắt đầu từ cột 0)
      if (columnCount == 0) {
        // Cột 0: Ngày tháng giờ giấc (Ví dụ: "2026/05/19 13:00")
        timeStr = field;
      }
      else if (columnCount == 1) {
        // Cột 1: Nhiệt độ ngoài trời (TempOut)
        tempOutVal = field.toFloat();
      }
      else if (columnCount == 15) {
        // Cột 12: Lượng mưa ngày (RainDaily) - Ca kiểm tra lại xem đúng số thứ tự cột trong file log của ca không nha
        rainDailyVal = field.toFloat();
      }

      columnCount++;
      if (commaIndex == -1) break; // Hết dòng thì dừng vòng cắt chuỗi
    }

    // 5. Xác định mốc giờ để nhét vào đúng vị trí trong mảng (0 - 23)
    // Chuỗi timeStr có dạng "2026/05/19 13:00" -> Giờ nằm ở ký tự thứ 11 và 12
    if (timeStr.length() >= 16) {
      int hourIndex = timeStr.substring(11, 13).toInt(); // Trích xuất ra số 13
      
      if (hourIndex >= 0 && hourIndex < 24) {
        history_TempOut[hourIndex] = tempOutVal;     // Nạp vào mảng nhiệt độ đúng vị trí giờ
        history_RainDaily[hourIndex] = rainDailyVal; // Nạp vào mảng lượng mưa đúng vị trí giờ
        
        Serial.printf("-> Đã nạp [%02dh:00] TempOut: %.1f°C | Rain: %.1fmm\n", hourIndex, tempOutVal, rainDailyVal);
      }
    }
  }

  dataFile.close(); // Đóng file sau khi đọc xong
  Serial.println("✔️ ĐÃ ĐỌC VÀ PHÂN TÍCH FILE SD THÀNH CÔNG!");
}


/***************************************************************************************
**                          Draw the clock digits
***************************************************************************************/
void drawTime(time_t local_time) {
  sprTime.fillSprite(TFT_BLACK);
  sprTime.setTextDatum(BC_DATUM);
  sprTime.setTextColor(TFT_GREENYELLOW, TFT_BLACK);

  // Mảng đệm ký tự tĩnh đa năng
  char timeBuf[32];

  // --- 1. IN THỨ VÀ NGÀY THÁNG (Font 20) ---
  sprTime.loadFont(AA_FONT_20, LittleFS);
  sprTime.setTextPadding(sprTime.textWidth("WEDNESDAY"));  // Sửa thành sprTime

  // In Thứ (Dùng hàm dayStr gốc của ca)
  sprTime.drawString(dayStr(weekday(local_time)), 70, 30);

  // Đúc chuỗi ngày tháng dạng DD/MM/YYYY bằng snprintf (Thay thế hoàn toàn cụm cộng String)
  snprintf(timeBuf, sizeof(timeBuf), "%d/%d/%d",
           day(local_time),
           month(local_time),
           year(local_time));
  sprTime.drawString(timeBuf, 70, 56);
  sprTime.unloadFont();  // Sửa thành sprTime.unloadFont() cho đúng chuẩn Sprite

  // --- 2. IN GIỜ PHÚT ĐỒNG HỒ (Font 70) ---
  sprTime.loadFont(AA_FONT_70, LittleFS);

  // Định dạng tự động thêm số 0 phía trước (%02d) cho Giờ và Phút, kèm dấu hai chấm
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d",
           hour(local_time),
           minute(local_time));

  sprTime.setTextPadding(sprTime.textWidth("44:44"));  // Sửa thành sprTime
  sprTime.drawString(timeBuf, 222, 68);
  sprTime.unloadFont();  // Sửa thành sprTime.unloadFont()

  // --- 3. ĐẨY SPRITE LÊN MÀN HÌNH ---
  sprTime.pushSprite(0, 0);
  sprTime.setTextPadding(0);  // Khóa đuôi padding an toàn

  // --- 4. TỰ ĐỘNG ĐIỀU CHỈNH ĐÈN NỀN (AUTO DIM BACKLIGHT) ---
  int lux = lightMeter.readLightLevel();
  if (lux >= 0 && lux < 20) ledcWrite(LEDpin, 50);
  else if (lux >= 20 && lux < 40) ledcWrite(LEDpin, 150);
  else if (lux >= 40 && lux < 80) ledcWrite(LEDpin, 200);
  else if (lux >= 80) ledcWrite(LEDpin, 255);
}

/***************************************************************************************
**                          Fetch the weather data  and update screen
***************************************************************************************/
// Update the Internet based information and update screen
void updateWeatherData(int h, int m) {
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
      // Đọc trực tiếp từ luồng mạng (Stream), cực nhẹ RAM
      DynamicJsonDocument doc(2048);
      DeserializationError error = deserializeJson(doc, http.getStream());

      if (!error) {
        JsonObject current = doc["currentConditions"];
        if (!current.isNull()) {
          // Đổ dữ liệu vào biến toàn cục weatherData
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

  // Nếu lấy được dữ liệu mới vẽ lên màn hình
  if (validData) {
    GetWeatherData = true;

    fillSegment(6, 27, 0, 360, 5, TFT_RED);
    fillSegment(6, 27, 0, 360, 6, TFT_BLACK);

    // Gọi hàm vẽ, bên trong hàm này ca nhớ sửa các biến
    // thành weatherData.temperature thay vì currentWeather->temperature nhé
    drawCurrentWeather(h, m);
  }
}

/***************************************************************************************
**                          Draw the current weather
***************************************************************************************/
void drawCurrentWeather(int h, int m) {
  // Mảng đệm ký tự tĩnh dùng chung cho toàn hàm, cực nhẹ
  char buf[32];

  // --- 1. IN THỜI GIAN CẬP NHẬT (Mép phải màn hình) ---
  tft.loadFont(AA_FONT_10, LittleFS);
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("12:30/12:00"));

  // Đúc thẳng chuỗi "|HH:MM/HH:MM" không qua substring hay cộng String
  // Cố định tọa độ X về 799 (Mép phải chuẩn của màn hình 800px)
  snprintf(buf, sizeof(buf), "|%02d:%02d/%.5s", h, m, weatherData.timeupdate);
  tft.drawString(buf, 799, 50);
  tft.unloadFont();

  // --- 2. VẼ ICON THỜI TIẾT METEOCON ---
  // Lấy tên icon (Hàm getMeteoconIcon nên sửa trả về const char* hoặc giữ nguyên nếu nó trả về String cũng được)
  String weathertest = getMeteoconIcon(weatherData.weatherIcon, weatherData.Cloudcover, weatherData.isday);
  snprintf(buf, sizeof(buf), "/icon/%s.bmp", weathertest.c_str());
  ui.drawBmp(buf, 0, 63);

  // --- 3. IN THÔNG TIN PHỤ: ĐỘ PHỦ MÂY & CHỈ SỐ UV (Font SMALL) ---
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TC_DATUM);

  // In Độ phủ mây
  snprintf(buf, sizeof(buf), "%d%%", weatherData.Cloudcover);
  tft.setTextPadding(tft.textWidth(" 100%"));
  tft.drawString(buf, 195, 94);

  // In Chỉ số UV
  snprintf(buf, sizeof(buf), "%d", weatherData.UVindex);
  tft.setTextPadding(tft.textWidth("88"));
  tft.drawString(buf, 273, 94);

  // In Tốc độ gió
  snprintf(buf, sizeof(buf), "%.1fkm/h", weatherData.windSpeedKmph);
  tft.setTextPadding(tft.textWidth("88,88km/h"));
  tft.drawString(buf, 195, 146);

  // In Lượng mưa
  snprintf(buf, sizeof(buf), "%.2fmm", weatherData.rainmm);
  tft.setTextPadding(tft.textWidth("88,88mm"));
  tft.drawString(buf, 273, 146);
  tft.unloadFont();

  // --- 4. VẼ ICON HƯỚNG GIÓ (Xử lý an toàn mảng la bàn) ---
  int windAngle = (int)((weatherData.windDirectionDegrees + 22.5) / 45.0);
  if (windAngle > 7 || windAngle < 0) windAngle = 0;  // Bảo hiểm chống tràn mảng

  // Dùng mảng hằng ký tự cố định (const char*) thay cho mảng String để tiết kiệm RAM
  const char* windDir[] = { "N", "NE", "E", "SE", "S", "SW", "W", "NW" };
  snprintf(buf, sizeof(buf), "/wind/%s.bmp", windDir[windAngle]);
  ui.drawBmp(buf, 114, 86);

  // --- 5. IN NHIỆT ĐỘ HIỆN TẠI (Font 40) ---
  tft.loadFont(AA_FONT_40, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("88,8°"));

  snprintf(buf, sizeof(buf), "%.1f°", weatherData.temperature);
  tft.drawString(buf, 322, 186);
  tft.unloadFont();

  // --- 6. IN ĐỘ ẨM HIỆN TẠI (Font 20) ---
  tft.loadFont(AA_FONT_20, LittleFS);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(tft.textWidth("100%"));

  snprintf(buf, sizeof(buf), "%d%%", weatherData.humidity);
  tft.drawString(buf, 322, 228);
  tft.unloadFont();

  tft.setTextPadding(0);  // Reset padding an toàn
}


void updateAstronomy(time_t local_time) {
  HTTPClient http;

  // Xây dựng URL lấy dữ liệu thiên văn (Mặt trời, mặt trăng)
  String url1d = "https://weather.visualcrossing.com/VisualCrossingWebServices/rest/services/timeline/";
  url1d += locationKey;
  url1d += "/today?unitGroup=metric&include=current&elements=moonrise,moonset,moonphase,sunrise,sunset&key=" + String(apiKey);

  bool validData = false;
  int retry = 0;

  while (!validData && retry < 3) {  // Thử lại tối đa 3 lần
    http.setConnectTimeout(4000);
    http.setTimeout(4000);
    http.begin(url1d);

    int httpResponseCode = http.GET();

    if (httpResponseCode == HTTP_CODE_OK) {
      // Dùng Stream để deserialize trực tiếp, cực kỳ tiết kiệm RAM
      DynamicJsonDocument doc(2048);
      DeserializationError error = deserializeJson(doc, http.getStream());

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

  // Nếu dữ liệu hợp lệ mới tiến hành vẽ lên màn hình
  if (validData) {
    GetAstronomy = true;

    // Gọi hàm vẽ - Ca nhớ vào hàm drawAstronomy sửa các dấu "->" thành dấu "." nhé
    // Ví dụ: astronomyData.sunrise thay vì dailyWeather->sunrise
    drawAstronomy(local_time);

    tft.unloadFont();  // Nhả font để giải phóng bộ nhớ
  }
}

/***************************************************************************************
**                          Draw Sun rise/set, Moon, cloud cover and humidity
***************************************************************************************/
void drawAstronomy(time_t local_time) {
  int d = day(local_time);
  int m = month(local_time);
  int y = year(local_time);

  // Lấy lịch âm (Giữ nguyên cấu trúc đỉnh cao của ca)
  LunarDate lunar = convertSolar2Lunar(d, m, y);

  // Mảng đệm ký tự tĩnh dùng chung cho toàn hàm
  char buf[32];

  // --- 1. IN NGÀY/THÁNG ÂM LỊCH (Font SMALL) ---
  tft.loadFont(AA_FONT_SMALL, LittleFS);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("30/12"));

  // Đúc thẳng chuỗi "Ngày/Tháng" âm lịch không qua toán tử cộng String
  snprintf(buf, sizeof(buf), "%d/%d", lunar.day, lunar.month);
  tft.drawString(buf, 279, 207);

  // --- 2. VẼ ICON MẶT TRĂNG THEO NGÀY ÂM LỊCH ---
  // Đường dẫn ảnh ví dụ: /moon/15.bmp
  snprintf(buf, sizeof(buf), "/moon/%d.bmp", lunar.day);
  ui.drawBmp(buf, 90, 167);

  // --- 3. CHUYỂN ĐỔI VÀ VẼ GIỜ MẶT TRỜI MỌC/LẶN (24H) ---
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("88 : 88"));
  tft.setTextDatum(BC_DATUM);

  // Giờ mặt trời mọc (Cắt chuỗi an toàn bằng %.8s rồi chuyển đổi 24h)
  char sunriseRaw[9];
  snprintf(sunriseRaw, sizeof(sunriseRaw), "%.8s", astronomyData.sunrise);
  String sunrise24 = convertTo24HourFormat(sunriseRaw);
  tft.drawString(sunrise24.c_str(), 40, 207);

  // Giờ mặt trời lặn
  char sunsetRaw[9];
  snprintf(sunsetRaw, sizeof(sunsetRaw), "%.8s", astronomyData.sunset);
  String sunset24 = convertTo24HourFormat(sunsetRaw);
  tft.drawString(sunset24.c_str(), 40, 248);

  // --- 4. CHUYỂN ĐỔI VÀ VẼ GIỜ MẶT TRĂNG MỌC/LẶN (24H) ---
  // Giờ mặt trăng mọc
  char moonriseRaw[9];
  snprintf(moonriseRaw, sizeof(moonriseRaw), "%.8s", astronomyData.moonrise);
  String moonrise24 = convertTo24HourFormat(moonriseRaw);
  tft.drawString(moonrise24.c_str(), 219, 207);

  // Giờ mặt trăng lặn
  char moonsetRaw[9];
  snprintf(moonsetRaw, sizeof(moonsetRaw), "%.8s", astronomyData.moonset);
  String moonset24 = convertTo24HourFormat(moonsetRaw);
  tft.drawString(moonset24.c_str(), 219, 248);

  tft.unloadFont();
  tft.setTextPadding(0);  // Reset padding an toàn
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

  // Thiết lập padding bằng chiều rộng màn hình (800) để khi chữ thay đổi
  // nó tự động xóa sạch vết chữ cũ ở nền sau, không bị lem chữ.
  tft.setTextPadding(800);

  // In dòng chữ thông báo (Dùng chuỗi tĩnh C-style)
  tft.drawString(text, 400, 260);

  // Vẽ thanh tiến trình phần trăm
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