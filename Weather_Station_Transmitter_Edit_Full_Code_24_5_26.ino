// ======================================================
// WEATHER STATION TX - ARDUINO NANO + LORA E220
// ======================================================

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

SoftwareSerial E220Serial(9, 8);   // RX, TX

// ======================================================
// BME280
// ======================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

#define SEALEVELPRESSURE_HPA (1013.25)

// ======================================================
// AS5600 WIND DIRECTION
// ======================================================

#include "AS5600.h"

AS5600 as5600;

// ======================================================
// ANALOG PINS
// ======================================================

#define cellPin  A1
#define SolarPin A2

// ======================================================
// WIND SENSOR
// ======================================================

const byte WindPin = 2;

// interrupt counter
volatile uint16_t windClicks = 0;

// debounce timer
volatile unsigned long lastWindIRQ = 0;

// ======================================================
// RAIN SENSOR
// ======================================================

const byte RainPin = 3;

// interrupt counter
volatile uint8_t rainTips = 0;

// debounce timer
volatile unsigned long lastRainIRQ = 0;

// ======================================================
// VARIABLES
// ======================================================

unsigned long lastMeasure = 0;

// đo mỗi 3 giây
const float WindMeasureTime = 3.0;

// calibration tốc độ gió của ca
const float WindCalibration = 1.3;

// sensor data
float bmeTemp;
float bmeHum;
float bmePress;
float bmeAltitude;

float ang;
float spd;
float rai;

float battery;
float solar;

unsigned long tim;

// ======================================================
// WIND INTERRUPT
// ======================================================

void countWind() {

  unsigned long now = millis();

  // chống bounce reed switch
  if (now - lastWindIRQ > 10) {

    windClicks++;

    lastWindIRQ = now;
  }
}

// ======================================================
// RAIN INTERRUPT
// ======================================================

void countRain() {

  unsigned long now = millis();

  // rain gauge bounce mạnh hơn
  if (now - lastRainIRQ > 50) {

    rainTips++;

    lastRainIRQ = now;
  }
}

// ======================================================
// SETUP
// ======================================================

void setup() {

  // Serial.begin(9600);

  E220Serial.begin(9600);

  Wire.begin();

  // ====================================================
  // BME280
  // ====================================================

  bme.begin(0x76);

  // ====================================================
  // AS5600
  // ====================================================

  as5600.begin(0x36);

  // ====================================================
  // INPUT PULLUP
  // ====================================================

  pinMode(WindPin, INPUT_PULLUP);

  pinMode(RainPin, INPUT_PULLUP);

  // ====================================================
  // INTERRUPTS
  // ====================================================

  attachInterrupt(
    digitalPinToInterrupt(WindPin),
    countWind,
    FALLING
  );

  attachInterrupt(
    digitalPinToInterrupt(RainPin),
    countRain,
    FALLING
  );

  // ====================================================

  lastMeasure = millis();
}

// ======================================================
// LOOP
// ======================================================

void loop() {

  // ====================================================
  // MEASURE EVERY 3 SECONDS
  // ====================================================

  if (millis() - lastMeasure >= 3000) {

    lastMeasure = millis();

    // ==================================================
    // COPY & RESET INTERRUPT COUNTERS
    // ==================================================

    noInterrupts();

    uint16_t windCount = windClicks;
    windClicks = 0;

    uint8_t rainCount = rainTips;
    rainTips = 0;

    interrupts();

    // ==================================================
    // WIND SPEED
    // ==================================================

    spd =
      ((float)windCount / WindMeasureTime)
      * WindCalibration;

    // ==================================================
    // RAIN
    // ==================================================
    // 1 tip = 0.25 mm

    rai = (float)rainCount * 0.25;

    // ==================================================
    // BATTERY
    // ==================================================

    analogRead(cellPin);

    delay(2);

    battery =
      analogRead(cellPin)
      * 4.1
      / 1000.0;

    // ==================================================
    // SOLAR
    // ==================================================

    analogRead(SolarPin);

    delay(2);

    solar =
      analogRead(SolarPin)
      * 20.3
      / 1000.0;

    // ==================================================
    // BME280
    // ==================================================

    bmeTemp = bme.readTemperature();

    bmeHum = bme.readHumidity();

    bmePress =
      (bme.readPressure() / 100.0F) + 55.84;

    bmeAltitude =
      bme.readAltitude(SEALEVELPRESSURE_HPA) + 14;

    // ==================================================
    // WIND DIRECTION
    // ==================================================

    ang =
      as5600.readAngle()
      / 11.40947075208914;

    // ==================================================
    // TIMER
    // ==================================================

    tim = millis() / 1000UL;

    // ==================================================
    // SEND DATA
    // ==================================================

    sendLoRa();
  }
}

// ======================================================
// SEND LORA
// ======================================================

void sendLoRa() {

  StaticJsonDocument<250> doc;

  doc["temperature"] = bmeTemp;

  doc["humidity"] = bmeHum;

  doc["angle"] = ang;

  doc["speed"] = spd;

  doc["rain"] = rai;

  doc["battery"] = battery;

  doc["solar"] = solar;

  doc["timerun"] = tim;

  doc["altitude"] = bmeAltitude;

  doc["pressure"] = bmePress;

  // ====================================================
  // NANO STABLE BUFFER
  // ====================================================

  char jsonBuffer[250];

  serializeJson(doc, jsonBuffer);

  E220Serial.println(jsonBuffer);

  // Serial.println(jsonBuffer);
}
