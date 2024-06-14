/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include "esp_camera.h"
#include <Wire.h>
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory
#include <HTTPClient.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <SPI.h>

// #include <Adafruit_Si7021.h>
#include <SensirionI2cSht4x.h>
#include <Wire.h>

#include <Adafruit_Sensor_Modified.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 1

// Pin definition for CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  1800        /* Time ESP32 will go to sleep (in seconds) */

int pictureNumber = 0;
const char *ssid = "pvltrv-TX1C";
const char *password = "m0WYLkvI";
const char *post_url = "https://rm.fsv.cvut.cz/upload-image"; // Location where images are POSTED
bool internet_connected = false;

// -----------------temp humid definitions start -----------------
// macro definitions
// make sure to use the proper definition of NO_ERROR
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

#define I2C_SDA 13  // Set your SDA pin (do not use pins: 12,2,16; ok:13,14)
#define I2C_SCL 14  // Set your SCL pin (ok pin:15; )

SensirionI2cSht4x sensor;

static char errorMessage[64];
static int16_t error;

float aTemperature = 0.0;
float aHumidity = 0.0;
// -----------------temp humid definitions end -----------------


// -----------------distance definitions start -----------------
#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2
#define XSHUT_PIN 13

// Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

int16_t aDistance = 0;
bool validDistance = false;
int attempts = 0;
// -----------------distance definitions end -----------------

bool init_wifi()
{
  int connAttempts = 5;
  Serial.println("\r\nConnecting to: " + String(ssid));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if (connAttempts > 10)
      return false;
    connAttempts++;
  }
  return true;
}

void setupCameraAndTakePhoto() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  // Serial.begin(115200);
  // Serial.setDebugOutput(true);
  //Serial.println();

  // Ensure GPIO 4 is set up before camera initialization
  pinMode(4, OUTPUT);
  // digitalWrite(4, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if(psramFound()){
    Serial.println("psramFound OK ");
    config.frame_size = FRAMESIZE_VGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 1;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_whitebal(s, 1); // Enable white balance
    s->set_awb_gain(s, 1); // Enable AWB gain
    s->set_wb_mode(s, 0);  // Set to auto white balance
  }

  //Serial.println("Starting SD Card");
  if(!SD_MMC.begin("/sdcard", true)){
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }

  // Ensure GPIO 4 is set up before camera initialization
  // pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  delay(10);
  // rtc_gpio_hold_en(GPIO_NUM_4);
    
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);

  // If pictureNumber is out of bounds, reset it
  if (pictureNumber < 0 || pictureNumber > 255) {
    pictureNumber = 0;
  }

  // Ensure GPIO 4 is set before sleep
  // pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  // rtc_gpio_hold_en(GPIO_NUM_4);

  pictureNumber = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";

  fs::FS &fs = SD_MMC; 
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();



  /////////////////////////
  HTTPClient http;

  Serial.print("[HTTP] begin...\n");
  // configure traged server and url

  http.begin(post_url); //HTTP

  Serial.print("[HTTP] POST...\n");
  // start connection and send HTTP header
  int httpCode = http.sendRequest("POST", fb->buf, fb->len); // we simply put the whole image in the post body.

  // httpCode will be negative on error
  if (httpCode > 0)
  {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] POST... code: %d\n", httpCode);

    // file found at server
    if (httpCode == HTTP_CODE_OK)
    {
      String payload = http.getString();
      Serial.println(payload);
    }
  }
  else
  {
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  esp_camera_fb_return(fb); 

}

void goToSleep(){
  delay(2000);
  Serial.println("Going to sleep now");
  delay(2000);
  WiFi.disconnect(true);    // Disconnect WiFi
  analogReadResolution(0);  // Equivalent to turning off ADC
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.flush(); // Ensure that all data you have sent out through the serial port has been transmitted
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}


void JSONsendingHTML() {
  HTTPClient http;

  Serial.print("[HTTP] begin...\n");
  // configure target server and URL
  const char* post_url = "https://rm.fsv.cvut.cz/upload-temperature"; // url to the flask app location
  http.begin(post_url); // HTTP

  Serial.print("[HTTP] POST...\n");

  // Create JSON object
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["temperature"] = aTemperature;
  jsonDoc["humidity"] = aHumidity;
  jsonDoc["distance"] = aDistance;
  String jsonStr;
  serializeJson(jsonDoc, jsonStr);

  // Specify content-type header
  http.addHeader("Content-Type", "application/json");

  // Start connection and send HTTP header and body
  int httpCode = http.POST(jsonStr);

  // httpCode will be negative on error
  if (httpCode > 0) {
    // HTTP header has been sent and Server response header has been handled
    Serial.printf("[HTTP] POST... code: %d\n", httpCode);

    // file found at server
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
    }
  } else {
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

void temperatureHumidity(){
  while (!Serial) {
      delay(100);
  }
  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA, I2C_SCL);
  sensor.begin(Wire, SHT40_I2C_ADDR_44);

  sensor.softReset();
  delay(10);
  uint32_t serialNumber = 0;
  error = sensor.serialNumber(serialNumber);
  if (error != NO_ERROR) {
      Serial.print("Error trying to execute serialNumber(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
  }
  Serial.print("serialNumber: ");
  Serial.print(serialNumber);
  Serial.println();

  delay(20);
  error = sensor.measureLowestPrecision(aTemperature, aHumidity);
  if (error != NO_ERROR) {
      Serial.print("Error trying to execute measureLowestPrecision(): ");
      errorToString(error, errorMessage, sizeof errorMessage);
      Serial.println(errorMessage);
      return;
  }
  Serial.print("Temperature: ");
  Serial.print(aTemperature);
  Serial.print("\t");
  Serial.print("Humidity: ");
  Serial.print(aHumidity);
  Serial.println();

  Wire.end();
}



void measureDistance (){
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println(F("measuring distance function start"));

  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("after  ! l53.begin"));
  
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  Serial.println(F("before startRanging"));

  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(10);
  }
  Serial.println(F("Ranging started"));

  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
  vl53.setTimingBudget(500);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  /*
  vl.VL53L1X_SetDistanceThreshold(100, 300, 3, 1);
  vl.VL53L1X_SetInterruptPolarity(0);
  */
  while (!validDistance && attempts < 5) {
    if (vl53.dataReady()) {
      // new measurement for the taking!
      aDistance = vl53.distance();
      if (aDistance == -1) {
        // something went wrong!
        Serial.print(F("Couldn't get distance: "));
        Serial.println(vl53.vl_status);
      } else if (aDistance == 0) {
        // too far or too close
        Serial.print(F("Too far or too close: "));
        Serial.println(vl53.vl_status);
      } else {
        // valid distance
        validDistance = true;
        Serial.print(F("Distance: "));
        Serial.print(aDistance);
        Serial.println(" mm");
      }

      // data is read out, time for another reading!
      vl53.clearInterrupt();
    }
    attempts++;
    delay(1000); // Small delay between attempts
  }

  if (!validDistance) {
    Serial.println(F("Failed to get a valid distance after 5 attempts."));
  }
}

void setup (){
  Serial.begin(115200);

  Serial.println("-= ESP32-CAM-CONTAINER =-");
  if (init_wifi())
  { // Connected to WiFi
    internet_connected = true;
    Serial.println("Internet connected");
  }

  if (internet_connected) {
    setupCameraAndTakePhoto(); // also sends a photo over HTTP protocol
  }
  temperatureHumidity();
  measureDistance();
  JSONsendingHTML(); // sends a string to the server

  delay(100);
  goToSleep();
}


void loop() {
}