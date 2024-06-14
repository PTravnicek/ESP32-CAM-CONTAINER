#include "esp_camera.h"
#include <Wire.h>
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
// #include "SD_MMC.h"            // SD Card ESP32
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
#define TIME_TO_SLEEP  300        /* Time ESP32 will go to sleep (in seconds) */

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
#include "SparkFun_VL53L1X.h"

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 13

SFEVL53L1X distanceSensor;

int16_t aDistance = 0;
bool validDistance = false;
int attempts = 0;
// -----------------distance definitions end -----------------

bool init_wifi() {
  int connAttempts = 5;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && connAttempts > 0) {
    delay(500);
    connAttempts--;
  }
  return WiFi.status() == WL_CONNECTED;
}

void setupCameraAndTakePhoto() {

  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  // Serial.begin(115200);
  // Serial.setDebugOutput(true);
  //Serial.println();

  // Ensure GPIO 4 is set up before camera initialization
  pinMode(4, OUTPUT);
  // digitalWrite(4, LOW);
  Serial.printf("Before init - Heap: %d, PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

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
    config.frame_size = FRAMESIZE_SVGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 12;
    config.fb_location = CAMERA_FB_IN_PSRAM;  // Use PSRAM for frame buffer
    config.fb_count = 2;
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

  Serial.printf("After init - Heap: %d, PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 0);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 0);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)2);  // 0 to 6
    s->set_bpc(s, 0);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    s->set_quality(s, 12);
  }

  /* sufficient time to let the YAVG value stabilize before esp_camera_fb_get, 
  than the photo is almost perfect and I do not suffer any more of the 
  "dark tint after restart"
  */
  delay(100);
  digitalWrite(4, HIGH); // turn on LED flash, delay after to let it turn on fully
  delay(100);
  // rtc_gpio_hold_en(GPIO_NUM_4);
    
  camera_fb_t * fb = NULL;
  
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  digitalWrite(4, LOW); // turn off LED flash

  const char *post_url = "https://rm.fsv.cvut.cz/upload-image";
  int maxRetries = 5;
  HTTPClient http;
  http.setTimeout(10000); // Set timeout to 10 seconds, to prevent it from waiting indefinitely.

  bool success = false;

  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("[HTTP] POST photo...\n");
      http.begin(post_url);
      int httpCode = http.sendRequest("POST", fb->buf, fb->len);
      Serial.print("after int httpCode ...\n");
      if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.println(payload);
        success = true;
        break;
      } else {
        Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }
      http.end();
    } else {
      Serial.println("WiFi not connected. Retrying...");
    }
    delay(1000);
  }

  if (!success) {
    Serial.println("Failed to send photo after multiple attempts.");
  }
  esp_camera_fb_return(fb);

  // Serial.print("[HTTP] POST photo...\n");
  // HTTPClient http;
  // http.begin(post_url);
  // int httpCode = http.sendRequest("POST", fb->buf, fb->len);
  // if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
  //   String payload = http.getString();
  //   Serial.println(payload);
  // } else {
  //   Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
  // }
  // http.end();
  // esp_camera_fb_return(fb);
}

void goToSleep(){
  Serial.println("Going to sleep now");
  delay(100);
  WiFi.disconnect(true);
  analogReadResolution(0);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}


void JSONsendingHTML() {
  const char* post_url = "https://rm.fsv.cvut.cz/upload-temperature"; // url to the flask app location
  int maxRetries = 5; // Number of retries for DNS and HTTP request
  int retryDelay = 2000; // Delay between retries in milliseconds
  bool success = false; // Flag to check if the request was successful

  // Create JSON object
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["temperature"] = aTemperature;
  jsonDoc["humidity"] = aHumidity;
  jsonDoc["distance"] = aDistance;
  String jsonStr;
  serializeJson(jsonDoc, jsonStr);
  
  for (int attempt = 1; attempt <= maxRetries; attempt++) {
    HTTPClient http;
    Serial.print("[HTTP] begin...\n");
    // configure target server and URL
    http.begin(post_url); // HTTP
  
    Serial.print("[HTTP] POST string...\n");
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(jsonStr);

    if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
      success = true; // Set success flag to true if request was successful
      break; // Exit the loop if the request was successful
    } else {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      if (http.errorToString(httpCode) == "connection refused" || http.errorToString(httpCode) == "DNS Failed") {
        Serial.printf("Retrying... (%d/%d)\n", attempt, maxRetries);
        delay(retryDelay); // Wait before retrying
      } else {
        break; // Exit the loop if the error is not recoverable
      }
    }
    http.end();
  }

  if (!success) {
    Serial.println("Failed to send HTTP POST request after multiple attempts.");
  }
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
  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");

  distanceSensor.startOneshotRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  aDistance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor

  Serial.print("Distance(mm): ");
  Serial.println(aDistance);
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