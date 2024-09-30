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
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "SparkFun_VL53L1X.h"
#include <Adafruit_ADS1015.h>

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

#define CHUNK_SIZE_PHOTO 4096 // Define chunk size as 4 KB

#define TIME_TO_SLEEP 3550ULL       // Time ESP32 will go to sleep (in seconds)
#define uS_TO_S_FACTOR 1000000ULL   // Conversion factor for micro seconds to seconds


int pictureNumber = 0;
const char *ssid = "pvltrv-TX1C";
const char *password = "m0WYLkvI";
const char *post_url_image = "https://rm.fsv.cvut.cz/upload_container_image"; // Location where images are POSTED
const char *post_url_data = "https://rm.fsv.cvut.cz/upload_container_data"; // Location where images are POSTED
// const char *post_url_image = "https://rm.fsv.cvut.cz/upload-image"; // Location where images are POSTED
// const char *post_url_data = "https://rm.fsv.cvut.cz/upload-temperature"; // Location where images are POSTED
bool internet_connected = false;


// macro definitions
// make sure to use the proper definition of NO_ERROR
#ifdef NO_ERROR
#undef NO_ERROR
#endif
#define NO_ERROR 0

#define I2C_SDA 13  // Set your SDA pin (do not use pins: 12,2,16; ok:13,14)
#define I2C_SCL 14  // Set your SCL pin (ok pin:15; )
#define GPS_switch 2 // pin number that is used to turn on and off the gps
#define LEDflash 4  //

#define rxPinGPS 15  // 15 pin na PCB
#define txPinGPS 12  // 12 pin na PCB

SoftwareSerial ss(rxPinGPS, txPinGPS);
SensirionI2cSht4x sensor;
TinyGPSPlus gps;
HTTPClient http;
SFEVL53L1X distanceSensor;
Adafruit_ADS1115 ads(0x48); // Create an instance of the ADS1115

static char errorMessage[64];
static int16_t error;

unsigned int timeLimitConnectionGPS = 90000; // Milliseconds to get the connection
unsigned long GPSstartTime = millis();  // Record the start time
bool gpsUpdated = false;

float aTemperature = 0.0;
float aHumidity = 0.0;
float aLatitude ;
float aLongitude ;
float aBattPercentage = 0.0;
unsigned int aContID = 3;
unsigned int aStatus = 0; // 0 je neodeslana fotka, 1 je odeslani uspesne

static const uint32_t GPSBaud = 9600;

// Define the frame buffer as a global variable
camera_fb_t * fb = NULL;

// -----------------temp humid definitions end -----------------


// -----------------distance definitions start -----------------


int16_t aDistance = 0;
bool validDistance = false;
int attempts = 0;
// -----------------distance definitions end -----------------

float getBatteryPercentage(float voltage) {
  // Define the minimum and maximum voltages
  const float minVoltage = 3.0;
  const float maxVoltage = 4.2;

  // Ensure the voltage is within the expected range
  if (voltage > maxVoltage) {
    voltage = maxVoltage;
  } else if (voltage < minVoltage) {
    voltage = minVoltage;
  }

  // Calculate the percentage
  float percentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100;
  return percentage;
}

void get_battery_voltage(){
  Wire.begin(I2C_SDA, I2C_SCL); // reset I2C 
    // Initialize the ADS1115
  ads.begin();
  // int sum = 0;
  // for(int i=0; i<100; i++) {
  //   sum = sum + analogRead(ADC_PIN);
  // }
  // float result = float(sum)/100.0;
  // return int(result * (1.3665));
  // Serial.println(result);
  
  int16_t adc0 = ads.readADC_Differential_0_1();
  
  // Convert the ADC value to voltage (assuming the default gain is used, i.e., 2/3x which gives 6.144V range)
  float voltage = -1 * (adc0 * 0.1875 / 1000); // ADS1115 gives 16-bit value, 0.1875mV per bit
  
  // Scale the voltage back up based on the voltage divider
  float batteryVoltage = voltage * (10.09 + 7.45) / 7.45; // R1 = 10.09 kOhm R2 = 7.45 kOhm

  // Calculate battery percentage
  float batteryPercentage = getBatteryPercentage(batteryVoltage);

  Serial.print("ADC0: "); 
  Serial.print(adc0);
  Serial.print(" Voltage: ");
  Serial.println(batteryVoltage);

  Serial.print("Battery Percentage: ");
  Serial.print(batteryPercentage);
  Serial.println("%");

  aBattPercentage = batteryPercentage; //Get the result of the measurement from the sensor
}

bool init_wifi() {
  int connAttempts = 15;
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && connAttempts > 0) {
    delay(1000);
    connAttempts--;
    Serial.println("... ");
  }
  return WiFi.status() == WL_CONNECTED;
}

void setupCameraAndTakePhoto() {

  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

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
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
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

  // Serial.printf("After init - Heap: %d, PSRAM: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    // s->set_brightness(s, 0);     // -2 to 2
    // s->set_contrast(s, 0);       // -2 to 2
    // s->set_saturation(s, 0);     // -2 to 2
    // s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    // s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    // s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    // s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    // s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    // s->set_ae_level(s, 0);       // -2 to 2
    // s->set_aec_value(s, 300);    // 0 to 1200
    // s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    // s->set_agc_gain(s, 0);       // 0 to 30
    // s->set_gainceiling(s, (gainceiling_t)2);  // 0 to 6
    s->set_bpc(s, 1);            // 0 = disable , 1 = enable
    // s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    // s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    // s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    // s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    // s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    s->set_aec2(s, 1);           // 0 = disable , 1 = enable


    s->set_vflip(s, 1);          // 0 = disable , 1 = enable
    s->set_hmirror(s, 1);        // 0 = disable , 1 = enable
    s->set_gain_ctrl(s, 1);         //auto gain 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);          // Auto White Balance ENABLE(0 = disable , 1 = enable)
    s->set_exposure_ctrl(s, 1);     // auto exposure ON
    // s->set_brightness(s, 0);        // (-2 to 2) - set brightness
    // s->set_agc_gain(s, 0);          // set gain manually (0 - 30)
    // s->set_aec_value(s, 110);         // set exposure manually  (0-1200)
  }


  // Ensure GPIO 4 is set up before camera initialization
  // pinMode(GPIO_NUM_4, OUTPUT);
  // digitalWrite(GPIO_NUM_4, HIGH); // Ensure the LED is off initially
  // digitalWrite(4, HIGH); // turn on LED flash, delay after to let it turn on fully
  // delay(300);
  // rtc_gpio_hold_en(GPIO_NUM_4);
  /* sufficient time to let the YAVG value stabilize before esp_camera_fb_get, 
  than the photo is almost perfect and I do not suffer any more of the 
  "dark tint after restart"
  */

  // After initializing the camera
  delay(3000); // Wait for 1 second

// Capture and discard initial frames
for (int i = 0; i < 5; i++) {
    camera_fb_t * temp_fb = esp_camera_fb_get();
    if (temp_fb) {
        esp_camera_fb_return(temp_fb);
    }
    delay(500); // Wait between frames
}


  // Now capture the actual frame you want to use
  fb = esp_camera_fb_get();
  delay(1000); // Wait for 1 second

  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
    // Turns off the ESP32-CAM white on-board LED (flash) connected to GPIO 4
  pinMode(LEDflash, OUTPUT);
  digitalWrite(LEDflash, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);
  
  int maxRetriesSendPhoto = 8;
  http.setTimeout(10000); // Set timeout to 10 seconds, to prevent it from waiting indefinitely.
  bool successSendPhoto = false;

  if (WiFi.status() == WL_CONNECTED) {
    http.begin(post_url_image);
    http.addHeader("Content-Type", "image/jpeg");
    for (int attemptSendPhoto = 1; attemptSendPhoto <= maxRetriesSendPhoto; attemptSendPhoto++) {
      Serial.print("[HTTP] POST photo...\n");

      size_t totalSize = fb->len;
      size_t offset = 0;

      int httpCode = http.sendRequest("POST", fb->buf, fb->len);
      if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        Serial.println(payload);
        successSendPhoto = true;
        aStatus = 1; // confirmation for the JSON message that a photo has been sent
        http.end(); // Ensure connection is closed
        break;
      } else {
        Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }

      http.end(); // Ensure connection is closed on failure
      Serial.printf("Retrying... (%d/%d)\n", attemptSendPhoto, maxRetriesSendPhoto);
      delay(1000); // Wait before retrying
    }

  } else {
    Serial.println("WiFi not connected. Please check your connection.");
  }

  if (!successSendPhoto) {
    Serial.println("Failed to send photo after multiple attempts.");
  }

    // After sending the photo or if an error occurs
  if (fb) {
    esp_camera_fb_return(fb);
    fb = NULL; // Reset the frame buffer pointer
  }

  http.end(); 
}

void goToSleep(){
  Serial.println("Going to sleep now\n\n");
  delay(100);
  WiFi.disconnect(true);
  digitalWrite(GPS_switch, LOW); // Turn GPS power OFF
  esp_sleep_enable_timer_wakeup((uint64_t)TIME_TO_SLEEP * (uint64_t)uS_TO_S_FACTOR);
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void JSONsendingHTML() {
  int maxRetriesSendJSON = 5; // Number of retries for DNS and HTTP request
  int retryDelaySendJSON = 1000; // Delay between retries in milliseconds
  bool successSendJSON = false; // Flag to check if the request was successful

  // Convert aTemperature and aHumidity to integers
  int intTemperature = static_cast<int>(aTemperature);
  int intHumidity = static_cast<int>(aHumidity);
  int intDistance = static_cast<int>(aDistance);
  int intBattPercentage = static_cast<int>(aBattPercentage);

  // Create JSON object
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["temperature"] = intTemperature;
  jsonDoc["humidity"] = intHumidity;
  jsonDoc["distance"] = intDistance;
  jsonDoc["latitude"] = aLatitude;
  jsonDoc["longitude"] = aLongitude;
  jsonDoc["container_id"] = aContID;
  jsonDoc["image_status"] = aStatus;
  jsonDoc["battery_status"] = intBattPercentage;

  esp_camera_fb_return(fb);
  fb = NULL; // Reset the frame buffer pointer

  String jsonStr;
  serializeJson(jsonDoc, jsonStr);
  Serial.print(jsonStr);

  for (int attemptSendJSON = 1; attemptSendJSON <= maxRetriesSendJSON; attemptSendJSON++) {
    HTTPClient http;
    Serial.print("[HTTP] begin...\n");
    // configure target server and URL
    http.begin(post_url_data); // HTTP
  
    Serial.print("[HTTP] POST string...\n");
    http.addHeader("Content-Type", "application/json");
    int httpCode = http.POST(jsonStr);

    if (httpCode > 0 && httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      Serial.println(payload);
      successSendJSON = true; // Set success flag to true if request was successful
      http.end(); // Ensure connection is closed
      break; // Exit the loop if the request was successful
    } else {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      http.end(); // Ensure connection is closed on failure
      if (http.errorToString(httpCode) == "connection refused" || http.errorToString(httpCode) == "DNS Failed") {
        Serial.printf("Retrying... (%d/%d)\n", attemptSendJSON, maxRetriesSendJSON);
        delay(retryDelaySendJSON); // Wait before retrying
      } else {
        break; // Exit the loop if the error is not recoverable
      }
    }
  }

  if (!successSendJSON) {
    Serial.println("Failed to send HTTP POST request after multiple attempts.");
  }
}

void temperatureHumidity(){
  // Initialize I2C with custom pins
  Wire.begin(I2C_SDA, I2C_SCL);
  sensor.begin(Wire, SHT40_I2C_ADDR_44);

  sensor.softReset();
  // delay(10);
  // uint32_t serialNumber = 0;
  // error = sensor.serialNumber(serialNumber);
  // if (error != NO_ERROR) {
  //     Serial.print("Error trying to execute serialNumber(): ");
  //     errorToString(error, errorMessage, sizeof errorMessage);
  //     Serial.println(errorMessage);
  //     return;
  // }
  // Serial.print("serialNumber: ");
  // Serial.print(serialNumber);
  // Serial.println();

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
    Serial.println("measureDistance Sensor failed to begin. Please check wiring. Freezing...");
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

void getGPS(){
  
  Serial.println("getting GPS function");

  GPSstartTime = millis(); // Record the start time

  while (millis() - GPSstartTime < timeLimitConnectionGPS) {  // Loop for limited time
    while (ss.available() > 0) {
      gps.encode(ss.read());
      if (gps.location.isUpdated()) {
        aLatitude = gps.location.lat();
        aLongitude = gps.location.lng();
        Serial.print("Latitude= ");
        Serial.print(aLatitude, 6);
        Serial.print(" Longitude= ");
        Serial.println(aLongitude, 6);
        gpsUpdated = true;
        Serial.println("GPS UPDATED");
        break;  // Exit the while loop if GPS location is updated
      }
    }
    if (gpsUpdated) {
      break;  // Exit the outer while loop if GPS location is updated
    }
  }

  if (!gpsUpdated) {
    Serial.println("GPS connection not established within 60 seconds.");
    aLatitude = 0.0;
    aLongitude = 0.0;
  }

  
}

void setup (){
  Serial.begin(9600);
  ss.begin(GPSBaud);

  Serial.println("-= ESP32-CAM-CONTAINER =-");

  // PINS SETUP //
  // rtc_gpio_hold_dis(GPIO_NUM_4);  // disable holding the LED pin low so it can turn on high again for a photo
  pinMode(GPS_switch, OUTPUT);   // GPS module power management
  digitalWrite(GPS_switch, HIGH); // Turn GPS power ON

  // temperatureHumidity();
  // measureDistance();
  // get_battery_voltage();
  // getGPS(); // pokud je GPS na tranzistoru tak je asi nutne zajistit, aby tam uz od zapnuti bylo dostatecne napeti,
  // // jinak tam jde po UARTu nejakej bordel a cely to jde do <>

  if (init_wifi())
  { // Connected to WiFi
    internet_connected = true;
    Serial.println("Internet connected");
  }

  if (internet_connected) {
    temperatureHumidity();
    measureDistance();
    get_battery_voltage();
    getGPS(); // pokud je GPS na tranzistoru tak je asi nutne zajistit, aby tam uz od zapnuti bylo dostatecne napeti,
    // jinak tam jde po UARTu nejakej bordel a cely to jde do <>

    setupCameraAndTakePhoto(); // also sends a photo over HTTP protocol
    JSONsendingHTML(); // sends a string to the server
  }
  goToSleep();
}


void loop() {
  Serial.println("LOOPING");
  // This should never be reached because the device goes to sleep in setup()
  // Serial.println("LOOPING LED");
  // digitalWrite(GPS_switch, HIGH); // Turn on
  // delay(2000);
  // goToSleep();

  // get_battery_voltage();
  // delay(1000);
}