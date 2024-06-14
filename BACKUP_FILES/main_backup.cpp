#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <EEPROM.h>
#include <EspMQTTClient.h>
#include "esp_sleep.h"
#include <esp_wifi.h>

#define EEPROM_SIZE 1

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

EspMQTTClient client(
  "pvltrv-TX1C",
  "m0WYLkvI",
  "test.mosquitto.org",
  "",
  "",
  "",
  1883
);

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else 
#define debug(x)
#define debugln(x)
#endif

int pictureNumber = 0;
bool messagePublished = false;
bool tookPhoto = false;
String pictureFileName = "";
const int wakeUpIntervalSeconds = 30;
const int wifiTimeoutSeconds = 60; // Increased WiFi timeout period

void setupAndTakePhoto() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached");
    return;
  }

  camera_fb_t *fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  String path = "/picture" + String(pictureNumber) + ".jpg";
  pictureFileName = path;

  fs::FS &fs = SD_MMC;
  Serial.printf("Picture file name: %s\n", path.c_str());

  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in writing mode");
  } else {
    file.write(fb->buf, fb->len);
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(fb);

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);
}

void onConnectionEstablished() {
  // client.publish("pjpn89/aithinker", "connection established to broker test.mosquitto.org");
}

void goToSleepNow() {
  Serial.println("Going to sleep now");
  esp_sleep_enable_timer_wakeup(wakeUpIntervalSeconds * 1000000);
  WiFi.disconnect();
  delay(100);
  esp_wifi_stop();
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void setup() {
  Serial.begin(115200);
  client.enableDebuggingMessages();
  client.enableHTTPWebUpdater();
  client.enableLastWillMessage("pjpn89/aithinker/lastwill", "I am going offline");
  Serial.println("PROJECT: ESP32-CAM-CONTAINER");
}

unsigned long startAttemptTime = millis();

void loop() {
  client.loop();
  if (!tookPhoto) {
    setupAndTakePhoto();
    tookPhoto = true;
  }

  if (WiFi.status() == WL_CONNECTED && !messagePublished) {
    messagePublished = client.publish("pjpn89/aithinker", pictureFileName);
  }

  if (millis() - startAttemptTime > wifiTimeoutSeconds * 1000) {
    debugln("going to sleep, message was NOT published - wifi timeout");
    goToSleepNow();
  }

  if (messagePublished) {
    debugln("going to sleep after a message was published");
    goToSleepNow();
  }

  delay(1000);
}
