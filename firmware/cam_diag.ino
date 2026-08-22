/*
 * cam_diag — why is the camera failing?
 *
 * 0x106 is ESP_ERR_NOT_SUPPORTED, which the camera driver returns when it cannot IDENTIFY the
 * sensor. That is nearly always one of three things, and this tells you which:
 *
 *   1. no PSRAM      -> the frame buffer cannot be allocated
 *   2. sensor silent -> ribbon or board-to-board connection (electrical)
 *   3. both fine     -> configuration, not hardware
 *
 * The useful part is the bus scan. The OV2640 answers on its own SCCB bus at 0x30 - but ONLY while
 * it is being clocked, so we have to generate XCLK first. A plain I2C scan without that clock finds
 * nothing even on a perfectly good camera, which is why "scan found nothing" is normally useless
 * here and why this sketch starts the clock before looking.
 */

#include <Wire.h>
#include "esp_camera.h"

// XIAO ESP32S3 Sense camera pins
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  delay(500);
  Serial.println("\n\n===== CAMERA DIAGNOSTIC =====\n");

  // ---- 1. PSRAM -----------------------------------------------------------
  Serial.println("[1] PSRAM");
  if (psramFound()) {
    Serial.printf("    FOUND, %u bytes free\n", (unsigned)ESP.getFreePsram());
  } else {
    Serial.println("    NOT FOUND  <-- this alone causes the failure");
    Serial.println("    Fix: Tools > PSRAM > OPI PSRAM, then re-upload");
  }

  // ---- 2. is the sensor electrically present? -----------------------------
  Serial.println("\n[2] sensor on the camera bus");
  Serial.println("    starting XCLK on GPIO10...");

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(XCLK_GPIO_NUM, 20000000, 1);
  ledcWrite(XCLK_GPIO_NUM, 1);
#else
  ledcSetup(0, 20000000, 1);
  ledcAttachPin(XCLK_GPIO_NUM, 0);
  ledcWrite(0, 1);
#endif
  delay(100);

  Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  Wire.setClock(100000);
  delay(50);

  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    device at 0x%02X", a);
      if (a == 0x30) Serial.print("   <-- OV2640, sensor is ALIVE");
      Serial.println();
      found++;
    }
  }
  if (!found) {
    Serial.println("    NOTHING on the camera bus.");
    Serial.println("    The sensor is not electrically reachable. That is the ribbon");
    Serial.println("    or the Sense board not mated - not a software problem.");
  }
  Wire.end();

  // ---- 3. what does the driver itself say? --------------------------------
  Serial.println("\n[3] camera driver");
  camera_config_t c;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;   c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_QVGA;     // small on purpose - rules out memory as a cause
  c.jpeg_quality = 12;
  c.fb_count     = 1;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&c);
  Serial.printf("    esp_camera_init -> 0x%X  (%s)\n", err, esp_err_to_name(err));

  if (err == ESP_OK) {
    sensor_t *s = esp_camera_sensor_get();
    Serial.printf("    sensor PID 0x%02X - camera is WORKING\n", s->id.PID);
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      Serial.printf("    captured a frame: %ux%u, %u bytes\n", fb->width, fb->height, (unsigned)fb->len);
      esp_camera_fb_return(fb);
    }
  }

  // ---- verdict ------------------------------------------------------------
  Serial.println("\n===== VERDICT =====");
  if (err == ESP_OK)          Serial.println("Camera fine. Whatever failed before was config in that sketch.");
  else if (!psramFound())     Serial.println("Enable PSRAM (Tools > PSRAM > OPI PSRAM). Fix that first.");
  else if (!found)            Serial.println("HARDWARE: reseat the ribbon and the Sense board screws.");
  else                        Serial.println("Sensor answers but driver rejects it - ribbon partially seated,");
  if (found && err != ESP_OK) Serial.println("or a cracked trace. Reseat, then check the gold contacts.");
}

void loop() { delay(1000); }
