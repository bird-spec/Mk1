#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <VL53L0X.h>

// Credentials live in secrets.h, which is gitignored. Copy
// secrets_example.h to secrets.h and put your own network in it.
// Never put a real password in a file you commit.
#include "secrets.h"
const uint16_t PC_PORT     = 5005;
const uint16_t LISTEN_PORT = 5006;

#define SDA_PIN D4
#define SCL_PIN D8
#define XSHUT_DOWN  D10
#define XSHUT_FRONT D6
#define MPU 0x68

const int MOTOR_PIN[4] = {D0, D1, D3, D9};

#define GW 60
#define GH 50
#define RES_M 0.10f
#define L_FREE (-6)
#define L_OCC  (14)
#define L_LIM  (64)
#define T_OCC  (8)
#define T_FREE (-8)
#define MAXR_M 1.26f
#define NBINS 180
#define BIN_CLEAR 1300          // "nothing within range in this direction"

int8_t grid[GW * GH];
uint16_t scanBins[NBINS];
bool scanning = false;

const float ORIGIN_X = GW * RES_M / 2;
const float ORIGIN_Y = GH * RES_M / 2;

WiFiUDP udp;
VL53L0X down, front;
bool haveDown = false, haveFront = false, haveImu = false;
char command[32] = "HOLD";
float heading = 0.0f;
float gyroBias = 0.0f;
uint32_t lastImu = 0, nextTelem = 0, nextMap = 0;

void unwedge() {
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);
  for (int i = 0; i < 16; i++) {
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  }
  pinMode(SCL_PIN, INPUT);
}

static inline bool inside(int cx, int cy) {
  return cx >= 0 && cx < GW && cy >= 0 && cy < GH;
}

void bump(int cx, int cy, int d) {
  if (!inside(cx, cy)) return;
  int i = cy * GW + cx;
  int v = grid[i] + d;
  if (v >  L_LIM) v =  L_LIM;
  if (v < -L_LIM) v = -L_LIM;
  grid[i] = (int8_t)v;
}

void integrateRay(float th, float dist_m) {
  bool hit = dist_m < MAXR_M - 0.01f;
  float d = hit ? dist_m : MAXR_M;

  int x0 = ORIGIN_X / RES_M;
  int y0 = ORIGIN_Y / RES_M;
  int x1 = (ORIGIN_X + cosf(th) * d) / RES_M;
  int y1 = (ORIGIN_Y + sinf(th) * d) / RES_M;

  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  while (x0 != x1 || y0 != y1) {
    bump(x0, y0, L_FREE);
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 <  dx) { err += dx; y0 += sy; }
  }
  if (hit) bump(x1, y1, L_OCC);
}

int mappedPct() {
  int n = 0;
  for (int i = 0; i < GW * GH; i++)
    if (grid[i] > T_OCC || grid[i] < T_FREE) n++;
  return 100 * n / (GW * GH);
}

int binsFilled() {
  int n = 0;
  for (int i = 0; i < NBINS; i++) if (scanBins[i]) n++;
  return n;
}

float gyroZ() {
  if (!haveImu) return 0;
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(MPU, 2) != 2) return 0;
  int16_t rz = (Wire.read() << 8) | Wire.read();
  return rz / 131.0f;
}

void readAccel(float* roll, float* pitch) {
  *roll = 0; *pitch = 0;
  if (!haveImu) return;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom(MPU, 6) != 6) return;
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  float fx = ax / 16384.0f, fy = ay / 16384.0f, fz = az / 16384.0f;
  *roll  = atan2(fy, fz) * 57.2958f;
  *pitch = atan2(-fx, sqrt(fy * fy + fz * fz)) * 57.2958f;
}

void sendTelem(uint16_t f, uint16_t d, float roll, float pitch) {
  char pkt[240];
  snprintf(pkt, sizeof(pkt),
    "{\"front_mm\":%u,"
    "\"down_mm\":%u,"
    "\"roll\":%.1f,"
    "\"pitch\":%.1f,"
    "\"head\":%.2f,"
    "\"mapped\":%d,"
    "\"cmd\":\"%s\"}",
    f, d, roll, pitch, heading, mappedPct(), command);
  udp.beginPacket(PC_IP, PC_PORT);
  udp.write((const uint8_t*)pkt, strlen(pkt));
  udp.endPacket();
}

void sendMap() {
  static uint8_t buf[1 + GW * GH / 4];
  buf[0] = 0xAA;
  memset(buf + 1, 0, sizeof(buf) - 1);
  for (int i = 0; i < GW * GH; i++) {
    uint8_t v = grid[i] > T_OCC ? 2 : (grid[i] < T_FREE ? 1 : 0);
    buf[1 + i / 4] |= v << ((i % 4) * 2);
  }
  udp.beginPacket(PC_IP, PC_PORT);
  udp.write(buf, sizeof(buf));
  udp.endPacket();
}

void sendScan() {
  static uint8_t buf[1 + NBINS * 2];
  buf[0] = 0xBB;
  memcpy(buf + 1, scanBins, NBINS * 2);
  udp.beginPacket(PC_IP, PC_PORT);
  udp.write(buf, sizeof(buf));
  udp.endPacket();
  Serial.printf("scan sent, %d of %d bins filled\n", binsFilled(), NBINS);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  for (int i = 0; i < 4; i++) {
    pinMode(MOTOR_PIN[i], OUTPUT);
    digitalWrite(MOTOR_PIN[i], LOW);
  }
  memset(grid, 0, sizeof(grid));
  memset(scanBins, 0, sizeof(scanBins));

  unwedge();
  pinMode(XSHUT_DOWN, INPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  digitalWrite(XSHUT_FRONT, LOW);
  delay(50);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(50);

  haveDown = down.init();
  if (haveDown) {
    down.setAddress(0x30);
    down.setTimeout(200);
    down.setMeasurementTimingBudget(20000);
    down.startContinuous();
  }
  pinMode(XSHUT_FRONT, INPUT);
  delay(50);
  haveFront = front.init();
  if (haveFront) {
    front.setSignalRateLimit(0.1);
    front.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    front.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    front.setTimeout(200);
    front.setMeasurementTimingBudget(33000);
    front.startContinuous();
  }

  Wire.beginTransmission(MPU);
  haveImu = (Wire.endTransmission() == 0);
  if (haveImu) {
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); Wire.write(0);
    Wire.endTransmission();
    delay(100);
  }
  Serial.printf("sensors: down %d front %d imu %d\n",
                haveDown, haveFront, haveImu);

  // A resting MPU6050 does not read zero. Integrating that offset puts the
  // heading tens of degrees out in under a minute and smears the whole map.
  if (haveImu) {
    Serial.println("calibrating gyro - hold completely still");
    delay(500);
    float sum = 0;
    for (int i = 0; i < 400; i++) { sum += gyroZ(); delay(3); }
    gyroBias = sum / 400.0f;
    Serial.printf("gyro bias %.2f deg/s\n", gyroBias);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.begin(SSID, PASS);
  Serial.print("wifi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("connected. drone is at ");
    Serial.println(WiFi.localIP());
    udp.begin(LISTEN_PORT);
  } else {
    Serial.println("wifi FAILED");
  }

  lastImu = micros();
  Serial.println("ready");
}

void loop() {
  int n = udp.parsePacket();
  if (n > 0) {
    int len = udp.read(command, sizeof(command) - 1);
    command[len > 0 ? len : 0] = 0;
    if (!strcmp(command, "SCAN")) {
      memset(scanBins, 0, sizeof(scanBins));
      scanning = true;
      Serial.println("SCANNING - now turn the drone slowly, one full circle");
    } else if (!strcmp(command, "SEND")) {
      scanning = false;
      sendScan();
    }
  }

  uint32_t now = micros();
  float dt = (now - lastImu) / 1000000.0f;
  lastImu = now;
  heading += (gyroZ() - gyroBias) * dt / 57.2958f;

  uint16_t f = haveFront ? front.readRangeContinuousMillimeters() : 0;
  if (f == 65535) f = 0;
  if (f > 0 && f < 8000) integrateRay(heading, f / 1000.0f);

  // One beam plus rotation IS a multi-beam scan, spread over time.
  //
  // Record EVERY direction, including the empty ones. "nothing within range
  // here" is what defines open space, and in an ordinary room that is most
  // of the sweep - skipping those is why only ten bins were filling.
  if (scanning) {
    int deg = (int)(heading * 57.2958f);
    int b = ((deg / 2) % NBINS + NBINS) % NBINS;
    if (f > 0 && f < 8000)  scanBins[b] = f;
    else if (f >= 8000)     scanBins[b] = BIN_CLEAR;
  }

  if (scanning) {
    static uint32_t sdbg = 0;
    if (millis() > sdbg) {
      sdbg = millis() + 500;
      Serial.printf("scanning... %3d/%d bins   heading %+6.0f deg\n",
                    binsFilled(), NBINS, heading * 57.2958f);
    }
  }

  if (millis() >= nextTelem) {
    nextTelem = millis() + 200;
    uint16_t d = haveDown ? down.readRangeContinuousMillimeters() : 0;
    if (d == 65535) d = 0;
    float roll, pitch;
    readAccel(&roll, &pitch);
    if (WiFi.status() == WL_CONNECTED) sendTelem(f, d, roll, pitch);
  }

  if (millis() >= nextMap) {
    nextMap = millis() + 1000;
    if (WiFi.status() == WL_CONNECTED) sendMap();
  }
}
