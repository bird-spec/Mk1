/*
 * imu_bringup — first real sensor. Four wires, no libraries to install.
 *
 * This talks to the MPU6050 with raw register reads instead of a library, on purpose. A library that
 * fails to find the chip just says "failed" and leaves you guessing. This scans the whole I2C bus and
 * tells you what it can actually see, which turns "it doesn't work" into a specific answer: nothing on
 * the bus at all (wiring), or the chip is there but not responding (power).
 *
 * WIRING  MPU6050 (GY-521)  ->  XIAO ESP32S3
 *   VCC  ->  3V3     <-- 3V3, NOT 5V. The XIAO is a 3.3V board.
 *   GND  ->  GND
 *   SDA  ->  D4
 *   SCL  ->  D5
 * Leave XDA, XCL, ADO and INT unconnected.
 *
 * Battery stays in the bag. This runs off the USB cable.
 */

#include <Wire.h>

#define MPU_ADDR   0x68     // 0x69 if the ADO pin is tied high; the scan below will tell you
#define REG_PWR    0x6B
#define REG_ACCEL  0x3B

bool found = false;
uint8_t addr = MPU_ADDR;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println("\n\n=== MPU6050 bring-up ===\n");

  Wire.begin(D4, D5);
  Wire.setClock(400000);

  // Scan first, always. This one loop separates "my soldering is bad" from "my code is bad".
  Serial.println("scanning I2C bus...");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X", a);
      if (a == 0x68 || a == 0x69) { Serial.print("  <-- MPU6050"); addr = a; found = true; }
      Serial.println();
      n++;
    }
  }

  if (n == 0) {
    Serial.println("  NOTHING on the bus.\n");
    Serial.println("That is a wiring fault, not a code fault. In order of likelihood:");
    Serial.println("  1. SDA and SCL swapped  (SDA->D4, SCL->D5)");
    Serial.println("  2. a joint that looks fine but never bonded - tug each of the four wires");
    Serial.println("  3. VCC not actually on 3V3, or GND not connected");
    return;
  }
  if (!found) {
    Serial.println("\nBus works, but no MPU6050 on it. Check VCC.");
    return;
  }

  // The chip boots asleep. Writing 0 to the power register wakes it - miss this and every reading
  // comes back as zero, which looks exactly like a dead sensor.
  Wire.beginTransmission(addr);
  Wire.write(REG_PWR);
  Wire.write(0);
  Wire.endTransmission();
  delay(100);

  Serial.println("\nawake. tilt the board and watch roll/pitch move.\n");
  Serial.println("   roll   pitch |    ax     ay     az  (g)");
}

void loop() {
  if (!found) { delay(2000); return; }

  Wire.beginTransmission(addr);
  Wire.write(REG_ACCEL);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)6);
  if (Wire.available() < 6) { Serial.println("read failed"); delay(500); return; }

  int16_t rx = (Wire.read() << 8) | Wire.read();
  int16_t ry = (Wire.read() << 8) | Wire.read();
  int16_t rz = (Wire.read() << 8) | Wire.read();

  // 16384 counts per g at the default +/-2g range.
  float ax = rx / 16384.0f, ay = ry / 16384.0f, az = rz / 16384.0f;

  // Gravity always points down, so which way it lands across the three axes IS the tilt. This is the
  // same trick the flight controller uses to know which way is up - no magic, just arctangent.
  float roll  = atan2(ay, az) * 57.2958f;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 57.2958f;

  Serial.printf("%7.1f %7.1f | %6.2f %6.2f %6.2f\n", roll, pitch, ax, ay, az);
  delay(100);
}
