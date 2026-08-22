/*
 * tof_bringup — get all four VL53L0X talking on one I2C bus.
 *
 * THE PROBLEM. Every VL53L0X powers up at address 0x29. Put four on a bus and you do not get four
 * sensors, you get one sensor answering four times, and the readings look plausible enough that you
 * can lose an evening to it. The address is writable but NOT persistent — it resets on every power
 * cycle — so this dance has to run at every boot, forever.
 *
 * THE FIX. Every sensor has an XSHUT pin that holds it in reset. Pull them all low, then bring them
 * up ONE at a time; while only one is awake it is the only thing answering at 0x29, so you can
 * re-address it safely before waking the next.
 *
 * RELEASE, DON'T DRIVE. XSHUT is released by setting the pin to INPUT and letting the breakout's own
 * pull-up take it high — not by driving 3.3V into it. The VL53L0X core runs at 2.8V and several
 * breakouts route XSHUT straight to it.
 *
 * WIRING (bench layout — the flight layout moves XSHUT to free up motor pins):
 *   all sensors: VIN->3V3, GND->GND, SDA->D4, SCL->D5
 *   XSHUT: sensor0->D0, sensor1->D1, sensor2->D3, sensor3->D8
 * Do not use D2/D6/D7. D2 is a strapping pin and the board will refuse to boot if it is held wrong;
 * D6/D7 are the serial console you are about to read this output on.
 *
 * Library: "VL53L0X" by Pololu (Library Manager). Adafruit's works too but its address API differs.
 */

#include <Wire.h>
#include <VL53L0X.h>

#define N_SENSORS 4

const uint8_t XSHUT_PIN[N_SENSORS] = { D0, D1, D3, D8 };
const uint8_t ADDRESS[N_SENSORS]   = { 0x30, 0x31, 0x32, 0x33 };   // anything unused, not 0x29
const char*   LABEL[N_SENSORS]     = { "front", "left ", "right", "down " };

VL53L0X sensor[N_SENSORS];
bool alive[N_SENSORS] = { false, false, false, false };

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  Serial.println("\n\n=== VL53L0X bring-up ===");

  // Hold every sensor in reset before touching the bus.
  for (int i = 0; i < N_SENSORS; i++) {
    pinMode(XSHUT_PIN[i], OUTPUT);
    digitalWrite(XSHUT_PIN[i], LOW);
  }
  delay(50);

  Wire.begin(D4, D5);
  Wire.setClock(400000);

  for (int i = 0; i < N_SENSORS; i++) {
    pinMode(XSHUT_PIN[i], INPUT);      // release this one only; the rest stay asleep
    delay(20);                          // it needs a moment to boot before it will answer

    sensor[i].setTimeout(500);
    if (!sensor[i].init()) {
      Serial.printf("  [%d] %s  NOT FOUND on XSHUT pin %d\n", i, LABEL[i], XSHUT_PIN[i]);
      pinMode(XSHUT_PIN[i], OUTPUT);    // put it back to sleep so it cannot squat on 0x29
      digitalWrite(XSHUT_PIN[i], LOW);
      continue;
    }
    sensor[i].setAddress(ADDRESS[i]);
    // 33ms budget is the sensible default: long enough to be quiet, fast enough for a 30Hz loop.
    sensor[i].setMeasurementTimingBudget(33000);
    sensor[i].startContinuous();
    alive[i] = true;
    Serial.printf("  [%d] %s  OK, moved to 0x%02X\n", i, LABEL[i], ADDRESS[i]);
  }

  int n = 0;
  for (int i = 0; i < N_SENSORS; i++) if (alive[i]) n++;
  Serial.printf("\n%d of %d sensors up.\n", n, N_SENSORS);
  if (n < N_SENSORS) {
    Serial.println("A missing sensor is almost always wiring: check that ITS XSHUT pin actually");
    Serial.println("goes to the pin listed above, and that VIN is on 3V3 and not 5V.");
  }
  Serial.println("\nfront  left   right  down     (mm, 8190 = nothing in range)\n");
}

void loop() {
  uint16_t mm[N_SENSORS];
  for (int i = 0; i < N_SENSORS; i++) {
    mm[i] = alive[i] ? sensor[i].readRangeContinuousMillimeters() : 0;
    if (alive[i] && sensor[i].timeoutOccurred()) mm[i] = 0;
    Serial.printf("%5u  ", mm[i]);
  }

  // THE FAILURE THAT LOOKS LIKE SUCCESS. If addressing silently failed you are reading one sensor
  // four times, so all four columns move together — which reads as "wow, very consistent" rather
  // than "broken". Wave your hand in front of ONE sensor: if all four numbers change, say so.
  bool allEqual = true;
  for (int i = 1; i < N_SENSORS; i++) if (mm[i] != mm[0]) allEqual = false;
  if (allEqual && mm[0] != 0) Serial.print("  <-- all four identical: re-addressing did not take");

  Serial.println();
  delay(100);
}
