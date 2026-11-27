/*
  T4 – Servo + TPS63020 step-up test (collar wiring)
  ---------------------------------------------------
  Wiring (see the wiring diagram):
    GPIO4 (J3 pin 15) -> TPS63020 EN      (100 kΩ EN -> GND if the board has no pull-down)
    GPIO6 (J3 pin 17) -> servo signal (orange)
    TPS63020 VIN/GND  -> battery via the Y adapter, OUT -> servo red, servo brown -> GND
    470 µF across servo red/brown, + to red

  BEFORE connecting the servo: measure TPS63020 OUT with a multimeter while
  this sketch says "5 V ON" - it must read about 5.0 V.

  What it does: every press of the PRG button
    1. switches the 5 V step-up on
    2. moves the servo to the other position (LOCK <-> RING)
    3. switches the 5 V off again (servo unpowered, holds by friction)
  Serial prints battery voltage before and during the move, so you can see
  how much the battery sags under servo load.

  Adjust SERVO_US_LOCK / SERVO_US_RING until the latch moves exactly as the
  bell team wants, then copy the values into vf_collar/config.h.
  Serial Monitor 115200. Type 'l' (lock), 'g' (ring) or a number 500-2500 (µs).
*/
#include "heltec_v4.h"

#define SERVO_US_LOCK   1000     // clapper locked  (bell silent)
#define SERVO_US_RING   2000     // clapper free    (bell rings when the cow moves)
#define SERVO_MOVE_MS   600      // time the servo gets to reach the position

void servoPulse(uint16_t us) {
  Serial.printf("battery before: %u mV\n", batteryMv());
  ledcAttach(PIN_SERVO_SIG, 50, 14);                  // 50 Hz, 14 bit
  ledcWrite(PIN_SERVO_SIG, (uint32_t)us * 16384 / 20000);
  digitalWrite(PIN_SERVO_PWR_EN, HIGH);               // 5 V on
  Serial.printf("5 V ON, moving to %u us\n", us);
  delay(150);
  Serial.printf("battery during move: %u mV\n", batteryMv());
  delay(SERVO_MOVE_MS - 150);
  digitalWrite(PIN_SERVO_PWR_EN, LOW);                // 5 V off
  ledcDetach(PIN_SERVO_SIG);
  pinMode(PIN_SERVO_SIG, OUTPUT);
  digitalWrite(PIN_SERVO_SIG, LOW);                   // no signal into an unpowered servo
  Serial.println("5 V OFF");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  gpio_hold_dis((gpio_num_t)PIN_SERVO_PWR_EN);
  pinMode(PIN_SERVO_PWR_EN, OUTPUT);
  digitalWrite(PIN_SERVO_PWR_EN, LOW);
  pinMode(PIN_SERVO_SIG, OUTPUT);
  digitalWrite(PIN_SERVO_SIG, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  Serial.println("\n=== T4 servo test === press PRG, or type l / g / 500-2500");
}

void loop() {
  static bool ring = false;
  if (digitalRead(PIN_BUTTON) == LOW) {
    ring = !ring;
    servoPulse(ring ? SERVO_US_RING : SERVO_US_LOCK);
    while (digitalRead(PIN_BUTTON) == LOW) delay(10);
  }
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s == "l") servoPulse(SERVO_US_LOCK);
    else if (s == "g") servoPulse(SERVO_US_RING);
    else if (s.toInt() >= 500 && s.toInt() <= 2500) servoPulse(s.toInt());
  }
}
