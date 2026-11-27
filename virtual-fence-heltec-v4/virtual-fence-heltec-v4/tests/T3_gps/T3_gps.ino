/*
  T3 – GPS test (Heltec V4 + L76K GNSS module from the kit)
  ----------------------------------------------------------
  Plug the GNSS module into the 8-pin GNSS connector BEFORE powering the board.
  Go outside or put the board at a window with open sky.

  Shows: time since power-on, satellites, HDOP, position, time-to-first-fix.
  First fix ("cold start") normally takes 30–60 s outside, up to a few minutes
  indoors near a window. Afterwards fixes come every second.

  Serial Monitor 115200 baud. Type 'r' to print the raw NMEA sentences for
  10 s (useful if nothing seems to happen: no text = wiring/power problem,
  text but no fix = sky view problem).

  Libraries: TinyGPSPlus, ThingPulse SSD1306.
*/
#include <Wire.h>
#include <SSD1306Wire.h>
#include <TinyGPSPlus.h>
#include "heltec_v4.h"

SSD1306Wire display(OLED_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
uint32_t t0, ttff = 0, rawUntil = 0;

// Send a $PCAS command to the L76K with the NMEA checksum added.
void l76kCommand(const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= *p;
  GPSSerial.printf("$%s*%02X\r\n", body, cs);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  vextOn();
  pinMode(PIN_OLED_RST, OUTPUT);
  digitalWrite(PIN_OLED_RST, LOW); delay(20); digitalWrite(PIN_OLED_RST, HIGH); delay(20);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // GNSS power on, out of reset, kept awake
  gpio_hold_dis((gpio_num_t)PIN_GNSS_EN);
  pinMode(PIN_GNSS_RST, OUTPUT);  digitalWrite(PIN_GNSS_RST, HIGH);
  pinMode(PIN_GNSS_WAKE, OUTPUT); digitalWrite(PIN_GNSS_WAKE, HIGH);
  pinMode(PIN_GNSS_EN, OUTPUT);   digitalWrite(PIN_GNSS_EN, LOW);
  GPSSerial.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
  delay(500);
  l76kCommand("PCAS04,7");        // use GPS + BeiDou + GLONASS
  t0 = millis();
  Serial.println("\n=== T3 GPS test === (type 'r' for raw NMEA)");
}

void loop() {
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    gps.encode(c);
    if (millis() < rawUntil) Serial.write(c);
  }
  if (Serial.available() && Serial.read() == 'r') rawUntil = millis() + 10000;

  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  const bool fix = gps.location.isValid() && gps.location.age() < 3000;
  if (fix && !ttff) ttff = millis() - t0;
  const uint32_t up = (millis() - t0) / 1000;

  if (millis() >= rawUntil) {
    if (gps.charsProcessed() < 10 && up > 5) {
      Serial.println("No data from the GNSS module - check that it is plugged in firmly.");
    } else {
      Serial.printf("t=%lus  sats %lu  hdop %.1f  %s", (unsigned long)up,
                    (unsigned long)gps.satellites.value(), gps.hdop.hdop(), fix ? "FIX " : "no fix");
      if (fix) Serial.printf(" %.6f, %.6f  alt %.0f m  ttff %lus", gps.location.lat(), gps.location.lng(),
                             gps.altitude.meters(), (unsigned long)(ttff / 1000));
      if (gps.time.isValid()) Serial.printf("  UTC %02u:%02u:%02u", gps.time.hour(), gps.time.minute(), gps.time.second());
      Serial.println();
    }
  }

  display.clear();
  display.drawString(0, 0, String("T3 GPS  t=") + up + "s");
  display.drawString(0, 12, String("sats ") + gps.satellites.value() + "  hdop " + String(gps.hdop.hdop(), 1));
  if (fix) {
    display.drawString(0, 26, String(gps.location.lat(), 6) + "  " + String(gps.location.lng(), 6));
    display.drawString(0, 38, String("first fix after ") + (ttff / 1000) + " s");
  } else {
    display.drawString(0, 26, gps.charsProcessed() < 10 ? "no data from GNSS!" : "searching...");
  }
  display.drawString(0, 50, String("NMEA chars ") + gps.charsProcessed());
  display.display();
}
