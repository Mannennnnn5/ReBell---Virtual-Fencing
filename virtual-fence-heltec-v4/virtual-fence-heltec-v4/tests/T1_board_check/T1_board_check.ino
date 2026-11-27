/*
  T1 – Board check (Heltec WiFi LoRa 32 V4)
  ------------------------------------------
  Proves that the board, display, LED, button, battery reading and the LoRa
  radio + front end all start. Run this first on EVERY Heltec board.

  Serial Monitor: 115200 baud.  Libraries: RadioLib, "ESP8266 and ESP32 OLED
  driver for SSD1306 displays" (ThingPulse).

  Expected:
    - OLED shows "T1 board check", battery voltage, radio OK, FEM type
    - LED blinks once per second, faster while the PRG button is pressed
    - Serial prints the same plus the board's MAC address (write it down,
      it identifies the board later)
*/
#include <Wire.h>
#include <SSD1306Wire.h>
#include <WiFi.h>
#include "heltec_v4.h"
#include "protocol.h"
#include "vf_radio.h"

SSD1306Wire display(OLED_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
int16_t radioState = -1;

void setup() {
  Serial.begin(115200);
  delay(1500);                                   // time to open the Serial Monitor (no waiting loop)
  Serial.println("\n=== T1 board check ===");

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // OLED: power via Vext, then reset pulse
  vextOn();
  pinMode(PIN_OLED_RST, OUTPUT);
  digitalWrite(PIN_OLED_RST, LOW);  delay(20);
  digitalWrite(PIN_OLED_RST, HIGH); delay(20);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  WiFi.mode(WIFI_STA);                            // only to read the MAC
  Serial.printf("MAC address : %s\n", WiFi.macAddress().c_str());
  WiFi.mode(WIFI_OFF);

  radioState = vfRadioBegin();
  Serial.printf("RadioLib    : %s\n", radioState == RADIOLIB_ERR_NONE ? "SX1262 OK" : "FAILED");
  if (radioState != RADIOLIB_ERR_NONE) Serial.printf("  error code %d (see RadioLib status codes)\n", radioState);
  Serial.printf("Front end   : %s\n", femName());
  radio.sleep();
}

void loop() {
  static uint32_t last = 0;
  const bool pressed = digitalRead(PIN_BUTTON) == LOW;
  digitalWrite(PIN_LED, (millis() / (pressed ? 100 : 500)) % 2);

  if (millis() - last < 1000) return;
  last = millis();
  const uint16_t mv = batteryMv();

  Serial.printf("battery %u mV (%u %%)  button %s\n", mv, batteryPercent(mv), pressed ? "PRESSED" : "-");

  display.clear();
  display.drawString(0, 0,  "T1 board check");
  display.drawString(0, 14, String("Battery: ") + mv + " mV  " + batteryPercent(mv) + "%");
  display.drawString(0, 26, String("Radio: ") + (radioState == RADIOLIB_ERR_NONE ? "OK" : "FAIL " + String(radioState)));
  display.drawString(0, 38, String("FEM: ") + femName());
  display.drawString(0, 50, pressed ? "Button: PRESSED" : "Button: -");
  display.display();
}
