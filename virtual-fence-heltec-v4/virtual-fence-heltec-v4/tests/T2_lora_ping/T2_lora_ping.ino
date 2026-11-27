/*
  T2 – LoRa ping between boards (Heltec WiFi LoRa 32 V4)
  -------------------------------------------------------
  Flash this SAME sketch on all three boards (2 collars + gateway).
  Every board:
    - sends a small "ping" every 5 s (with random jitter so boards don't collide)
    - listens the rest of the time and shows every ping it hears from the others,
      with RSSI (signal strength) and SNR (signal quality)
  Press the PRG button to send a ping immediately.

  Uses exactly the virtual-fence radio settings: 869.525 MHz, SF9, 125 kHz,
  CR 4/5, sync word 0x12. Airtime ~165 ms every 5 s = 3.3 % (limit 10 %).

  Read the result:
    RSSI  > -100 dBm  very good   |  -100 .. -115  ok  |  < -120  near the limit
    SNR   > 0 dB      very good   |  down to about -12 dB still decodes at SF9

  Libraries: RadioLib, ThingPulse SSD1306.
*/
#include <Wire.h>
#include <SSD1306Wire.h>
#include <WiFi.h>
#include "heltec_v4.h"
#include "protocol.h"
#include "vf_radio.h"

SSD1306Wire display(OLED_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);

struct __attribute__((packed)) Ping {
  char     magic[2];     // "VP"
  uint16_t from;         // last two bytes of the MAC
  uint32_t counter;
  int8_t   lastRssiHeard;
};

volatile bool rxFlag = false;
void IRAM_ATTR onRx() { rxFlag = true; }

uint16_t myId;
uint32_t sent = 0, heard = 0;
uint32_t nextTx = 0;
String lines[3];

void show() {
  display.clear();
  display.drawString(0, 0, String("T2 ping  id ") + String(myId, HEX) + "  tx " + sent);
  display.drawString(0, 12, String("heard ") + heard + "  FEM " + (g_fem == FEM_GC1109 ? "GC1109" : "KCT8103L"));
  for (int i = 0; i < 3; i++) display.drawString(0, 26 + i * 12, lines[i]);
  display.display();
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  vextOn();
  pinMode(PIN_OLED_RST, OUTPUT);
  digitalWrite(PIN_OLED_RST, LOW); delay(20); digitalWrite(PIN_OLED_RST, HIGH); delay(20);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  uint8_t mac[6];
  WiFi.mode(WIFI_STA); WiFi.macAddress(mac); WiFi.mode(WIFI_OFF);
  myId = (mac[4] << 8) | mac[5];

  int16_t s = vfRadioBegin(onRx);
  Serial.printf("\n=== T2 LoRa ping, id %04X, radio %s (%d), FEM %s ===\n", myId,
                s == RADIOLIB_ERR_NONE ? "OK" : "FAILED", s, femName());
  if (s != RADIOLIB_ERR_NONE) { lines[0] = "RADIO FAILED " + String(s); show(); while (true) delay(1000); }
  vfStartReceive();
  randomSeed(esp_random());
  nextTx = millis() + random(1000, 5000);
  show();
}

void sendPing() {
  static int8_t lastRssi = 0;
  Ping p = {{'V', 'P'}, myId, ++sent, lastRssi};
  digitalWrite(PIN_LED, HIGH);
  int16_t s = vfTransmit((uint8_t *)&p, sizeof(p));
  digitalWrite(PIN_LED, LOW);
  Serial.printf("[tx] ping #%lu %s\n", (unsigned long)sent, s == RADIOLIB_ERR_NONE ? "sent" : "FAILED");
  rxFlag = false;                       // transmit() also fires DIO1
  vfStartReceive();
  show();
}

void loop() {
  if (rxFlag) {
    rxFlag = false;
    Ping p;
    size_t len = radio.getPacketLength();
    int16_t s = radio.readData((uint8_t *)&p, sizeof(p));
    float rssi = radio.getRSSI(), snr = radio.getSNR();
    if (s == RADIOLIB_ERR_NONE && len == sizeof(p) && p.magic[0] == 'V' && p.magic[1] == 'P') {
      heard++;
      String l = String(p.from, HEX) + " #" + p.counter + "  " + String(rssi, 0) + "dBm " + String(snr, 1) + "dB";
      lines[2] = lines[1]; lines[1] = lines[0]; lines[0] = l;
      Serial.printf("[rx] from %04X  #%lu  RSSI %.0f dBm  SNR %.1f dB  (they heard us at %d dBm)\n",
                    p.from, (unsigned long)p.counter, rssi, snr, p.lastRssiHeard);
      show();
    } else if (s == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("[rx] packet with CRC error (too weak or collision)");
    }
    vfStartReceive();
  }
  if (millis() > nextTx || digitalRead(PIN_BUTTON) == LOW) {
    sendPing();
    nextTx = millis() + 5000 + random(0, 1500);
    while (digitalRead(PIN_BUTTON) == LOW) delay(10);
  }
}
