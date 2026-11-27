// =============================================================================
//  BENCH MODE radio: ESP-NOW instead of LoRa, for plain ESP32 boards.
//
//  Offers the same functions as the LoRa part of vf_radio.h
//  (vfRadioBegin, vfTransmit, vfStartReceive, vfTimeOnAirMs and the few
//  radio.xxx() calls), so vf_collar and vf_gateway run unchanged:
//  same packets (protocol.h), same replies, same timing logic.
//
//  Behaves like the LoRa link where it matters:
//   - every packet is a broadcast, filtered by netId and deviceId as before
//   - the collar only "hears" packets that arrive after its own transmission
//   - vfTimeOnAirMs() returns the real LoRa airtime (SF9/125 kHz), so the
//     collar's duty-cycle limit behaves exactly as on the Heltec
//
//  ESP-NOW and Wi-Fi share one radio, so both must use the same channel. The
//  gateway's channel is fixed by the phone hotspot. The gateway therefore also
//  runs a small access point "VF-GW-<netId>" (on that same channel) and the
//  collars scan for it to find the channel. Nobody has to join that network.
// =============================================================================
#pragma once
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "protocol.h"

#define RADIOLIB_ERR_NONE 0
#ifndef VF_RADIO_DBM
#define VF_RADIO_DBM 5
#endif
#define VF_BENCH_AP_PASS  "vf-bench-only"     // beacon network, nobody needs to join

#ifdef VF_ROLE_GATEWAY
#define VF_GW_WIFI_MODE   WIFI_AP_STA         // hotspot (STA) + beacon for the collars (AP)
#endif

namespace vfnow {

struct Pkt { uint8_t len; int8_t rssi; int8_t noise; uint8_t data[250]; };
static const uint8_t kBroadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static Pkt ring[16];
static volatile uint8_t head = 0, tail = 0;   // write / read index
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static void (*irq)(void) = nullptr;
static int8_t   lastRssi = 0, lastNoise = -95;
static uint8_t  channel = 1;
static volatile uint32_t lastRxMs = 0;
static uint32_t lastScanMs = 0;

static inline bool empty() { return head == tail; }

// Gateway keeps uplinks, collar keeps downlinks (a LoRa receiver would hear
// both, but the firmware ignores the other direction anyway).
static bool wanted(const uint8_t *d, int len) {
  if (len < (int)sizeof(VfHeader) || len > (int)sizeof(Pkt::data) || d[1] != NET_ID) return false;
#ifdef VF_ROLE_GATEWAY
  return d[0] < VF_MSG_ACK;
#else
  return d[0] >= VF_MSG_ACK;
#endif
}

static void push(const uint8_t *d, int len, int rssi, int noise) {
  if (!wanted(d, len)) return;
  portENTER_CRITICAL(&mux);
  const uint8_t next = (head + 1) % 16;
  if (next != tail) {
    Pkt &p = ring[head];
    p.len = len; p.rssi = rssi; p.noise = noise;
    memcpy(p.data, d, len);
    head = next;
  }
  portEXIT_CRITICAL(&mux);
  lastRxMs = millis();
  if (irq) irq();                    // like DIO1 "packet received"
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *d, int len) {
  push(d, len, info->rx_ctrl ? info->rx_ctrl->rssi : -50, info->rx_ctrl ? info->rx_ctrl->noise_floor : -95);
}
#else
static void onRecv(const uint8_t *, const uint8_t *d, int len) { push(d, len, -50, -95); }
#endif

static void flush() { portENTER_CRITICAL(&mux); tail = head; portEXIT_CRITICAL(&mux); }

static void setChannel(uint8_t ch) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  channel = ch;
}

static String beaconName() { char b[16]; snprintf(b, sizeof(b), "VF-GW-%02X", NET_ID); return String(b); }

// Collar: look for the gateway's beacon network and move to its channel.
// quick = only check the current channel (~0.1 s), else all channels (~2.5 s).
static bool findGateway(bool quick) {
  lastScanMs = millis();
  const String name = beaconName();
  const int n = WiFi.scanNetworks(false, false, false, quick ? 120 : 200, quick ? channel : 0, name.c_str());
  int ch = 0;
  for (int i = 0; i < n && !ch; i++) if (WiFi.SSID(i) == name) ch = WiFi.channel(i);
  WiFi.scanDelete();
  if (!ch) { setChannel(channel); if (!quick) Serial.printf("[espnow] gateway beacon %s not found\n", name.c_str()); return false; }
  if (ch != channel || !quick) Serial.printf("[espnow] gateway found on Wi-Fi channel %d\n", ch);
  setChannel(ch);
  return true;
}

}  // namespace vfnow

// The few RadioLib calls the firmware makes, answered from the receive buffer.
class VfEspNowRadio {
 public:
  size_t getPacketLength() { return vfnow::empty() ? 0 : vfnow::ring[vfnow::tail].len; }
  int16_t readData(uint8_t *buf, size_t len) {
    if (vfnow::empty()) return -1;
    portENTER_CRITICAL(&vfnow::mux);
    const vfnow::Pkt &p = vfnow::ring[vfnow::tail];
    memcpy(buf, p.data, p.len < len ? p.len : len);
    vfnow::lastRssi = p.rssi; vfnow::lastNoise = p.noise;
    vfnow::tail = (vfnow::tail + 1) % 16;
    portEXIT_CRITICAL(&vfnow::mux);
    return RADIOLIB_ERR_NONE;
  }
  float   getRSSI() { return vfnow::lastRssi; }
  float   getSNR()  { return vfnow::lastRssi - vfnow::lastNoise; }   // signal above noise floor
  int16_t standby() { return RADIOLIB_ERR_NONE; }
  int16_t sleep()   { return RADIOLIB_ERR_NONE; }
  int16_t setRxBoostedGainMode(bool) { return RADIOLIB_ERR_NONE; }
};
static VfEspNowRadio radio;

static int16_t vfRadioBegin(void (*irq)(void) = nullptr, int8_t dbm = VF_RADIO_DBM) {
  (void)dbm;
  vfnow::irq = irq;
#ifdef VF_ROLE_GATEWAY
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);                       // Wi-Fi power save would miss ESP-NOW packets
  WiFi.softAP(vfnow::beaconName().c_str(), VF_BENCH_AP_PASS, 1, 0, 1);
  vfnow::channel = WiFi.channel();
  Serial.printf("[espnow] gateway beacon %s (moves to the hotspot's channel once connected)\n",
                vfnow::beaconName().c_str());
#else
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  for (int i = 0; i < 3 && !vfnow::findGateway(false); i++) delay(500);
#endif
  if (esp_now_init() != ESP_OK) return -2;
  esp_now_register_recv_cb(vfnow::onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, vfnow::kBroadcast, 6);
  peer.channel = 0;                           // = whatever channel the radio is on
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return -3;
  return RADIOLIB_ERR_NONE;
}

static int16_t vfTransmit(const uint8_t *buf, size_t len) {
#ifndef VF_ROLE_GATEWAY
  // No downlink for 2 min: the gateway may have moved to another channel
  // (hotspot changed). Check again, at most every 30 s.
  if (millis() - vfnow::lastRxMs > 120000 && millis() - vfnow::lastScanMs > 30000)
    if (!vfnow::findGateway(true)) vfnow::findGateway(false);
  vfnow::flush();                             // LoRa: nothing is heard before our own TX
#endif
  return esp_now_send(vfnow::kBroadcast, buf, len) == ESP_OK ? RADIOLIB_ERR_NONE : -1;
}

static int16_t vfStartReceive() {
  if (!vfnow::empty() && vfnow::irq) vfnow::irq();   // packet already waiting -> signal again
  return RADIOLIB_ERR_NONE;
}

// Real LoRa time on air (SX126x formula), so duty-cycle limits match the Heltec.
static uint32_t vfTimeOnAirMs(size_t len) {
  const double tSym = (double)(1UL << VF_SF) / VF_BW_HZ * 1000.0;     // ms
  const int de = (VF_SF >= 11 && VF_BW_HZ <= 125000) ? 1 : 0;
  const double num = 8.0 * len - 4.0 * VF_SF + 28 + 16;               // CRC on, explicit header
  const double payload = 8 + fmax(ceil(num / (4.0 * (VF_SF - 2 * de))) * VF_CR, 0.0);
  return (uint32_t)((VF_PREAMBLE + 4.25 + payload) * tSym);
}
