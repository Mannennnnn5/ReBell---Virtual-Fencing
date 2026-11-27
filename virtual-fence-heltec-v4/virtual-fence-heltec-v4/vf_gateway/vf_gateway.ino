// =============================================================================
//  Alpine Virtual Fence – GATEWAY firmware (Heltec WiFi LoRa 32 V4)
//  Prototype: internet through a phone hotspot (Wi-Fi). An LTE modem can
//  replace the Wi-Fi part later; the radio part stays the same.
//
//  radioTask (core 1) – listens on 869.525 MHz, answers collars within ~50 ms
//                       (ACK / CONFIG / FENCE chunks), queues records
//  cloudTask (core 0) – Wi-Fi + MQTT: position batches, events immediately,
//                       status every minute, fence/config commands from the
//                       cloud; updates the OLED
//
//  Libraries: RadioLib, PubSubClient, ArduinoJson, ThingPulse SSD1306
// =============================================================================
#include "config.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#if GATEWAY_GNSS
#include <TinyGPSPlus.h>
#endif
#if !VF_BENCH
#include <SSD1306Wire.h>
#endif

#include "heltec_v4.h"
#include "protocol.h"
#include "geofence.h"
#include "vf_radio.h"

#ifndef VF_GW_WIFI_MODE
#define VF_GW_WIFI_MODE WIFI_STA        // bench mode (ESP-NOW) uses WIFI_AP_STA, see vf_radio_espnow.h
#endif

#if VF_DEBUG
#define LOG(...) Serial.printf(__VA_ARGS__)
#else
#define LOG(...) do {} while (0)
#endif

#if MQTT_TLS
WiFiClientSecure net;
#else
WiFiClient net;
#endif
PubSubClient mqtt(net);
Preferences  prefs;
SSD1306Wire  display(OLED_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);

// ------------------------------------------------- desired network state ----
#define VF_MAX_COLLARS 300                         // device ids 1..299
static uint32_t collarSeenMs[VF_MAX_COLLARS];       // last packet per collar (screen "Collars: n")


struct DesiredState {
  Geofence fence;
  VfConfig cfg;
  bool     haveCfg = false;
  uint8_t  mode[VF_MAX_COLLARS] = {0};              // VfBellMode per collar (farmer's choice)
  bool     ringNow[VF_MAX_COLLARS] = {false};       // "Ring now" waiting for the collar's next report
} desired;

// Herd bell switch + daily silent hours (from the website, kept in flash)
struct BellPlan {
  uint8_t  enabled = 1;                             // herd switch
  uint8_t  schedEnabled = 0;                        // daily silent hours on/off
  uint16_t offMin = 20 * 60;                        // bells off from (minutes after midnight)
  uint16_t onMin = 6 * 60;                          // bells on again at
} bells;
bool bellsActive = true;                            // what the collars are told right now
SemaphoreHandle_t desiredMutex;

struct RxRecord {
  uint32_t   rxMillis;
  int16_t    rssi;
  int8_t     snrX4;
  VfPosition p;
};
QueueHandle_t upQueue;

uint32_t statRx = 0, statBad = 0, statDown = 0;   // stats, read by the display task
int16_t  lastRssi = 0;
uint16_t lastId = 0;
static TaskHandle_t radioTaskHandle;
static bool publishStatusSoon = false;
static volatile bool radioReady = false;          // cloudTask waits for the radio set-up

// ---------------------------------------------------------------- own GNSS --
#if GATEWAY_GNSS
HardwareSerial GPSSerial(1);
TinyGPSPlus gnss;
static bool gnssFix() { return gnss.location.isValid() && gnss.location.age() < 5000; }

// Satellites HEARD (with a signal), from the $xxGSV sentences - shown while searching.
// (gnss.satellites = satellites USED for a fix, which stays 0 until the fix.)
static uint8_t gsvHeard[4], gsvBest[4];          // per system: GPS, BeiDou, GLONASS, Galileo
static uint8_t gsvTmpHeard[4], gsvTmpBest[4];
static uint32_t gsvSeenMs[4];
static void parseGsv(char *line) {
  int sys = !strncmp(line + 1, "GPGSV", 5) ? 0 : (!strncmp(line + 1, "BDGSV", 5) || !strncmp(line + 1, "GBGSV", 5)) ? 1
          : !strncmp(line + 1, "GLGSV", 5) ? 2 : !strncmp(line + 1, "GAGSV", 5) ? 3 : -1;
  if (sys < 0) return;
  char *f[24]; int n = 0; f[n++] = line;
  for (char *p = line; *p && n < 24; p++) if (*p == ',' || *p == '*') { *p = 0; f[n++] = p + 1; }
  if (n < 4) return;
  const int msg = atoi(f[2]), total = atoi(f[1]);
  if (msg == 1) { gsvTmpHeard[sys] = 0; gsvTmpBest[sys] = 0; }
  for (int i = 4; i + 3 < n; i += 4) {
    const int snr = atoi(f[i + 3]);
    if (*f[i] && snr > 0) { gsvTmpHeard[sys]++; if (snr > gsvTmpBest[sys]) gsvTmpBest[sys] = snr; }
  }
  if (msg == total) { gsvHeard[sys] = gsvTmpHeard[sys]; gsvBest[sys] = gsvTmpBest[sys]; gsvSeenMs[sys] = millis(); }
}
static int gnssHeard(int &best) {
  int h = 0; best = 0;
  for (int s = 0; s < 4; s++) {
    if (millis() - gsvSeenMs[s] > 5000) continue;
    h += gsvHeard[s]; if (gsvBest[s] > best) best = gsvBest[s];
  }
  return h;
}
static void gnssTask(void *) {
  static char line[120];
  static uint8_t len = 0;
  for (;;) {
    while (GPSSerial.available()) {
      const char c = GPSSerial.read();
      gnss.encode(c);
      if (c == '$') len = 0;
      if (c == '\r' || c == '\n') { if (len > 6) { line[len] = 0; parseGsv(line); } len = 0; }
      else if (len < sizeof(line) - 1) line[len++] = c;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
static void gnssBegin() {
  pinMode(PIN_GNSS_RST, OUTPUT);  digitalWrite(PIN_GNSS_RST, HIGH);
  pinMode(PIN_GNSS_WAKE, OUTPUT); digitalWrite(PIN_GNSS_WAKE, HIGH);
  pinMode(PIN_GNSS_EN, OUTPUT);   digitalWrite(PIN_GNSS_EN, LOW);      // GNSS on
  delay(200);
  GPSSerial.setRxBufferSize(2048);
  GPSSerial.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
  delay(300);
  GPSSerial.print("$PCAS04,7*1E\r\n");                                 // GPS + BeiDou + GLONASS
  xTaskCreatePinnedToCore(gnssTask, "gnss", 3072, nullptr, 1, nullptr, 0);
}
#endif

static int activeCollars() {                        // heard in the last 5 minutes
  int n = 0;
  for (int i = 1; i < VF_MAX_COLLARS; i++)
    if (collarSeenMs[i] && millis() - collarSeenMs[i] < 300000UL) n++;
  return n;
}

static void IRAM_ATTR onRadioIrq() {
  BaseType_t woken = pdFALSE;
  if (radioTaskHandle) vTaskNotifyGiveFromISR(radioTaskHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

// =============================================================================
//  Persistence of the desired fence / config
// =============================================================================
static void loadDesired() {
  prefs.begin("gw", true);
  if (prefs.getBytesLength("fence") == sizeof(Geofence)) prefs.getBytes("fence", &desired.fence, sizeof(Geofence));
  if (prefs.getBytesLength("cfg") == sizeof(VfConfig)) { prefs.getBytes("cfg", &desired.cfg, sizeof(VfConfig)); desired.haveCfg = true; }
  if (prefs.getBytesLength("modes") == sizeof(desired.mode)) prefs.getBytes("modes", desired.mode, sizeof(desired.mode));
  if (prefs.getBytesLength("bells") == sizeof(BellPlan)) prefs.getBytes("bells", &bells, sizeof(BellPlan));
  prefs.end();
#if VF_BENCH
  if (!desired.fence.valid()) {                   // bench: start with the website's default fence
    desired.fence.version = 1;
    desired.fence.count = BENCH_FENCE_N;
    for (uint8_t i = 0; i < BENCH_FENCE_N; i++)
      desired.fence.pts[i] = {(int32_t)lround(BENCH_FENCE[i][0] * 1e7), (int32_t)lround(BENCH_FENCE[i][1] * 1e7)};
  }
#endif
  // Always have a collar config, with the report period from config.h. A new or changed
  // period gets a new version, so every collar receives it at its next report.
  if (!desired.haveCfg) {
    desired.cfg = {};
    desired.cfg.nearDistM = 15; desired.cfg.bellEnabled = bells.enabled ? 1 : 0; desired.cfg.cueMaxS = 20;
    desired.cfg.cuePauseS = 10; desired.cfg.maxEpisodes = 5; desired.cfg.escapeDistM = 100;
    desired.cfg.cfgVer = 100;                     // unlikely to match a collar's old version
    desired.haveCfg = true;
  }
  if (desired.cfg.reportPeriodS != COLLAR_REPORT_PERIOD_S) {
    desired.cfg.reportPeriodS = COLLAR_REPORT_PERIOD_S;
    desired.cfg.cfgVer = desired.cfg.cfgVer % 255 + 1;
    prefs.begin("gw", false);
    prefs.putBytes("cfg", &desired.cfg, sizeof(VfConfig));
    prefs.end();
  }
  LOG("[nvs] fence v%u (%u pts), cfg %s v%u\n", desired.fence.version, desired.fence.count,
      desired.haveCfg ? "set" : "none", desired.cfg.cfgVer);
}

static void saveDesired() {
  prefs.begin("gw", false);
  prefs.putBytes("fence", &desired.fence, sizeof(Geofence));
  if (desired.haveCfg) prefs.putBytes("cfg", &desired.cfg, sizeof(VfConfig));
  prefs.putBytes("modes", desired.mode, sizeof(desired.mode));
  prefs.putBytes("bells", &bells, sizeof(BellPlan));
  prefs.end();
}

// =============================================================================
//  RADIO TASK
// =============================================================================
static void sendDown(const uint8_t *buf, size_t len) {
  const int16_t s = vfTransmit(buf, len);
  ulTaskNotifyTake(pdTRUE, 0);                    // drop the "TX done" interrupt
  if (s == RADIOLIB_ERR_NONE) statDown++;
  else LOG("[lora] downlink tx fail %d\n", s);
}

static void replyTo(const VfPosition &p) {
  const uint16_t id = p.h.deviceId;
  if (p.h.type == VF_MSG_EVENT || p.event != VF_EVT_NONE) {   // 1. ACK events
    VfAck a = {{VF_MSG_ACK, NET_ID, id, 0}, p.h.seq};
    delay(10);
    sendDown((uint8_t *)&a, sizeof(a));
  }
  xSemaphoreTake(desiredMutex, portMAX_DELAY);
  Geofence f = desired.fence;
  VfConfig c = desired.cfg;
  const bool haveCfg = desired.haveCfg;
  const uint8_t wantMode = id < VF_MAX_COLLARS ? desired.mode[id] : VF_MODE_AUTO;
  const bool ring = id < VF_MAX_COLLARS && desired.ringNow[id];
  if (ring) desired.ringNow[id] = false;                       // "ring now" is sent once
  xSemaphoreGive(desiredMutex);

  if ((p.mode & 0x0F) != wantMode || ring) {                   // 0. bell mode / ring now
    VfMode m = {{VF_MSG_MODE, NET_ID, id, 0}, wantMode, (uint8_t)(ring ? RING_NOW_S : 0)};
    delay(20);
    sendDown((uint8_t *)&m, sizeof(m));
    LOG("[down] bell %s%s -> %u\n", wantMode == VF_MODE_SILENT ? "SILENT" : "AUTO", ring ? " + ring now" : "", id);
  }

  if (haveCfg && p.cfgVer != c.cfgVer) {                       // 2. config behind?
    c.h = {VF_MSG_CONFIG, NET_ID, id, 0};
    delay(20);
    sendDown((uint8_t *)&c, sizeof(c));
    LOG("[down] cfg v%u -> %u\n", c.cfgVer, id);
  }
  if (f.valid() && p.fenceVer != f.version) {                  // 3. fence behind?
    for (uint8_t first = 0; first < f.count; first += VF_FENCE_CHUNK_PTS) {
      VfFenceChunk ch;
      ch.h = {VF_MSG_FENCE, NET_ID, id, 0};
      ch.fenceVer = f.version;
      ch.totalPts = f.count;
      ch.firstIdx = first;
      ch.count = min<uint8_t>(VF_FENCE_CHUNK_PTS, f.count - first);
      memcpy(ch.pts, &f.pts[first], ch.count * sizeof(VfFencePoint));
      delay(30);                                               // collar re-arms its receiver
      sendDown((uint8_t *)&ch, vfFenceChunkLen(ch.count));
    }
    LOG("[down] fence v%u (%u pts) -> %u\n", f.version, f.count, id);
  }
}

static void radioTask(void *) {
  const int16_t s = vfRadioBegin(onRadioIrq, VF_RADIO_DBM);
  radioReady = true;
  if (s != RADIOLIB_ERR_NONE) { LOG("[lora] radio init FAILED %d\n", s); vTaskDelete(nullptr); }
  radio.setRxBoostedGainMode(true);
  vfStartReceive();
  LOG("[lora] listening on %.3f MHz SF%u, front end %s\n", VF_FREQ_MHZ, VF_SF, femName());
  for (;;) {
    if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000))) { vfStartReceive(); continue; }
    uint8_t buf[256];
    const size_t len = radio.getPacketLength();
    const int16_t st = (len > 0 && len <= sizeof(buf)) ? radio.readData(buf, len) : -1;
    if (st != RADIOLIB_ERR_NONE) {
      if (st != -1) statBad++;
      vfStartReceive();
      continue;
    }
    const VfHeader *h = (const VfHeader *)buf;
    if (len == sizeof(VfPosition) && h->netId == NET_ID &&
        (h->type == VF_MSG_POSITION || h->type == VF_MSG_EVENT) &&
        h->deviceId > 0 && h->deviceId < VF_BROADCAST_ID) {
      RxRecord r;
      r.rxMillis = millis();
      r.rssi = (int16_t)radio.getRSSI();
      r.snrX4 = (int8_t)(radio.getSNR() * 4);
      memcpy(&r.p, buf, sizeof(VfPosition));
      statRx++; lastRssi = r.rssi; lastId = h->deviceId;
      replyTo(r.p);                                            // collar is listening now
      if (xQueueSend(upQueue, &r, 0) != pdTRUE) LOG("[q] full, record dropped\n");
      LOG("[rx] id %u seq %u rssi %d snr %.1f ev %u\n", h->deviceId, h->seq, r.rssi, r.snrX4 / 4.0, r.p.event);
      if (h->deviceId < VF_MAX_COLLARS) collarSeenMs[h->deviceId] = millis() | 1;
    } else {
      statBad++;
    }
    vfStartReceive();
  }
}

// =============================================================================
//  CLOUD TASK – Wi-Fi + MQTT + display
// =============================================================================
static const char *zoneName(uint8_t z) {
  static const char *n[] = {"unknown", "inside", "near", "outside", "escaped"};
  return z < 5 ? n[z] : "?";
}
static const char *eventName(uint8_t e) {
  static const char *n[] = {"", "exit", "return", "escaped", "low_batt", "boot"};
  return e < 6 ? n[e] : "?";
}

// =============================================================================
//  Herd bell switch + daily silent hours. The clock comes from the internet
//  (NTP). Changes go to the collars as a CONFIG (bellEnabled) at their next report.
// =============================================================================
static bool clockValid() { return time(nullptr) > 1700000000; }

static void applyBells() {
  if (bells.enabled && bells.schedEnabled && !clockValid()) return;   // wait for the internet clock
  bool active = bells.enabled;
  if (bells.schedEnabled && clockValid()) {
    const time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    const int m = t.tm_hour * 60 + t.tm_min, a = bells.offMin, b = bells.onMin;
    const bool silentHours = (a < b) ? (m >= a && m < b) : (m >= a || m < b);   // may cross midnight
    if (silentHours) active = false;
  }
  bool changed = false;
  xSemaphoreTake(desiredMutex, portMAX_DELAY);
  if (!desired.haveCfg && !active) {                          // first change: start from collar defaults
    desired.cfg = {};
    desired.cfg.cfgVer = 0;
    desired.cfg.reportPeriodS = COLLAR_REPORT_PERIOD_S;
    desired.cfg.nearDistM = 15; desired.cfg.bellEnabled = 1; desired.cfg.cueMaxS = 20;
    desired.cfg.cuePauseS = 10; desired.cfg.maxEpisodes = 5; desired.cfg.escapeDistM = 100;
    desired.haveCfg = true;
  }
  if (desired.haveCfg && (desired.cfg.bellEnabled != 0) != active) {
    desired.cfg.bellEnabled = active;
    desired.cfg.cfgVer = desired.cfg.cfgVer % 255 + 1;
    saveDesired();
    changed = true;
  }
  xSemaphoreGive(desiredMutex);
  if (changed) LOG("[bells] all bells %s (config v%u)\n", active ? "ON" : "OFF", desired.cfg.cfgVer);
  bellsActive = active;
}

static uint16_t parseHhMm(const char *s, uint16_t fallback) {
  int h, m;
  if (!s || sscanf(s, "%d:%d", &h, &m) != 2 || h < 0 || h > 23 || m < 0 || m > 59) return fallback;
  return h * 60 + m;
}

// "COW-001" -> 1 (0 = not a collar id)
static uint16_t collarId(const String &name) {
  const int dash = name.lastIndexOf('-');
  const long id = name.substring(dash + 1).toInt();
  return (id > 0 && id < VF_MAX_COLLARS) ? (uint16_t)id : 0;
}

// Tell the website whether the gateway took a command (not that a collar did it).
static void publishAck(const String &commandId, const String &deviceId, const char *status, const char *reason = nullptr) {
  JsonDocument d;
  d["type"] = "ack";
  d["commandId"] = commandId;
  d["deviceId"] = deviceId;
  d["status"] = status;
  if (reason) d["reason"] = reason;
  String out; serializeJson(d, out);
  mqtt.publish((String(TOPIC_PREFIX) + "/event").c_str(), out.c_str());
  LOG("[mqtt] ack %s %s\n", commandId.c_str(), status);
}

static void onMqttMessage(char *topic, byte *payload, unsigned int len) {
  JsonDocument doc;
  if (deserializeJson(doc, payload, len)) { LOG("[mqtt] bad json on %s\n", topic); return; }
  const char *type = doc["type"] | "";
  const String commandId = doc["commandId"] | "";
  const String target = doc["deviceId"] | "ALL";
  const char *ack = nullptr, *reason = nullptr;
  xSemaphoreTake(desiredMutex, portMAX_DELAY);
  if (!strcmp(type, "fence")) {
    JsonArray pts = doc["points"].as<JsonArray>();
    if (pts.size() >= 3 && pts.size() <= VF_MAX_FENCE_POINTS) {
      desired.fence.version = doc["version"] | 1;
      desired.fence.count = pts.size();
      uint8_t i = 0;
      for (JsonArray pt : pts) {
        desired.fence.pts[i].lat = lround(pt[0].as<double>() * 1e7);
        desired.fence.pts[i].lon = lround(pt[1].as<double>() * 1e7);
        i++;
      }
      LOG("[cmd] fence v%u, %u points\n", desired.fence.version, desired.fence.count);
    }
  } else if (!strcmp(type, "config") && doc["boundary"].is<JsonArray>()) {
    // ReBell website "Send fence to gateway": {"type":"config","boundary":[[lat,lon],..]}
    JsonArray pts = doc["boundary"].as<JsonArray>();
    if (pts.size() >= 3 && pts.size() <= VF_MAX_FENCE_POINTS) {
      Geofence f;
      f.count = pts.size();
      uint8_t i = 0;
      for (JsonArray pt : pts) {
        f.pts[i].lat = lround(pt[0].as<double>() * 1e7);
        f.pts[i].lon = lround(pt[1].as<double>() * 1e7);
        i++;
      }
      const bool same = f.count == desired.fence.count &&
                        !memcmp(f.pts, desired.fence.pts, f.count * sizeof(VfFencePoint));
      f.version = same ? desired.fence.version : (uint8_t)(desired.fence.version % 255 + 1);
      desired.fence = f;
      ack = "accepted";
      LOG("[cmd] website fence v%u, %u points%s\n", f.version, f.count, same ? " (unchanged)" : "");
    } else {
      ack = "rejected"; reason = "fence needs 3-32 points";
    }
  } else if (!strcmp(type, "command")) {
    // Website per-cow bell: AUTO = bell on, SILENT = bell off, SOUND = ring now
    const uint16_t id = collarId(target);
    const char *mode = doc["mode"] | "";
    if (!id) { ack = "rejected"; reason = "unknown collar"; }
    else if (!strcmp(mode, "AUTO"))   { desired.mode[id] = VF_MODE_AUTO;   desired.ringNow[id] = false; ack = "accepted"; }
    else if (!strcmp(mode, "SILENT")) { desired.mode[id] = VF_MODE_SILENT; desired.ringNow[id] = false; ack = "accepted"; }
    else if (!strcmp(mode, "SOUND"))  { desired.ringNow[id] = true; ack = "accepted"; }
    else { ack = "rejected"; reason = "mode must be AUTO, SILENT or SOUND"; }
    LOG("[cmd] bell %s for collar %u: %s\n", mode, id, ack);
  } else if (!strcmp(type, "bells")) {
    // Website herd switch + daily silent hours: {"type":"bells","enabled":true,"schedule":{"enabled":true,"off":"20:00","on":"06:00"}}
    bells.enabled = (doc["enabled"] | true) ? 1 : 0;
    JsonObject sc = doc["schedule"];
    if (!sc.isNull()) {
      bells.schedEnabled = (sc["enabled"] | false) ? 1 : 0;
      bells.offMin = parseHhMm(sc["off"] | "", bells.offMin);
      bells.onMin  = parseHhMm(sc["on"] | "", bells.onMin);
    }
    ack = "accepted";
    LOG("[cmd] bells %s, silent hours %s %02u:%02u-%02u:%02u\n", bells.enabled ? "on" : "off", bells.schedEnabled ? "on" : "off",
        bells.offMin / 60, bells.offMin % 60, bells.onMin / 60, bells.onMin % 60);
  } else if (!strcmp(type, "config")) {
    VfConfig &c = desired.cfg;
    c.cfgVer        = doc["version"] | 1;
    c.reportPeriodS = doc["reportPeriodS"] | COLLAR_REPORT_PERIOD_S;
    c.nearDistM     = doc["nearDistM"] | 15;
    c.bellEnabled   = (doc["bellEnabled"] | true) ? 1 : 0;
    c.cueMaxS       = doc["cueMaxS"] | 20;
    c.cuePauseS     = doc["cuePauseS"] | 10;
    c.maxEpisodes   = doc["maxEpisodes"] | 5;
    c.escapeDistM   = doc["escapeDistM"] | 100;
    desired.haveCfg = true;
    LOG("[cmd] config v%u\n", c.cfgVer);
  }
  saveDesired();
  xSemaphoreGive(desiredMutex);
  if (!strcmp(type, "bells")) { applyBells(); publishStatusSoon = true; }
  if (ack && commandId.length()) publishAck(commandId, target, ack, reason);
}

static bool wifiUp() {
  if (WiFi.status() == WL_CONNECTED) return true;
  static uint32_t lastTry = 0;
  if (lastTry && millis() - lastTry < 15000) return false;
  lastTry = millis();
  LOG("[wifi] connecting to %s ...\n", WIFI_SSID);
  WiFi.mode(VF_GW_WIFI_MODE);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) vTaskDelay(pdMS_TO_TICKS(250));
  if (WiFi.status() != WL_CONNECTED) { LOG("[wifi] not connected (status %d)\n", WiFi.status()); return false; }
  LOG("[wifi] connected, IP %s, RSSI %d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  return true;
}

static bool mqttUp() {
  if (mqtt.connected()) return true;
  static uint32_t lastTry = 0;
  if (lastTry && millis() - lastTry < 10000) return false;
  lastTry = millis();
#if MQTT_TLS
  net.setInsecure();                              // prototype: no certificate check
#endif
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(BATCH_MAX_BYTES + 512);
  mqtt.setKeepAlive(60);
  mqtt.setCallback(onMqttMessage);
  const String will = String(TOPIC_PREFIX) + "/online";
  const String clientId = String("vf-") + GATEWAY_ID + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  const bool ok = strlen(MQTT_USER)
      ? mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, will.c_str(), 1, true, "0")
      : mqtt.connect(clientId.c_str(), will.c_str(), 1, true, "0");
  if (!ok) { LOG("[mqtt] connect failed, state %d\n", mqtt.state()); return false; }
  mqtt.publish(will.c_str(), "1", true);
  mqtt.subscribe((String(TOPIC_PREFIX) + "/cmd/#").c_str(), 1);
  LOG("[mqtt] connected to %s, topics %s/...\n", MQTT_HOST, TOPIC_PREFIX);
  return true;
}

static void addRecord(JsonArray arr, const RxRecord &r) {
  const VfPosition &p = r.p;
  JsonObject o = arr.add<JsonObject>();
  o["id"]  = p.h.deviceId;
  o["seq"] = p.h.seq;
  o["age"] = millis() - r.rxMillis;
  if (p.flags & VF_FLAG_FIX_VALID) {
    o["lat"]  = serialized(String(p.lat * 1e-7, 7));
    o["lon"]  = serialized(String(p.lon * 1e-7, 7));
    o["hacc"] = p.hAccDm / 10.0;
  }
  o["zone"] = zoneName((p.flags >> VF_FLAG_ZONE_SHIFT) & 0x07);
  if (p.fenceDistM != INT16_MAX) o["dist"] = p.fenceDistM;
  o["bell"] = (p.flags & VF_FLAG_BELL_ON) ? 1 : 0;
  o["batt"] = p.battMv;
  o["temp"] = p.tempC;
  o["sats"] = p.sats;
  o["fv"]   = p.fenceVer;
  o["cv"]   = p.cfgVer;
  if (p.event) o["ev"] = eventName(p.event);
  o["rssi"] = r.rssi;
  o["snr"]  = r.snrX4 / 4.0;
}

static void publishStatus() {
  JsonDocument d;
  const uint16_t mv = batteryMv();
#if CLOUD_FORMAT_REBELL
  d["type"]      = "gateway_position";        // website shows the gateway on the map
  d["deviceId"]  = GATEWAY_ID;
#if GATEWAY_GNSS
  { int best; d["satsHeard"] = gnssHeard(best); d["satsBestDb"] = best; }
  if (gnssFix()) {                               // own GPS position
    d["gpsValid"] = true;
    JsonObject loc = d["location"].to<JsonObject>();
    loc["latitude"]  = serialized(String(gnss.location.lat(), 7));
    loc["longitude"] = serialized(String(gnss.location.lng(), 7));
    loc["accuracyM"] = gnss.hdop.hdop() * 2.5;
    d["sats"] = gnss.satellites.value();
  } else
#endif
  {
    d["gpsValid"]  = (GATEWAY_LAT != 0.0 || GATEWAY_LON != 0.0);
    if (GATEWAY_LAT != 0.0 || GATEWAY_LON != 0.0) {
      JsonObject loc = d["location"].to<JsonObject>();
      loc["latitude"]  = GATEWAY_LAT;
      loc["longitude"] = GATEWAY_LON;
    }
  }
  if (VF_BENCH) d["simulated"] = true;
#endif
  d["gw"]        = GATEWAY_ID;
  d["uptime_s"]  = millis() / 1000;
  d["batt_mv"]   = mv;
  d["batt_pct"]  = batteryPercent(mv);
  d["temp_c"]    = serialized(String(temperatureRead(), 1));
  d["wifi_rssi"] = WiFi.RSSI();
  d["rx"]        = statRx;
  d["rx_bad"]    = statBad;
  d["down"]      = statDown;
  xSemaphoreTake(desiredMutex, portMAX_DELAY);
  JsonObject b = d["bells"].to<JsonObject>();
  b["enabled"] = bells.enabled != 0;
  b["active"]  = bellsActive;
  JsonObject sc = b["schedule"].to<JsonObject>();
  char hm[6];
  sc["enabled"] = bells.schedEnabled != 0;
  snprintf(hm, sizeof(hm), "%02u:%02u", bells.offMin / 60, bells.offMin % 60); sc["off"] = hm;
  snprintf(hm, sizeof(hm), "%02u:%02u", bells.onMin / 60, bells.onMin % 60);   sc["on"] = hm;
  if (clockValid()) {
    const time_t now = time(nullptr); struct tm t; localtime_r(&now, &t);
    snprintf(hm, sizeof(hm), "%02d:%02d", t.tm_hour, t.tm_min); b["clock"] = hm;
  }
  d["fence_ver"] = desired.fence.version;
  d["cfg_ver"]   = desired.cfg.cfgVer;
  xSemaphoreGive(desiredMutex);
  String s; serializeJson(d, s);
  mqtt.publish((String(TOPIC_PREFIX) + "/status").c_str(), s.c_str(), true);
  LOG("[mqtt] status %s\n", s.c_str());
}

// ReBell website format: one telemetry message per report, plus an event message.
static void publishRebell(const RxRecord &r) {
  const VfPosition &p = r.p;
  char cow[12];
  snprintf(cow, sizeof(cow), "COW-%03u", p.h.deviceId);
  const bool fix = p.flags & VF_FLAG_FIX_VALID;
  const uint8_t zone = (p.flags >> VF_FLAG_ZONE_SHIFT) & 0x07;

  JsonDocument d;
  d["type"]     = "telemetry";
  d["deviceId"] = cow;
  if (VF_BENCH) d["simulated"] = true;         // bench: GPS positions are simulated
  d["gpsValid"] = fix;
  if (fix) {
    JsonObject loc = d["location"].to<JsonObject>();
    loc["latitude"]  = serialized(String(p.lat * 1e-7, 7));
    loc["longitude"] = serialized(String(p.lon * 1e-7, 7));
    loc["accuracyM"] = p.hAccDm / 10.0;
  }
  d["batteryPercent"] = batteryPercent(p.battMv);
  d["batteryMv"]      = p.battMv;
  d["rssiDbm"]        = r.rssi;
  d["snrDb"]          = r.snrX4 / 4.0;
  d["soundMode"]      = (p.mode & VF_MODE_SOUNDING) ? "SOUND" : ((p.mode & 0x0F) == VF_MODE_SILENT ? "SILENT" : "AUTO");
  d["clapperState"]   = (p.flags & VF_FLAG_BELL_ON) ? "RELEASED" : "LOCKED";
  d["zone"]           = zoneName(zone);
  if (p.fenceDistM != INT16_MAX) d["fenceDistM"] = p.fenceDistM;
  d["sats"]     = p.sats;
  d["tempC"]    = p.tempC;
  d["seq"]      = p.h.seq;
  d["fenceVer"] = p.fenceVer;
  d["cfgVer"]   = p.cfgVer;
  String out; serializeJson(d, out);
  mqtt.publish((String(TOPIC_PREFIX) + "/up").c_str(), out.c_str());
  LOG("[mqtt] %s %s %s %d m%s\n", cow, zoneName(zone), (p.flags & VF_FLAG_BELL_ON) ? "BELL" : "", 
      p.fenceDistM == INT16_MAX ? 0 : p.fenceDistM, p.event ? " +event" : "");

  if (p.event == VF_EVT_NONE) return;
  JsonDocument e;
  e["type"] = "event";
  e["deviceId"] = cow;
  e["eventType"] = eventName(p.event);
  e["severity"] = (p.event == VF_EVT_EXIT || p.event == VF_EVT_ESCAPED || p.event == VF_EVT_LOW_BATT) ? "alert" : "info";
  char title[48], desc[96];
  switch (p.event) {
    case VF_EVT_EXIT:
      e["eventType"] = "boundary_breach";
      snprintf(title, sizeof(title), "Virtual fence crossed · %s", cow);
      snprintf(desc, sizeof(desc), "%d m outside. Bell cue started on the collar.", -p.fenceDistM);
      break;
    case VF_EVT_RETURN:
      snprintf(title, sizeof(title), "%s back inside", cow);
      snprintf(desc, sizeof(desc), "Returned inside the fence. Bell locked.");
      break;
    case VF_EVT_ESCAPED:
      snprintf(title, sizeof(title), "%s escaped – fetch the cow", cow);
      snprintf(desc, sizeof(desc), "Bell cues did not work (%d m outside). Bell stopped for animal welfare.", -p.fenceDistM);
      break;
    case VF_EVT_LOW_BATT:
      snprintf(title, sizeof(title), "Low battery · %s", cow);
      snprintf(desc, sizeof(desc), "%u mV", p.battMv);
      break;
    default:
      snprintf(title, sizeof(title), "%s started", cow);
      snprintf(desc, sizeof(desc), "Collar booted, fence v%u.", p.fenceVer);
  }
  e["title"] = title;
  e["description"] = desc;
  out = "";
  serializeJson(e, out);
  mqtt.publish((String(TOPIC_PREFIX) + "/event").c_str(), out.c_str());
}

// Screen:
//   G  WiFi ok  MQTT ok     t=178s
//   Collars: 2
//   FIX 9 used  +-8 m
//   46.53902 N  6.62114 E
static void showStatus() {
  const uint32_t up = millis() / 1000;
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, String("G  ") + (WiFi.status() == WL_CONNECTED ? "WiFi ok" : "no WiFi") +
                           "  " + (mqtt.connected() ? "MQTT ok" : "MQTT --"));
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, up < 10000 ? "t=" + String(up) + "s" : "t=" + String(up / 60) + "m");
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 14, "Collars: " + String(activeCollars()));
#if GATEWAY_GNSS
  if (gnssFix()) {
    const double lat = gnss.location.lat(), lon = gnss.location.lng();
    display.drawString(0, 28, "FIX " + String(gnss.satellites.value()) + " used  +-" + String(gnss.hdop.hdop() * 2.5, 0) + " m");
    display.drawString(0, 42, String(fabs(lat), 5) + (lat >= 0 ? " N  " : " S  ") + String(fabs(lon), 5) + (lon >= 0 ? " E" : " W"));
  } else {
    int best;
    const int heard = gnssHeard(best);
    display.drawString(0, 28, "GPS searching... heard " + String(heard));
    display.drawString(0, 42, gnss.charsProcessed() < 10 ? String("no GNSS data - check module")
                              : heard == 0 ? String("no satellites - open sky / antenna")
                              : "best signal " + String(best) + " dB" + (heard < 4 ? "  need 4+" : "  wait..."));
  }
#else
  display.drawString(0, 28, "GPS: none (bench gateway)");
#endif
  display.display();
}

static void cloudTask(void *) {
  while (!radioReady) vTaskDelay(pdMS_TO_TICKS(50));   // ESP-NOW bench: Wi-Fi is set up by the radio first
  JsonDocument batch;
  JsonArray recs = batch["recs"].to<JsonArray>();
  batch["gw"] = GATEWAY_ID;
  uint32_t lastBatch = millis(), lastStatus = 0, lastShow = 0;

  for (;;) {
    if (millis() - lastShow > 2000) { showStatus(); lastShow = millis(); }
    if (!wifiUp() || !mqttUp()) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }   // records stay queued
    static bool ntpStarted = false;
    if (!ntpStarted) { configTzTime(GW_TIMEZONE, "pool.ntp.org", "time.google.com"); ntpStarted = true; }
    static uint32_t lastBells = 0;
    if (!lastBells || millis() - lastBells > 10000) { applyBells(); lastBells = millis(); }
    if (publishStatusSoon) { publishStatusSoon = false; lastStatus = 0; }
    mqtt.loop();

    RxRecord r;
    bool urgent = false;
    while (xQueueReceive(upQueue, &r, pdMS_TO_TICKS(200)) == pdTRUE) {
#if CLOUD_FORMAT_REBELL
      publishRebell(r);                            // website: every report right away
      continue;
#endif
      addRecord(recs, r);
      if (r.p.event != VF_EVT_NONE) urgent = true;
      if (measureJson(batch) > BATCH_MAX_BYTES - 200) break;
    }
    const bool due = millis() - lastBatch > BATCH_INTERVAL_S * 1000UL;
    if (recs.size() && (urgent || due || measureJson(batch) > BATCH_MAX_BYTES - 200)) {
      String s; serializeJson(batch, s);
      const String topic = String(TOPIC_PREFIX) + (urgent ? "/event" : "/up");
      if (mqtt.publish(topic.c_str(), s.c_str())) {
        LOG("[mqtt] %s %u recs, %u B\n", urgent ? "event" : "batch", recs.size(), s.length());
        batch.clear();
        recs = batch["recs"].to<JsonArray>();
        batch["gw"] = GATEWAY_ID;
        lastBatch = millis();
      }
    }
    if (!lastStatus || millis() - lastStatus > STATUS_INTERVAL_S * 1000UL) { publishStatus(); lastStatus = millis(); }
  }
}

// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1500);                                    // no waiting loop: runs without USB too
  LOG("\n=== gateway %s (net 0x%02X) ===\n", GATEWAY_ID, NET_ID);
  vextOn();
  pinMode(PIN_OLED_RST, OUTPUT);
  digitalWrite(PIN_OLED_RST, LOW); delay(20); digitalWrite(PIN_OLED_RST, HIGH); delay(20);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Virtual fence gateway");
  display.drawString(0, 14, "starting...");
  display.display();
#if GATEWAY_GNSS
  gnssBegin();
#endif

  desiredMutex = xSemaphoreCreateMutex();
  upQueue = xQueueCreate(200, sizeof(RxRecord));
  loadDesired();
  xTaskCreatePinnedToCore(radioTask, "radio", 8192, nullptr, 5, &radioTaskHandle, 1);
  xTaskCreatePinnedToCore(cloudTask, "cloud", 12288, nullptr, 2, nullptr, 0);
}

void loop() { vTaskDelay(portMAX_DELAY); }
