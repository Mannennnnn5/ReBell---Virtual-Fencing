// =============================================================================
//  Alpine Virtual Fence – COLLAR firmware (Heltec WiFi LoRa 32 V4)
//
//  One cycle:
//    1. get a GPS fix from the L76K (NMEA, TinyGPSPlus)
//    2. signed distance to the fence polygon stored on the collar
//    3. bell cue: servo moves the clapper latch (RING / LOCK), with welfare
//       limits (max episodes, escape distance) and events (exit/return/escaped)
//    4. report position in this collar's TDMA slot, listen for the gateway's
//       reply (ACK, CONFIG, FENCE chunks)
//    5. far inside the fence -> deep sleep until the next slot
//       near the fence / outside -> stay awake, fix every 5 s / 1 s
//
//  Arduino IDE: board "Heltec WiFi LoRa 32(V3)" or "ESP32S3 Dev Module"
//  (see README). Libraries: RadioLib, TinyGPSPlus, ThingPulse SSD1306.
// =============================================================================
#include "config.h"                // first: holds the VF_BENCH switch
#include <Wire.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#if !VF_BENCH
#include <SSD1306Wire.h>
#endif
#include "esp_sleep.h"
#include "driver/gpio.h"

#include "heltec_v4.h"
#include "protocol.h"
#include "geofence.h"
#include "vf_radio.h"

#if VF_DEBUG
#define LOG(...) Serial.printf(__VA_ARGS__)
#else
#define LOG(...) do {} while (0)
#endif

// ------------------------------------------------------------- devices ------
HardwareSerial GPSSerial(1);
TinyGPSPlus    gps;
Preferences    prefs;
SSD1306Wire    display(OLED_ADDR, PIN_OLED_SDA, PIN_OLED_SCL);
uint16_t       DEVICE_ID = 1;

// --------------------------------------------- state kept across deep sleep -
struct CollarConfig {
  uint8_t  ver;
  uint16_t reportPeriodS;
  uint8_t  nearDistM, bellEnabled, cueMaxS, cuePauseS, maxEpisodes, escapeDistM;
};

struct PendingEvent { uint8_t event, seq, tries; };

#define STATE_MAGIC 0xC0FFEE05
struct RtcState {
  uint32_t magic;
  uint8_t  seq;
  uint8_t  zone;               // VfZone
  uint8_t  outsideCount;       // consecutive outside fixes (debounce)
  uint8_t  episodes;           // bell cue episodes in this excursion
  bool     lowBattSent;
  bool     haveTime;
  bool     servoRing;          // last commanded servo position
  uint64_t epochMsAtSleep;
  uint64_t sleepMs;
  uint64_t nextSlotMs;         // UTC ms of next TDMA slot
  int32_t  lat, lon;           // last good fix
  int16_t  distM;
  uint8_t  sats, hAccDm;
  bool     fixValid;
  uint32_t airtimeMsThisHour;
  uint32_t airtimeHour;
  PendingEvent events[4];
  uint8_t  nEvents;
  uint32_t txCount, rxCount;
  uint8_t  mode;               // VfBellMode chosen by the farmer
};

RTC_DATA_ATTR RtcState     st;
RTC_DATA_ATTR CollarConfig cfg;
RTC_DATA_ATTR Geofence     fence;
RTC_DATA_ATTR Geofence     staging;          // fence being received in chunks
RTC_DATA_ATTR uint32_t     stagingMask;

// ------------------------------------------------------- runtime (RAM) ------
static uint64_t epochMsRef = 0;
static uint32_t millisRef = 0;
static bool     bellOn = false;
static uint32_t cueStartMs = 0, pauseStartMs = 0;
static uint32_t soundUntilMs = 0;               // "ring now" from the website, 0 = off
static int8_t   linkOk = -1;                   // last report answered by the gateway? -1 = not sent yet
static volatile bool radioIrq = false;
static void IRAM_ATTR onRadioIrq() { radioIrq = true; }

// =============================================================================
//  Time, waiting, persistence
// =============================================================================
static uint64_t nowEpochMs() { return st.haveTime ? epochMsRef + (uint32_t)(millis() - millisRef) : 0; }
static void setEpoch(uint64_t ms) { epochMsRef = ms; millisRef = millis(); st.haveTime = true; }

// Satellites HEARD (with a signal), from the $xxGSV sentences - shown on the screen while
// searching (gps.satellites = satellites USED for a fix, which stays 0 until the fix).
static uint8_t gsvHeard[4], gsvBest[4];          // per system: GPS, BeiDou, GLONASS, Galileo
static uint8_t gsvTmpHeard[4], gsvTmpBest[4];
static uint32_t gsvSeenMs[4];
static void parseGsv(char *line) {
  int sys = !strncmp(line + 1, "GPGSV", 5) ? 0 : (!strncmp(line + 1, "BDGSV", 5) || !strncmp(line + 1, "GBGSV", 5)) ? 1
          : !strncmp(line + 1, "GLGSV", 5) ? 2 : !strncmp(line + 1, "GAGSV", 5) ? 3 : -1;
  if (sys < 0) return;
  char *f[24]; int n = 0; f[n++] = line;
  for (char *q = line; *q && n < 24; q++) if (*q == ',' || *q == '*') { *q = 0; f[n++] = q + 1; }
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

static void showStatus();

static void feedGps() {
#if VF_BENCH
  benchGpsFeed(gps, DEVICE_ID);        // simulated L76K: NMEA from a walk in Parc de Milan
#else
  static char line[120];
  static uint8_t len = 0;
  while (GPSSerial.available()) {
    const char c = GPSSerial.read();
    gps.encode(c);
    if (c == '$') len = 0;
    if (c == '\r' || c == '\n') { if (len > 6) { line[len] = 0; parseGsv(line); } len = 0; }
    else if (len < sizeof(line) - 1) line[len++] = c;
  }
#endif
#if VF_DEBUG
  static uint32_t lastShow = 0;                // keep the screen live while searching / waiting
  if (millis() - lastShow > 2000) { lastShow = millis(); showStatus(); }
#endif
}

// Wait while still reading GPS. Returns early if wakeOnRadio and DIO1 fired.
static void waitMs(uint32_t ms, bool wakeOnRadio = false) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    feedGps();
    if (wakeOnRadio && radioIrq) return;
    delay(2);
  }
}

// days since 1970-01-01 for a civil date (Howard Hinnant's algorithm)
static int64_t daysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static void loadDefaultsConfig() {
  cfg = {0, DEF_REPORT_PERIOD_S, DEF_NEAR_DIST_M, DEF_BELL_ENABLED, DEF_CUE_MAX_S,
         DEF_CUE_PAUSE_S, DEF_MAX_EPISODES, DEF_ESCAPE_DIST_M};
}

static void loadFromFlash() {
  prefs.begin("vf", true);
  if (prefs.getBytesLength("cfg") == sizeof(cfg)) prefs.getBytes("cfg", &cfg, sizeof(cfg));
  else loadDefaultsConfig();
  if (prefs.getBytesLength("fence") == sizeof(fence)) prefs.getBytes("fence", &fence, sizeof(fence));
  else fence.count = 0;
  st.mode = prefs.getUChar("mode", VF_MODE_AUTO);
  prefs.end();
  LOG("[nvs] cfg v%u, fence v%u (%u pts)\n", cfg.ver, fence.version, fence.count);
}
static void saveConfig() { prefs.begin("vf", false); prefs.putBytes("cfg", &cfg, sizeof(cfg)); prefs.end(); }
static void saveMode()   { prefs.begin("vf", false); prefs.putUChar("mode", st.mode); prefs.end(); }
static void saveFence()  { prefs.begin("vf", false); prefs.putBytes("fence", &fence, sizeof(fence)); prefs.end(); }

static void queueEvent(uint8_t ev) {
  if (st.nEvents >= 4) { memmove(&st.events[0], &st.events[1], sizeof(PendingEvent) * 3); st.nEvents = 3; }
  st.events[st.nEvents++] = {ev, 0, 0};
  LOG("[evt] queued %u\n", ev);
}

// =============================================================================
//  Bell = servo on the clapper latch, powered only while it moves
// =============================================================================
static void servoMove(bool ring) {
  const uint16_t us = ring ? SERVO_US_RING : SERVO_US_LOCK;
  gpio_hold_dis((gpio_num_t)PIN_SERVO_PWR_EN);
  gpio_hold_dis((gpio_num_t)PIN_SERVO_SIG);
  ledcAttach(PIN_SERVO_SIG, 50, 14);
  ledcWrite(PIN_SERVO_SIG, (uint32_t)us * 16384 / 20000);
  pinMode(PIN_SERVO_PWR_EN, OUTPUT);
  digitalWrite(PIN_SERVO_PWR_EN, HIGH);        // 5 V on
  waitMs(SERVO_MOVE_MS);
  digitalWrite(PIN_SERVO_PWR_EN, LOW);         // 5 V off
  ledcDetach(PIN_SERVO_SIG);
  pinMode(PIN_SERVO_SIG, OUTPUT);
  digitalWrite(PIN_SERVO_SIG, LOW);
  st.servoRing = ring;
}

static bool soundActive() { return soundUntilMs && (int32_t)(soundUntilMs - millis()) > 0; }

static void setBell(bool on) {
  if (soundActive()) on = true;                                  // farmer pressed "Ring now"
  else if (on && (!cfg.bellEnabled || st.mode == VF_MODE_SILENT)) on = false;
  if (on == bellOn) return;
  bellOn = on;
#if VF_BENCH
  digitalWrite(PIN_LED, on ? HIGH : LOW);   // bench: on-board LED = bell ringing
#endif
  servoMove(on);
  LOG("[bell] %s\n", on ? "RING" : "lock");
}

// Herd switch (CONFIG), per-collar mode and "ring now" can change at any
// downlink; this applies them once per loop (not inside the receive window).
static void applyBellPolicy() {
  if (soundUntilMs && !soundActive()) {
    soundUntilMs = 0;
    LOG("[bell] ring-now finished\n");
    setBell(false);
  } else if (soundActive()) {
    setBell(true);
  } else if (bellOn && (!cfg.bellEnabled || st.mode == VF_MODE_SILENT)) {
    setBell(false);
  }
}

// =============================================================================
//  GNSS (L76K, NMEA 9600 baud)
// =============================================================================
static void l76kCommand(const char *body) {
#if VF_BENCH
  return;                              // no GNSS module on the bench
#endif
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= *p;
  GPSSerial.printf("$%s*%02X\r\n", body, cs);
}

static void gnssPowerOn() {
#if VF_BENCH
  return;                              // GNSS is simulated (see bench_esp32.h)
#endif
  gpio_hold_dis((gpio_num_t)PIN_GNSS_EN);
  gpio_hold_dis((gpio_num_t)PIN_GNSS_WAKE);
  gpio_hold_dis((gpio_num_t)PIN_GNSS_RST);
  pinMode(PIN_GNSS_RST, OUTPUT);  digitalWrite(PIN_GNSS_RST, HIGH);
  pinMode(PIN_GNSS_WAKE, OUTPUT); digitalWrite(PIN_GNSS_WAKE, HIGH);
  pinMode(PIN_GNSS_EN, OUTPUT);   digitalWrite(PIN_GNSS_EN, LOW);     // LOW = on
  GPSSerial.begin(GNSS_BAUD, SERIAL_8N1, PIN_GNSS_RX, PIN_GNSS_TX);
}

static bool getFix(uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    feedGps();
    if (gps.date.isValid() && gps.time.isValid() && gps.date.year() >= 2024 && gps.time.isUpdated()) {
      const int64_t days = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
      const uint64_t sec = (uint64_t)days * 86400ULL + gps.time.hour() * 3600UL +
                           gps.time.minute() * 60UL + gps.time.second();
      setEpoch(sec * 1000ULL + gps.time.centisecond() * 10UL + gps.time.age());
    }
    if (gps.location.isUpdated() && gps.location.isValid() && gps.location.age() < 1500 &&
        gps.hdop.isValid() && gps.hdop.hdop() <= GNSS_MAX_HDOP && gps.satellites.value() >= 4) {
      st.lat = lround(gps.location.lat() * 1e7);
      st.lon = lround(gps.location.lng() * 1e7);
      st.sats = gps.satellites.value();
      const float accM = gps.hdop.hdop() * 2.5f;           // rough horizontal accuracy
      st.hAccDm = accM * 10 > 254 ? 255 : (uint8_t)(accM * 10);
      st.fixValid = true;
      LOG("[gnss] fix %.6f %.6f  sats %u  hdop %.1f  (%lu ms)\n",
          st.lat * 1e-7, st.lon * 1e-7, st.sats, gps.hdop.hdop(), millis() - t0);
      return true;
    }
    delay(5);
  }
  st.fixValid = false;
  LOG("[gnss] no fix within %lu ms (sats %u)\n", timeoutMs, (unsigned)gps.satellites.value());
  return false;
}

// Square test fence around the first fix (only if no real fence is stored).
static void maybeCreateTestFence() {
#if TEST_FENCE_ENABLED
  if (fence.valid() || !st.fixValid) return;
  const double dLat = TEST_FENCE_HALF_M / 111320.0;
  const double dLon = TEST_FENCE_HALF_M / (111320.0 * cos(st.lat * 1e-7 * M_PI / 180.0));
  const int32_t a = lround(dLat * 1e7), o = lround(dLon * 1e7);
  fence.version = 0;
  fence.count = 4;
  fence.pts[0] = {st.lat - a, st.lon - o};
  fence.pts[1] = {st.lat - a, st.lon + o};
  fence.pts[2] = {st.lat + a, st.lon + o};
  fence.pts[3] = {st.lat + a, st.lon - o};
  LOG("[fence] TEST fence created: square +/-%d m around %.6f, %.6f\n", TEST_FENCE_HALF_M, st.lat * 1e-7, st.lon * 1e-7);
#endif
}

// =============================================================================
//  Radio
// =============================================================================
static bool airtimeAllowed(size_t len, bool urgent) {
  const uint32_t hour = (uint32_t)(nowEpochMs() / 3600000ULL);
  if (hour != st.airtimeHour) { st.airtimeHour = hour; st.airtimeMsThisHour = 0; }
  const uint32_t toa = vfTimeOnAirMs(len);
  if (!urgent && st.airtimeMsThisHour + toa > 3600UL * DUTY_LIMIT_PERMILLE) return false;
  st.airtimeMsThisHour += toa;
  return true;
}

static void handleDownlink(const uint8_t *buf, size_t len);

// Transmit, then listen for the gateway's replies. True if 'ackSeq' was acked.
static bool sendAndListen(const uint8_t *buf, size_t len, int ackSeq = -1) {
  if (vfTransmit(buf, len) != RADIOLIB_ERR_NONE) { LOG("[lora] tx fail\n"); return false; }
  st.txCount++;
  const uint32_t rxBefore = st.rxCount;
  bool acked = false;
  for (int rx = 0; rx < 10; rx++) {           // ACK, CONFIG and up to 4 FENCE chunks
    radioIrq = false;
    vfStartReceive();
    waitMs(VF_RX_WINDOW_MS, true);
    if (!radioIrq) break;                      // nothing came
    radioIrq = false;
    uint8_t in[256];
    const size_t n = radio.getPacketLength();
    if (n == 0 || n > sizeof(in) || radio.readData(in, n) != RADIOLIB_ERR_NONE) break;
    st.rxCount++;
    if (n >= sizeof(VfAck) && in[0] == VF_MSG_ACK && ackSeq >= 0 && ((VfAck *)in)->ackSeq == (uint8_t)ackSeq) acked = true;
    handleDownlink(in, n);
  }
  radio.standby();
  linkOk = st.rxCount != rxBefore;
  return acked;
}

static void handleDownlink(const uint8_t *buf, size_t len) {
  if (len < sizeof(VfHeader)) return;
  const VfHeader *h = (const VfHeader *)buf;
  if (h->netId != NET_ID) return;
  if (h->deviceId != DEVICE_ID && h->deviceId != VF_BROADCAST_ID) return;

  switch (h->type) {
    case VF_MSG_ACK: {
      const VfAck *a = (const VfAck *)buf;
      for (uint8_t i = 0; i < st.nEvents; i++)
        if (st.events[i].seq == a->ackSeq && st.events[i].tries > 0) {
          LOG("[evt] %u acked\n", st.events[i].event);
          memmove(&st.events[i], &st.events[i + 1], sizeof(PendingEvent) * (st.nEvents - i - 1));
          st.nEvents--;
          break;
        }
      break;
    }
    case VF_MSG_CONFIG: {
      if (len < sizeof(VfConfig)) return;
      const VfConfig *c = (const VfConfig *)buf;
      if (c->cfgVer == cfg.ver) return;
      cfg.ver = c->cfgVer;
      const uint16_t period = c->reportPeriodS;
      cfg.reportPeriodS = period < 5 ? 5 : period;
      cfg.nearDistM = c->nearDistM;  cfg.bellEnabled = c->bellEnabled;
      cfg.cueMaxS = c->cueMaxS;      cfg.cuePauseS = c->cuePauseS;
      cfg.maxEpisodes = c->maxEpisodes; cfg.escapeDistM = c->escapeDistM;
      saveConfig();
      st.nextSlotMs = 0;
      LOG("[cfg] v%u period %u s, bell %u\n", cfg.ver, cfg.reportPeriodS, cfg.bellEnabled);
      break;
    }
    case VF_MSG_MODE: {
      if (len < sizeof(VfMode)) return;
      const VfMode *m = (const VfMode *)buf;
      if (m->mode <= VF_MODE_SILENT && m->mode != st.mode) { st.mode = m->mode; saveMode(); }
      const uint16_t s = m->ringS > SOUND_MAX_S ? SOUND_MAX_S : m->ringS;
      soundUntilMs = (s && st.mode != VF_MODE_SILENT) ? (millis() + s * 1000UL) | 1 : 0;
      LOG("[bell] mode %s%s\n", st.mode == VF_MODE_SILENT ? "SILENT" : "AUTO", soundUntilMs ? ", ring now" : "");
      break;
    }
    case VF_MSG_FENCE: {
      const VfFenceChunk *c = (const VfFenceChunk *)buf;
      if (len < vfFenceChunkLen(0) || c->count > VF_FENCE_CHUNK_PTS || len < vfFenceChunkLen(c->count) ||
          c->totalPts < 3 || c->totalPts > VF_MAX_FENCE_POINTS || c->firstIdx + c->count > c->totalPts) return;
      if (staging.version != c->fenceVer || staging.count != c->totalPts) {
        staging.version = c->fenceVer; staging.count = c->totalPts; stagingMask = 0;
      }
      for (uint8_t i = 0; i < c->count; i++) {
        staging.pts[c->firstIdx + i] = c->pts[i];
        stagingMask |= (1UL << (c->firstIdx + i));
      }
      const uint32_t full = (c->totalPts >= 32) ? 0xFFFFFFFFUL : ((1UL << c->totalPts) - 1);
      if (stagingMask == full) {
        fence = staging;
        saveFence();
        st.zone = VF_ZONE_UNKNOWN; st.outsideCount = 0; st.episodes = 0;
        LOG("[fence] v%u installed, %u points\n", fence.version, fence.count);
      }
      break;
    }
    default: break;
  }
}

// =============================================================================
//  Uplinks
// =============================================================================
static void buildPosition(VfPosition &p, uint8_t event) {
  memset(&p, 0, sizeof(p));
  p.h = {event ? (uint8_t)VF_MSG_EVENT : (uint8_t)VF_MSG_POSITION, NET_ID, DEVICE_ID, ++st.seq};
  const uint16_t mv = batteryMv();
  p.flags = (st.fixValid ? VF_FLAG_FIX_VALID : 0) | (bellOn ? VF_FLAG_BELL_ON : 0) |
            (mv < LOW_BATT_MV ? VF_FLAG_LOW_BATT : 0) | (fence.valid() ? 0 : VF_FLAG_NO_FENCE) |
            (st.zone << VF_FLAG_ZONE_SHIFT);
  p.lat = st.lat; p.lon = st.lon;
  p.fenceDistM = st.distM;
  p.battMv = mv;
  p.tempC = (int8_t)temperatureRead();
  p.sats = st.sats; p.hAccDm = st.hAccDm;
  p.fenceVer = fence.version; p.cfgVer = cfg.ver;
  p.event = event;
  p.mode = st.mode | (soundActive() ? VF_MODE_SOUNDING : 0);
}

static void flushEvents() {
  while (st.nEvents > 0) {
    PendingEvent &e = st.events[0];
    if (e.tries >= EVENT_RETRIES) { LOG("[evt] %u not acked, rides in next report\n", e.event); return; }
    VfPosition p;
    buildPosition(p, e.event);
    e.seq = p.h.seq;
    e.tries++;
    waitMs(random(0, 1500 * e.tries));
    if (!airtimeAllowed(sizeof(p), true)) return;
    LOG("[evt] send %u (try %u)\n", e.event, e.tries);
    sendAndListen((uint8_t *)&p, sizeof(p), p.h.seq);
    if (st.nEvents && st.events[0].seq == p.h.seq) continue;   // not acked -> retry
  }
}

static uint64_t computeNextSlot(uint64_t nowMs) {
  const uint64_t period = (uint64_t)cfg.reportPeriodS * 1000ULL;
  const uint32_t nSlots = period / VF_SLOT_MS;
  const uint64_t offset = (uint64_t)(DEVICE_ID % nSlots) * VF_SLOT_MS;
  uint64_t slot = (nowMs / period) * period + offset;
  while (slot < nowMs + 100) slot += period;
  return slot;
}

static void sendPeriodicReport() {
  if (st.haveTime) {
    const uint64_t now = nowEpochMs();
    if (st.nextSlotMs > now && st.nextSlotMs - now < 60000) waitMs((uint32_t)(st.nextSlotMs - now));
  }
  const uint8_t ev = st.nEvents ? st.events[0].event : VF_EVT_NONE;
  VfPosition p;
  buildPosition(p, ev);
  if (ev) { st.events[0].seq = p.h.seq; st.events[0].tries = 1; }
  if (airtimeAllowed(sizeof(p), false)) {
    LOG("[lora] report seq %u zone %u dist %d m\n", p.h.seq, st.zone, st.distM);
    sendAndListen((uint8_t *)&p, sizeof(p), ev ? p.h.seq : -1);
  }
  st.nextSlotMs = st.haveTime ? computeNextSlot(nowEpochMs() + 1000) : 0;
}

// =============================================================================
//  Fence state machine + bell cue
// =============================================================================
static void updateZone() {
  if (!fence.valid() || !st.fixValid) {
    st.distM = INT16_MAX;
    if (!fence.valid()) st.zone = VF_ZONE_UNKNOWN;
    setBell(false);
    return;
  }
  const double d = geo::signedDistance(fence, st.lat, st.lon);
  const long dl = lround(d);
  st.distM = (int16_t)(dl < -32000 ? -32000 : (dl > 32000 ? 32000 : dl));
  const uint8_t prev = st.zone;

  if (prev == VF_ZONE_ESCAPED) {                         // silent until clearly back inside
    if (d >= ZONE_HYSTERESIS_M) {
      st.zone = (d < cfg.nearDistM) ? VF_ZONE_NEAR : VF_ZONE_INSIDE;
      st.episodes = 0; st.outsideCount = 0;
      queueEvent(VF_EVT_RETURN);
    }
    return;
  }
  if (d < 0) {                                           // outside
    if (prev != VF_ZONE_OUTSIDE) {
      if (++st.outsideCount < OUTSIDE_CONFIRM_FIXES) return;
      st.zone = VF_ZONE_OUTSIDE; st.episodes = 0;
      queueEvent(VF_EVT_EXIT);
      setBell(true); cueStartMs = millis();
      return;
    }
    const bool escapedFar = (-d) > cfg.escapeDistM;
    if (bellOn && (millis() - cueStartMs) > cfg.cueMaxS * 1000UL) {
      setBell(false); pauseStartMs = millis(); st.episodes++;
    } else if (!bellOn && (millis() - pauseStartMs) > cfg.cuePauseS * 1000UL) {
      setBell(true); cueStartMs = millis();
    }
    if (st.episodes >= cfg.maxEpisodes || escapedFar) {  // welfare limit
      setBell(false);
      st.zone = VF_ZONE_ESCAPED;
      queueEvent(VF_EVT_ESCAPED);
    }
    return;
  }
  st.outsideCount = 0;                                   // inside
  if (prev == VF_ZONE_OUTSIDE) {
    if (d < ZONE_HYSTERESIS_M) return;
    setBell(false);
    queueEvent(VF_EVT_RETURN);
  }
  st.zone = (d < cfg.nearDistM) ? VF_ZONE_NEAR : VF_ZONE_INSIDE;
  st.episodes = 0;
}

// =============================================================================
//  Display (bench mode only)
// =============================================================================
// Screen (same layout as the gateway):
//   C1  LoRa ok  rx 4       t=178s
//   INSIDE  12 m  bell off
//   FIX 9 used  +-8 m
//   46.53902 N  6.62114 E
static void showStatus() {
#if VF_DEBUG
  static const char *zn[] = {"no fence", "INSIDE", "NEAR", "OUTSIDE", "ESCAPED"};
  const uint32_t up = millis() / 1000;
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, String("C") + DEVICE_ID + "  " +
                           (linkOk < 0 ? "LoRa ..." : linkOk ? "LoRa ok" : "LoRa --") + "  rx " + st.rxCount);
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(128, 0, up < 10000 ? "t=" + String(up) + "s" : "t=" + String(up / 60) + "m");
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 14, String(zn[st.zone < 5 ? st.zone : 0]) +
                            (st.distM == INT16_MAX ? String("") : "  " + String(st.distM) + " m") +
                            (bellOn ? "  RINGING" : st.mode == VF_MODE_SILENT ? "  bell off" : "  bell auto"));
  if (st.fixValid) {
    const double lat = st.lat * 1e-7, lon = st.lon * 1e-7;
    display.drawString(0, 28, "FIX " + String(st.sats) + " used  +-" + String(st.hAccDm / 10.0f, 0) + " m");
    display.drawString(0, 42, String(fabs(lat), 5) + (lat >= 0 ? " N  " : " S  ") + String(fabs(lon), 5) + (lon >= 0 ? " E" : " W"));
  } else {
    int best;
    const int heard = gnssHeard(best);
    display.drawString(0, 28, "GPS searching... heard " + String(heard));
    display.drawString(0, 42, gps.charsProcessed() < 10 ? String("no GNSS data - check module")
                              : heard == 0 ? String("no satellites - open sky / antenna")
                              : "best signal " + String(best) + " dB" + (heard < 4 ? "  need 4+" : "  wait..."));
  }
  display.display();
#endif
}

// =============================================================================
//  Deep sleep (field mode). In bench mode we just wait, so USB stays alive.
// =============================================================================
static void sleepUntilNextSlot() {
  uint64_t sleepMs = 60000;
  if (st.haveTime && st.nextSlotMs) {
    const uint64_t now = nowEpochMs();
    const uint64_t wake = st.nextSlotMs - GNSS_LEAD_S * 1000ULL;
    sleepMs = (wake > now) ? (wake - now) : 1000;
  }
#if VF_DEBUG
  LOG("[sleep] bench mode: waiting %llu ms (no deep sleep)\n", sleepMs);
  showStatus();
  waitMs((uint32_t)sleepMs);
#else
  st.epochMsAtSleep = nowEpochMs();
  st.sleepMs = sleepMs;
  radio.sleep();
  femSleep();
  // freeze outputs: servo power off, signal low, GNSS on (or off)
  digitalWrite(PIN_SERVO_PWR_EN, LOW);
  digitalWrite(PIN_SERVO_SIG, LOW);
  gpio_hold_en((gpio_num_t)PIN_SERVO_PWR_EN);
  gpio_hold_en((gpio_num_t)PIN_SERVO_SIG);
  digitalWrite(PIN_GNSS_EN, GNSS_OFF_IN_SLEEP ? HIGH : LOW);
  gpio_hold_en((gpio_num_t)PIN_GNSS_EN);
  gpio_hold_en((gpio_num_t)PIN_GNSS_WAKE);
  gpio_hold_en((gpio_num_t)PIN_GNSS_RST);
  digitalWrite(PIN_LED, LOW);
  vextOff();
  gpio_hold_en((gpio_num_t)PIN_VEXT);
  gpio_deep_sleep_hold_en();
  esp_sleep_enable_timer_wakeup(sleepMs * 1000ULL);
  esp_deep_sleep_start();
#endif
}

// =============================================================================
void setup() {
  const bool wokeFromSleep = esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER;
  gpio_hold_dis((gpio_num_t)PIN_SERVO_PWR_EN);
  gpio_hold_dis((gpio_num_t)PIN_SERVO_SIG);
  gpio_hold_dis((gpio_num_t)PIN_VEXT);
  pinMode(PIN_SERVO_PWR_EN, OUTPUT); digitalWrite(PIN_SERVO_PWR_EN, LOW);
  pinMode(PIN_SERVO_SIG, OUTPUT);    digitalWrite(PIN_SERVO_SIG, LOW);
  pinMode(PIN_LED, OUTPUT);
  setCpuFrequencyMhz(80);

#if VF_DEBUG
  Serial.begin(115200);
  delay(1500);                                  // no while(!Serial): never block without USB
  vextOn();
  pinMode(PIN_OLED_RST, OUTPUT);
  digitalWrite(PIN_OLED_RST, LOW); delay(20); digitalWrite(PIN_OLED_RST, HIGH); delay(20);
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
#else
  vextOff();
#endif

  const uint64_t mac = ESP.getEfuseMac();
  DEVICE_ID = DEVICE_ID_FIXED ? DEVICE_ID_FIXED : (uint16_t)(((mac >> 32) & 0xFFFF) % 299) + 1;

  const bool coldBoot = (st.magic != STATE_MAGIC) || !wokeFromSleep;
  if (coldBoot) {
    memset(&st, 0, sizeof(st));
    st.magic = STATE_MAGIC;
    staging.version = 0; staging.count = 0; stagingMask = 0;
    loadFromFlash();
    queueEvent(VF_EVT_BOOT);
    LOG("\n=== collar %u boot (net 0x%02X) ===\n", DEVICE_ID, NET_ID);
  } else if (st.epochMsAtSleep) {
    setEpoch(st.epochMsAtSleep + st.sleepMs);
  }
  randomSeed(esp_random());

  gnssPowerOn();
  if (coldBoot) { delay(300); l76kCommand("PCAS04,7"); }   // GPS + BeiDou + GLONASS
  const int16_t rs = vfRadioBegin(onRadioIrq);
  LOG("[lora] %s (%d), front end %s\n", rs == RADIOLIB_ERR_NONE ? "ok" : "FAILED", rs, femName());
  if (coldBoot) servoMove(false);               // known state: latch locked
  bellOn = false;
}

void loop() {
  const bool fast = (st.zone == VF_ZONE_NEAR || st.zone == VF_ZONE_OUTSIDE);
  const bool quick = cfg.reportPeriodS < 60;                  // short report period (test)
  const bool cold = !st.haveTime || (!st.fixValid && !fast && !quick);
  getFix(cold ? GNSS_COLD_TIMEOUT_MS : (fast || quick) ? 3000 : GNSS_FIX_TIMEOUT_MS);

  maybeCreateTestFence();
  updateZone();
  if (!st.lowBattSent && batteryMv() < LOW_BATT_MV) { st.lowBattSent = true; queueEvent(VF_EVT_LOW_BATT); }

  if (st.haveTime && !st.nextSlotMs) st.nextSlotMs = computeNextSlot(nowEpochMs());
  flushEvents();
  if (!st.haveTime || (st.nextSlotMs && nowEpochMs() + 1500 >= st.nextSlotMs)) sendPeriodicReport();
  applyBellPolicy();
  showStatus();

  if (st.zone == VF_ZONE_OUTSIDE || soundActive()) waitMs(OUT_FIX_INTERVAL_MS);
  else if (st.zone == VF_ZONE_NEAR) waitMs(NEAR_FIX_INTERVAL_MS);
  else                              sleepUntilNextSlot();     // INSIDE, ESCAPED, UNKNOWN
}
