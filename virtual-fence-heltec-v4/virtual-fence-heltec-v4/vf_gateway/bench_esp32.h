// =============================================================================
//  BENCH MODE (VF_BENCH 1 in config.h) – plain ESP32 dev board, no Heltec V4.
//
//  Used instead of the Heltec part of heltec_v4.h. Same names, so the collar
//  and gateway code stays the same:
//   - pins moved to safe GPIOs of a plain ESP32 (GPIO 6-11 are its flash!)
//   - RF front end and Vext: nothing to switch
//   - battery: fixed value (no divider on a dev board)
//   - OLED: empty stand-in (the Serial log shows the same information)
//   - GNSS: simulated L76K. NMEA sentences are fed into TinyGPSPlus, so the
//     collar's getFix() runs unchanged. The cow walks a route in Parc de
//     Milan (Lausanne) and leaves the virtual fence on purpose.
//  The radio part (ESP-NOW instead of LoRa) is in vf_radio.h.
// =============================================================================
#pragma once
#include <Arduino.h>
#include <math.h>
#include "driver/gpio.h"

// ------------------------------------------------------------ pins ----------
// Only pins that are safe on an ESP32-WROOM DevKit. Nothing has to be wired.
#define PIN_LED           2      // on-board LED of most DevKits = BELL indicator
#define PIN_BUTTON        0      // BOOT button
#define PIN_SERVO_SIG     4      // servo signal wire on D4 (servo powered from VIN + GND)
#define PIN_SERVO_PWR_EN  26     // not wired on the bench (servo always on VIN)
#define PIN_VEXT          27     // not used on the bench
#define PIN_OLED_SDA      21
#define PIN_OLED_SCL      22
#define PIN_OLED_RST      14
#define OLED_ADDR         0x3C
#define PIN_GNSS_TX       17     // GNSS is simulated – these are never driven
#define PIN_GNSS_RX       16     //   (except held LOW/HIGH before deep sleep)
#define PIN_GNSS_EN       32
#define PIN_GNSS_WAKE     33
#define PIN_GNSS_RST      13
#define PIN_GNSS_PPS      35
#define GNSS_BAUD         9600

// ------------------------------------------------------------ RF front end --
enum FemType : uint8_t { FEM_NONE = 0, FEM_GC1109, FEM_KCT8103L };
static FemType g_fem = FEM_NONE;
static inline FemType femInit()        { return FEM_NONE; }
static inline const char *femName()    { return "none (ESP-NOW bench)"; }
static inline void femTx()             {}
static inline void femRx()             {}
static inline void femSleep()          {}
static inline void femReleaseHolds()   {}

// ------------------------------------------------------------ power ---------
static inline uint16_t batteryMv() { return 3950; }      // ~76 %, never "low"
static inline uint8_t batteryPercent(uint16_t mv) {
  if (mv >= 4150) return 100;
  if (mv <= 3300) return 0;
  return (uint8_t)((mv - 3300) * 100UL / (4150 - 3300));
}
static inline void vextOn()  {}
static inline void vextOff() {}

// ------------------------------------------------------------ OLED ----------
// Stand-in with the same methods as ThingPulse SSD1306Wire (draws nothing).
static const uint8_t ArialMT_Plain_10[1] = {0};
enum OLEDDISPLAY_TEXT_ALIGNMENT { TEXT_ALIGN_LEFT, TEXT_ALIGN_RIGHT, TEXT_ALIGN_CENTER, TEXT_ALIGN_CENTER_BOTH };
class SSD1306Wire {
 public:
  SSD1306Wire(uint8_t, int, int) {}
  bool init() { return true; }
  void flipScreenVertically() {}
  void setFont(const uint8_t *) {}
  void setTextAlignment(OLEDDISPLAY_TEXT_ALIGNMENT) {}
  void clear() {}
  void drawString(int16_t, int16_t, const String &) {}
  void display() {}
};

// ------------------------------------------------------------ test pasture --
// Parc de Milan, Lausanne – the same fence as the ReBell website's default.
// The gateway sends it to the collars if no fence came from the website yet.
static const double BENCH_FENCE[][2] = {
  {46.51412, 6.62184}, {46.51439, 6.62142}, {46.51477, 6.62137}, {46.51503, 6.62179},
  {46.51505, 6.62231}, {46.51483, 6.62273}, {46.51438, 6.62275}, {46.51413, 6.62235}};
#define BENCH_FENCE_N (sizeof(BENCH_FENCE) / sizeof(BENCH_FENCE[0]))
#define BENCH_CENTER_LAT 46.51459
#define BENCH_CENTER_LON 6.62207

// ------------------------------------------------------------ simulated GNSS -
// Waypoints in metres from the pasture centre (x = east, y = north) and how
// long the cow stands there. The fence edge is ~50 m from the centre.
struct BenchWaypoint { float x, y; uint16_t dwellS; };

// Route 1: grazes, walks 12 m out of the north side for ~1 min, comes back.
//          -> EXIT event, bell cue on/off, RETURN event.
static const BenchWaypoint BENCH_ROUTE_1[] = {
  {0, 0, 20}, {-15, 10, 20}, {10, 20, 15}, {5, 35, 5}, {5, 62, 40},
  {5, 30, 20}, {-20, -10, 30}, {15, -25, 20}};
// Route 2: walks 19 m out of the east side and stays ~3 min.
//          -> EXIT, 5 bell cues, ESCAPED (bell stops), later RETURN.
static const BenchWaypoint BENCH_ROUTE_2[] = {
  {0, -10, 20}, {20, 0, 20}, {40, 5, 5}, {70, 5, 200}, {30, 0, 20},
  {0, -20, 30}, {-25, 0, 20}};

#ifndef BENCH_ROUTE
#define BENCH_ROUTE 0
#endif
#define BENCH_WALK_MPS 1.0f      // walking speed of the simulated cow

// Position on the route at time t (s). Loops forever.
static void benchRoutePos(uint8_t route, float t, float &x, float &y) {
  const BenchWaypoint *w = route == 1 ? BENCH_ROUTE_1 : BENCH_ROUTE_2;
  const size_t n = route == 1 ? sizeof(BENCH_ROUTE_1) / sizeof(BenchWaypoint)
                              : sizeof(BENCH_ROUTE_2) / sizeof(BenchWaypoint);
  float total = 0;
  for (size_t i = 0; i < n; i++) {
    const BenchWaypoint &a = w[i], &b = w[(i + 1) % n];
    total += hypotf(b.x - a.x, b.y - a.y) / BENCH_WALK_MPS + b.dwellS;
  }
  t = fmodf(t, total);
  for (size_t i = 0; i < n; i++) {
    const BenchWaypoint &a = w[i], &b = w[(i + 1) % n];
    const float walk = hypotf(b.x - a.x, b.y - a.y) / BENCH_WALK_MPS;
    if (t < walk) { const float f = walk > 0 ? t / walk : 1; x = a.x + f * (b.x - a.x); y = a.y + f * (b.y - a.y); return; }
    t -= walk;
    if (t < b.dwellS) { x = b.x; y = b.y; return; }
    t -= b.dwellS;
  }
  x = w[0].x; y = w[0].y;
}

// Fake UTC clock: starts at the moment the sketch was compiled.
static uint32_t benchCompileEpoch() {
  static const char *mon = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *d = __DATE__, *t = __TIME__;            // "Nov 22 2026", "10:55:00"
  int m = (strstr(mon, String(d).substring(0, 3).c_str()) - mon) / 3 + 1;
  int day = atoi(d + 4), y = atoi(d + 7);
  y -= m <= 2;                                        // days from civil (H. Hinnant)
  const int era = y / 400;
  const unsigned yoe = y - era * 400;
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const long days = era * 146097L + doe - 719468L;
  return days * 86400UL + atoi(t) * 3600UL + atoi(t + 3) * 60UL + atoi(t + 6);
}

template <class GPS>
static void benchNmea(GPS &gps, const char *body) {
  uint8_t cs = 0;
  for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
  char line[128];
  snprintf(line, sizeof(line), "$%s*%02X\r\n", body, cs);
  for (const char *p = line; *p; p++) gps.encode(*p);
}

// Call often (instead of reading the GNSS UART). Emits one GGA + RMC per second.
template <class GPS>
static void benchGpsFeed(GPS &gps, uint16_t deviceId) {
  static uint32_t lastS = UINT32_MAX;
  static uint32_t epoch0 = 0;
  const uint32_t s = millis() / 1000;
  if (s == lastS) return;
  lastS = s;
  if (!epoch0) epoch0 = benchCompileEpoch();

  const uint8_t route = BENCH_ROUTE ? BENCH_ROUTE : (deviceId % 2 ? 1 : 2);
  float x, y;
  benchRoutePos(route, (float)s, x, y);
  x += (float)random(-80, 81) / 100.0f;                // ~1 m GNSS noise
  y += (float)random(-80, 81) / 100.0f;
  const double lat = BENCH_CENTER_LAT + y / 111320.0;
  const double lon = BENCH_CENTER_LON + x / (111320.0 * cos(BENCH_CENTER_LAT * M_PI / 180.0));

  // UTC date / time
  const uint32_t e = epoch0 + s;
  const long z = e / 86400 + 719468;
  const long era = z / 146097;
  const unsigned doe = z - era * 146097;
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  const unsigned day = doy - (153 * mp + 2) / 5 + 1;
  const unsigned mon = mp < 10 ? mp + 3 : mp - 9;
  const unsigned yr = yoe + era * 400 + (mon <= 2);
  const unsigned hh = (e % 86400) / 3600, mm = (e % 3600) / 60, ss = e % 60;

  char la[16], lo[16], body[110];
  const int latD = (int)lat, lonD = (int)lon;
  snprintf(la, sizeof(la), "%02d%08.5f", latD, (lat - latD) * 60.0);
  snprintf(lo, sizeof(lo), "%03d%08.5f", lonD, (lon - lonD) * 60.0);
  snprintf(body, sizeof(body), "GPGGA,%02u%02u%02u.00,%s,N,%s,E,1,09,0.9,480.0,M,48.0,M,,", hh, mm, ss, la, lo);
  benchNmea(gps, body);
  snprintf(body, sizeof(body), "GPRMC,%02u%02u%02u.00,A,%s,N,%s,E,0.8,45.0,%02u%02u%02u,,,A",
           hh, mm, ss, la, lo, day, mon, yr % 100);
  benchNmea(gps, body);
}
