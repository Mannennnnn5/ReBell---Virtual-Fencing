// =============================================================================
//  Collar (cow locator) – settings
//  Hardware: Heltec WiFi LoRa 32 V4 + L76K GNSS module + TPS63020 5 V step-up
//            + MG90S/MF90 servo on the bell latch
// =============================================================================
#pragma once

// ------------------------------------------------------------- bench --------
// 1 = BENCH MODE: plain ESP32 dev board, ESP-NOW instead of LoRa, simulated
//     GPS (the cow walks a route in Parc de Milan), on-board LED = bell.
// 0 = real collar: Heltec V4 + LoRa + L76K + servo.
#define VF_BENCH          0
// Radio: 1 = ESP-NOW (to the ESP32 test gateway), 0 = LoRa.
// Heltec V4 with its REAL GPS as a collar in the 3-board ESP-NOW test:
//   VF_BENCH 0  and  VF_RADIO_ESPNOW 1
#define VF_RADIO_ESPNOW   0
// Bench route: 0 = automatic (odd id -> 1, even id -> 2)
//              1 = short trip out of the fence and back (bell cue, RETURN)
//              2 = stays outside ~3 min (5 bell cues, ESCAPED, later RETURN)
// Set 1 on one board and 2 on the other to see both.
#define BENCH_ROUTE       0

// ------------------------------------------------------------ identity ------
// 0 = automatic (derived from the board's MAC address, printed at boot).
// Or set 1..299 by hand, different on every collar.
#define DEVICE_ID_FIXED   1        // <-- collar 1 = 1, collar 2 = 2
#define NET_ID            0x2A     // must match the gateway

// ------------------------------------------------------------- debug --------
// 1 = bench mode: Serial log, never deep sleep (USB stays connected),
//     OLED shows zone / distance / satellites.
// 0 = field mode: lowest power, deep sleep between reports, OLED off.
#define VF_DEBUG          1

// ------------------------------------------------------------- servo --------
// Pulse widths found with test sketch T4. LOCK = bell silent, RING = bell free.
#if VF_BENCH
// Bench: MF90 on D4 turns a full 180 degrees (0 deg = inside / silent, 180 deg = outside / bell)
#define SERVO_US_LOCK     500
#define SERVO_US_RING     2500
#define SERVO_MOVE_MS     800     // 180 degrees takes ~0.4-0.5 s
#else
#define SERVO_US_LOCK     1000
#define SERVO_US_RING     2000
#define SERVO_MOVE_MS     600
#endif

// ------------------------------------------------------- test fence ---------
// For garden / field tests without the cloud: if no fence has been received,
// build a square fence of +/- TEST_FENCE_HALF_M metres around the FIRST good
// GPS fix after power-on. Walk away from the start point to cross it.
// Set to 0 once fences come from the gateway.
#define TEST_FENCE_ENABLED   (!VF_RADIO_ESPNOW)   // ESP-NOW test: the fence comes from the gateway / website
#define TEST_FENCE_HALF_M    25

// ------------------------------------------------------ default behaviour ----
// All of these can be changed over the air with a CONFIG downlink.
// TEST SETTING: a report every 5 s (field use: VF_DEFAULT_PERIOD_S = 120 s, which saves
// battery and airtime). The gateway's COLLAR_REPORT_PERIOD_S must be the same.
#define DEF_REPORT_PERIOD_S   5
#define DEF_NEAR_DIST_M       15     // closer than this -> fast GNSS mode (prototype: small fence)
#define DEF_BELL_ENABLED      1
#define DEF_CUE_MAX_S         20     // bell on at most 20 s per episode
#define DEF_CUE_PAUSE_S       10     // then 10 s silence
#define DEF_MAX_EPISODES      5      // 5 failed cues -> ESCAPED, bell stops
#define DEF_ESCAPE_DIST_M     100    // or 100 m outside -> ESCAPED
#define SOUND_MAX_S           60     // "Ring now" from the website rings at most this long

// --------------------------------------------------------- GNSS -------------
// Keep the L76K powered during deep sleep -> fix in 1-5 s after wake
// (~27 mA continuous). Set to 1 to cut its power between reports (saves power,
// but every fix is a cold start of 30-60 s).
#define GNSS_OFF_IN_SLEEP     0
#define GNSS_FIX_TIMEOUT_MS   20000   // normal wake
#define GNSS_COLD_TIMEOUT_MS  180000  // first fix after power-up
#define GNSS_MAX_HDOP         3.0f    // accept fix only with HDOP <= 3
#define GNSS_LEAD_S           1       // wake this long before the TDMA slot (field use: 6)

// --------------------------------------------------------- tuning -----------
#define NEAR_FIX_INTERVAL_MS  5000    // fix interval in NEAR zone
#define OUT_FIX_INTERVAL_MS   5000    // fix interval while outside (every 5 s)
#define ZONE_HYSTERESIS_M     3       // must be 3 m back inside to count as returned
#define OUTSIDE_CONFIRM_FIXES 2       // consecutive outside fixes before the bell
#define LOW_BATT_MV           3450    // ~10 % left
#define EVENT_RETRIES         3
#define DUTY_LIMIT_PERMILLE   60      // self-limit to 6 % airtime per hour (legal: 10 %)
                                      // a report every 5 s uses ~4 % (206 ms / 5 s)
