// =============================================================================
//  Gateway – settings  (Heltec WiFi LoRa 32 V4, internet via phone hotspot)
// =============================================================================
#pragma once

// ------------------------------------------------------------- bench --------
// 1 = BENCH MODE: plain ESP32 dev board, ESP-NOW instead of LoRa (the collars
//     must be in bench mode too). 0 = real gateway: Heltec V4 + LoRa.
#define VF_BENCH        0
// Radio to the collars: 1 = ESP-NOW (test collars on plain ESP32s), 0 = LoRa.
// Heltec V4 as gateway for the ESP-NOW test collars: VF_BENCH 0, VF_RADIO_ESPNOW 1.
#define VF_RADIO_ESPNOW 0
// Gateway reads its own L76K GNSS (Heltec only) and reports that position
#define GATEWAY_GNSS    (!VF_BENCH)
#define VF_ROLE_GATEWAY 1               // tells vf_radio.h this is the gateway

#define NET_ID          0x2A            // must match every collar
#define GATEWAY_ID      "gw01"
#define VF_DEBUG        1               // Serial log

// ------------------------------------------------------------- Wi-Fi --------
// Your phone's hotspot. 2.4 GHz only! iPhone: turn on "Maximise Compatibility".
#define WIFI_SSID       "Will S25"
#define WIFI_PASS       "Tissemand"

// ------------------------------------------------------------- MQTT ---------
// Prototype default: free public test broker (no account, but anyone who knows
// the topic can read it – fine for tests, never for real farm data).
// For a private broker (e.g. HiveMQ Cloud free tier): set MQTT_TLS 1, port 8883,
// host/user/password from its console.
#define MQTT_HOST       "broker.hivemq.com"
#define MQTT_PORT       1883
#define MQTT_TLS        0
#define MQTT_USER       ""
#define MQTT_PASS       ""
// Pick something unique so you don't collide with other people on the public broker
#define TOPIC_PREFIX    "vf-alp-test-7q3/" GATEWAY_ID
// topics: <prefix>/up | event | status | online      (gateway -> cloud)
//         <prefix>/cmd/fence, <prefix>/cmd/config     (cloud -> gateway, retained)

#define BATCH_INTERVAL_S   30          // publish position batches at most this often
#define BATCH_MAX_BYTES    1800
#define STATUS_INTERVAL_S  30          // gateway health (website marks it stale after 60 s)

// ------------------------------------------------------------- website ------
// 1 = publish in the ReBell website's format: one "telemetry" message per
//     collar report on <prefix>/up, "event"/"ack" on <prefix>/event,
//     "gateway_position" on <prefix>/status. The website "Wi-Fi gateway topic"
//     is TOPIC_PREFIX. Accepts the website's "Send fence to gateway".
// 0 = the original compact batch format ({"gw":..,"recs":[..]}).
#define CLOUD_FORMAT_REBELL 1

// ------------------------------------------------------------- bells --------
#define RING_NOW_S         60          // website "Ring now": seconds (collar caps it too)
#define GW_TIMEZONE        "CET-1CEST,M3.5.0,M10.5.0/3"   // Switzerland/Denmark, for silent hours
// Collars' report period, sent to every collar in a CONFIG.
// TEST SETTING: 5 s (field use: VF_DEFAULT_PERIOD_S = 120 s). Same as the collar's DEF_REPORT_PERIOD_S.
#define COLLAR_REPORT_PERIOD_S 5

// Fixed position of the gateway (shown on the website map). 0, 0 = unknown.
// With GATEWAY_GNSS the GPS fix is used instead (this is the fallback without fix).
#if VF_BENCH
#define GATEWAY_LAT     46.51455        // bench: inside Parc de Milan
#define GATEWAY_LON     6.62172
#else
#define GATEWAY_LAT     0.0
#define GATEWAY_LON     0.0
#endif

// LoRa downlink power at the SX1262 (+~11 dB from the V4 front end)
#define VF_RADIO_DBM       5
