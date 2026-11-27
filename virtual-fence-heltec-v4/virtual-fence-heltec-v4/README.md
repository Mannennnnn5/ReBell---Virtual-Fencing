# Alpine Virtual Fence – Heltec V4 prototype firmware

```
heltec-v4/
├── common/                 shared headers (master copies)
│   ├── heltec_v4.h         pins, RF front end, battery, Vext
│   ├── vf_radio.h          LoRa setup (RadioLib 7.x and 8.x)
│   ├── protocol.h          packet formats, radio settings, TDMA
│   └── geofence.h          point-in-polygon + signed distance
├── tests/
│   ├── T1_board_check      display, LED, button, battery, radio + front end
│   ├── T2_lora_ping        same sketch on all boards: ping + RSSI/SNR
│   ├── T3_gps              L76K GNSS: satellites, fix, time to first fix
│   └── T4_servo            TPS63020 + servo, finds LOCK/RING pulse widths
├── vf_collar/              collar firmware  (edit config.h)
├── vf_gateway/             gateway firmware (edit config.h: hotspot + MQTT)
└── sync_common.sh          copies common/ into every sketch folder

common/bench_esp32.h       BENCH MODE: plain ESP32 stand-ins (pins, fake GPS walk, no OLED)
common/vf_radio_espnow.h   BENCH MODE: ESP-NOW instead of LoRa, same functions
```

Every sketch folder already contains its own copy of the headers, so each
folder opens directly in the Arduino IDE. If you change a file in `common/`,
run `sync_common.sh` (or copy the file by hand).

## Arduino IDE settings (all sketches)

1. Boards Manager: **esp32 by Espressif Systems**, version 3.3.x
   (board package URL `https://espressif.github.io/arduino-esp32/package_esp32_index.json`)
2. Tools → Board → esp32 → **Heltec WiFi LoRa 32(V4)**
3. Tools → **USB CDC On Boot: Enabled** (the V4 has native USB – without this the Serial Monitor stays empty)
4. Tools → Flash Size: **16MB**
5. Libraries (Library Manager): **RadioLib** (7.x or 8.x), **TinyGPSPlus**,
   **PubSubClient**, **ArduinoJson** (7.x), **ESP8266 and ESP32 OLED driver for SSD1306 displays** (ThingPulse)

Compiled and checked with esp32 core 3.3.12, RadioLib 8.0.0 and 7.8.1,
TinyGPSPlus 1.1.0, PubSubClient 2.8, ArduinoJson 7.4.3, ThingPulse SSD1306 4.6.2.
Not yet run on real hardware.

Upload fails ("Failed to connect")? Hold **PRG**, press and release **RST**, release PRG, upload again.

## Sending a fence / config to the collars (MQTT)

Publish **retained** JSON to `<TOPIC_PREFIX>/cmd/fence` and `<TOPIC_PREFIX>/cmd/config`
(e.g. with MQTT Explorer or MQTTX, broker from `vf_gateway/config.h`).

```json
{"type":"fence","version":1,
 "points":[[46.35000,7.15000],[46.35000,7.15050],[46.35030,7.15050],[46.35030,7.15000]]}
```

```json
{"type":"config","version":1,"reportPeriodS":60,"nearDistM":15,
 "bellEnabled":true,"cueMaxS":20,"cuePauseS":10,"maxEpisodes":5,"escapeDistM":100}
```

Bump `version` (1–255) on every change. Collars pick it up at their next report.
Subscribe to `<TOPIC_PREFIX>/#` to see positions, events and gateway status.

## Bench mode – 3 plain ESP32 boards, no Heltec (VF_BENCH)

`VF_BENCH 1` in `vf_collar/config.h` and `vf_gateway/config.h` (the default in
this copy) runs the **same collar and gateway firmware** on plain ESP32 dev
boards, before the Heltec V4 boards arrive:

| Real system (VF_BENCH 0)        | Bench mode (VF_BENCH 1)                                   |
|---------------------------------|-----------------------------------------------------------|
| LoRa 869.525 MHz (RadioLib)     | ESP-NOW broadcast, same packets, same replies             |
| L76K GNSS                       | simulated NMEA: the cow walks a route in Parc de Milan    |
| servo on the bell latch         | on-board LED (GPIO2) = bell ringing (servo on GPIO25 optional) |
| OLED                            | nothing (read the Serial Monitor)                         |
| battery divider                 | fixed 3950 mV                                             |

Unchanged: protocol, fence maths, zone state machine, bell cue limits
(20 s on / 10 s off, 5 cues -> ESCAPED), events with ACK and retries, TDMA
slots, fence download in chunks, the duty-cycle limit (ESP-NOW reports the
real LoRa airtime).

**Setup**
1. Arduino IDE board: **ESP32 Dev Module** (all three boards). Libraries:
   TinyGPSPlus, PubSubClient, ArduinoJson 7. (RadioLib and the OLED library are
   not needed in bench mode.)
2. `vf_gateway/config.h`: your hotspot `WIFI_SSID` / `WIFI_PASS`, and a unique
   `TOPIC_PREFIX`. Upload to board 1. Turn the hotspot on first.
3. `vf_collar/config.h`: `BENCH_ROUTE 1` -> upload to board 2;
   `BENCH_ROUTE 2` -> upload to board 3.
4. Website: enter `TOPIC_PREFIX` (default `vf-alp-test-7q3/gw01`) under
   "Wi-Fi gateway topic" and press "Connect online".

The gateway also opens a small Wi-Fi network `VF-GW-2A` on the hotspot's
channel; the collars scan for it to find the channel (ESP-NOW and Wi-Fi share
one radio). Nobody needs to join it.

**Website messages** (`CLOUD_FORMAT_REBELL 1`, also usable later with LoRa):
`<prefix>/up` telemetry per report, `<prefix>/event` events + ACKs,
`<prefix>/status` gateway_position (+ health fields). The website's
"Send fence to gateway" becomes the collars' fence (downloaded at their next
report). Bells (22 Nov): `<prefix>/cmd/sound` per cow (AUTO = bell on, SILENT = bell
off, SOUND = ring now for `RING_NOW_S` = 60 s) -> new downlink `VF_MSG_MODE`,
the collar keeps its mode in flash and reports it in the uplink (`mode` byte,
uplink now 25 bytes, same LoRa airtime). `<prefix>/cmd/bells` (retained):
herd switch + daily silent hours, e.g.
`{"type":"bells","enabled":true,"schedule":{"enabled":true,"off":"20:00","on":"06:00"}}`.
The gateway keeps the plan in flash, takes the time from the internet (NTP,
`GW_TIMEZONE`), and sends `bellEnabled` to the collars in a CONFIG whenever the
result changes. Its status message reports `bells` (enabled, active, schedule,
clock). Collars pick changes up at their next report.

**Back to the real boards:** set `VF_BENCH 0` in both `config.h` files and
choose the Heltec V4 board again. Nothing else changes.

**Fix in this version (real system too):** the collar's duty-cycle limit was
1.08 s of airtime per hour instead of 108 s (`36UL` -> `3600UL`), which would
have blocked reports after ~5 per hour.

### Heltec V4 with real GPS in the ESP-NOW test (VF_RADIO_ESPNOW)

`vf_collar/config.h` now has two switches:

| VF_BENCH | VF_RADIO_ESPNOW | Collar |
|---|---|---|
| 1 | 1 | plain ESP32, simulated GPS walk, ESP-NOW (3-board test) |
| 0 | 1 | **Heltec V4 with its real L76K GPS**, OLED, ESP-NOW to the ESP32 test gateway |
| 0 | 0 | real collar: Heltec V4, LoRa |

The gateway follows VF_BENCH (ESP32 + ESP-NOW, or Heltec + LoRa).
With real GPS the cow is wherever you are: draw a fence around it on the
website (it is sent to the collar automatically). Until then the gateway's
default Parc de Milan fence applies and the collar reports "escaped".
