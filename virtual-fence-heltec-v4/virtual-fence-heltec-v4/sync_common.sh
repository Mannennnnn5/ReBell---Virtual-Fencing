#!/bin/sh
# Copies the shared headers into every sketch folder (Arduino IDE needs them next to the .ino)
cd "$(dirname "$0")"
for d in vf_collar vf_gateway tests/T1_board_check tests/T2_lora_ping; do cp common/heltec_v4.h common/protocol.h common/geofence.h common/vf_radio.h "$d/"; done
# bench mode (plain ESP32 + ESP-NOW) files, only needed by the collar and the gateway
for d in vf_collar vf_gateway; do cp common/bench_esp32.h common/vf_radio_espnow.h "$d/"; done
for d in tests/T3_gps tests/T4_servo; do cp common/heltec_v4.h "$d/"; done
