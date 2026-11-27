// =============================================================================
//  Heltec WiFi LoRa 32 V4 – board pins and helpers
//
//  Sources (checked Nov 2026):
//   - Heltec "WiFi LoRa 32 V4.3.1" datasheet, header tables J2/J3
//   - Meshtastic firmware variants/esp32s3/heltec_v4 (variant.h, LoRaFEMInterface)
//
//  The V4 has an RF front end (FEM) with a power amplifier between the SX1262
//  and the antenna. It MUST be powered and enabled, otherwise nothing is sent
//  or received. Two FEM chips exist:
//    V4.2  GC1109   : GPIO2 = enable (CSD), GPIO46 = PA on/bypass (CPS)
//    V4.3  KCT8103L : GPIO2 = enable (CSD), GPIO5  = TX/RX-bypass (CTX)
//  Both: GPIO7 switches the FEM's LDO. DIO2 of the SX1262 does TX/RX switching.
//  femInit() detects which one is fitted (same method as Meshtastic).
// =============================================================================
#pragma once

// Bench mode (VF_BENCH 1 in config.h): plain ESP32 dev board -> see bench_esp32.h
#if defined(VF_BENCH) && VF_BENCH
#include "bench_esp32.h"
#else

#include <Arduino.h>
#include "driver/gpio.h"

// ------------------------------------------------------------ SX1262 --------
#define PIN_LORA_NSS      8
#define PIN_LORA_SCK      9
#define PIN_LORA_MOSI     10
#define PIN_LORA_MISO     11
#define PIN_LORA_RST      12
#define PIN_LORA_BUSY     13
#define PIN_LORA_DIO1     14
#define LORA_TCXO_V       1.8f

// ------------------------------------------------------------ RF front end --
#define PIN_FEM_POWER     7      // VFEM LDO enable (HIGH = on)
#define PIN_FEM_EN        2      // CSD on both chips (HIGH = on)
#define PIN_FEM_GC_PA     46     // GC1109 CPS: HIGH = full PA, LOW = bypass
#define PIN_FEM_KCT_CTX   5      // KCT8103L CTX: HIGH = TX / RX bypass, LOW = RX LNA
// Net TX gain of the FEM is about +11 dB at low drive (Meshtastic measurements):
// SX1262 set to 5 dBm -> ~16 dBm (40 mW) at the antenna.

// ------------------------------------------------------------ GNSS (L76K) ---
// Plug-in GNSS module on the 8-pin SH1.25 GNSS connector.
#define PIN_GNSS_TX       38     // ESP32 -> GNSS
#define PIN_GNSS_RX       39     // GNSS  -> ESP32
#define PIN_GNSS_EN       34     // VGNSS_Ctrl, LOW = GNSS powered
#define PIN_GNSS_WAKE     40     // HIGH = keep awake, LOW = allow standby
#define PIN_GNSS_PPS      41
#define PIN_GNSS_RST      42     // LOW > 100 ms resets the L76K
#define GNSS_BAUD         9600

// ------------------------------------------------------------ board ---------
#define PIN_VEXT          36     // LOW = Vext (OLED + 3.3 V "Ve" pins) on
#define PIN_LED           35     // white LED, HIGH = on
#define PIN_BUTTON        0      // PRG button, LOW = pressed
#define PIN_VBAT_ADC      1      // battery divider 390k/100k
#define PIN_VBAT_CTRL     37     // HIGH = connect divider (only while measuring)
#define VBAT_MULTIPLIER   (4.9f * 1.045f)
#define PIN_OLED_SDA      17
#define PIN_OLED_SCL      18
#define PIN_OLED_RST      21
#define OLED_ADDR         0x3C

// ------------------------------------------------------------ our wiring ----
// Collar only. Both are RTC GPIOs on header J3 (no other use on the V4).
#define PIN_SERVO_SIG     6      // J3 pin 17 -> servo signal (orange)
#define PIN_SERVO_PWR_EN  4      // J3 pin 15 -> TPS63020 EN (HIGH = 5 V on)

// =============================================================================
//  RF front end control
// =============================================================================
enum FemType : uint8_t { FEM_NONE = 0, FEM_GC1109, FEM_KCT8103L };
static FemType g_fem = FEM_NONE;

static inline void femReleaseHolds() {
  gpio_hold_dis((gpio_num_t)PIN_FEM_POWER);
  gpio_hold_dis((gpio_num_t)PIN_FEM_EN);
  gpio_hold_dis((gpio_num_t)PIN_FEM_GC_PA);
  gpio_hold_dis((gpio_num_t)PIN_FEM_KCT_CTX);
}

// Call once before radio.begin(). Returns the detected FEM type.
static inline FemType femInit() {
  femReleaseHolds();
  pinMode(PIN_FEM_POWER, OUTPUT);
  digitalWrite(PIN_FEM_POWER, HIGH);
  delay(5);
  pinMode(PIN_FEM_EN, INPUT);              // KCT8103L pulls CSD high, GC1109 does not
  delay(1);
  if (digitalRead(PIN_FEM_EN) == HIGH) {
    g_fem = FEM_KCT8103L;
    pinMode(PIN_FEM_EN, OUTPUT);       digitalWrite(PIN_FEM_EN, HIGH);
    pinMode(PIN_FEM_KCT_CTX, OUTPUT);  digitalWrite(PIN_FEM_KCT_CTX, LOW);   // RX LNA
  } else {
    g_fem = FEM_GC1109;
    pinMode(PIN_FEM_EN, OUTPUT);       digitalWrite(PIN_FEM_EN, HIGH);
    pinMode(PIN_FEM_GC_PA, OUTPUT);    digitalWrite(PIN_FEM_GC_PA, LOW);
  }
  return g_fem;
}

static inline const char *femName() {
  return g_fem == FEM_GC1109 ? "GC1109 (V4.2)" : g_fem == FEM_KCT8103L ? "KCT8103L (V4.3)" : "none";
}

// Call right before radio.transmit()
static inline void femTx() {
  digitalWrite(PIN_FEM_POWER, HIGH);
  digitalWrite(PIN_FEM_EN, HIGH);
  if (g_fem == FEM_GC1109)   digitalWrite(PIN_FEM_GC_PA, HIGH);
  if (g_fem == FEM_KCT8103L) digitalWrite(PIN_FEM_KCT_CTX, HIGH);
}

// Call right after transmit() and before startReceive()
static inline void femRx() {
  digitalWrite(PIN_FEM_POWER, HIGH);
  digitalWrite(PIN_FEM_EN, HIGH);
  if (g_fem == FEM_GC1109)   digitalWrite(PIN_FEM_GC_PA, LOW);
  if (g_fem == FEM_KCT8103L) digitalWrite(PIN_FEM_KCT_CTX, LOW);
}

// Before deep sleep: FEM off (µA) and pins frozen in that state.
static inline void femSleep() {
  digitalWrite(PIN_FEM_EN, LOW);
  if (g_fem == FEM_GC1109)   digitalWrite(PIN_FEM_GC_PA, LOW);
  if (g_fem == FEM_KCT8103L) { digitalWrite(PIN_FEM_KCT_CTX, LOW); digitalWrite(PIN_FEM_POWER, LOW); }
  gpio_hold_en((gpio_num_t)PIN_FEM_POWER);
  gpio_hold_en((gpio_num_t)PIN_FEM_EN);
  if (g_fem == FEM_GC1109)   gpio_hold_en((gpio_num_t)PIN_FEM_GC_PA);
  if (g_fem == FEM_KCT8103L) gpio_hold_en((gpio_num_t)PIN_FEM_KCT_CTX);
}

// =============================================================================
//  Battery voltage (mV). Divider is only connected while measuring.
// =============================================================================
static inline uint16_t batteryMv() {
  pinMode(PIN_VBAT_CTRL, OUTPUT);
  digitalWrite(PIN_VBAT_CTRL, HIGH);
  analogSetPinAttenuation(PIN_VBAT_ADC, ADC_2_5db);   // 0..~1.05 V range
  delay(5);
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += analogReadMilliVolts(PIN_VBAT_ADC);
  digitalWrite(PIN_VBAT_CTRL, LOW);
  return (uint16_t)((sum / 8) * VBAT_MULTIPLIER);
}

// Rough LiPo state of charge from voltage (no load), 0..100 %
static inline uint8_t batteryPercent(uint16_t mv) {
  if (mv >= 4150) return 100;
  if (mv <= 3300) return 0;
  return (uint8_t)((mv - 3300) * 100UL / (4150 - 3300));
}

// =============================================================================
//  Vext (powers the OLED and the "Ve" header pins)
// =============================================================================
static inline void vextOn()  { pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, LOW);  delay(50); }
static inline void vextOff() { pinMode(PIN_VEXT, OUTPUT); digitalWrite(PIN_VEXT, HIGH); }

#endif  // VF_BENCH
