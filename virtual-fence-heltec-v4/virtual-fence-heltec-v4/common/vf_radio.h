// =============================================================================
//  LoRa radio setup for the Heltec V4 – works with RadioLib 7.x and 8.x
//  (RadioLib 8 changed begin(): config struct, frequency in Hz, setIrqAction).
// =============================================================================
#pragma once

#if defined(VF_RADIO_ESPNOW) && VF_RADIO_ESPNOW
#include "vf_radio_espnow.h"     // bench mode: ESP-NOW stand-in with the same functions
#else

#include <SPI.h>
#include <RadioLib.h>
#include "heltec_v4.h"
#include "protocol.h"

// SX1262 output power. The V4 front end adds ~+11 dB:
//   5 dBm -> ~16 dBm (40 mW) at the antenna. Legal limit on 869.525 MHz: 27 dBm.
#ifndef VF_RADIO_DBM
#define VF_RADIO_DBM 5
#endif

static SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);

// Returns RADIOLIB_ERR_NONE (0) on success.
static int16_t vfRadioBegin(void (*irq)(void) = nullptr, int8_t dbm = VF_RADIO_DBM) {
  femInit();
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_NSS);
  int16_t s;
#if defined(RADIOLIB_VERSION_MAJOR) && (RADIOLIB_VERSION_MAJOR >= 8)
  radio.tcxoVoltage = RadioLibTCXOVoltage_t::Voltage1V8;
  ConfigLoRa_t c;
  c.frequency       = VF_FREQ_HZ;
  c.bandwidth       = VF_BW_HZ;
  c.spreadingFactor = VF_SF;
  c.codingRate      = VF_CR;
  c.syncWord        = VF_SYNC_WORD;
  c.power           = dbm;
  c.preambleLength  = VF_PREAMBLE;
  s = radio.begin(c);
  if (s == RADIOLIB_ERR_NONE && irq) radio.setIrqAction(irq);
#else
  s = radio.begin(VF_FREQ_HZ / 1.0e6f, VF_BW_HZ / 1.0e3f, VF_SF, VF_CR, VF_SYNC_WORD,
                  dbm, VF_PREAMBLE, LORA_TCXO_V, false);
  if (s == RADIOLIB_ERR_NONE && irq) radio.setPacketReceivedAction(irq);
#endif
  if (s != RADIOLIB_ERR_NONE) return s;
  radio.setDio2AsRfSwitch(true);       // DIO2 drives the FEM TX/RX line
  radio.setCRC(2);
  radio.setCurrentLimit(140);
  femRx();
  return RADIOLIB_ERR_NONE;
}

// Blocking transmit with the front end switched to TX for the duration.
static int16_t vfTransmit(const uint8_t *buf, size_t len) {
  femTx();
  int16_t s = radio.transmit(buf, len);
  femRx();
  return s;
}

static int16_t vfStartReceive() {
  femRx();
  return radio.startReceive();
}

// Time on air in ms for a payload of len bytes (works on 7.x and 8.x).
static uint32_t vfTimeOnAirMs(size_t len) {
  return (uint32_t)(radio.getTimeOnAir(len) / 1000);
}

#endif  // VF_RADIO_ESPNOW
