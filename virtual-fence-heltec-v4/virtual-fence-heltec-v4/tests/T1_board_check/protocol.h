// =============================================================================
//  Alpine Virtual Fence – shared radio protocol (collar <-> gateway)
//  Included by both firmware projects. Keep both sides in sync!
// =============================================================================
#pragma once
#include <stdint.h>

// ---------------------------------------------------------------- radio -----
// EU868, sub-band 869.40–869.65 MHz: 10 % duty cycle, max 500 mW ERP.
// One channel for uplink and downlink keeps the gateway single-radio.
#define VF_FREQ_HZ           869525000UL
#define VF_BW_HZ             125000UL
#define VF_FREQ_MHZ          869.525f
#define VF_SF                9          // ~206 ms airtime for the 24-byte uplink
#define VF_CR                5          // 4/5
#define VF_SYNC_WORD         0x12       // private LoRa network (not LoRaWAN 0x34)
#define VF_PREAMBLE          8

#define VF_PROTO_VERSION     1

// --------------------------------------------------------------- TDMA -------
// Every collar knows UTC from GNSS. The report period is split into slots and
// each collar transmits only in slot (deviceId % VF_NUM_SLOTS).
// Period 120 s / 0.4 s = 300 slots -> max 300 collars per gateway.
#define VF_SLOT_MS           400
#define VF_DEFAULT_PERIOD_S  120
#define VF_RX_WINDOW_MS      600        // collar listens this long after each TX

// --------------------------------------------------------------- limits -----
#define VF_MAX_FENCE_POINTS  32
#define VF_FENCE_CHUNK_PTS   8          // vertices per downlink chunk
#define VF_BROADCAST_ID      0xFFFF

// ---------------------------------------------------------- message types ---
enum VfMsgType : uint8_t {
  // uplink (collar -> gateway)
  VF_MSG_POSITION  = 0x01,   // periodic report in TDMA slot
  VF_MSG_EVENT     = 0x02,   // immediate, needs ACK, retried
  // downlink (gateway -> collar)
  VF_MSG_ACK       = 0x10,
  VF_MSG_CONFIG    = 0x11,
  VF_MSG_FENCE     = 0x12,   // one chunk of the fence polygon
  VF_MSG_MODE      = 0x13,   // bell mode for one collar (+ optional "ring now")
};

// Per-collar bell mode (farmer's choice, kept by the collar)
enum VfBellMode : uint8_t {
  VF_MODE_AUTO     = 0,      // bell follows the fence (and the herd switch in CONFIG)
  VF_MODE_SILENT   = 1,      // this collar never rings
};
#define VF_MODE_SOUNDING  0x80  // uplink only: "ring now" is active on the collar

// Collar zone state (also used in flags)
enum VfZone : uint8_t {
  VF_ZONE_UNKNOWN = 0,       // no fence or no fix
  VF_ZONE_INSIDE  = 1,       // > nearDist inside
  VF_ZONE_NEAR    = 2,       // inside, within nearDist of the edge
  VF_ZONE_OUTSIDE = 3,       // outside -> bell cue active
  VF_ZONE_ESCAPED = 4,       // cue gave up, farmer must fetch the cow
};

enum VfEvent : uint8_t {
  VF_EVT_NONE      = 0,
  VF_EVT_EXIT      = 1,      // crossed the fence, bell cue started
  VF_EVT_RETURN    = 2,      // came back inside after a cue
  VF_EVT_ESCAPED   = 3,      // cue limit reached, bell disabled
  VF_EVT_LOW_BATT  = 4,
  VF_EVT_BOOT      = 5,
};

// flags bit field (uplink)
#define VF_FLAG_FIX_VALID   0x01
#define VF_FLAG_BELL_ON     0x02
#define VF_FLAG_LOW_BATT    0x04
#define VF_FLAG_NO_FENCE    0x08
#define VF_FLAG_ZONE_SHIFT  4       // zone stored in bits 4..6

// ------------------------------------------------------------ packets -------
// All multi-byte fields are little-endian (ESP32 native). Packed, no padding.
#pragma pack(push, 1)

struct VfHeader {
  uint8_t  type;          // VfMsgType
  uint8_t  netId;         // herd / farm network id, filters neighbours
  uint16_t deviceId;      // 1..65534, 0xFFFF = broadcast (downlink)
  uint8_t  seq;
};

// 25 bytes total -> ~206 ms airtime at SF9/125 kHz (same symbols as 24 bytes)
struct VfPosition {
  VfHeader h;
  uint8_t  flags;         // VF_FLAG_* | zone << 4
  int32_t  lat;           // degrees * 1e7
  int32_t  lon;           // degrees * 1e7
  int16_t  fenceDistM;    // signed distance to fence: + inside, - outside, 32767 = n/a
  uint16_t battMv;
  int8_t   tempC;
  uint8_t  sats;
  uint8_t  hAccDm;        // horizontal accuracy in decimetres (255 = >25 m)
  uint8_t  fenceVer;      // fence version the collar currently holds
  uint8_t  cfgVer;        // config version the collar currently holds
  uint8_t  event;         // VfEvent (VF_EVT_NONE for periodic reports)
  uint8_t  mode;          // VfBellMode | VF_MODE_SOUNDING
};

struct VfAck {
  VfHeader h;
  uint8_t  ackSeq;
};

struct VfConfig {
  VfHeader h;
  uint8_t  cfgVer;
  uint16_t reportPeriodS;   // TDMA period, multiple of slot length
  uint8_t  nearDistM;       // switch to fast GNSS inside this distance
  uint8_t  bellEnabled;     // 0 = monitoring only (training phase)
  uint8_t  cueMaxS;         // max bell-on seconds per cue episode
  uint8_t  cuePauseS;       // pause between episodes
  uint8_t  maxEpisodes;     // after this many failed cues -> ESCAPED
  uint8_t  escapeDistM;     // or this far outside -> ESCAPED
};

struct VfMode {
  VfHeader h;
  uint8_t  mode;            // VfBellMode
  uint8_t  ringS;           // >0: ring now for this many seconds (collar caps it), 0: stop ringing
};

struct VfFencePoint {
  int32_t lat;              // degrees * 1e7
  int32_t lon;
};

struct VfFenceChunk {
  VfHeader     h;
  uint8_t      fenceVer;
  uint8_t      totalPts;    // total vertices in the polygon
  uint8_t      firstIdx;    // index of pts[0] in the polygon
  uint8_t      count;       // vertices in this chunk (<= VF_FENCE_CHUNK_PTS)
  VfFencePoint pts[VF_FENCE_CHUNK_PTS];
};

#pragma pack(pop)

static_assert(sizeof(VfPosition) == 25, "uplink size changed - update docs/airtime");
static_assert(sizeof(VfFenceChunk) <= 255, "LoRa max payload");

// Byte length of a fence chunk that carries n points
inline uint8_t vfFenceChunkLen(uint8_t n) {
  return (uint8_t)(sizeof(VfFenceChunk) - sizeof(VfFencePoint) * (VF_FENCE_CHUNK_PTS - n));
}
