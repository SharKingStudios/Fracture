/*
  Infinite MIDI Keyboard – ESP32-S3 (Polished Version)

  Features:
  - 2 octaves per board (24 keys) + 8 extra keys (32 total)
  - 2x 74HC4067 (16+16 channels)
  - Hall-effect velocity sensing (time delta based)
  - WS2812 LEDs (45 total):
      * 1–32 under keys (0–31 index)
      * 33–44 status (32–43 index)
      * 45th LED (index 44) top-left:
          - Yellow while remapping
          - Light green pulse when network healthy
  - LED base color: light blue under all keys
  - Keypress ripple: white “splash in the pond” based on MIDI note distances,
    propagating across all boards.
  - Buzzer:
      * Startup jingle
      * Polyphonic (up to 12 notes) for this board’s keys only
  - I2S DAC (ES9023P) sine-wave poly synth
  - RS-485 (SP3485) bus:
      * Physical left/right chain mapping via IO1/IO2
      * Each board gets an index 0..N-1, left→right
      * NOTE events shared across boards
      * Board 0 pings others, auto-remap on missing nodes
      * Auto-remap on new ANNOUNCE IDs
  - USB MIDI:
      * Uses Adafruit TinyUSB Library (Adafruit_USBD_MIDI)
      * Any board plugged into USB outputs MIDI for the whole keyboard

  Neighbor wiring assumption:
    - IO43 = LEFT neighbor signal (INPUT_PULLUP).
    - IO41 = RIGHT neighbor signal (OUTPUT).
    - Each board’s IO2 connects to the next board’s IO1 on its right.
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>
#include <HardwareSerial.h>
#include <Adafruit_TinyUSB.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/timer.h"

// ========================= PIN DEFINITIONS =========================

// Physical neighbor chain pins
#define PIN_NEIGHBOR_LEFT    43   // IO43: left neighbor signal (input)
#define PIN_NEIGHBOR_RIGHT   41   // IO41: right neighbor signal (output)

// Encoder (rewired to free IO1/IO2)
#define PIN_ENC_A            1
#define PIN_ENC_B            2
#define PIN_ENC_SW           15

// LEDs
#define PIN_LED_DATA         14   // WS2812, 45 LEDs

// Buzzer (now driven by timer-based polyphonic synth)
#define PIN_BUZZER           13

// RS-485 (SP3485)
#define PIN_RS485_MODE       21   // DE/RE
#define PIN_RS485_TX         17
#define PIN_RS485_RX         18

// Multiplexer selects (74HC4067)
#define PIN_MUX_S0           6
#define PIN_MUX_S1           7
#define PIN_MUX_S2           8
#define PIN_MUX_S3           9

// Multiplexer analog inputs
#define PIN_AM0              4   // Extra keys + main keys
#define PIN_AM1              5   // Main keys

// I2S DAC (ES9023P)
#define PIN_I2S_BCLK         12
#define PIN_I2S_LRCK         11
#define PIN_I2S_DOUT         10
#define PIN_DAC_MUTE_B       16

// ========================= CONSTANTS ===============================

// Keys
const uint8_t NUM_MAIN_KEYS   = 24;   // musical keys
const uint8_t NUM_EXTRA_KEYS  = 8;    // function keys
const uint8_t TOTAL_KEYS      = 32;   // 24 + 8

// LEDs
const uint16_t NUM_LEDS       = 45;   // 32 keys, 12 status, 1 top-left

// MIDI note layout
const uint8_t GLOBAL_BASE_NOTE = 48;  // C3 as first board's first note
const uint8_t BOARD_NOTE_SPAN  = 24;  // 2 octaves per board

// RS-485
HardwareSerial RS485(1);
const uint32_t RS485_BAUD       = 250000;
const uint8_t  RS485_MAX_NODES  = 16;

// RS-485 message types
const uint8_t  RS485_ANNOUNCE_MSG      = 0x01; // nodeId + boardIndex
const uint8_t  RS485_NOTE_MSG          = 0x02; // nodeId + note + vel + on/off
const uint8_t  RS485_PING_MSG          = 0x03; // ping sequence
const uint8_t  RS485_PING_REPLY_MSG    = 0x04; // nodeId + seq
const uint8_t  RS485_REMAP_REQUEST_MSG = 0x05; // trigger re-discovery

// Discovery / mapping
const uint32_t DISCOVERY_MAX_MS        = 1000; // total time budget for chain setup

// Velocity sensing
const uint16_t VELOCITY_ON_DELTA     = 80;    // change from baseline to detect press start
const uint16_t VELOCITY_FULL_DELTA   = 300;   // change from baseline to consider “fully pressed”
const uint16_t VELOCITY_MIN_DT_US    = 1000;  // very fast strike
const uint32_t VELOCITY_MAX_DT_US    = 60000; // very slow strike

// Audio synth (I2S)
const i2s_port_t I2S_PORT = I2S_NUM_0;
const uint32_t AUDIO_SAMPLE_RATE = 44100;
const uint8_t MAX_VOICES = 8;
const float MASTER_VOLUME = 0.4f;

// Buzzer synth
const uint8_t  BUZZER_MAX_VOICES  = 12;
const float    BUZZER_SAMPLE_RATE = 20000.0f; // 20 kHz

// Encoder
const int32_t ENCODER_MIN = 0;
const int32_t ENCODER_MAX = 127;

// Ping / remap
const uint32_t PING_INTERVAL_MS = 1000;
const uint32_t PING_TIMEOUT_MS  = 200;

// Ripple (LED splash)
const float RIPPLE_SPEED = 0.02f;  // "radius" in semitones/ms

// ========================= TYPES & STRUCTS =========================

struct NodeInfo {
  uint32_t nodeId;
  uint8_t  boardIndex; // 0..N-1
};

struct NodeList {
  uint8_t  count = 0;
  NodeInfo nodes[RS485_MAX_NODES];
};

struct KeyDef {
  uint8_t muxIndex;   // 0 = AM0, 1 = AM1
  uint8_t muxChannel; // 0–15
  bool    isExtra;    // true for extra keys
};

struct KeyState {
  uint16_t raw = 0;
  uint16_t baseline = 0;
  bool     pressed = false;
  bool     velocityComputed = false;
  uint32_t pressStartUs = 0;
  uint8_t  velocity = 0;
};

struct Voice {
  bool active = false;
  uint8_t note = 0;
  float phase = 0.0f;
  float phaseInc = 0.0f;
  float amplitude = 0.0f;
};

struct Ripple {
  bool active = false;
  float centerNote = 0.0f;   // MIDI note
  float radius = 0.0f;       // in semitones
  float maxRadius = 24.0f;
  uint32_t startMs = 0;
  uint8_t baseVelocity = 80;
};

struct RxState {
  uint8_t state = 0;
  uint8_t len = 0;
  uint8_t type = 0;
  uint8_t buf[32];
  uint8_t pos = 0;
  uint8_t checksum = 0;
};

struct BuzzerVoice {
  bool active;
  uint8_t note;
  float phase;
  float phaseInc;
};

// ========================= FORWARD DECLARATIONS ====================

void onLocalNoteOn(uint8_t note, uint8_t velocity);
void onLocalNoteOff(uint8_t note);
void onRemoteNoteOn(uint8_t note, uint8_t velocity);
void onRemoteNoteOff(uint8_t note);
void handleExtraKeyPressed(uint8_t extraIndex);
void rs485SendPacket(uint8_t type, const uint8_t *payload, uint8_t len);
void startDiscovery(bool fromRemap);
uint8_t midiNoteToBoardOffset(uint8_t note);

// ========================= GLOBALS =================================

Adafruit_NeoPixel leds(NUM_LEDS, PIN_LED_DATA, NEO_GRB + NEO_KHZ800);

// USB MIDI via TinyUSB
Adafruit_USBD_MIDI usb_midi;

// Key definitions
KeyDef keyDefs[TOTAL_KEYS];
KeyState keyStates[TOTAL_KEYS];

// Musical mapping: main key index -> semitone offset from boardBaseNote
// For now: black keys first (2 octaves, left→right), then white keys.
// You can remap this later to match your physical layout.
const uint8_t keyToNote[NUM_MAIN_KEYS] = {
  // Black keys (2 octaves)
  1, 3, 6, 8, 10,    // C#1, D#1, F#1, G#1, A#1
  13, 15, 18, 20, 22,// C#2, D#2, F#2, G#2, A#2
  // White keys (2 octaves)
  0, 2, 4, 5, 7, 9, 11,   // C1 D1 E1 F1 G1 A1 B1
  12, 14, 16, 17, 19, 21, 23 // C2 D2 E2 F2 G2 A2 B2
};

// Node discovery
uint32_t myNodeId;
NodeList nodeList;
uint8_t myBoardIndex = 0;
uint8_t totalBoards = 1;
bool discoveryDone = false;
bool remapRequested = false;

// Network state
bool isRemapping     = false;
bool networkHealthy  = false;

// Note mapping
uint8_t boardBaseNote = GLOBAL_BASE_NOTE;

// Synth (I2S)
Voice voices[MAX_VOICES];

// Encoder
volatile int32_t encoderValue = 64;
uint8_t encoderLastState = 0;

// Modes
bool buzzerEnabled = false;
bool audioEnabled  = true;

// Neighbor info
bool hasLeftNeighbor = false;

// RS-485 RX state
RxState rx;

// Ripple + key LED base velocities
Ripple ripple;
uint8_t keyBaseVelocity[TOTAL_KEYS]; // 0..127

// Ping
uint8_t pingSequence = 0;
bool awaitingPingReplies = false;
uint32_t pingStartMs = 0;
uint32_t lastPingMs = 0;
bool pingReceived[RS485_MAX_NODES];

// Buzzer synth
volatile BuzzerVoice buzzerVoices[BUZZER_MAX_VOICES];
hw_timer_t* buzzerTimer = nullptr;
portMUX_TYPE buzzerMux = portMUX_INITIALIZER_UNLOCKED;

// ========================= UTILITY HELPERS =========================

uint32_t readRandom32() {
  return ((uint32_t)esp_random());
}

float midiNoteToFreq(uint8_t note) {
  return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

// Given a note, find local main key index that maps to it, or 0xFF if not on this board
uint8_t keyIndexFromNote(uint8_t note) {
  uint8_t offset = note - boardBaseNote;
  for (uint8_t i = 0; i < NUM_MAIN_KEYS; i++) {
    if (keyToNote[i] == offset) return i;
  }
  return 0xFF;
}

// ========================= MUX & KEYS ==============================

void setMuxChannel(uint8_t channel) {
  digitalWrite(PIN_MUX_S0, channel & 0x01);
  digitalWrite(PIN_MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(PIN_MUX_S2, (channel >> 2) & 0x01);
  digitalWrite(PIN_MUX_S3, (channel >> 3) & 0x01);
}

uint16_t readKeyRaw(const KeyDef &def) {
  setMuxChannel(def.muxChannel);
  delayMicroseconds(5);
  int pin = (def.muxIndex == 0) ? PIN_AM0 : PIN_AM1;
  return analogRead(pin);
}

/*
  Physical mapping:

  - AM0:0–7   => extra keys (8)
  - AM0:8–15  => main keys (8)
  - AM1:0–15  => main keys (16)
  Total main keys = 24, extra = 8.

  main key indices 0–23 correspond to keyToNote[] offsets.
*/
void initKeyDefs() {
  // Extra keys: indices 24–31 on AM0:0–7
  for (uint8_t i = 0; i < NUM_EXTRA_KEYS; i++) {
    uint8_t keyIndex = NUM_MAIN_KEYS + i; // 24–31
    keyDefs[keyIndex].muxIndex   = 0;     // AM0
    keyDefs[keyIndex].muxChannel = i;     // 0–7
    keyDefs[keyIndex].isExtra    = true;
  }

  // Main keys 0–7 on AM0:8–15
  for (uint8_t i = 0; i < 8; i++) {
    keyDefs[i].muxIndex   = 0;        // AM0
    keyDefs[i].muxChannel = 8 + i;    // 8–15
    keyDefs[i].isExtra    = false;
  }

  // Main keys 8–23 on AM1:0–15
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t keyIndex = 8 + i;        // 8–23
    keyDefs[keyIndex].muxIndex   = 1; // AM1
    keyDefs[keyIndex].muxChannel = i; // 0–15
    keyDefs[keyIndex].isExtra    = false;
  }
}

void calibrateKeys() {
  const uint8_t samples = 16;
  for (uint8_t k = 0; k < TOTAL_KEYS; k++) {
    uint32_t sum = 0;
    for (uint8_t s = 0; s < samples; s++) {
      sum += readKeyRaw(keyDefs[k]);
      delay(1);
    }
    keyStates[k].baseline = (uint16_t)(sum / samples);
    keyStates[k].raw = keyStates[k].baseline;
  }
}

uint8_t computeVelocity(uint16_t delta, uint32_t dtUs) {
  (void)delta;
  if (dtUs <= VELOCITY_MIN_DT_US) return 127;
  if (dtUs >= VELOCITY_MAX_DT_US) return 10;

  float t = (float)(dtUs - VELOCITY_MIN_DT_US) /
            (float)(VELOCITY_MAX_DT_US - VELOCITY_MIN_DT_US);
  t = constrain(1.0f - t, 0.0f, 1.0f);
  uint8_t vel = (uint8_t)(10 + t * (127 - 10));
  if (vel < 1) vel = 1;
  return vel;
}

void setKeyLedBase(uint8_t keyIndex, uint8_t velocity, bool on);
void startRipple(uint8_t note, uint8_t velocity);

void scanKeys() {
  for (uint8_t k = 0; k < TOTAL_KEYS; k++) {
    KeyDef &def = keyDefs[k];
    KeyState &st = keyStates[k];

    uint16_t raw = readKeyRaw(def);
    st.raw = raw;
    int16_t delta = (int16_t)raw - (int16_t)st.baseline;
    if (delta < 0) delta = -delta;

    uint32_t nowUs = micros();

    if (!st.pressed) {
      if (delta > VELOCITY_ON_DELTA) {
        st.pressed = true;
        st.pressStartUs = nowUs;
        st.velocityComputed = false;
      }
    } else {
      if (!st.velocityComputed && delta > VELOCITY_FULL_DELTA) {
        uint32_t dtUs = nowUs - st.pressStartUs;
        st.velocity = computeVelocity(delta, dtUs);
        st.velocityComputed = true;

        if (def.isExtra) {
          uint8_t extraIndex = k - NUM_MAIN_KEYS;
          handleExtraKeyPressed(extraIndex);
          setKeyLedBase(k, st.velocity, true);
          // Extras do not spawn cross-board ripple (no MIDI note)
        } else {
          // main key k -> MIDI note via keyToNote[]
          uint8_t note = boardBaseNote + keyToNote[k];
          uint8_t vel = st.velocity;
          if (vel == 0) vel = 1;
          onLocalNoteOn(note, vel);
        }
      }

      // Release detection
      if (delta < (VELOCITY_ON_DELTA / 2)) {
        if (def.isExtra) {
          setKeyLedBase(k, 0, false);
        } else {
          uint8_t note = boardBaseNote + keyToNote[k];
          onLocalNoteOff(note);
        }
        st.pressed = false;
        st.velocityComputed = false;
      }
    }
  }
}

// ========================= LED HANDLING ============================

void setKeyLedBase(uint8_t keyIndex, uint8_t velocity, bool on) {
  if (keyIndex >= TOTAL_KEYS) return;
  keyBaseVelocity[keyIndex] = on ? velocity : 0;
}

void startRipple(uint8_t note, uint8_t velocity) {
  ripple.active = true;
  ripple.centerNote = (float)note;
  ripple.radius = 0.0f;
  ripple.maxRadius = 24.0f;
  ripple.startMs = millis();
  ripple.baseVelocity = (velocity > 0 ? velocity : 80);
}

void updateStatusLeds() {
  // Board index in LEDs 33–36 (indices 32–35)
  for (uint8_t i = 0; i < 4; i++) {
    bool bit = (myBoardIndex >> i) & 0x01;
    uint16_t ledIndex = 32 + i;
    if (bit) {
      leds.setPixelColor(ledIndex, leds.Color(0, 50, 0));
    } else {
      leds.setPixelColor(ledIndex, 0);
    }
  }
  // LED 37 (index 36): buzzer mode
  leds.setPixelColor(36, buzzerEnabled ? leds.Color(50, 10, 0) : 0);
  // LED 38 (index 37): audio enabled
  leds.setPixelColor(37, audioEnabled ? leds.Color(0, 0, 50) : 0);
  // LED 39 (index 38): left neighbor present
  leds.setPixelColor(38, hasLeftNeighbor ? leds.Color(0, 30, 0) : 0);
  // LED 40 (index 39): reserved
  leds.setPixelColor(39, 0);
  // LEDs 41–44 (indices 40–43): free
  for (uint8_t i = 40; i <= 43; i++) {
    leds.setPixelColor(i, 0);
  }
}

void renderLeds() {
  uint32_t nowMs = millis();

  // Update ripple radius
  if (ripple.active) {
    uint32_t dt = nowMs - ripple.startMs;
    ripple.radius = dt * RIPPLE_SPEED;
    if (ripple.radius > ripple.maxRadius) {
      ripple.active = false;
    }
  }

  // Key LEDs 0–31
  for (uint8_t k = 0; k < TOTAL_KEYS; k++) {
    uint8_t baseVel = keyBaseVelocity[k];

    // Base color: soft light blue
    uint8_t r = 10;
    uint8_t g = 40;
    uint8_t b = 80;

    // If key is currently active, make it brighter based on velocity
    if (baseVel > 0) {
      uint8_t extra = map(baseVel, 1, 127, 10, 100);
      b = min<uint16_t>(255, b + extra);
      g = min<uint16_t>(255, g + extra / 2);
    }

    // Ripple overlay: white, based on MIDI note distance
    uint8_t overlay = 0;
    if (ripple.active) {
      bool hasNote = false;
      float noteVal = 0.0f;

      if (k < NUM_MAIN_KEYS) {
        uint8_t note = boardBaseNote + keyToNote[k];
        noteVal = (float)note;
        hasNote = true;
      } else {
        hasNote = false; // extras don't follow MIDI ripple
      }

      if (hasNote) {
        float dist = fabsf(noteVal - ripple.centerNote);
        float diff = fabsf(dist - ripple.radius);
        const float bandWidth = 1.5f;
        if (diff < bandWidth) {
          float t = 1.0f - (diff / bandWidth); // 0..1
          uint8_t maxOverlay = map(ripple.baseVelocity > 0 ? ripple.baseVelocity : 80,
                                   1, 127, 50, 255);
          overlay = (uint8_t)(t * maxOverlay);
        }
      }
    }

    if (overlay > 0) {
      r = min<uint16_t>(255, r + overlay);
      g = min<uint16_t>(255, g + overlay);
      b = min<uint16_t>(255, b + overlay);
    }

    leds.setPixelColor(k, leds.Color(r, g, b));
  }

  // Status LEDs
  updateStatusLeds();

  // Top-left LED (index 44)
  uint8_t topIndex = 44;
  if (isRemapping) {
    // Yellow while remapping
    leds.setPixelColor(topIndex, leds.Color(80, 80, 0));
  } else if (networkHealthy) {
    // Light green pulse when healthy
    float phase = (nowMs % 1000) / 1000.0f * 2.0f * PI;
    float t = (sinf(phase) + 1.0f) * 0.5f; // 0..1
    uint8_t g = (uint8_t)(30 + t * 80);    // 30..110
    uint8_t r = 5;
    uint8_t b = 5;
    leds.setPixelColor(topIndex, leds.Color(r, g, b));
  } else {
    // Off when not healthy / unknown
    leds.setPixelColor(topIndex, 0);
  }

  leds.show();
}

void showAllLedsStartup() {
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, leds.Color(0, 0, 50));
  }
  leds.show();
  delay(200);
  for (uint16_t i = 0; i < NUM_LEDS; i++) {
    leds.setPixelColor(i, 0);
  }
  leds.show();
}

// ========================= BUZZER POLYPHONIC SYNTH =================

void IRAM_ATTR onBuzzerTimer() {
  float acc = 0.0f;
  bool any = false;

  for (int i = 0; i < BUZZER_MAX_VOICES; i++) {
    if (buzzerVoices[i].active) {
      any = true;
      // simple square wave per voice
      acc += (buzzerVoices[i].phase < PI ? 1.0f : -1.0f);
      buzzerVoices[i].phase += buzzerVoices[i].phaseInc;
      if (buzzerVoices[i].phase >= 2.0f * PI) {
        buzzerVoices[i].phase -= 2.0f * PI;
      }
    }
  }

  int level = 0;
  if (any) {
    level = (acc >= 0.0f) ? 1 : 0;
  } else {
    level = 0;
  }
  gpio_set_level((gpio_num_t)PIN_BUZZER, level);
}

void buzzerStartNoteInternal(uint8_t note) {
  float freq = midiNoteToFreq(note);
  float phaseInc = 2.0f * PI * freq / BUZZER_SAMPLE_RATE;

  portENTER_CRITICAL(&buzzerMux);
  int idx = -1;
  for (int i = 0; i < BUZZER_MAX_VOICES; i++) {
    if (!buzzerVoices[i].active) { idx = i; break; }
  }
  if (idx < 0) idx = 0; // steal first if all busy

  buzzerVoices[idx].active   = true;
  buzzerVoices[idx].note     = note;
  buzzerVoices[idx].phase    = 0.0f;
  buzzerVoices[idx].phaseInc = phaseInc;
  portEXIT_CRITICAL(&buzzerMux);
}

void buzzerStopNoteInternal(uint8_t note) {
  portENTER_CRITICAL(&buzzerMux);
  for (int i = 0; i < BUZZER_MAX_VOICES; i++) {
    if (buzzerVoices[i].active && buzzerVoices[i].note == note) {
      buzzerVoices[i].active = false;
    }
  }
  portEXIT_CRITICAL(&buzzerMux);
}

void buzzerStopAllInternal() {
  portENTER_CRITICAL(&buzzerMux);
  for (int i = 0; i < BUZZER_MAX_VOICES; i++) {
    buzzerVoices[i].active = false;
  }
  portEXIT_CRITICAL(&buzzerMux);
}

// Keyboard-level buzzer handlers (respect buzzerEnabled)
void buzzerNoteOn(uint8_t note, uint8_t velocity) {
  (void)velocity;
  if (!buzzerEnabled) return;
  buzzerStartNoteInternal(note);
}

void buzzerNoteOff(uint8_t note) {
  if (!buzzerEnabled) return;
  buzzerStopNoteInternal(note);
}

// Startup jingle using internal buzzer voices
void playStartupJingle() {
  const uint8_t notes[] = {
    79, 76, 72, 67, 72, 76, 79 // G5, E5, C5, G4, C5, E5, G5
  };
  const uint16_t durMs[] = {
    120, 120, 120, 120, 120, 120, 250
  };

  bool oldEnabled = buzzerEnabled;
  buzzerEnabled = true;

  for (uint8_t i = 0; i < sizeof(notes); i++) {
    buzzerStartNoteInternal(notes[i]);
    delay(durMs[i]);
    buzzerStopNoteInternal(notes[i]);
    delay(20);
  }

  buzzerStopAllInternal();
  buzzerEnabled = oldEnabled;
}

// ========================= RS-485 PROTOCOL =========================

void rs485SetTx(bool tx) {
  digitalWrite(PIN_RS485_MODE, tx ? HIGH : LOW);
}

void rs485SendPacket(uint8_t type, const uint8_t *payload, uint8_t len) {
  uint8_t checksum = type ^ len;
  rs485SetTx(true);
  RS485.write(0xAA);
  RS485.write(len);
  RS485.write(type);
  for (uint8_t i = 0; i < len; i++) {
    RS485.write(payload[i]);
    checksum ^= payload[i];
  }
  RS485.write(checksum);
  RS485.flush();
  rs485SetTx(false);
}

void registerNode(uint32_t nodeId, uint8_t boardIndex);

void handleAnnouncePacket(const uint8_t *payload, uint8_t len) {
  if (len != 5) return;
  uint32_t otherId = ((uint32_t)payload[0]) |
                     ((uint32_t)payload[1] << 8) |
                     ((uint32_t)payload[2] << 16) |
                     ((uint32_t)payload[3] << 24);
  uint8_t otherIdx = payload[4];
  registerNode(otherId, otherIdx);
}

void handleNotePacket(const uint8_t *payload, uint8_t len) {
  if (len != 7) return;
  uint32_t srcId = ((uint32_t)payload[0]) |
                   ((uint32_t)payload[1] << 8) |
                   ((uint32_t)payload[2] << 16) |
                   ((uint32_t)payload[3] << 24);
  uint8_t note = payload[4];
  uint8_t vel  = payload[5];
  uint8_t on   = payload[6];

  if (srcId == myNodeId) return;

  if (on) {
    onRemoteNoteOn(note, vel);
  } else {
    onRemoteNoteOff(note);
  }
}

void handlePingPacket(const uint8_t *payload, uint8_t len) {
  if (len != 1) return;
  uint8_t seq = payload[0];

  uint8_t reply[5];
  reply[0] = (myNodeId & 0xFF);
  reply[1] = (myNodeId >> 8) & 0xFF;
  reply[2] = (myNodeId >> 16) & 0xFF;
  reply[3] = (myNodeId >> 24) & 0xFF;
  reply[4] = seq;
  rs485SendPacket(RS485_PING_REPLY_MSG, reply, 5);
}

void handlePingReplyPacket(const uint8_t *payload, uint8_t len) {
  if (len != 5) return;
  uint32_t srcId = ((uint32_t)payload[0]) |
                   ((uint32_t)payload[1] << 8) |
                   ((uint32_t)payload[2] << 16) |
                   ((uint32_t)payload[3] << 24);
  uint8_t seq = payload[4];

  if (!awaitingPingReplies) return;
  if (seq != pingSequence) return;

  for (uint8_t i = 0; i < nodeList.count; i++) {
    if (nodeList.nodes[i].nodeId == srcId) {
      pingReceived[i] = true;
      break;
    }
  }
}

void handleRemapRequestPacket(const uint8_t *payload, uint8_t len) {
  (void)payload;
  (void)len;
  remapRequested = true;
  networkHealthy = false;
}

void processRs485() {
  while (RS485.available()) {
    uint8_t b = RS485.read();
    switch (rx.state) {
      case 0:
        if (b == 0xAA) {
          rx.state = 1;
          rx.checksum = 0;
        }
        break;
      case 1:
        rx.len = b;
        rx.pos = 0;
        rx.state = 2;
        break;
      case 2:
        rx.type = b;
        rx.checksum = rx.type ^ rx.len;
        rx.state = 3;
        break;
      case 3:
        if (rx.pos < sizeof(rx.buf)) {
          rx.buf[rx.pos++] = b;
          rx.checksum ^= b;
        }
        if (rx.pos >= rx.len) {
          rx.state = 4;
        }
        break;
      case 4:
        if (rx.checksum == b) {
          if (rx.type == RS485_ANNOUNCE_MSG) {
            handleAnnouncePacket(rx.buf, rx.len);
          } else if (rx.type == RS485_NOTE_MSG) {
            handleNotePacket(rx.buf, rx.len);
          } else if (rx.type == RS485_PING_MSG) {
            handlePingPacket(rx.buf, rx.len);
          } else if (rx.type == RS485_PING_REPLY_MSG) {
            handlePingReplyPacket(rx.buf, rx.len);
          } else if (rx.type == RS485_REMAP_REQUEST_MSG) {
            handleRemapRequestPacket(rx.buf, rx.len);
          }
        }
        rx.state = 0;
        break;
    }
  }
}

void sendIdAnnounce(uint8_t boardIndex) {
  uint8_t payload[5];
  payload[0] = (myNodeId & 0xFF);
  payload[1] = (myNodeId >> 8) & 0xFF;
  payload[2] = (myNodeId >> 16) & 0xFF;
  payload[3] = (myNodeId >> 24) & 0xFF;
  payload[4] = boardIndex;
  rs485SendPacket(RS485_ANNOUNCE_MSG, payload, 5);
  registerNode(myNodeId, boardIndex);
}

void broadcastNoteEvent(uint8_t note, uint8_t velocity, bool on) {
  uint8_t payload[7];
  payload[0] = (myNodeId & 0xFF);
  payload[1] = (myNodeId >> 8) & 0xFF;
  payload[2] = (myNodeId >> 16) & 0xFF;
  payload[3] = (myNodeId >> 24) & 0xFF;
  payload[4] = note;
  payload[5] = velocity;
  payload[6] = on ? 1 : 0;
  rs485SendPacket(RS485_NOTE_MSG, payload, sizeof(payload));
}

void sendPing() {
  pingSequence++;
  uint8_t payload[1];
  payload[0] = pingSequence;
  rs485SendPacket(RS485_PING_MSG, payload, 1);

  for (uint8_t i = 0; i < RS485_MAX_NODES; i++) pingReceived[i] = false;
  for (uint8_t i = 0; i < nodeList.count; i++) {
    if (nodeList.nodes[i].nodeId == myNodeId) {
      pingReceived[i] = true;
      break;
    }
  }

  awaitingPingReplies = true;
  pingStartMs = millis();
}

void registerNode(uint32_t nodeId, uint8_t boardIndex) {
  bool found = false;
  bool added = false;
  for (uint8_t i = 0; i < nodeList.count; i++) {
    if (nodeList.nodes[i].nodeId == nodeId) {
      nodeList.nodes[i].boardIndex = boardIndex;
      found = true;
      break;
    }
  }
  if (!found && nodeList.count < RS485_MAX_NODES) {
    nodeList.nodes[nodeList.count].nodeId = nodeId;
    nodeList.nodes[nodeList.count].boardIndex = boardIndex;
    nodeList.count++;
    added = true;
  }

  // New node after discovery: trigger remap from board 0
  if (discoveryDone && added && myBoardIndex == 0) {
    uint8_t dummy = 0;
    rs485SendPacket(RS485_REMAP_REQUEST_MSG, &dummy, 0);
    remapRequested = true;
    networkHealthy = false;
  }
}

// ========================= USB / MIDI ==============================

void midiSendNoteOn(uint8_t note, uint8_t velocity) {
  usb_midi.sendNoteOn(note, velocity, 0);
}

void midiSendNoteOff(uint8_t note, uint8_t velocity) {
  usb_midi.sendNoteOff(note, velocity, 0);
}

// ========================= I2S SYNTH ===============================

void synthNoteOn(uint8_t note, uint8_t velocity) {
  if (!audioEnabled) return;

  float freq = midiNoteToFreq(note);
  float phaseInc = 2.0f * PI * freq / (float)AUDIO_SAMPLE_RATE;
  float amp = (velocity / 127.0f) * MASTER_VOLUME;

  int freeIndex = -1;
  float minAmp = 1e9f;
  int minIndex = 0;
  for (uint8_t i = 0; i < MAX_VOICES; i++) {
    if (!voices[i].active) {
      freeIndex = i;
      break;
    }
    if (voices[i].amplitude < minAmp) {
      minAmp = voices[i].amplitude;
      minIndex = i;
    }
  }
  uint8_t idx = (freeIndex >= 0) ? freeIndex : minIndex;
  voices[idx].active = true;
  voices[idx].note = note;
  voices[idx].phase = 0.0f;
  voices[idx].phaseInc = phaseInc;
  voices[idx].amplitude = amp;
}

void synthNoteOff(uint8_t note) {
  for (uint8_t i = 0; i < MAX_VOICES; i++) {
    if (voices[i].active && voices[i].note == note) {
      voices[i].active = false;
    }
  }
}

void audioGenerateAndWrite() {
  static int16_t buffer[256 * 2];
  const size_t frames = 256;

  for (size_t i = 0; i < frames; i++) {
    float sample = 0.0f;
    for (uint8_t v = 0; v < MAX_VOICES; v++) {
      if (!voices[v].active) continue;
      sample += sinf(voices[v].phase) * voices[v].amplitude;
      voices[v].phase += voices[v].phaseInc;
      if (voices[v].phase > 2.0f * PI) voices[v].phase -= 2.0f * PI;
    }
    sample = constrain(sample, -1.0f, 1.0f);
    int16_t s16 = (int16_t)(sample * 32767.0f);
    buffer[2 * i]     = s16;
    buffer[2 * i + 1] = s16;
  }

  size_t bytesWritten = 0;
  i2s_write(I2S_PORT, buffer, frames * sizeof(int16_t) * 2, &bytesWritten, 0);
}

void audioInit() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pinConfig = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_LRCK,
    .data_out_num = PIN_I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_PORT, &config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pinConfig);
  i2s_set_clk(I2S_PORT, AUDIO_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

  pinMode(PIN_DAC_MUTE_B, OUTPUT);
  digitalWrite(PIN_DAC_MUTE_B, HIGH);
}

// ========================= NOTE HANDLERS ===========================

void onLocalNoteOn(uint8_t note, uint8_t velocity) {
  uint8_t keyIndex = keyIndexFromNote(note);
  if (keyIndex != 0xFF) {
    setKeyLedBase(keyIndex, velocity, true);
  }
  startRipple(note, velocity);

  midiSendNoteOn(note, velocity);
  synthNoteOn(note, velocity);
  buzzerNoteOn(note, velocity);
  broadcastNoteEvent(note, velocity, true);
}

void onLocalNoteOff(uint8_t note) {
  uint8_t keyIndex = keyIndexFromNote(note);
  if (keyIndex != 0xFF) {
    setKeyLedBase(keyIndex, 0, false);
  }

  midiSendNoteOff(note, 0);
  synthNoteOff(note);
  buzzerNoteOff(note);
  broadcastNoteEvent(note, 0, false);
}

void onRemoteNoteOn(uint8_t note, uint8_t velocity) {
  uint8_t keyIndex = keyIndexFromNote(note);
  if (keyIndex != 0xFF) {
    setKeyLedBase(keyIndex, velocity, true);
  }
  startRipple(note, velocity);

  midiSendNoteOn(note, velocity);
  synthNoteOn(note, velocity);
}

void onRemoteNoteOff(uint8_t note) {
  uint8_t keyIndex = keyIndexFromNote(note);
  if (keyIndex != 0xFF) {
    setKeyLedBase(keyIndex, 0, false);
  }

  midiSendNoteOff(note, 0);
  synthNoteOff(note);
}

// ========================= EXTRA KEYS / MODES ======================

void handleExtraKeyPressed(uint8_t extraIndex) {
  switch (extraIndex) {
    case 0: // toggle buzzer
      buzzerEnabled = !buzzerEnabled;
      break;
    case 1: // toggle audio
      audioEnabled = !audioEnabled;
      break;
    case 2: { // manual remap
      uint8_t dummy = 0;
      rs485SendPacket(RS485_REMAP_REQUEST_MSG, &dummy, 0);
      remapRequested = true;
      networkHealthy = false;
      break;
    }
    case 3: // octave up
      boardBaseNote += 12;
      break;
    case 4: // octave down
      boardBaseNote -= 12;
      break;
    default:
      break;
  }
}

// ========================= ENCODER ================================

void IRAM_ATTR encoderISR() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t state = (a << 1) | b;
  if ((encoderLastState == 0b00 && state == 0b01) ||
      (encoderLastState == 0b01 && state == 0b11) ||
      (encoderLastState == 0b11 && state == 0b10) ||
      (encoderLastState == 0b10 && state == 0b00)) {
    encoderValue++;
  } else if ((encoderLastState == 0b00 && state == 0b10) ||
             (encoderLastState == 0b10 && state == 0b11) ||
             (encoderLastState == 0b11 && state == 0b01) ||
             (encoderLastState == 0b01 && state == 0b00)) {
    encoderValue--;
  }
  encoderValue = constrain(encoderValue, ENCODER_MIN, ENCODER_MAX);
  encoderLastState = state;
}

void encoderInit() {
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  encoderLastState = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

  pinMode(PIN_ENC_SW, INPUT_PULLUP);
}

// Encoder button toggles buzzer mode
void handleEncoderButton() {
  static bool prev = HIGH;
  bool cur = digitalRead(PIN_ENC_SW);
  if (prev == HIGH && cur == LOW) {
    buzzerEnabled = !buzzerEnabled;
  }
  prev = cur;
}

// ========================= PHYSICAL CHAIN DISCOVERY ===============

void pulseRightNeighbor() {
  digitalWrite(PIN_NEIGHBOR_RIGHT, LOW);
  delay(3);
  digitalWrite(PIN_NEIGHBOR_RIGHT, HIGH);
}

void startDiscovery(bool fromRemap) {
  (void)fromRemap;
  discoveryDone = false;
  isRemapping = true;
  networkHealthy = false;

  myBoardIndex = 0xFF;
  nodeList.count = 0;

  for (uint8_t i = 0; i < TOTAL_KEYS; i++) {
    keyBaseVelocity[i] = 0;
  }

  pinMode(PIN_NEIGHBOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_NEIGHBOR_RIGHT, OUTPUT);
  digitalWrite(PIN_NEIGHBOR_RIGHT, LOW);
  delay(5);
  hasLeftNeighbor = (digitalRead(PIN_NEIGHBOR_LEFT) == LOW);
  bool isRoot = !hasLeftNeighbor;

  digitalWrite(PIN_NEIGHBOR_RIGHT, HIGH);
  delay(2);

  uint32_t tStart = millis();

  if (isRoot) {
    myBoardIndex = 0;
    sendIdAnnounce(myBoardIndex);
    pulseRightNeighbor();
  }

  int leftPrev = digitalRead(PIN_NEIGHBOR_LEFT);

  while (millis() - tStart < DISCOVERY_MAX_MS) {
    processRs485();

    int leftNow = digitalRead(PIN_NEIGHBOR_LEFT);
    if (leftPrev == HIGH && leftNow == LOW && myBoardIndex == 0xFF) {
      uint8_t maxIdx = (myBoardIndex == 0xFF) ? 0 : myBoardIndex;
      for (uint8_t i = 0; i < nodeList.count; i++) {
        uint8_t idx = nodeList.nodes[i].boardIndex;
        if (idx != 0xFF && idx > maxIdx) {
          maxIdx = idx;
        }
      }
      uint8_t newIdx = maxIdx + 1;
      myBoardIndex = newIdx;
      sendIdAnnounce(myBoardIndex);
      pulseRightNeighbor();
    }
    leftPrev = leftNow;

    if (myBoardIndex != 0xFF && (millis() - tStart) > 100) {
      break;
    }

    delay(1);
  }

  uint8_t maxIdx = (myBoardIndex == 0xFF) ? 0 : myBoardIndex;
  for (uint8_t i = 0; i < nodeList.count; i++) {
    if (nodeList.nodes[i].boardIndex != 0xFF &&
        nodeList.nodes[i].boardIndex > maxIdx) {
      maxIdx = nodeList.nodes[i].boardIndex;
    }
  }
  totalBoards = maxIdx + 1;

  if (myBoardIndex == 0xFF) {
    myBoardIndex = 0;
  }

  boardBaseNote = GLOBAL_BASE_NOTE + myBoardIndex * BOARD_NOTE_SPAN;

  discoveryDone = true;
  isRemapping = false;
  networkHealthy = true;
}

// ========================= SETUP & LOOP ============================

void setup() {
  Serial.begin(115200);

  // USB MIDI via TinyUSB
  TinyUSBDevice.begin();
  usb_midi.begin();

  // Mux pins
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);

  // RS-485
  pinMode(PIN_RS485_MODE, OUTPUT);
  rs485SetTx(false);
  RS485.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);

  // Analog
  analogReadResolution(12);

  // LEDs
  leds.begin();
  leds.clear();
  leds.show();

  // Buzzer pin & timer
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  for (int i = 0; i < BUZZER_MAX_VOICES; i++) {
    buzzerVoices[i].active = false;
  }
  buzzerTimer = timerBegin(0, 80, true); // 1 MHz
  timerAttachInterrupt(buzzerTimer, &onBuzzerTimer, true);
  timerAlarmWrite(buzzerTimer, 50, true); // 50us => 20kHz
  timerAlarmEnable(buzzerTimer);

  // Keys
  initKeyDefs();
  calibrateKeys();

  // Encoder
  encoderInit();

  // I2S audio
  audioInit();

  // Node ID
  myNodeId = readRandom32();
  nodeList.count = 0;

  // Ping state
  awaitingPingReplies = false;
  lastPingMs = millis();

  // Startup polish
  showAllLedsStartup();
  playStartupJingle();

  // Initial discovery
  startDiscovery(false);
}

void loop() {
  TinyUSBDevice.task();

  if (remapRequested) {
    remapRequested = false;
    startDiscovery(true);
  }

  processRs485();

  scanKeys();
  handleEncoderButton();

  audioGenerateAndWrite();

  uint32_t nowMs = millis();
  if (myBoardIndex == 0 && totalBoards > 1) {
    if (!awaitingPingReplies && (nowMs - lastPingMs > PING_INTERVAL_MS)) {
      sendPing();
      lastPingMs = nowMs;
    } else if (awaitingPingReplies && (nowMs - pingStartMs > PING_TIMEOUT_MS)) {
      bool missing = false;
      for (uint8_t i = 0; i < nodeList.count; i++) {
        if (!pingReceived[i]) {
          missing = true;
          break;
        }
      }
      awaitingPingReplies = false;
      if (missing) {
        networkHealthy = false;
        uint8_t dummy = 0;
        rs485SendPacket(RS485_REMAP_REQUEST_MSG, &dummy, 0);
        remapRequested = true;
      } else {
        networkHealthy = true;
      }
    }
  }

  renderLeds();

  delay(2);
}
