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
      * Uses ESP32 USBMIDI / TinyUSB when USB-OTG mode is selected
      * Any board plugged into USB outputs MIDI for the whole keyboard

  Neighbor wiring assumption:
    - IO40 = LEFT neighbor signal (INPUT_PULLUP).
    - IO42 = RIGHT neighbor signal (OUTPUT).
    - Each board’s IO42 connects to the next board’s IO40 on its right.
*/

#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32) && __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#if defined(ARDUINO_ARCH_ESP32) && __has_include(<soc/soc_caps.h>)
#include <soc/soc_caps.h>
#endif
#include <Adafruit_NeoPixel.h>
#include <driver/i2s.h>
#include <HardwareSerial.h>
#include <math.h>
#include "driver/gpio.h"
#include "FractureSongSystem.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(SOC_USB_OTG_SUPPORTED) && SOC_USB_OTG_SUPPORTED && defined(ARDUINO_USB_MODE) && !ARDUINO_USB_MODE
#include "USB.h"
#include <USBMIDI.h>
#if defined(CONFIG_TINYUSB_MIDI_ENABLED) && CONFIG_TINYUSB_MIDI_ENABLED
#define FRACTURE_USB_DEVICE_ENABLED 1
#define FRACTURE_USB_MIDI_ENABLED 1
#else
#error "Fracture requires TinyUSB MIDI support in the selected ESP32 board configuration."
#endif
#else
#define FRACTURE_USB_DEVICE_ENABLED 0
#define FRACTURE_USB_MIDI_ENABLED 0
#endif

#if defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE
#error "Set Tools > USB Mode to USB-OTG (TinyUSB) for Fracture MIDI."
#endif
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#error "Disable USB CDC On Boot; it starts TinyUSB before Fracture can select MIDI and song-loader MSC."
#endif
#if defined(ARDUINO_USB_MSC_ON_BOOT) && ARDUINO_USB_MSC_ON_BOOT
#error "Disable USB Firmware MSC On Boot; Fracture supplies its own song-loader MSC."
#endif
#if defined(ARDUINO_USB_DFU_ON_BOOT) && ARDUINO_USB_DFU_ON_BOOT
#error "Disable USB DFU On Boot; Fracture must be the first code to start TinyUSB."
#endif

#ifndef FRACTURE_ENABLE_I2S_AUDIO
#define FRACTURE_ENABLE_I2S_AUDIO 0
#endif

#ifndef FRACTURE_ENABLE_EXTRA_KEYS
#define FRACTURE_ENABLE_EXTRA_KEYS 1
#endif

#ifndef FRACTURE_ENABLE_BUZZER_SYNTH
#define FRACTURE_ENABLE_BUZZER_SYNTH 1
#endif
// ========================= PIN DEFINITIONS =========================

// Physical neighbor chain pins
#define PIN_NEIGHBOR_LEFT    40   // IO40: left neighbor signal (input)
#define PIN_NEIGHBOR_RIGHT   42   // IO42: right neighbor signal (output)

// Encoder
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
const uint8_t  RS485_CONTROL_MSG       = 0x06; // nodeId + command + value
const uint8_t  RS485_REMAP_FINAL_MSG   = 0x07; // generation + board count

const uint8_t  CONTROL_BUZZER_ENABLED  = 0x01;
const uint8_t  CONTROL_OCTAVE_OFFSET   = 0x02;
const uint8_t  CONTROL_AUDIO_ENABLED   = 0x03;
const uint8_t  CONTROL_SIGNED_BIAS     = 64;

// Discovery / mapping
const uint32_t DISCOVERY_MAX_MS        = 1000; // total time budget for chain setup
const uint32_t REMAP_VISUAL_MS         = 900;  // visible refresh flash after discovery
const uint32_t DISCOVERY_NEIGHBOR_SETTLE_MS = 35; // hold neighbor detect state long enough to synchronize
const uint32_t DISCOVERY_RETRY_MS      = 80;   // repeat the left-to-right discovery token
const uint32_t DISCOVERY_PARENT_ANNOUNCE_MS = 30; // announce-to-pulse pairing window
const uint32_t REMAP_START_DELAY_MS   = 140;  // let every board enter refresh before discovery
const uint8_t  REMAP_REQUEST_REPEATS  = 3;

// Velocity sensing
const uint16_t VELOCITY_ON_DELTA     = 80;    // change from baseline to detect press start
const uint16_t VELOCITY_FULL_DELTA   = 120;   // lower threshold keeps weaker Hall sensors playable
const uint16_t VELOCITY_MIN_DT_US    = 1000;  // very fast strike
const uint32_t VELOCITY_MAX_DT_US    = 60000; // very slow strike
const uint8_t  KEY_ADC_AVERAGE_SAMPLES = 4;
const uint8_t  KEY_PRESS_DEBOUNCE_SCANS = 3;
const uint8_t  KEY_RELEASE_DEBOUNCE_SCANS = 2;
const uint8_t  KEY_NOTE_ON_MAX_SCANS = 6;
const uint16_t KEY_MUX_SETTLE_US = 80;
const uint16_t KEY_ADC_SETTLE_US = 20;
const uint32_t AUX_HOLD_US = 220000UL;
const uint32_t SETTINGS_AFTER_KEYS_COOLDOWN_MS = 1200;

// Audio synth (I2S)
const i2s_port_t I2S_PORT = I2S_NUM_0;
const uint32_t AUDIO_SAMPLE_RATE = 44100;
const uint8_t MAX_VOICES = 8;
const float MASTER_VOLUME = 0.4f;

// Buzzer synth
const uint8_t  BUZZER_PWM_RESOLUTION = 10;
const uint16_t BUZZER_PWM_DUTY = 256;               // 25% duty is less harsh than 50%
const int8_t   BUZZER_NOTE_TRANSPOSE = 12;          // small buzzers speak cleaner one octave up
const uint8_t  BUZZER_PWM_CHANNEL = 0;               // Arduino-ESP32 2.x LEDC channel
const int8_t   OCTAVE_SHIFT_MIN = -4;                // C3 down to MIDI note 0
const int8_t   OCTAVE_SHIFT_MAX = 4;                 // one-board upper limit

// Encoder
const int8_t ENCODER_EDGES_PER_DETENT = 2;           // EC11E: 9 pulses / 18 detents

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define FRACTURE_LEDC_PIN_API 1
#else
#define FRACTURE_LEDC_PIN_API 0
#endif

// Ping / remap
const uint32_t PING_INTERVAL_MS = 1000;
const uint32_t PING_TIMEOUT_MS  = 200;

// Ripple (LED splash)
const float RIPPLE_SPEED = 0.018f; // wave radius in semitones/ms
const uint8_t MAX_RIPPLES = 8;
const uint8_t STATUS_ROWS = 4;
const uint8_t STATUS_COLS = 3;
const uint16_t STATUS_FIRST_LED = 32;
const uint32_t STATUS_FADE_MS = 147;
const uint8_t STATUS_DISPLAY_BRIGHTNESS_SCALE = 128;

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
  bool     extraHandled = false;
  bool     extraSuppressed = false;
  uint8_t  aboveCount = 0;
  uint8_t  belowCount = 0;
  uint32_t pressStartUs = 0;
  uint8_t  velocity = 0;
  uint8_t  activeNote = 0;
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


// ========================= FORWARD DECLARATIONS ====================

void onLocalNoteOn(uint8_t note, uint8_t velocity);
void onLocalNoteOff(uint8_t note);
void onRemoteNoteOn(uint8_t note, uint8_t velocity, uint8_t sourceBoard);
void onRemoteNoteOff(uint8_t note);
void handleExtraKeyPressed(uint8_t extraIndex);
void rs485SendPacket(uint8_t type, const uint8_t *payload, uint8_t len);
void startDiscovery(bool fromRemap);
void renderLeds();
void buzzerApplySelectedNote();
void buzzerStopAllInternal();
void setSharedBuzzerEnabled(bool enabled, bool broadcast);
void setSharedOctave(int8_t octave, bool broadcast);
void broadcastSharedState();
uint8_t midiNoteToBoardOffset(uint8_t note);

// ========================= GLOBALS =================================

Adafruit_NeoPixel leds(NUM_LEDS, PIN_LED_DATA, NEO_GRB + NEO_KHZ800);

// Register MIDI before setup so TinyUSB always sees it before USB.begin().
#if FRACTURE_USB_MIDI_ENABLED
USBMIDI usbMidi("Fracture Keyboard");
#endif

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
bool remapScheduled = false;
uint32_t remapStartAtMs = 0;
uint16_t currentRemapToken = 0;
uint16_t lastCompletedRemapToken = 0xFFFF;
uint16_t localRemapCounter = 0;
uint8_t lastDiscoveryAnnounceIndex = 0xFF;
uint32_t lastDiscoveryAnnounceMs = 0;

// Network state
bool isRemapping     = false;
bool networkHealthy  = false;
uint32_t lastMainKeyActivityMs = 0;
uint32_t networkRefreshEffectUntilMs = 0;
char statusGlyphCurrent = 0;
char statusGlyphPrevious = 0;
uint32_t statusGlyphTransitionMs = 0;
uint32_t statusGlyphColor = 0;
uint32_t statusGlyphPreviousColor = 0;

// Note mapping
int16_t boardBaseNote = GLOBAL_BASE_NOTE;
int8_t octaveOffset = 0; // shared semitone mapping in one-octave steps

// Synth (I2S)
Voice voices[MAX_VOICES];

// Encoder
volatile int16_t encoderPendingEdges = 0;
int16_t encoderEdgeAccumulator = 0;
uint8_t encoderLastState = 0;
uint8_t encoderLastRoute = 0;

// Modes
bool buzzerEnabled = false;
bool audioEnabled  = FRACTURE_ENABLE_I2S_AUDIO;
bool audioReady    = false;

// Neighbor info
bool hasLeftNeighbor = false;

// RS-485 RX state
RxState rx;

// Ripples + key LED base velocities
Ripple ripples[MAX_RIPPLES];
uint8_t keyBaseVelocity[TOTAL_KEYS]; // 0..127
uint32_t keyBaseStartMs[TOTAL_KEYS];

// Ping
uint8_t pingSequence = 0;
bool awaitingPingReplies = false;
uint32_t pingStartMs = 0;
uint32_t lastPingMs = 0;
bool pingReceived[RS485_MAX_NODES];

// Buzzer synth
bool buzzerPwmReady = false;
uint8_t buzzerNoteCount[128];
uint32_t buzzerNoteOrder[128];
uint8_t buzzerNoteSourceBoard[128];
uint32_t buzzerOrderCounter = 0;
portMUX_TYPE buzzerMux = portMUX_INITIALIZER_UNLOCKED;

// ========================= UTILITY HELPERS =========================

void bootLog(const char *message) {
  Serial.println(message);
  Serial.flush();
}

float midiNoteToFreq(uint8_t note) {
  return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

int8_t minSharedOctave() {
  return OCTAVE_SHIFT_MIN;
}

int8_t maxSharedOctave() {
  int8_t span = totalBoards == 0 ? 1 : (int8_t)totalBoards;
  int8_t maxOctave = OCTAVE_SHIFT_MAX - 2 * (span - 1);
  return maxOctave < OCTAVE_SHIFT_MIN ? OCTAVE_SHIFT_MIN : maxOctave;
}

int8_t clampOctave(int8_t octave) {
  int8_t minimum = minSharedOctave();
  int8_t maximum = maxSharedOctave();
  if (octave < minimum) return minimum;
  if (octave > maximum) return maximum;
  return octave;
}

void updateBoardBaseNote() {
  uint8_t index = (myBoardIndex == 0xFF) ? 0 : myBoardIndex;
  boardBaseNote = (int16_t)GLOBAL_BASE_NOTE +
                  (int16_t)octaveOffset * 12 + (int16_t)index * BOARD_NOTE_SPAN;
}

bool applyOctaveLocal(int8_t octave) {
  int8_t clamped = clampOctave(octave);
  bool changed = clamped != octaveOffset;
  octaveOffset = clamped;
  updateBoardBaseNote();
  return changed;
}

uint8_t localNoteForMainKey(uint8_t keyIndex) {
  int16_t note = boardBaseNote + keyToNote[keyIndex];
  if (note < 0) return 0;
  if (note > 127) return 127;
  return (uint8_t)note;
}

void triggerNetworkRefreshEffect() {
  networkRefreshEffectUntilMs = millis() + REMAP_VISUAL_MS;
}

void finishNetworkRefreshVisual() {
  networkRefreshEffectUntilMs = 0;
  statusGlyphCurrent = 0;
  statusGlyphPrevious = 0;
  statusGlyphTransitionMs = 0;
}

void scheduleNetworkRemap(uint16_t token) {
  if (discoveryDone && token == lastCompletedRemapToken) return;
  if (token == currentRemapToken && (remapScheduled || isRemapping)) return;
  currentRemapToken = token;
  remapScheduled = true;
  remapStartAtMs = millis() + REMAP_START_DELAY_MS;
  isRemapping = true;
  networkHealthy = false;
  triggerNetworkRefreshEffect();
}

void requestNetworkRemap() {
  uint16_t token = (uint16_t)(++localRemapCounter ^ (uint16_t)millis() ^
                              (uint16_t)(myNodeId >> 16));
  if (token == lastCompletedRemapToken) ++token;
  scheduleNetworkRemap(token);

  uint8_t payload[6];
  payload[0] = myNodeId & 0xFF;
  payload[1] = (myNodeId >> 8) & 0xFF;
  payload[2] = (myNodeId >> 16) & 0xFF;
  payload[3] = (myNodeId >> 24) & 0xFF;
  payload[4] = token & 0xFF;
  payload[5] = (token >> 8) & 0xFF;
  for (uint8_t attempt = 0; attempt < REMAP_REQUEST_REPEATS; ++attempt) {
    rs485SendPacket(RS485_REMAP_REQUEST_MSG, payload, sizeof(payload));
    if (attempt + 1 < REMAP_REQUEST_REPEATS) delay(4);
  }
}

bool buzzerPwmAttach(uint8_t pin, uint32_t freq, uint8_t resolution) {
#if FRACTURE_LEDC_PIN_API
  return ledcAttach(pin, freq, resolution);
#else
  if (ledcSetup(BUZZER_PWM_CHANNEL, freq, resolution) <= 0) return false;
  ledcAttachPin(pin, BUZZER_PWM_CHANNEL);
  return true;
#endif
}

void buzzerPwmWrite(uint32_t duty) {
#if FRACTURE_LEDC_PIN_API
  ledcWrite(PIN_BUZZER, duty);
#else
  ledcWrite(BUZZER_PWM_CHANNEL, duty);
#endif
}

void buzzerPwmChangeFrequency(uint32_t freq) {
#if FRACTURE_LEDC_PIN_API
  ledcChangeFrequency(PIN_BUZZER, freq, BUZZER_PWM_RESOLUTION);
#else
  ledcChangeFrequency(BUZZER_PWM_CHANNEL, freq, BUZZER_PWM_RESOLUTION);
#endif
}

// Given a note, find local main key index that maps to it, or 0xFF if not on this board
uint8_t keyIndexFromNote(uint8_t note) {
  int16_t offset = (int16_t)note - boardBaseNote;
  if (offset < 0 || offset >= BOARD_NOTE_SPAN) return 0xFF;
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
  delayMicroseconds(KEY_MUX_SETTLE_US);
}

uint16_t readKeyRaw(const KeyDef &def) {
  setMuxChannel(def.muxChannel);
  int pin = (def.muxIndex == 0) ? PIN_AM0 : PIN_AM1;
  analogRead(pin);
  analogRead(pin);
  delayMicroseconds(KEY_ADC_SETTLE_US);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < KEY_ADC_AVERAGE_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(10);
  }
  return sum / KEY_ADC_AVERAGE_SAMPLES;
}

/*
  Physical mapping confirmed from PCB/user notes:

  - S1-S8   / AM0:0-7          => aux keys, LEDs 1-8
  - S9-S18  / AM0:8-15+AM1:0-1 => black keys, LEDs 9-18
  - S19-S32 / AM1:2-15         => white keys, LEDs 19-32

  keyDefs[] keeps musical keys first for keyToNote[], then aux keys.
  Use physicalLedIndexForKeyIndex() whenever touching the LED strip.
*/
void initKeyDefs() {
  // Aux switches S1-S8: logical indices 24-31 on AM0:0-7
  for (uint8_t i = 0; i < NUM_EXTRA_KEYS; i++) {
    uint8_t keyIndex = NUM_MAIN_KEYS + i; // logical aux 24-31
    keyDefs[keyIndex].muxIndex   = 0;     // AM0
    keyDefs[keyIndex].muxChannel = i;     // physical S1-S8
    keyDefs[keyIndex].isExtra    = true;
  }

  // Black switches S9-S16: main key indices 0-7 on AM0:8-15
  for (uint8_t i = 0; i < 8; i++) {
    keyDefs[i].muxIndex   = 0;        // AM0
    keyDefs[i].muxChannel = 8 + i;    // physical S9-S16
    keyDefs[i].isExtra    = false;
  }

  // Black switches S17-S18, then white switches S19-S32: main indices 8-23 on AM1:0-15
  for (uint8_t i = 0; i < 16; i++) {
    uint8_t keyIndex = 8 + i;        // logical main 8-23
    keyDefs[keyIndex].muxIndex   = 1; // AM1
    keyDefs[keyIndex].muxChannel = i; // physical S17-S32
    keyDefs[keyIndex].isExtra    = false;
  }
}

void calibrateKeys() {
  const uint8_t samples = 32;
  for (uint8_t k = 0; k < TOTAL_KEYS; k++) {
    uint32_t sum = 0;
    for (uint8_t s = 0; s < samples; s++) {
      sum += readKeyRaw(keyDefs[k]);
      delay(1);
    }
    keyStates[k].baseline = (uint16_t)(sum / samples);
    keyStates[k].raw = keyStates[k].baseline;
    keyStates[k].pressed = false;
    keyStates[k].velocityComputed = false;
    keyStates[k].extraHandled = false;
    keyStates[k].extraSuppressed = false;
    keyStates[k].activeNote = 0;
    keyStates[k].aboveCount = 0;
    keyStates[k].belowCount = 0;
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

bool anyMainKeyActive() {
  for (uint8_t i = 0; i < NUM_MAIN_KEYS; i++) {
    if (keyStates[i].velocityComputed) return true;
  }
  return false;
}

bool settingsChangesAllowed() {
  return !anyMainKeyActive() && (millis() - lastMainKeyActivityMs) >= SETTINGS_AFTER_KEYS_COOLDOWN_MS;
}

bool songModeImmediateExtraKey(uint8_t keyIndex) {
  if (!FractureSongs::isActive() || keyIndex < NUM_MAIN_KEYS) return false;
  uint8_t extraIndex = keyIndex - NUM_MAIN_KEYS;
  return extraIndex == 0 || extraIndex >= 6;
}

uint32_t extraKeyHoldUs(uint8_t keyIndex) {
  (void)keyIndex;
  return FractureSongs::isActive() ? (AUX_HOLD_US / 2) : AUX_HOLD_US;
}

bool extraKeyActionAllowed(uint8_t keyIndex) {
  if (FractureSongs::isActive()) return true;
  if (songModeImmediateExtraKey(keyIndex)) return true;
  return settingsChangesAllowed();
}

void scanKeys() {
  const uint16_t releaseDelta = VELOCITY_ON_DELTA / 2;

  for (uint8_t k = 0; k < TOTAL_KEYS; k++) {
    KeyDef &def = keyDefs[k];
    KeyState &st = keyStates[k];

    uint16_t raw = readKeyRaw(def);
    st.raw = raw;
    int16_t travel = (int16_t)st.baseline - (int16_t)raw;
    uint16_t delta = travel > 0 ? (uint16_t)travel : 0;
    uint32_t nowUs = micros();
    uint32_t nowMs = millis();

    if (!def.isExtra && st.velocityComputed) {
      lastMainKeyActivityMs = nowMs;
    }

    if (delta >= VELOCITY_ON_DELTA) {
      if (st.aboveCount == 0 && !st.pressed) {
        st.pressStartUs = nowUs;
      }
      if (st.aboveCount < 255) st.aboveCount++;
      st.belowCount = 0;
    } else {
      if (!st.pressed) {
        st.aboveCount = 0;
      }
      if (delta <= releaseDelta) {
        if (st.belowCount < 255) st.belowCount++;
        st.aboveCount = 0;
      } else if (st.pressed) {
        st.belowCount = 0;
      }
    }

    if (!st.pressed && st.aboveCount >= KEY_PRESS_DEBOUNCE_SCANS) {
      st.pressed = true;
      st.velocityComputed = false;
      if (def.isExtra) {
        st.extraSuppressed = !extraKeyActionAllowed(k);
      }
    }

    if (st.pressed && !st.velocityComputed &&
        (delta >= VELOCITY_FULL_DELTA || st.aboveCount >= KEY_NOTE_ON_MAX_SCANS)) {
      uint32_t dtUs = nowUs - st.pressStartUs;
      st.velocity = computeVelocity(delta, dtUs);
      st.velocityComputed = true;

      if (def.isExtra) {
        setKeyLedBase(k, st.velocity, true);
      } else {
        uint8_t note = localNoteForMainKey(k);
        uint8_t vel = st.velocity;
        if (vel == 0) vel = 1;
        st.activeNote = note;
        onLocalNoteOn(note, vel);
      }
    }

    if (def.isExtra && st.pressed && st.velocityComputed && !st.extraHandled &&
        (songModeImmediateExtraKey(k) || (nowUs - st.pressStartUs) >= extraKeyHoldUs(k)) &&
        !st.extraSuppressed && extraKeyActionAllowed(k)) {
      uint8_t extraIndex = k - NUM_MAIN_KEYS;
      handleExtraKeyPressed(extraIndex);
      st.extraHandled = true;
      setKeyLedBase(k, 127, true);
    }
    if (st.pressed && st.belowCount >= KEY_RELEASE_DEBOUNCE_SCANS) {
      if (st.velocityComputed) {
        if (def.isExtra) {
          setKeyLedBase(k, 0, false);
        } else {
          uint8_t note = st.activeNote;
          onLocalNoteOff(note);
          st.activeNote = 0;
        }
      }
      st.pressed = false;
      st.velocityComputed = false;
      st.extraHandled = false;
      st.extraSuppressed = false;
      st.velocity = 0;
      st.aboveCount = 0;
      st.belowCount = 0;
    }
  }
}
// ========================= LED HANDLING ============================

uint8_t physicalSwitchForKeyIndex(uint8_t keyIndex) {
  if (keyIndex < NUM_MAIN_KEYS) {
    return keyIndex + NUM_EXTRA_KEYS + 1; // main key 0 is physical S9
  }
  if (keyIndex < TOTAL_KEYS) {
    return keyIndex - NUM_MAIN_KEYS + 1; // aux key 0 is physical S1
  }
  return 0;
}

uint8_t physicalLedIndexForSwitch(uint8_t switchNumber) {
  if (switchNumber >= 1 && switchNumber <= 15) {
    return switchNumber - 1;
  }
  if (switchNumber >= 16 && switchNumber <= 30) {
    return 45 - switchNumber; // S16-S30 are wired to LEDs 30-16
  }
  if (switchNumber >= 31 && switchNumber <= 32) {
    return switchNumber - 1;
  }
  return 0xFF;
}

uint8_t physicalLedIndexForKeyIndex(uint8_t keyIndex) {
  return physicalLedIndexForSwitch(physicalSwitchForKeyIndex(keyIndex));
}

uint8_t physicalSwitchForLedIndex(uint8_t ledIndex) {
  uint8_t ledNumber = ledIndex + 1;
  if (ledNumber >= 1 && ledNumber <= 15) {
    return ledNumber;
  }
  if (ledNumber >= 16 && ledNumber <= 30) {
    return 46 - ledNumber;
  }
  if (ledNumber >= 31 && ledNumber <= 32) {
    return ledNumber;
  }
  return 0;
}

int8_t mainKeyIndexForPhysicalLed(uint8_t ledIndex) {
  uint8_t switchNumber = physicalSwitchForLedIndex(ledIndex);
  if (switchNumber >= 9 && switchNumber <= 32) {
    return (int8_t)(switchNumber - 9);
  }
  return -1;
}

void setKeyLedBase(uint8_t keyIndex, uint8_t velocity, bool on) {
  uint8_t ledIndex = physicalLedIndexForKeyIndex(keyIndex);
  if (ledIndex >= TOTAL_KEYS) return;
  keyBaseVelocity[ledIndex] = on ? velocity : 0;
  keyBaseStartMs[ledIndex] = on ? millis() : 0;
}

void startRipple(uint8_t note, uint8_t velocity) {
  uint8_t slot = 0;
  uint32_t oldestAge = 0;
  uint32_t nowMs = millis();

  for (uint8_t i = 0; i < MAX_RIPPLES; i++) {
    if (!ripples[i].active) {
      slot = i;
      oldestAge = 0xFFFFFFFFUL;
      break;
    }
    uint32_t age = nowMs - ripples[i].startMs;
    if (age >= oldestAge) {
      oldestAge = age;
      slot = i;
    }
  }

  ripples[slot].active = true;
  ripples[slot].centerNote = (float)note;
  ripples[slot].radius = 0.0f;
  ripples[slot].maxRadius = 28.0f;
  ripples[slot].startMs = nowMs;
  ripples[slot].baseVelocity = (velocity > 0 ? velocity : 80);
}

uint16_t statusLedIndex(uint8_t row, uint8_t col) {
  return STATUS_FIRST_LED + row * STATUS_COLS + col;
}

struct StatusGlyphDef {
  char glyph;
  uint16_t mask;
};

const StatusGlyphDef STATUS_GLYPHS[] = {
  {'A', 0b111101111101},
  {'B', 0b110111101111},
  {'C', 0b111100100111},
  {'D', 0b110101101110},
  {'E', 0b111100110111},
  {'F', 0b111100110100},
  {'G', 0b111100101111},
  {'H', 0b101111101101},
  {'I', 0b111010010111},
  {'J', 0b011001101010},
  {'K', 0b101110101101},
  {'L', 0b100100100111},
  {'M', 0b111111101101},
  {'N', 0b111101101101},
  {'O', 0b111101101111},
  {'P', 0b111101111100},
  {'Q', 0b010101101011},
  {'R', 0b110101110101},
  {'S', 0b110100010110},
  {'T', 0b111010010010},
  {'U', 0b101101101111},
  {'V', 0b101101101010},
  {'W', 0b101101111111},
  {'X', 0b101010101101},
  {'Y', 0b101111010010},
  {'Z', 0b110010100110},
  {'a', 0b000011101011},
  {'b', 0b100110101110},
  {'c', 0b000110100110},
  {'d', 0b001011101011},
  {'e', 0b000111101110},
  {'f', 0b010100110100},
  {'g', 0b110110010110},
  {'h', 0b100110101101},
  {'i', 0b010000010010},
  {'j', 0b010010010100},
  {'k', 0b100101110101},
  {'l', 0b100100100010},
  {'m', 0b000111111101},
  {'n', 0b000110101101},
  {'o', 0b000111101111},
  {'p', 0b110101110100},
  {'q', 0b011101011001},
  {'r', 0b000110100100},
  {'s', 0b000011010110},
  {'t', 0b010111010011},
  {'u', 0b000101101111},
  {'v', 0b000101101010},
  {'w', 0b000101111111},
  {'x', 0b000101010101},
  {'y', 0b101011001010},
  {'z', 0b000110010011},
  {'0', 0b010101101010},
  {'1', 0b010110010010},
  {'2', 0b110001110111},
  {'3', 0b110001011111},
  {'4', 0b101101111001},
  {'5', 0b111100011110},
  {'6', 0b100111101011},
  {'7', 0b111001001001},
  {'8', 0b010111101111},
  {'9', 0b111111001110},
  {'(', 0b010100100010},
  {')', 0b100010010100},
  {'[', 0b110100100110},
  {']', 0b110010010110},
  {'+', 0b000010111010},
  {'-', 0b000000111000},
  {':', 0b000010000010},
  {'=', 0b000111000111},
  {'/', 0b010010100100},
  {'\\', 0b100011110001},
  {'{', 0b011010110011},
  {'}', 0b110010011110},
  {'?', 0b110001011010},
  {'!', 0b010010000010},
  {'.', 0b000000000010},
  {',', 0b000000010010},
  {';', 0b000010010100},
  {'\'', 0b010010000000},
  {'"', 0b101101000000},
  {'*', 0b010101010000},
};

const uint8_t STATUS_GLYPH_COUNT = sizeof(STATUS_GLYPHS) / sizeof(STATUS_GLYPHS[0]);

uint16_t statusGlyphMask(char glyph) {
  for (uint8_t i = 0; i < STATUS_GLYPH_COUNT; i++) {
    if (STATUS_GLYPHS[i].glyph == glyph) {
      return STATUS_GLYPHS[i].mask;
    }
  }
  return 0;
}

char octaveStatusGlyph() {
  int16_t displayedOctave = (int16_t)GLOBAL_BASE_NOTE / 12 - 1 + octaveOffset;
  if (myBoardIndex != 0xFF) {
    displayedOctave += (int16_t)myBoardIndex * 2;
  }
  if (displayedOctave < 0) return 'L';
  if (displayedOctave <= 9) return (char)('0' + displayedOctave);
  return 'A';
}

uint32_t scaleColor(uint32_t color, uint8_t scale) {
  uint8_t r = ((color >> 16) & 0xFF) * scale / 255;
  uint8_t g = ((color >> 8) & 0xFF) * scale / 255;
  uint8_t b = (color & 0xFF) * scale / 255;
  return leds.Color(r, g, b);
}

void drawStatusGlyph(char glyph, uint32_t color) {
  uint16_t mask = statusGlyphMask(glyph);
  uint32_t displayColor = scaleColor(color, STATUS_DISPLAY_BRIGHTNESS_SCALE);
  for (uint8_t row = 0; row < STATUS_ROWS; row++) {
    for (uint8_t col = 0; col < STATUS_COLS; col++) {
      uint8_t bit = row * STATUS_COLS + col;
      if (mask & (1 << (11 - bit))) {
        leds.setPixelColor(statusLedIndex(row, col), displayColor);
      }
    }
  }
}

void updateStatusLeds() {
  for (uint8_t i = 0; i < 12; i++) {
    leds.setPixelColor(STATUS_FIRST_LED + i, 0);
  }

  uint32_t nowMs = millis();
  bool refreshActive = isRemapping || nowMs < networkRefreshEffectUntilMs;
  char songGlyph = FractureSongs::statusGlyph();
  bool songStatus = FractureSongs::isActive() && songGlyph != 0;
  char target = refreshActive ? 'R' : (songStatus ? songGlyph : octaveStatusGlyph());
  uint32_t targetColor = buzzerEnabled ? leds.Color(40, 18, 0) : leds.Color(0, 35, 45);
  uint8_t songRed = 0;
  uint8_t songGreen = 0;
  uint8_t songBlue = 0;
  if (songStatus && FractureSongs::statusColor(songRed, songGreen, songBlue)) {
    targetColor = leds.Color(songRed, songGreen, songBlue);
  } else {
    if (!networkHealthy) targetColor = leds.Color(30, 30, 0);
    if (audioEnabled) targetColor = leds.Color(10, 20, 55);
  }
  if (refreshActive) targetColor = leds.Color(75, 65, 0);

  bool immediateGlyph = refreshActive || songStatus;
  if (immediateGlyph) {
    statusGlyphCurrent = target;
    statusGlyphColor = targetColor;
    statusGlyphPrevious = 0;
    statusGlyphTransitionMs = 0;
  } else if (statusGlyphCurrent == 0) {
    statusGlyphCurrent = target;
    statusGlyphColor = targetColor;
  } else if (target != statusGlyphCurrent || targetColor != statusGlyphColor) {
    statusGlyphPrevious = statusGlyphCurrent;
    statusGlyphPreviousColor = statusGlyphColor;
    statusGlyphCurrent = target;
    statusGlyphColor = targetColor;
    statusGlyphTransitionMs = nowMs;
  }

  uint32_t elapsed = nowMs - statusGlyphTransitionMs;
  if (statusGlyphTransitionMs != 0 && elapsed < STATUS_FADE_MS * 2) {
    if (elapsed < STATUS_FADE_MS) {
      uint8_t scale = 255 - (uint8_t)(elapsed * 255 / STATUS_FADE_MS);
      drawStatusGlyph(statusGlyphPrevious, scaleColor(statusGlyphPreviousColor, scale));
    } else {
      uint8_t scale = (uint8_t)((elapsed - STATUS_FADE_MS) * 255 / STATUS_FADE_MS);
      drawStatusGlyph(statusGlyphCurrent, scaleColor(statusGlyphColor, scale));
    }
  } else {
    drawStatusGlyph(statusGlyphCurrent, statusGlyphColor);
  }
}

void renderLeds() {
  uint32_t nowMs = millis();

  for (uint8_t i = 0; i < MAX_RIPPLES; i++) {
    if (ripples[i].active) {
      uint32_t ageMs = nowMs - ripples[i].startMs;
      ripples[i].radius = ageMs * RIPPLE_SPEED;
      if (ripples[i].radius > ripples[i].maxRadius) {
        ripples[i].active = false;
      }
    }
  }

  // Physical key LEDs 0-31
  for (uint8_t k = 0; k < TOTAL_KEYS; k++) {
    uint8_t baseVel = keyBaseVelocity[k];

    // Base color: soft light blue
    uint8_t r = 10;
    uint8_t g = 40;
    uint8_t b = 80;

    // Briefly acknowledge a new press, then fade the blue boost even while held.
    if (baseVel > 0) {
      uint32_t ageMs = nowMs - keyBaseStartMs[k];
      if (ageMs < 180) {
        uint8_t extra = map(baseVel, 1, 127, 4, 24);
        uint8_t faded = (uint16_t)extra * (180 - ageMs) / 180;
        b = min<uint16_t>(255, b + faded);
        g = min<uint16_t>(255, g + faded / 3);
      }
    }

    float overlay = 0.0f;
    int8_t mainKeyIndex = mainKeyIndexForPhysicalLed(k);
    if (mainKeyIndex >= 0) {
      float noteVal = (float)(boardBaseNote + keyToNote[mainKeyIndex]);

      for (uint8_t i = 0; i < MAX_RIPPLES; i++) {
        if (!ripples[i].active) continue;

        Ripple &rp = ripples[i];
        uint32_t ageMs = nowMs - rp.startMs;
        float dist = fabsf(noteVal - rp.centerNote);
        float radius = rp.radius;
        float amplitude = (float)map(rp.baseVelocity > 0 ? rp.baseVelocity : 80, 1, 127, 70, 170);

        // Keep the source bright but local, and let the moving front be narrow.
        float source = 0.0f;
        if (dist < 3.0f) {
          float sourceShape = 1.0f - (dist / 3.0f);
          sourceShape *= sourceShape;
          source = amplitude * sourceShape * expf(-(float)ageMs / 180.0f);
        }

        float front = 0.0f;
        const float bandWidth = 0.9f;
        float diff = fabsf(dist - radius);
        if (diff < bandWidth) {
          float shape = 1.0f - (diff / bandWidth);
          shape *= shape;
          float travelDecay = expf(-(float)ageMs / 850.0f) / (1.0f + radius * 0.35f);
          front = amplitude * shape * travelDecay;
        }

        float candidate = source + front;
        if (candidate > overlay) overlay = candidate;
      }
    }

    if (overlay > 0.0f) {
      if (overlay > 160.0f) overlay = 160.0f;
      uint16_t amount = (uint16_t)overlay;
      r = min<uint16_t>(255, r + amount);
      g = min<uint16_t>(255, g + amount);
      b = min<uint16_t>(255, b + amount);
    }

    if (mainKeyIndex >= 0) {
      uint8_t midiNote = localNoteForMainKey((uint8_t)mainKeyIndex);
      uint16_t globalPosition = (uint16_t)myBoardIndex * BOARD_NOTE_SPAN +
                                keyToNote[mainKeyIndex];
      FractureSongs::keyOverlay(midiNote, globalPosition, r, g, b);
    } else {
      uint8_t switchNumber = physicalSwitchForLedIndex(k);
      if (switchNumber >= 1 && switchNumber <= NUM_EXTRA_KEYS) {
        FractureSongs::buttonOverlay(switchNumber - 1, r, g, b);
      }
    }

    leds.setPixelColor(k, leds.Color(r, g, b));
  }

  // Status LEDs
  updateStatusLeds();

  // Top-left LED (index 44)
  uint8_t topIndex = 44;
  if (isRemapping || nowMs < networkRefreshEffectUntilMs) {
    float phase = (nowMs % 280) / 280.0f * 2.0f * PI;
    float t = (sinf(phase) + 1.0f) * 0.5f;
    uint8_t level = (uint8_t)(55 + t * 80);
    leds.setPixelColor(topIndex, leds.Color(level, level, 0));
  } else {
    uint8_t songRed = 0;
    uint8_t songGreen = 0;
    uint8_t songBlue = 0;
    if (FractureSongs::topLedOverlay(songRed, songGreen, songBlue)) {
      leds.setPixelColor(topIndex, leds.Color(songRed, songGreen, songBlue));
    } else if (networkHealthy) {
      // Light green pulse when healthy
      float phase = (nowMs % 2000) / 2000.0f * 2.0f * PI;
      float t = (sinf(phase) + 1.0f) * 0.5f; // 0..1
      uint8_t g = (uint8_t)(30 + t * 80);    // 30..110
      uint8_t r = 5;
      uint8_t b = 5;
      leds.setPixelColor(topIndex, leds.Color(r, g, b));
    } else {
      // Off when not healthy / unknown
      leds.setPixelColor(topIndex, 0);
    }
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

// ========================= BUZZER MONO SYNTH ======================

uint32_t buzzerFrequencyForNote(uint8_t note) {
  int16_t shiftedNote = (int16_t)note + BUZZER_NOTE_TRANSPOSE;
  if (shiftedNote < 0) shiftedNote = 0;
  if (shiftedNote > 127) shiftedNote = 127;

  float freq = midiNoteToFreq((uint8_t)shiftedNote);
  if (freq < 1.0f) return 1;
  return (uint32_t)(freq + 0.5f);
}

uint8_t buzzerSelectNewestActiveNote() {
  uint8_t selectedNote = 0xFF;
  uint32_t newestOrder = 0;

  for (uint8_t note = 0; note < 128; note++) {
    if (buzzerNoteCount[note] > 0 && buzzerNoteOrder[note] >= newestOrder) {
      newestOrder = buzzerNoteOrder[note];
      selectedNote = note;
    }
  }

  return selectedNote;
}

uint8_t buzzerSelectAssignedActiveNote() {
  if (!discoveryDone || totalBoards <= 1 || myBoardIndex == 0xFF) {
    return buzzerSelectNewestActiveNote();
  }

  uint8_t slotCount = totalBoards;
  if (slotCount > RS485_MAX_NODES) slotCount = RS485_MAX_NODES;
  uint8_t boardSlot = myBoardIndex % slotCount;
  uint8_t assigned[RS485_MAX_NODES];
  for (uint8_t i = 0; i < RS485_MAX_NODES; i++) {
    assigned[i] = 0xFF;
  }

  uint32_t lastOrder = 0;

  while (true) {
    uint8_t bestNote = 0xFF;
    uint32_t bestOrder = 0xFFFFFFFFUL;

    for (uint8_t note = 0; note < 128; note++) {
      uint32_t order = buzzerNoteOrder[note];
      if (buzzerNoteCount[note] > 0 && order > lastOrder && order < bestOrder) {
        bestOrder = order;
        bestNote = note;
      }
    }

    if (bestNote == 0xFF) break;

    uint8_t preferredSlot = buzzerNoteSourceBoard[bestNote];
    if (preferredSlot == 0xFF || preferredSlot >= slotCount) {
      preferredSlot = boardSlot;
    }

    uint8_t targetSlot = preferredSlot;
    if (assigned[targetSlot] != 0xFF) {
      uint8_t closestFreeSlot = 0xFF;
      uint8_t closestDistance = 0xFF;

      // Boards form a line, so spill a new note to the nearest available
      // physical neighbor instead of wrapping around the chain.
      for (uint8_t candidate = 0; candidate < slotCount; candidate++) {
        if (assigned[candidate] == 0xFF) {
          uint8_t distance = (candidate > preferredSlot)
            ? candidate - preferredSlot
            : preferredSlot - candidate;
          if (distance > 0 && distance < closestDistance) {
            closestDistance = distance;
            closestFreeSlot = candidate;
          }
        }
      }

      // No buzzer is free: the newest note replaces the older note on its
      // source board. That keeps the takeover local to the new key press.
      if (closestFreeSlot != 0xFF) {
        targetSlot = closestFreeSlot;
      }
    }

    assigned[targetSlot] = bestNote;
    lastOrder = bestOrder;
  }

  return assigned[boardSlot];
}

void buzzerApplySelectedNote() {
  if (!buzzerPwmReady) return;

  uint8_t note = buzzerSelectAssignedActiveNote();
  if (note == 0xFF) {
    buzzerPwmWrite(0);
    return;
  }

  uint32_t freq = buzzerFrequencyForNote(note);
  buzzerPwmChangeFrequency(freq);
  buzzerPwmWrite(BUZZER_PWM_DUTY);
}

void buzzerStartNoteInternal(uint8_t note, uint8_t sourceBoard) {
  if (note >= 128) return;

  if (buzzerNoteCount[note] < 255) {
    buzzerNoteCount[note]++;
  }
  buzzerOrderCounter++;
  if (buzzerOrderCounter == 0) buzzerOrderCounter = 1;
  buzzerNoteOrder[note] = buzzerOrderCounter;
  buzzerNoteSourceBoard[note] = sourceBoard;

  buzzerApplySelectedNote();
}

void buzzerStopNoteInternal(uint8_t note) {
  if (note >= 128) return;

  if (buzzerNoteCount[note] > 0) {
    buzzerNoteCount[note]--;
    if (buzzerNoteCount[note] == 0) {
      buzzerNoteOrder[note] = 0;
      buzzerNoteSourceBoard[note] = 0xFF;
    }
  }

  buzzerApplySelectedNote();
}

void buzzerStopAllInternal() {
  for (uint8_t note = 0; note < 128; note++) {
    buzzerNoteCount[note] = 0;
    buzzerNoteOrder[note] = 0;
    buzzerNoteSourceBoard[note] = 0xFF;
  }
  buzzerApplySelectedNote();
}

// Keyboard-level buzzer handlers (respect buzzerEnabled)
void buzzerNoteOnFromBoard(uint8_t note, uint8_t velocity, uint8_t sourceBoard) {
  (void)velocity;
#if !FRACTURE_ENABLE_BUZZER_SYNTH
  (void)note;
  (void)sourceBoard;
  return;
#endif
  if (!buzzerEnabled) return;
  buzzerStartNoteInternal(note, sourceBoard);
}

void buzzerNoteOn(uint8_t note, uint8_t velocity) {
  buzzerNoteOnFromBoard(note, velocity, myBoardIndex);
}

void buzzerNoteOff(uint8_t note) {
#if !FRACTURE_ENABLE_BUZZER_SYNTH
  (void)note;
  return;
#endif
  if (!buzzerEnabled) return;
  buzzerStopNoteInternal(note);
}

// Startup jingle using the buzzer synth
void playStartupJingle() {
#if !FRACTURE_ENABLE_BUZZER_SYNTH
  return;
#endif
  if (!buzzerEnabled) return;
  const uint8_t notes[] = {
    79, 76, 72, 67, 72, 76, 79 // G5, E5, C5, G4, C5, E5, G5
  };
  const uint16_t durMs[] = {
    120, 120, 120, 120, 120, 120, 250
  };

  bool oldEnabled = buzzerEnabled;
  buzzerEnabled = true;

  for (uint8_t i = 0; i < sizeof(notes); i++) {
    buzzerStartNoteInternal(notes[i], myBoardIndex);
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
  if (len > sizeof(rx.buf) || (len > 0 && payload == nullptr)) return;
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

void sendSharedControl(uint8_t command, uint8_t value) {
  uint8_t payload[6];
  payload[0] = (myNodeId & 0xFF);
  payload[1] = (myNodeId >> 8) & 0xFF;
  payload[2] = (myNodeId >> 16) & 0xFF;
  payload[3] = (myNodeId >> 24) & 0xFF;
  payload[4] = command;
  payload[5] = value;
  rs485SendPacket(RS485_CONTROL_MSG, payload, sizeof(payload));
  delay(2);
  rs485SendPacket(RS485_CONTROL_MSG, payload, sizeof(payload));
}

void applySharedBuzzerEnabled(bool enabled) {
#if FRACTURE_ENABLE_BUZZER_SYNTH
  buzzerEnabled = enabled;
  if (!buzzerEnabled) {
    buzzerStopAllInternal();
  } else {
    buzzerApplySelectedNote();
  }
#else
  (void)enabled;
  buzzerEnabled = false;
#endif
}

void setSharedBuzzerEnabled(bool enabled, bool broadcast) {
  applySharedBuzzerEnabled(enabled);
  if (broadcast) {
    sendSharedControl(CONTROL_BUZZER_ENABLED, enabled ? 1 : 0);
  }
}

void setSharedOctave(int8_t octave, bool broadcast) {
  bool changed = applyOctaveLocal(octave);
  if (broadcast && changed) {
    sendSharedControl(CONTROL_OCTAVE_OFFSET, (uint8_t)(octaveOffset + CONTROL_SIGNED_BIAS));
  }
}

void setSharedAudioEnabled(bool enabled, bool broadcast) {
  audioEnabled = audioReady && enabled;
  if (broadcast) {
    sendSharedControl(CONTROL_AUDIO_ENABLED, audioEnabled ? 1 : 0);
  }
}

void broadcastSharedState() {
  sendSharedControl(CONTROL_BUZZER_ENABLED, buzzerEnabled ? 1 : 0);
  delay(2);
  sendSharedControl(CONTROL_OCTAVE_OFFSET, (uint8_t)(octaveOffset + CONTROL_SIGNED_BIAS));
  delay(2);
  sendSharedControl(CONTROL_AUDIO_ENABLED, audioEnabled ? 1 : 0);
}

void handleControlPacket(const uint8_t *payload, uint8_t len) {
  if (len != 6) return;
  uint32_t srcId = ((uint32_t)payload[0]) |
                   ((uint32_t)payload[1] << 8) |
                   ((uint32_t)payload[2] << 16) |
                   ((uint32_t)payload[3] << 24);
  uint8_t command = payload[4];
  uint8_t value = payload[5];

  if (srcId == myNodeId) return;

  if (command == CONTROL_BUZZER_ENABLED) {
    setSharedBuzzerEnabled(value != 0, false);
  } else if (command == CONTROL_OCTAVE_OFFSET) {
    setSharedOctave((int8_t)value - (int8_t)CONTROL_SIGNED_BIAS, false);
  } else if (command == CONTROL_AUDIO_ENABLED) {
    setSharedAudioEnabled(value != 0, false);
  }
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
  if (otherId != myNodeId) {
    lastDiscoveryAnnounceIndex = otherIdx;
    lastDiscoveryAnnounceMs = millis();
  }
}

uint8_t boardIndexForNodeId(uint32_t nodeId) {
  for (uint8_t i = 0; i < nodeList.count; i++) {
    if (nodeList.nodes[i].nodeId == nodeId) {
      return nodeList.nodes[i].boardIndex;
    }
  }
  return 0xFF;
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

  uint8_t sourceBoard = boardIndexForNodeId(srcId);
  if (on) {
    onRemoteNoteOn(note, vel, sourceBoard);
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
  // Give each board a short bus slot so replies do not collide on RS-485.
  delay((uint32_t)myBoardIndex * 3);
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
  if (len != 6) return;
  uint32_t sourceId = ((uint32_t)payload[0]) |
                      ((uint32_t)payload[1] << 8) |
                      ((uint32_t)payload[2] << 16) |
                      ((uint32_t)payload[3] << 24);
  if (sourceId == myNodeId) return;
  uint16_t token = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
  scheduleNetworkRemap(token);
}

void handleRemapFinalPacket(const uint8_t *payload, uint8_t len) {
  if (len != 3) return;
  uint16_t token = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  if (token != currentRemapToken) return;
  totalBoards = payload[2] == 0 ? 1 : payload[2];
  lastCompletedRemapToken = token;
  remapScheduled = false;
  isRemapping = false;
  networkHealthy = true;
  finishNetworkRefreshVisual();
  applyOctaveLocal(octaveOffset);
  FractureSongs::setTopology(myNodeId, myBoardIndex, totalBoards);
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
        rx.state = rx.len <= sizeof(rx.buf) ? 2 : 0;
        break;
      case 2:
        rx.type = b;
        rx.checksum = rx.type ^ rx.len;
        rx.state = rx.len == 0 ? 4 : 3;
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
          } else if (rx.type == RS485_CONTROL_MSG) {
            handleControlPacket(rx.buf, rx.len);
          } else if (rx.type == RS485_REMAP_FINAL_MSG) {
            handleRemapFinalPacket(rx.buf, rx.len);
          } else {
            FractureSongs::handlePacket(rx.type, rx.buf, rx.len);
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

  // New node after discovery: trigger a synchronized remap from board 0.
  if (discoveryDone && added && myBoardIndex == 0 && !isRemapping &&
      !FractureSongs::blocksNetworkMaintenance()) {
    requestNetworkRemap();
  }
}

// ========================= USB / MIDI ==============================

void midiSendNoteOn(uint8_t note, uint8_t velocity) {
#if FRACTURE_USB_MIDI_ENABLED
  usbMidi.noteOn(note, velocity, 1);
#else
  (void)note;
  (void)velocity;
#endif
}

void midiSendNoteOff(uint8_t note, uint8_t velocity) {
#if FRACTURE_USB_MIDI_ENABLED
  usbMidi.noteOff(note, velocity, 1);
#else
  (void)note;
  (void)velocity;
#endif
}

// ========================= I2S SYNTH ===============================

void synthNoteOn(uint8_t note, uint8_t velocity) {
  if (!audioEnabled || !audioReady) return;

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
  if (!audioEnabled || !audioReady) return;
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
  pinMode(PIN_DAC_MUTE_B, OUTPUT);
  digitalWrite(PIN_DAC_MUTE_B, LOW);

#if FRACTURE_ENABLE_I2S_AUDIO
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = AUDIO_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 128,
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

  esp_err_t err = i2s_driver_install(I2S_PORT, &config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("I2S driver install failed: %d\n", (int)err);
    audioEnabled = false;
    audioReady = false;
    return;
  }

  err = i2s_set_pin(I2S_PORT, &pinConfig);
  if (err != ESP_OK) {
    Serial.printf("I2S pin setup failed: %d\n", (int)err);
    i2s_driver_uninstall(I2S_PORT);
    audioEnabled = false;
    audioReady = false;
    return;
  }

  err = i2s_set_clk(I2S_PORT, AUDIO_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
  if (err != ESP_OK) {
    Serial.printf("I2S clock setup failed: %d\n", (int)err);
    i2s_driver_uninstall(I2S_PORT);
    audioEnabled = false;
    audioReady = false;
    return;
  }

  audioReady = true;
  audioEnabled = true;
  digitalWrite(PIN_DAC_MUTE_B, HIGH);
#else
  audioReady = false;
  audioEnabled = false;
  bootLog("I2S audio disabled at build time; set FRACTURE_ENABLE_I2S_AUDIO to 1 after boot is stable.");
#endif
}
// ========================= NOTE HANDLERS ===========================

void onLocalNoteOn(uint8_t note, uint8_t velocity) {
  uint8_t keyIndex = keyIndexFromNote(note);
  if (keyIndex != 0xFF) {
    setKeyLedBase(keyIndex, velocity, true);
  }
  startRipple(note, velocity);
  FractureSongs::localNoteOn(note);

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

void onRemoteNoteOn(uint8_t note, uint8_t velocity, uint8_t sourceBoard) {
  uint8_t keyIndex = keyIndexFromNote(note);
  if (keyIndex != 0xFF) {
    setKeyLedBase(keyIndex, velocity, true);
  }
  startRipple(note, velocity);
  FractureSongs::noteOn(note);

  midiSendNoteOn(note, velocity);
  synthNoteOn(note, velocity);
  buzzerNoteOnFromBoard(note, velocity, sourceBoard);
}

void onRemoteNoteOff(uint8_t note) {
  uint8_t keyIndex = keyIndexFromNote(note);
  if (keyIndex != 0xFF) {
    setKeyLedBase(keyIndex, 0, false);
  }

  midiSendNoteOff(note, 0);
  synthNoteOff(note);
  buzzerNoteOff(note);
}

// ========================= EXTRA KEYS / MODES ======================

void handleExtraKeyPressed(uint8_t extraIndex) {
#if !FRACTURE_ENABLE_EXTRA_KEYS
#if FRACTURE_ENABLE_BUZZER_SYNTH
  if (extraIndex == 0) {
    setSharedBuzzerEnabled(!buzzerEnabled, true);
  }
#else
  (void)extraIndex;
#endif
  return;
#endif

  if (FractureSongs::isActive()) {
    switch (extraIndex) {
      case 0: // S1: escape/back from song and USB loader modes
        FractureSongs::back();
        break;
      case 1:
        setSharedOctave(octaveOffset - 1, true);
        break;
      case 2:
        setSharedOctave(octaveOffset + 1, true);
        break;
      case 3:
        setSharedOctave(0, true);
        break;
      case 4: // S5: hold to reveal the selected song title while song mode is active
        FractureSongs::showTitle();
        break;
      case 5: // S6 is intentionally inactive in song modes.
        break;
      case 6: // S7: previous song/browser item
        FractureSongs::encoderDelta(-1);
        break;
      case 7: // S8: next song/browser item
        FractureSongs::nextButton();
        break;
      default:
        break;
    }
    return;
  }

  if (!settingsChangesAllowed()) return;

  switch (extraIndex) {
    case 0: // S1: toggle buzzer
      setSharedBuzzerEnabled(!buzzerEnabled, true);
      break;
    case 1: // S2: octave down
      setSharedOctave(octaveOffset - 1, true);
      break;
    case 2: // S3: octave up
      setSharedOctave(octaveOffset + 1, true);
      break;
    case 3: // S4: reset chain to the base octave
      setSharedOctave(0, true);
      break;
    case 4: // S5: toggle audio
      if (audioReady) setSharedAudioEnabled(!audioEnabled, true);
      break;
    case 5: // S6: enter distributed song browser
      noInterrupts();
      encoderPendingEdges = 0;
      interrupts();
      encoderEdgeAccumulator = 0;
      FractureSongs::enterBrowser();
      break;
    case 6: // S7: refresh/reorder moved one button to the right
      requestNetworkRemap();
      break;
    case 7: // S8: reserved in normal mode; next item while song mode is active
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
    if (encoderPendingEdges > INT16_MIN) --encoderPendingEdges;
  } else if ((encoderLastState == 0b00 && state == 0b10) ||
             (encoderLastState == 0b10 && state == 0b11) ||
             (encoderLastState == 0b11 && state == 0b01) ||
             (encoderLastState == 0b01 && state == 0b00)) {
    if (encoderPendingEdges < INT16_MAX) ++encoderPendingEdges;
  }
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

// The encoder button keeps only electrical debounce; it is exempt from the
// main-key accidental-press cooldown.
void handleEncoderButton() {
  static bool prev = HIGH;
  static uint32_t lastPressMs = 0;
  bool cur = digitalRead(PIN_ENC_SW);
  uint32_t nowMs = millis();
  if (prev == HIGH && cur == LOW && (nowMs - lastPressMs) >= 250) {
    if (FractureSongs::isActive()) {
      FractureSongs::confirm();
    } else {
#if FRACTURE_ENABLE_BUZZER_SYNTH
      setSharedBuzzerEnabled(!buzzerEnabled, true);
#endif
    }
    lastPressMs = nowMs;
  }
  prev = cur;
}

void handleEncoderMovement() {
  noInterrupts();
  int16_t edges = encoderPendingEdges;
  encoderPendingEdges = 0;
  interrupts();

  bool menuEncoder = FractureSongs::usesEncoderForMenu();
  bool octaveEncoder = FractureSongs::usesEncoderForOctaves();
  uint8_t route = menuEncoder ? 1 : (octaveEncoder ? 2 : 0);
  if (route != encoderLastRoute) {
    encoderLastRoute = route;
    encoderEdgeAccumulator = 0;
  }
  if (route == 0) {
    return;
  }

  encoderEdgeAccumulator += edges;
  while (encoderEdgeAccumulator >= ENCODER_EDGES_PER_DETENT) {
    encoderEdgeAccumulator -= ENCODER_EDGES_PER_DETENT;
    if (menuEncoder) FractureSongs::encoderDelta(1);
    else setSharedOctave(octaveOffset + 1, true);
  }
  while (encoderEdgeAccumulator <= -ENCODER_EDGES_PER_DETENT) {
    encoderEdgeAccumulator += ENCODER_EDGES_PER_DETENT;
    if (menuEncoder) FractureSongs::encoderDelta(-1);
    else setSharedOctave(octaveOffset - 1, true);
  }
}
// ========================= PHYSICAL CHAIN DISCOVERY ===============

void pulseRightNeighbor() {
  digitalWrite(PIN_NEIGHBOR_RIGHT, LOW);
  delay(3);
  digitalWrite(PIN_NEIGHBOR_RIGHT, HIGH);
}

void startDiscovery(bool fromRemap) {
  if (fromRemap) {
    triggerNetworkRefreshEffect();
  }

  discoveryDone = false;
  isRemapping = true;
  networkHealthy = false;
  triggerNetworkRefreshEffect();

  myBoardIndex = 0xFF;
  nodeList.count = 0;
  lastDiscoveryAnnounceIndex = 0xFF;
  lastDiscoveryAnnounceMs = 0;

  for (uint8_t i = 0; i < TOTAL_KEYS; i++) {
    keyBaseVelocity[i] = 0;
    keyBaseStartMs[i] = 0;
  }

  pinMode(PIN_NEIGHBOR_LEFT, INPUT_PULLUP);
  pinMode(PIN_NEIGHBOR_RIGHT, OUTPUT);
  digitalWrite(PIN_NEIGHBOR_RIGHT, LOW);
  delay(DISCOVERY_NEIGHBOR_SETTLE_MS);
  hasLeftNeighbor = (digitalRead(PIN_NEIGHBOR_LEFT) == LOW);
  bool isRoot = !hasLeftNeighbor;

  digitalWrite(PIN_NEIGHBOR_RIGHT, HIGH);
  delay(2);

  uint32_t tStart = millis();
  uint32_t nextDiscoverySignalMs = tStart + DISCOVERY_RETRY_MS;

  if (isRoot) {
    myBoardIndex = 0;
    sendIdAnnounce(myBoardIndex);
    pulseRightNeighbor();
  }

  int leftPrev = digitalRead(PIN_NEIGHBOR_LEFT);

  while (millis() - tStart < DISCOVERY_MAX_MS) {
    processRs485();

    int leftNow = digitalRead(PIN_NEIGHBOR_LEFT);
    bool parentAnnounced = lastDiscoveryAnnounceIndex != 0xFF &&
      (uint32_t)(millis() - lastDiscoveryAnnounceMs) <= DISCOVERY_PARENT_ANNOUNCE_MS;
    if (leftPrev == HIGH && leftNow == LOW && parentAnnounced) {
      // The announce is sent immediately before the physical pulse, so it
      // carries the index of this board's direct left neighbor.
      myBoardIndex = lastDiscoveryAnnounceIndex + 1;
      hasLeftNeighbor = true;
      isRoot = false;
      sendIdAnnounce(myBoardIndex);
      pulseRightNeighbor();
    }
    leftPrev = leftNow;

    // Root repeats its token. Each board forwards the paired announce and
    // pulse, allowing a late-starting board to correct a provisional index.
    if (isRoot && millis() >= nextDiscoverySignalMs) {
      sendIdAnnounce(myBoardIndex);
      pulseRightNeighbor();
      nextDiscoverySignalMs = millis() + DISCOVERY_RETRY_MS;
    }

    renderLeds();
    delay(1);
  }

  // A late edge can still leave a non-root unassigned. The most recent
  // announce is paired with the repeated physical token, so use it as a
  // final fallback instead of incorrectly becoming a second board zero.
  if (myBoardIndex == 0xFF) {
    myBoardIndex = (hasLeftNeighbor && lastDiscoveryAnnounceIndex != 0xFF)
      ? lastDiscoveryAnnounceIndex + 1
      : 0;
    sendIdAnnounce(myBoardIndex);
    delay(6);
    processRs485();
  }

  uint8_t maxIdx = myBoardIndex;
  for (uint8_t i = 0; i < nodeList.count; i++) {
    if (nodeList.nodes[i].boardIndex != 0xFF &&
        nodeList.nodes[i].boardIndex > maxIdx) {
      maxIdx = nodeList.nodes[i].boardIndex;
    }
  }
  totalBoards = maxIdx + 1;
  if (totalBoards == 0) {
    totalBoards = 1;
  }

  applyOctaveLocal(octaveOffset);

  discoveryDone = true;
  remapScheduled = false;
  isRemapping = false;
  networkHealthy = true;
  lastCompletedRemapToken = currentRemapToken;
  finishNetworkRefreshVisual();
  buzzerApplySelectedNote();
  FractureSongs::setTopology(myNodeId, myBoardIndex, totalBoards);

  if (myBoardIndex == 0) {
    uint8_t finalPayload[3] = {
      (uint8_t)(currentRemapToken & 0xFF),
      (uint8_t)((currentRemapToken >> 8) & 0xFF),
      totalBoards
    };
    for (uint8_t attempt = 0; attempt < REMAP_REQUEST_REPEATS; ++attempt) {
      rs485SendPacket(RS485_REMAP_FINAL_MSG, finalPayload, sizeof(finalPayload));
      if (attempt + 1 < REMAP_REQUEST_REPEATS) delay(4);
    }
    broadcastSharedState();
  }
}
// ========================= SETUP & LOOP ============================

void setup() {
  Serial.begin(115200);
  delay(300);
  bootLog("BOOT 01: serial ready");

  // Configure the song loader before adding MIDI and starting TinyUSB.
  FractureSongs::beginBeforeUsb(rs485SendPacket);
  bool loaderBoot = FractureSongs::isUsbLoaderBoot();
#if FRACTURE_USB_DEVICE_ENABLED
  USB.manufacturerName("Fracture");
  USB.productName(loaderBoot ? "Fracture Keyboard + Song Loader" : "Fracture Keyboard");
#endif
#if FRACTURE_USB_MIDI_ENABLED
  usbMidi.begin();
#endif
#if FRACTURE_USB_DEVICE_ENABLED
  USB.begin();
  bootLog(loaderBoot ? "BOOT 02: USB loader ready" : "BOOT 02: USB device ready");
#else
  Serial.println("USB MIDI/song drive disabled. Set Tools > USB Mode to USB-OTG (TinyUSB).");
  bootLog("BOOT 02: USB device skipped");
#endif

  // Mux pins
  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);

  // RS-485
  pinMode(PIN_RS485_MODE, OUTPUT);
  rs485SetTx(false);
  RS485.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  bootLog("BOOT 03: RS485 ready");

  // Analog
  analogReadResolution(12);

  // LEDs
  leds.begin();
  leds.clear();
  leds.show();
  bootLog("BOOT 04: LEDs ready");

  // Buzzer PWM
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
#if FRACTURE_ENABLE_BUZZER_SYNTH
  buzzerPwmReady = buzzerPwmAttach(PIN_BUZZER, 1000, BUZZER_PWM_RESOLUTION);
  buzzerPwmWrite(0);
  buzzerStopAllInternal();
  bootLog(buzzerPwmReady ? "BOOT 05: buzzer PWM ready" : "BOOT 05: buzzer PWM failed");
#else
  bootLog("BOOT 05: buzzer synth disabled");
#endif
  // Keys
  initKeyDefs();
  calibrateKeys();
  bootLog("BOOT 06: keys calibrated");

  // Encoder
  encoderInit();
  bootLog("BOOT 07: encoder ready");

  // I2S audio
  audioInit();
  bootLog("BOOT 08: audio init complete");

  // Node ID
  uint64_t chipId = ESP.getEfuseMac();
  myNodeId = (uint32_t)chipId ^ (uint32_t)(chipId >> 32);
  nodeList.count = 0;
  FractureSongs::setTopology(myNodeId, 0, 1);

  // Ping state
  awaitingPingReplies = false;
  lastPingMs = millis();

  // Startup polish
  showAllLedsStartup();
  playStartupJingle();
  bootLog("BOOT 09: startup effects complete");

  // Initial discovery
  startDiscovery(false);
  bootLog("BOOT 10: discovery complete");
}

void loop() {
  if (remapScheduled && (int32_t)(millis() - remapStartAtMs) >= 0) {
    remapScheduled = false;
    startDiscovery(true);
  }

  processRs485();

  scanKeys();
  handleEncoderButton();
  handleEncoderMovement();
  FractureSongs::update();

  audioGenerateAndWrite();

  uint32_t nowMs = millis();
  if (FractureSongs::blocksNetworkMaintenance()) {
    awaitingPingReplies = false;
  } else if (myBoardIndex == 0 && totalBoards > 1) {
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
        requestNetworkRemap();
      } else {
        networkHealthy = true;
      }
    }
  }

  renderLeds();

  delay(2);
}
