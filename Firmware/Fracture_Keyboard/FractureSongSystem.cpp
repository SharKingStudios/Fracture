#include "FractureSongSystem.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <FFat.h>
#include <Preferences.h>
#if defined(ARDUINO_ARCH_ESP32) && __has_include(<soc/soc_caps.h>)
#include <soc/soc_caps.h>
#endif
#if defined(ARDUINO_ARCH_ESP32) && __has_include(<esp_heap_caps.h>)
#include <esp_heap_caps.h>
#define FRACTURE_HAS_HEAP_CAPS 1
#else
#define FRACTURE_HAS_HEAP_CAPS 0
#endif
#include "src/MD_MIDIFile.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(SOC_USB_OTG_SUPPORTED) && SOC_USB_OTG_SUPPORTED && defined(ARDUINO_USB_MODE) && !ARDUINO_USB_MODE && __has_include("USBMSC.h")
#include "USBMSC.h"
#include "esp_attr.h"
#include "esp_partition.h"
#include "wear_levelling.h"
#if defined(CONFIG_TINYUSB_MSC_ENABLED) && CONFIG_TINYUSB_MSC_ENABLED
#define FRACTURE_SONG_MSC_ENABLED 1
#else
#define FRACTURE_SONG_MSC_ENABLED 0
#endif
#else
#define FRACTURE_SONG_MSC_ENABLED 0
#endif

namespace FractureSongs {
namespace {

constexpr uint8_t MAX_LOCAL_SONGS = 16;
constexpr uint8_t MAX_NETWORK_SONGS = 64;
constexpr uint8_t MAX_TRACKS = 16;
constexpr uint8_t MAX_STREAMS = 32;
constexpr uint8_t MAX_GUIDE_NOTES = 22;
constexpr size_t MAX_GUIDE_EVENTS = 64000;
constexpr uint8_t SONG_NAME_LENGTH = 24;
constexpr uint8_t TITLE_LENGTH = 26;
constexpr uint8_t TITLE_FLAG_LOAD_ITEM = 0x80;
constexpr uint8_t TITLE_FLAG_ATTENTION = 0x40;
constexpr uint8_t TITLE_COLOR_MASK = 0x3F;
constexpr uint8_t CATALOG_LOCAL_INDEX_MASK = 0x0F;
constexpr uint8_t CATALOG_FLAG_HAS_NOTES = 0x40;
constexpr uint8_t CATALOG_FLAG_SPAN_WIDE = 0x80;
constexpr uint8_t NOTES_PER_BOARD = 24;
constexpr uint16_t MSC_FALLBACK_BLOCK_SIZE = 512;
constexpr uint32_t FALLBACK_MSC_BLOCK_COUNT = 16;
constexpr char LOADER_IMAGE_PATH[] = "/__song_loader.img";
constexpr uint16_t LOADER_IMAGE_BLOCK_SIZE = 512;
constexpr uint32_t LOADER_IMAGE_BLOCK_COUNT = 2048;
constexpr uint32_t LOADER_IMAGE_BYTE_COUNT = LOADER_IMAGE_BLOCK_COUNT * LOADER_IMAGE_BLOCK_SIZE;
constexpr uint8_t LOADER_IMAGE_SECTORS_PER_CLUSTER = 4;
constexpr uint16_t LOADER_IMAGE_RESERVED_SECTORS = 1;
constexpr uint8_t LOADER_IMAGE_FAT_COUNT = 1;
constexpr uint16_t LOADER_IMAGE_FAT_SECTORS = 2;
constexpr uint16_t LOADER_IMAGE_ROOT_ENTRIES = 64;
constexpr uint32_t LOADER_IMAGE_ROOT_SECTORS =
    ((uint32_t)LOADER_IMAGE_ROOT_ENTRIES * 32 + LOADER_IMAGE_BLOCK_SIZE - 1) / LOADER_IMAGE_BLOCK_SIZE;
constexpr uint32_t LOADER_IMAGE_ROOT_START_SECTOR =
    LOADER_IMAGE_RESERVED_SECTORS + LOADER_IMAGE_FAT_COUNT * LOADER_IMAGE_FAT_SECTORS;
constexpr uint32_t LOADER_IMAGE_DATA_START_SECTOR =
    LOADER_IMAGE_ROOT_START_SECTOR + LOADER_IMAGE_ROOT_SECTORS;
constexpr uint16_t LOADER_IMAGE_MAX_CLUSTER =
    (LOADER_IMAGE_BLOCK_COUNT - LOADER_IMAGE_DATA_START_SECTOR) / LOADER_IMAGE_SECTORS_PER_CLUSTER + 1;
constexpr uint16_t LOADER_IMPORT_NAME_LENGTH = 64;
constexpr uint16_t LOADER_IMPORT_PATH_LENGTH = 96;
constexpr uint32_t CATALOG_REPLY_GAP_MS = 6;
constexpr uint32_t CATALOG_TIMEOUT_MS = 320;
constexpr uint32_t GUIDE_ADVANCE_DELAY_MS = 120;
constexpr uint32_t USB_SETTLE_MS = 400;
constexpr uint32_t USB_ENTER_RESTART_DELAY_MS = 90;
constexpr uint32_t USB_EXIT_RESTART_DELAY_MS = 120;
constexpr uint32_t USB_LOAD_RTC_MAGIC = 0x46534D43UL;  // "FSMC"
constexpr uint32_t TITLE_FRAME_MS = 280;
constexpr uint32_t MODE_HEARTBEAT_MS = 900;
constexpr uint32_t MODE_STALE_MS = MODE_HEARTBEAT_MS * 4;

static_assert(6 + TITLE_LENGTH <= 32, "song title packet exceeds RS-485 payload");
static_assert(MAX_LOCAL_SONGS <= CATALOG_LOCAL_INDEX_MASK + 1, "catalog index mask too small");
static_assert(8 + SONG_NAME_LENGTH <= 32, "catalog packet exceeds RS-485 payload");
static_assert(10 + MAX_GUIDE_NOTES <= 32, "guide packet exceeds RS-485 payload");

constexpr uint8_t HAND_UNKNOWN = 0;
constexpr uint8_t HAND_LEFT = 1;
constexpr uint8_t HAND_RIGHT = 2;
constexpr uint8_t HAND_BOTH = 3;

enum SongMode : uint8_t {
  MODE_NORMAL = 0,
  MODE_BROWSER = 1,
  MODE_USB_LOAD = 2,
  MODE_LOADING = 3,
  MODE_HAND_SELECT = 4,
  MODE_LEARNING = 5,
  MODE_COMPLETE = 6,
};

enum SongCommand : uint8_t {
  COMMAND_NAVIGATE = 1,
  COMMAND_CONFIRM = 2,
  COMMAND_BACK = 3,
  COMMAND_LOAD_SELECTED = 4,
  COMMAND_START_LEARNING = 5,
  COMMAND_CANCEL_OWNER = 6,
  COMMAND_NEXT_OR_LOAD = 7,
  COMMAND_ENTER_USB_LOADER = 8,
  COMMAND_EXIT_USB_LOADER = 9,
  COMMAND_IMPORT_COMPLETE = 10,
};

constexpr uint8_t GUIDE_FLAG_END = 0x01;

struct LocalSong {
  char title[SONG_NAME_LENGTH + 1];
  char path[96];
  uint32_t size;
  uint8_t minNote;
  uint8_t maxNote;
  bool hasNotes;
};

struct NetworkSong {
  uint8_t ownerBoard;
  uint8_t localIndex;
  uint8_t flags;
  char title[SONG_NAME_LENGTH + 1];
};

struct GuideEvent {
  uint32_t tick;
  uint8_t note;
  uint8_t stream;
  uint8_t hand;
};

struct StreamStats {
  uint8_t id;
  uint8_t hand;
  uint32_t noteCount;
  uint32_t noteSum;
  uint8_t minNote;
  uint8_t maxNote;
};

PacketSender gSendPacket = nullptr;
uint32_t gNodeId = 0;
uint8_t gBoardIndex = 0;
uint8_t gBoardCount = 1;
SongMode gMode = MODE_NORMAL;
uint32_t gCoordinatorId = 0;
uint8_t gSession = 0;
uint8_t gSessionCounter = 0;
char gTitle[TITLE_LENGTH + 1] = "";
uint8_t gTitleFlags = 0;
uint32_t gTitleEpochMs = 0;
uint32_t gTitleRevealUntilMs = 0;
uint32_t gModeEpochMs = 0;
uint32_t gLastModeHeartbeatMs = 0;
uint32_t gLastModePacketMs = 0;
char gSelectedSongTitle[SONG_NAME_LENGTH + 1] = "";
uint8_t gSelectedSongFlags = 0;

bool gStorageMounted = false;
bool gStorageWasInitialized = false;
bool gStorageAvailable = false;
bool gBootUsbLoad = false;
bool gPostImportPending = false;
bool gInitialTopologySeen = false;
LocalSong gLocalSongs[MAX_LOCAL_SONGS];
uint8_t gLocalSongCount = 0;
uint32_t gStorageSignature = 0;

NetworkSong gNetworkSongs[MAX_NETWORK_SONGS];
uint8_t gNetworkSongCount = 0;
uint8_t gBrowserSelection = 0;
bool gCatalogScanning = false;
uint8_t gCatalogTarget = 0;
bool gCatalogWaiting = false;
bool gCatalogTargetComplete = false;
uint32_t gCatalogDeadlineMs = 0;
uint32_t gCatalogNextActionMs = 0;

bool gCatalogReplyPending = false;
uint32_t gCatalogReplyCoordinator = 0;
uint8_t gCatalogReplySession = 0;
uint8_t gCatalogReplyIndex = 0;
bool gCatalogReplySentEmpty = false;
uint32_t gCatalogReplyNextMs = 0;
bool gPreferFirstSongAfterScan = false;

bool gPendingLoad = false;
uint8_t gPendingLocalSong = 0;
uint8_t gSongOwner = 0xFF;
uint8_t gAvailableHands = HAND_UNKNOWN;
uint8_t gHandChoice = HAND_BOTH;

MD_MIDIFile gMidiFile;
GuideEvent *gEvents = nullptr;
size_t gEventCount = 0;
size_t gEventCapacity = 0;
bool gEventOverflow = false;
bool gStreamOverflow = false;
char gTrackNames[MAX_TRACKS][33];
StreamStats gStreams[MAX_STREAMS];
uint8_t gStreamCount = 0;
bool gParsingMidi = false;
bool gAnalyzingMidi = false;
bool gAnalysisHasNotes = false;
uint8_t gAnalysisMinNote = 127;
uint8_t gAnalysisMaxNote = 0;
size_t gEventCursor = 0;
uint16_t gGuideSequence = 0;
uint8_t gGuideNotes[MAX_GUIDE_NOTES];
bool gGuideLeft[MAX_GUIDE_NOTES];
bool gGuideHit[MAX_GUIDE_NOTES];
uint8_t gGuideCount = 0;
uint16_t gLastGuideSequence = 0;
bool gHaveGuideSequence = false;
bool gAdvancePending = false;
uint32_t gAdvanceAtMs = 0;
uint8_t gLearningHand = HAND_BOTH;

uint32_t gImportWaveStartMs = 0;

#if FRACTURE_SONG_MSC_ENABLED
RTC_NOINIT_ATTR uint32_t gRtcUsbLoadMagic;
RTC_NOINIT_ATTR uint32_t gRtcUsbLoadMagicInverse;
// Created only for a loader boot, before USB.begin(). Normal boots stay MIDI-only.
USBMSC *gMsc = nullptr;
const esp_partition_t *gFatPartition = nullptr;
wl_handle_t gRawHandle = WL_INVALID_HANDLE;
size_t gRawSize = 0;
size_t gRawEraseSize = 0;
uint16_t gMscBlockSize = MSC_FALLBACK_BLOCK_SIZE;
uint8_t *gMscScratch = nullptr;
bool gMscReady = false;
bool gMscMediaPresent = false;
volatile bool gMscDirty = false;
volatile bool gMscEjectRequested = false;
bool gUsbExitPending = false;
uint32_t gUsbExitAtMs = 0;
bool gUsbLoadModeAnnounced = false;
bool gRestartPending = false;
uint32_t gRestartAtMs = 0;
bool gMscUsesFallback = false;
bool gMscUsesImage = false;
File gLoaderImageFile;
bool gLoaderImageOpen = false;
uint8_t gFallbackMscDisk[FALLBACK_MSC_BLOCK_COUNT][MSC_FALLBACK_BLOCK_SIZE];
#endif

uint32_t decodeU32(const uint8_t *bytes) {
  return ((uint32_t)bytes[0]) |
         ((uint32_t)bytes[1] << 8) |
         ((uint32_t)bytes[2] << 16) |
         ((uint32_t)bytes[3] << 24);
}

void encodeU32(uint8_t *bytes, uint32_t value) {
  bytes[0] = value & 0xFF;
  bytes[1] = (value >> 8) & 0xFF;
  bytes[2] = (value >> 16) & 0xFF;
  bytes[3] = (value >> 24) & 0xFF;
}

void sendReliable(uint8_t type, const uint8_t *payload, uint8_t length) {
  if (!gSendPacket) return;
  gSendPacket(type, payload, length);
  delay(2);
  gSendPacket(type, payload, length);
}

bool isCoordinator() {
  return gCoordinatorId == gNodeId;
}

char lowerAscii(char value) {
  return (value >= 'A' && value <= 'Z') ? (char)(value + ('a' - 'A')) : value;
}

int compareText(const char *left, const char *right) {
  while (*left && *right) {
    char a = lowerAscii(*left++);
    char b = lowerAscii(*right++);
    if (a != b) return (int)(uint8_t)a - (int)(uint8_t)b;
  }
  return (int)(uint8_t)*left - (int)(uint8_t)*right;
}

bool endsWithMidiExtension(const char *name) {
  size_t length = strlen(name);
  if (length >= 4 && compareText(name + length - 4, ".mid") == 0) return true;
  return length >= 5 && compareText(name + length - 5, ".midi") == 0;
}

void normalizeTitle(char *destination, size_t destinationSize, const char *source) {
  if (destinationSize == 0) return;
  const char *base = strrchr(source, '/');
  base = base ? base + 1 : source;
  size_t length = strlen(base);
  if (length >= 5 && compareText(base + length - 5, ".midi") == 0) length -= 5;
  else if (length >= 4 && compareText(base + length - 4, ".mid") == 0) length -= 4;
  if (length >= destinationSize) length = destinationSize - 1;
  for (size_t i = 0; i < length; ++i) {
    char value = base[i];
    destination[i] = (value == '_') ? ' ' : value;
  }
  destination[length] = '\0';
}

uint32_t fnv1a(uint32_t hash, const void *data, size_t length) {
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  for (size_t i = 0; i < length; ++i) {
    hash ^= bytes[i];
    hash *= 16777619UL;
  }
  return hash;
}

void setTitleLocal(const char *title, uint8_t flags = 0) {
  char normalized[TITLE_LENGTH + 1];
  strncpy(normalized, title ? title : "", TITLE_LENGTH);
  normalized[TITLE_LENGTH] = '\0';
  for (size_t i = 0; normalized[i]; ++i) {
    if (normalized[i] == '_') normalized[i] = ' ';
  }
  if (flags == gTitleFlags && strcmp(normalized, gTitle) == 0) return;
  memcpy(gTitle, normalized, sizeof(gTitle));
  gTitleFlags = flags;
  gTitleEpochMs = millis() + 40;
}

void broadcastTitle(const char *title, uint8_t flags = 0) {
  setTitleLocal(title, flags);
  if (!gSendPacket) return;
  uint8_t payload[32];
  encodeU32(payload, gCoordinatorId);
  payload[4] = gSession;
  payload[5] = gTitleFlags;
  uint8_t titleLength = strlen(gTitle);
  if (titleLength > TITLE_LENGTH) titleLength = TITLE_LENGTH;
  memcpy(payload + 6, gTitle, titleLength);
  sendReliable(PACKET_TITLE, payload, 6 + titleLength);
}

void applyMode(SongMode mode) {
  if (gMode != mode) {
    gModeEpochMs = millis();
    gTitleRevealUntilMs = 0;
  }
  gMode = mode;
  if (mode != MODE_LEARNING && mode != MODE_COMPLETE) {
    gGuideCount = 0;
    gAdvancePending = false;
  }
  if (mode == MODE_LOADING) gHaveGuideSequence = false;
}

uint16_t modeAgeSeconds() {
  if (gMode == MODE_NORMAL) return 0;
  uint32_t nowMs = millis();
  uint32_t ageSeconds = (nowMs >= gModeEpochMs ? nowMs - gModeEpochMs : 0) / 1000UL;
  return ageSeconds > 0xFFFFUL ? 0xFFFF : (uint16_t)ageSeconds;
}

void carryIncomingModeAge(uint16_t ageSeconds) {
  if (gMode == MODE_NORMAL || ageSeconds == 0) return;
  uint32_t ageMs = (uint32_t)ageSeconds * 1000UL;
  uint32_t nowMs = millis();
  gModeEpochMs = nowMs > ageMs ? nowMs - ageMs : 0;
}

void applyModeArgument(SongMode mode, uint8_t argument) {
  if (mode == MODE_LOADING || mode == MODE_LEARNING || mode == MODE_USB_LOAD) {
    gSongOwner = argument;
  } else if (mode == MODE_HAND_SELECT) {
    gAvailableHands = argument;
  } else if (mode == MODE_NORMAL) {
    gSongOwner = 0xFF;
  }
}

void broadcastMode(SongMode mode, uint8_t argument = 0) {
  applyMode(mode);
  if (!gSendPacket) return;
  uint8_t payload[9];
  encodeU32(payload, gCoordinatorId);
  payload[4] = gSession;
  payload[5] = (uint8_t)mode;
  payload[6] = argument;
  uint16_t age = modeAgeSeconds();
  payload[7] = age & 0xFF;
  payload[8] = (age >> 8) & 0xFF;
  sendReliable(PACKET_MODE, payload, sizeof(payload));
}

void releaseGuideEvents();

void clearLoadedSong() {
  releaseGuideEvents();
  gEventCount = 0;
  gEventCapacity = 0;
  gEventOverflow = false;
  gEventCursor = 0;
  gGuideCount = 0;
  gHaveGuideSequence = false;
  gAdvancePending = false;
  gSongOwner = 0xFF;
  gAvailableHands = HAND_UNKNOWN;
  gSelectedSongFlags = 0;
  gSelectedSongTitle[0] = '\0';
}

bool mountStorage() {
  if (gStorageMounted) return true;
  if (!gStorageWasInitialized) {
    Preferences preferences;
    if (preferences.begin("fractureSong", true)) {
      gStorageWasInitialized = preferences.getBool("fat-ready", false);
      preferences.end();
    }
  }

  gStorageAvailable = FFat.begin(false, "/ffat", 10, "ffat");
  if (!gStorageAvailable && !gStorageWasInitialized) {
    gStorageAvailable = FFat.begin(true, "/ffat", 10, "ffat");
  }
  if (gStorageAvailable && !gStorageWasInitialized) {
    gStorageWasInitialized = true;
    Preferences preferences;
    if (preferences.begin("fractureSong", false)) {
      preferences.putBool("fat-ready", true);
      preferences.end();
    }
  }
  gStorageMounted = gStorageAvailable;
  return gStorageMounted;
}

void unmountStorage() {
  if (!gStorageMounted) return;
  FFat.end();
  gStorageMounted = false;
}

void midiEventCallback(midi_event *event);
void midiMetaCallback(const meta_event *event);
bool analyzeMidiFile(const char *path, uint8_t &minNote, uint8_t &maxNote);

void sortLocalSongs() {
  for (uint8_t i = 1; i < gLocalSongCount; ++i) {
    LocalSong value = gLocalSongs[i];
    int8_t j = i - 1;
    while (j >= 0 && compareText(gLocalSongs[j].title, value.title) > 0) {
      gLocalSongs[j + 1] = gLocalSongs[j];
      --j;
    }
    gLocalSongs[j + 1] = value;
  }
}

void scanLocalSongs() {
  gLocalSongCount = 0;
  uint32_t signature = 2166136261UL;
  if (!gStorageMounted) {
    gStorageSignature = 0;
    return;
  }

  File root = FFat.open("/");
  if (!root || !root.isDirectory()) {
    if (root) root.close();
    gStorageSignature = 0;
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      const char *path = file.path();
      const char *name = file.name();
      if (path && name && endsWithMidiExtension(name)) {
        uint32_t fileSize = (uint32_t)file.size();
        signature = fnv1a(signature, path, strlen(path));
        signature = fnv1a(signature, &fileSize, sizeof(fileSize));
        if (gLocalSongCount < MAX_LOCAL_SONGS) {
          LocalSong &song = gLocalSongs[gLocalSongCount++];
          normalizeTitle(song.title, sizeof(song.title), name);
          strncpy(song.path, path, sizeof(song.path) - 1);
          song.path[sizeof(song.path) - 1] = '\0';
          song.size = fileSize;
          song.minNote = 0;
          song.maxNote = 0;
          song.hasNotes = false;
        }
      }
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();
  for (uint8_t i = 0; i < gLocalSongCount; ++i) {
    gLocalSongs[i].hasNotes = analyzeMidiFile(gLocalSongs[i].path,
                                              gLocalSongs[i].minNote,
                                              gLocalSongs[i].maxNote);
  }
  sortLocalSongs();
  gStorageSignature = signature;
}

#if FRACTURE_SONG_MSC_ENABLED
int32_t mscRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufferSize);
int32_t mscWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufferSize);
bool mscStartStop(uint8_t powerCondition, bool start, bool loadEject);

void putLe16(uint8_t *destination, uint16_t value) {
  destination[0] = value & 0xFF;
  destination[1] = (value >> 8) & 0xFF;
}

void putLe32(uint8_t *destination, uint32_t value) {
  destination[0] = value & 0xFF;
  destination[1] = (value >> 8) & 0xFF;
  destination[2] = (value >> 16) & 0xFF;
  destination[3] = (value >> 24) & 0xFF;
}

void copyPadded(uint8_t *destination, size_t length, const char *textValue) {
  memset(destination, ' ', length);
  if (!textValue) return;
  size_t textLength = strlen(textValue);
  if (textLength > length) textLength = length;
  memcpy(destination, textValue, textLength);
}

void setFallbackFat12Pair(uint8_t *fat, uint16_t index, uint16_t left, uint16_t right) {
  uint16_t offset = (uint16_t)(index * 3);
  fat[offset] = left & 0xFF;
  fat[offset + 1] = ((left >> 8) & 0x0F) | ((right << 4) & 0xF0);
  fat[offset + 2] = (right >> 4) & 0xFF;
}

void buildFallbackMscDisk(const char *message) {
  memset(gFallbackMscDisk, 0, sizeof(gFallbackMscDisk));

  uint8_t *boot = gFallbackMscDisk[0];
  boot[0] = 0xEB;
  boot[1] = 0x3C;
  boot[2] = 0x90;
  copyPadded(boot + 3, 8, "FRACMSC");
  putLe16(boot + 11, MSC_FALLBACK_BLOCK_SIZE);
  boot[13] = 1;
  putLe16(boot + 14, 1);
  boot[16] = 1;
  putLe16(boot + 17, 16);
  putLe16(boot + 19, FALLBACK_MSC_BLOCK_COUNT);
  boot[21] = 0xF8;
  putLe16(boot + 22, 1);
  putLe16(boot + 24, 1);
  putLe16(boot + 26, 1);
  putLe32(boot + 28, 0);
  putLe32(boot + 32, 0);
  boot[38] = 0x29;
  putLe32(boot + 39, 0x534C4452UL);
  copyPadded(boot + 43, 11, "SONG LOADER");
  copyPadded(boot + 54, 8, "FAT12");
  boot[510] = 0x55;
  boot[511] = 0xAA;

  uint8_t *fat = gFallbackMscDisk[1];
  setFallbackFat12Pair(fat, 0, 0xFF8, 0xFFF);
  setFallbackFat12Pair(fat, 1, 0xFFF, 0x000);

  uint8_t *root = gFallbackMscDisk[2];
  copyPadded(root, 11, "SONG LOADER");
  root[11] = 0x08;

  uint8_t *entry = root + 32;
  copyPadded(entry, 8, "README");
  copyPadded(entry + 8, 3, "TXT");
  entry[11] = 0x20;
  putLe16(entry + 26, 2);

  const char *prefix =
      "Fracture Song Loader USB is alive, but the firmware could not open the FFat song partition.\r\n"
      "Files copied to this diagnostic disk will not be saved.\r\n\r\n";
  uint8_t *data = gFallbackMscDisk[3];
  size_t written = 0;
  size_t prefixLength = strlen(prefix);
  if (prefixLength > MSC_FALLBACK_BLOCK_SIZE) prefixLength = MSC_FALLBACK_BLOCK_SIZE;
  memcpy(data, prefix, prefixLength);
  written += prefixLength;
  if (message && written < MSC_FALLBACK_BLOCK_SIZE) {
    size_t messageLength = strlen(message);
    if (messageLength > MSC_FALLBACK_BLOCK_SIZE - written) {
      messageLength = MSC_FALLBACK_BLOCK_SIZE - written;
    }
    memcpy(data + written, message, messageLength);
    written += messageLength;
  }
  putLe32(entry + 28, (uint32_t)written);
}

bool configureFallbackMsc(const char *message) {
  if (!gMsc) gMsc = new USBMSC();
  if (!gMsc) return false;
  buildFallbackMscDisk(message);
  gMscUsesFallback = true;
  gMscUsesImage = false;
  gMsc->vendorID("Fracture");
  gMsc->productID("Fracture Songs");
  gMsc->productRevision("1.0");
  gMsc->onRead(mscRead);
  gMsc->onWrite(mscWrite);
  gMsc->onStartStop(mscStartStop);
  gMsc->mediaPresent(true);
  gMsc->isWritable(true);
  gMscBlockSize = MSC_FALLBACK_BLOCK_SIZE;
  gMscReady = gMsc->begin(FALLBACK_MSC_BLOCK_COUNT, MSC_FALLBACK_BLOCK_SIZE);
  gMscMediaPresent = gMscReady;
  if (!gMscReady) {
    gMsc->mediaPresent(false);
    gMscUsesFallback = false;
  }
  return gMscReady;
}

bool writeLoaderImageSector(File &file, uint32_t sectorIndex, const uint8_t *sector) {
  if (!file.seek((uint32_t)(sectorIndex * LOADER_IMAGE_BLOCK_SIZE))) return false;
  return file.write(sector, LOADER_IMAGE_BLOCK_SIZE) == LOADER_IMAGE_BLOCK_SIZE;
}

bool formatLoaderImage() {
  if (!mountStorage()) return false;
  if (gLoaderImageOpen) {
    gLoaderImageFile.close();
    gLoaderImageOpen = false;
  }

  FFat.remove(LOADER_IMAGE_PATH);
  File file = FFat.open(LOADER_IMAGE_PATH, FILE_WRITE);
  if (!file) return false;

  uint8_t sector[LOADER_IMAGE_BLOCK_SIZE];
  memset(sector, 0, sizeof(sector));
  sector[0] = 0xEB;
  sector[1] = 0x3C;
  sector[2] = 0x90;
  copyPadded(sector + 3, 8, "FRACLOAD");
  putLe16(sector + 11, LOADER_IMAGE_BLOCK_SIZE);
  sector[13] = LOADER_IMAGE_SECTORS_PER_CLUSTER;
  putLe16(sector + 14, LOADER_IMAGE_RESERVED_SECTORS);
  sector[16] = LOADER_IMAGE_FAT_COUNT;
  putLe16(sector + 17, LOADER_IMAGE_ROOT_ENTRIES);
  putLe16(sector + 19, LOADER_IMAGE_BLOCK_COUNT);
  sector[21] = 0xF8;
  putLe16(sector + 22, LOADER_IMAGE_FAT_SECTORS);
  putLe16(sector + 24, 1);
  putLe16(sector + 26, 1);
  putLe32(sector + 28, 0);
  putLe32(sector + 32, 0);
  sector[38] = 0x29;
  putLe32(sector + 39, 0x46524143UL);
  copyPadded(sector + 43, 11, "SONG LOADER");
  copyPadded(sector + 54, 8, "FAT12");
  sector[510] = 0x55;
  sector[511] = 0xAA;
  if (!writeLoaderImageSector(file, 0, sector)) {
    file.close();
    return false;
  }

  memset(sector, 0, sizeof(sector));
  setFallbackFat12Pair(sector, 0, 0xFF8, 0xFFF);
  for (uint16_t i = 0; i < LOADER_IMAGE_FAT_SECTORS; ++i) {
    if (!writeLoaderImageSector(file, LOADER_IMAGE_RESERVED_SECTORS + i, sector)) {
      file.close();
      return false;
    }
    memset(sector, 0, sizeof(sector));
  }

  memset(sector, 0, sizeof(sector));
  copyPadded(sector, 11, "SONG LOADER");
  sector[11] = 0x08;
  for (uint32_t i = 0; i < LOADER_IMAGE_ROOT_SECTORS; ++i) {
    if (!writeLoaderImageSector(file, LOADER_IMAGE_ROOT_START_SECTOR + i, sector)) {
      file.close();
      return false;
    }
    memset(sector, 0, sizeof(sector));
  }

  memset(sector, 0, sizeof(sector));
  for (uint32_t sectorIndex = LOADER_IMAGE_DATA_START_SECTOR;
       sectorIndex < LOADER_IMAGE_BLOCK_COUNT; ++sectorIndex) {
    if (!writeLoaderImageSector(file, sectorIndex, sector)) {
      file.close();
      return false;
    }
  }

  file.flush();
  file.close();
  return true;
}

bool ensureLoaderImageOpen() {
  if (gLoaderImageOpen && gLoaderImageFile) return true;
  if (!gStorageMounted && !mountStorage()) return false;
  gLoaderImageFile = FFat.open(LOADER_IMAGE_PATH, "r+");
  if (!gLoaderImageFile || gLoaderImageFile.size() < LOADER_IMAGE_BYTE_COUNT) {
    if (gLoaderImageFile) gLoaderImageFile.close();
    gLoaderImageOpen = false;
    return false;
  }
  gLoaderImageOpen = true;
  return true;
}

void closeLoaderImage() {
  if (!gLoaderImageOpen) return;
  gLoaderImageFile.flush();
  gLoaderImageFile.close();
  gLoaderImageOpen = false;
}

bool readLoaderImageBytes(uint32_t address, uint8_t *buffer, uint32_t length) {
  if (address + length > LOADER_IMAGE_BYTE_COUNT || !ensureLoaderImageOpen()) return false;
  if (!gLoaderImageFile.seek(address)) return false;
  return gLoaderImageFile.read(buffer, length) == (int)length;
}

bool writeLoaderImageBytes(uint32_t address, const uint8_t *buffer, uint32_t length) {
  if (address + length > LOADER_IMAGE_BYTE_COUNT || !ensureLoaderImageOpen()) return false;
  if (!gLoaderImageFile.seek(address)) return false;
  return gLoaderImageFile.write(buffer, length) == length;
}

uint16_t loaderFat12Entry(const uint8_t *fat, size_t fatSize, uint16_t cluster) {
  size_t offset = cluster + cluster / 2;
  if (offset + 1 >= fatSize) return 0xFFF;
  if ((cluster & 1) == 0) {
    return (uint16_t)(fat[offset] | ((fat[offset + 1] & 0x0F) << 8));
  }
  return (uint16_t)(((fat[offset] >> 4) & 0x0F) | (fat[offset + 1] << 4));
}

void sfnToName(const uint8_t *entry, char *name, size_t nameSize) {
  size_t out = 0;
  int endBase = 7;
  while (endBase >= 0 && entry[endBase] == ' ') --endBase;
  for (int i = 0; i <= endBase && out + 1 < nameSize; ++i) {
    name[out++] = (char)entry[i];
  }
  int endExt = 10;
  while (endExt >= 8 && entry[endExt] == ' ') --endExt;
  if (endExt >= 8 && out + 1 < nameSize) name[out++] = '.';
  for (int i = 8; i <= endExt && out + 1 < nameSize; ++i) {
    name[out++] = (char)entry[i];
  }
  name[out] = '\0';
}

void clearLoaderLfn(char *lfn, bool &active) {
  lfn[0] = '\0';
  active = false;
}

void readLoaderLfnPart(const uint8_t *entry, char *lfn, size_t lfnSize, bool &active) {
  static const uint8_t offsets[13] = {1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30};
  uint8_t order = entry[0] & 0x1F;
  if (order == 0) return;
  if (entry[0] & 0x40) memset(lfn, 0, lfnSize);
  size_t base = (size_t)(order - 1) * 13;
  if (base >= lfnSize - 1) return;
  for (uint8_t i = 0; i < 13 && base + i < lfnSize - 1; ++i) {
    uint16_t value = (uint16_t)entry[offsets[i]] | ((uint16_t)entry[offsets[i] + 1] << 8);
    if (value == 0x0000 || value == 0xFFFF) break;
    lfn[base + i] = (value >= 32 && value < 127) ? (char)value : '_';
  }
  active = true;
}

void sanitizeImportName(const char *source, char *name, size_t nameSize) {
  size_t out = 0;
  while (*source == ' ' || *source == '.' || *source == '/') ++source;
  for (size_t i = 0; source[i] && out + 1 < nameSize; ++i) {
    char c = source[i];
    bool bad = c < 32 || c == '/' || c == '\\' || c == ':' || c == '*' ||
               c == '?' || c == '"' || c == '<' || c == '>' || c == '|';
    name[out++] = bad ? '_' : c;
  }
  while (out > 0 && (name[out - 1] == ' ' || name[out - 1] == '.')) --out;
  if (out == 0) {
    strncpy(name, "song.mid", nameSize);
    name[nameSize - 1] = '\0';
    return;
  }
  name[out] = '\0';
}

void uniqueImportPath(const char *fileName, char *path, size_t pathSize) {
  char clean[LOADER_IMPORT_NAME_LENGTH];
  sanitizeImportName(fileName, clean, sizeof(clean));

  char base[LOADER_IMPORT_NAME_LENGTH];
  char ext[12] = "";
  const char *dot = strrchr(clean, '.');
  if (dot && dot != clean) {
    size_t baseLength = (size_t)(dot - clean);
    if (baseLength >= sizeof(base)) baseLength = sizeof(base) - 1;
    memcpy(base, clean, baseLength);
    base[baseLength] = '\0';
    strncpy(ext, dot, sizeof(ext) - 1);
    ext[sizeof(ext) - 1] = '\0';
  } else {
    strncpy(base, clean, sizeof(base) - 1);
    base[sizeof(base) - 1] = '\0';
    strncpy(ext, ".mid", sizeof(ext) - 1);
  }

  snprintf(path, pathSize, "/%s%s", base, ext);
  if (!FFat.exists(path)) return;
  for (uint8_t suffix = 1; suffix < 100; ++suffix) {
    snprintf(path, pathSize, "/%s-%u%s", base, suffix, ext);
    if (!FFat.exists(path)) return;
  }
}

bool copyImageFileToStorage(const char *fileName, uint16_t startCluster,
                            uint32_t fileSize, const uint8_t *fat, size_t fatSize) {
  char path[LOADER_IMPORT_PATH_LENGTH];
  uniqueImportPath(fileName, path, sizeof(path));

  File out = FFat.open(path, FILE_WRITE);
  if (!out) return false;
  if (fileSize == 0) {
    out.close();
    return true;
  }
  if (startCluster < 2 || startCluster > LOADER_IMAGE_MAX_CLUSTER) {
    out.close();
    FFat.remove(path);
    return false;
  }

  uint8_t buffer[LOADER_IMAGE_BLOCK_SIZE];
  uint32_t remaining = fileSize;
  uint16_t cluster = startCluster;
  uint16_t guard = 0;
  while (remaining > 0 && cluster >= 2 && cluster < 0xFF8 && guard++ < LOADER_IMAGE_MAX_CLUSTER) {
    uint32_t firstSector = LOADER_IMAGE_DATA_START_SECTOR +
                           (uint32_t)(cluster - 2) * LOADER_IMAGE_SECTORS_PER_CLUSTER;
    for (uint8_t s = 0; s < LOADER_IMAGE_SECTORS_PER_CLUSTER && remaining > 0; ++s) {
      uint32_t chunk = remaining > LOADER_IMAGE_BLOCK_SIZE ? LOADER_IMAGE_BLOCK_SIZE : remaining;
      uint32_t address = (firstSector + s) * LOADER_IMAGE_BLOCK_SIZE;
      if (!readLoaderImageBytes(address, buffer, chunk) || out.write(buffer, chunk) != chunk) {
        out.close();
        FFat.remove(path);
        return false;
      }
      remaining -= chunk;
    }
    if (remaining == 0) break;
    cluster = loaderFat12Entry(fat, fatSize, cluster);
  }

  out.flush();
  out.close();
  if (remaining != 0) {
    FFat.remove(path);
    return false;
  }
  return true;
}

bool importLoaderImage() {
  if (!ensureLoaderImageOpen()) return false;
  const size_t fatSize = (size_t)LOADER_IMAGE_FAT_SECTORS * LOADER_IMAGE_BLOCK_SIZE;
  uint8_t *fat = static_cast<uint8_t *>(malloc(fatSize));
  if (!fat) return false;
  bool ok = readLoaderImageBytes(LOADER_IMAGE_RESERVED_SECTORS * LOADER_IMAGE_BLOCK_SIZE,
                                 fat, fatSize);
  if (!ok) {
    free(fat);
    return false;
  }

  uint8_t sector[LOADER_IMAGE_BLOCK_SIZE];
  char lfn[LOADER_IMPORT_NAME_LENGTH];
  bool lfnActive = false;
  clearLoaderLfn(lfn, lfnActive);
  uint8_t imported = 0;
  for (uint32_t rootSector = 0; rootSector < LOADER_IMAGE_ROOT_SECTORS; ++rootSector) {
    uint32_t address = (LOADER_IMAGE_ROOT_START_SECTOR + rootSector) * LOADER_IMAGE_BLOCK_SIZE;
    if (!readLoaderImageBytes(address, sector, sizeof(sector))) {
      ok = false;
      break;
    }
    for (uint16_t offset = 0; offset < LOADER_IMAGE_BLOCK_SIZE; offset += 32) {
      uint8_t *entry = sector + offset;
      if (entry[0] == 0x00) {
        rootSector = LOADER_IMAGE_ROOT_SECTORS;
        break;
      }
      if (entry[0] == 0xE5) {
        clearLoaderLfn(lfn, lfnActive);
        continue;
      }
      uint8_t attr = entry[11];
      if ((attr & 0x0F) == 0x0F) {
        readLoaderLfnPart(entry, lfn, sizeof(lfn), lfnActive);
        continue;
      }
      if (attr & 0x18) {
        clearLoaderLfn(lfn, lfnActive);
        continue;
      }

      char name[LOADER_IMPORT_NAME_LENGTH];
      if (lfnActive && lfn[0]) {
        strncpy(name, lfn, sizeof(name) - 1);
        name[sizeof(name) - 1] = '\0';
      } else {
        sfnToName(entry, name, sizeof(name));
      }
      clearLoaderLfn(lfn, lfnActive);
      if (!endsWithMidiExtension(name)) continue;

      uint16_t startCluster = (uint16_t)entry[26] | ((uint16_t)entry[27] << 8);
      uint32_t fileSize = ((uint32_t)entry[28]) |
                          ((uint32_t)entry[29] << 8) |
                          ((uint32_t)entry[30] << 16) |
                          ((uint32_t)entry[31] << 24);
      if (copyImageFileToStorage(name, startCluster, fileSize, fat, fatSize)) {
        ++imported;
      } else {
        ok = false;
      }
    }
  }

  free(fat);
  Serial.printf("Imported %u MIDI file(s) from loader image\n", imported);
  return ok;
}

bool configureImageMsc() {
  if (!ensureLoaderImageOpen()) return false;
  if (!gMsc) gMsc = new USBMSC();
  if (!gMsc) return false;
  gMscUsesFallback = false;
  gMscUsesImage = true;
  gMsc->vendorID("Fracture");
  gMsc->productID("Song Loader");
  gMsc->productRevision("1.0");
  gMsc->onRead(mscRead);
  gMsc->onWrite(mscWrite);
  gMsc->onStartStop(mscStartStop);
  gMsc->mediaPresent(true);
  gMsc->isWritable(true);
  gMscBlockSize = LOADER_IMAGE_BLOCK_SIZE;
  gMscReady = gMsc->begin(LOADER_IMAGE_BLOCK_COUNT, LOADER_IMAGE_BLOCK_SIZE);
  gMscMediaPresent = gMscReady;
  if (!gMscReady) {
    gMsc->mediaPresent(false);
    gMscUsesImage = false;
  }
  return gMscReady;
}

bool mountRawStorage() {
  if (!gFatPartition || gRawHandle != WL_INVALID_HANDLE) return gRawHandle != WL_INVALID_HANDLE;
  if (wl_mount(gFatPartition, &gRawHandle) != ESP_OK) {
    gRawHandle = WL_INVALID_HANDLE;
    return false;
  }
  gRawSize = wl_size(gRawHandle);
  gRawEraseSize = wl_sector_size(gRawHandle);
  // FFat's FAT boot sector is formatted with this sector size; USB must match it.
  bool validGeometry = gRawEraseSize > 0 && gRawEraseSize <= UINT16_MAX &&
                       gRawSize >= gRawEraseSize &&
                       (gRawSize % gRawEraseSize) == 0;
  if (!validGeometry) {
    wl_unmount(gRawHandle);
    gRawHandle = WL_INVALID_HANDLE;
    gRawSize = 0;
    gRawEraseSize = 0;
    gMscBlockSize = MSC_FALLBACK_BLOCK_SIZE;
    return false;
  }
  gMscBlockSize = (uint16_t)gRawEraseSize;
  return true;
}

void unmountRawStorage() {
  if (gRawHandle == WL_INVALID_HANDLE) return;
  wl_unmount(gRawHandle);
  gRawHandle = WL_INVALID_HANDLE;
  gRawSize = 0;
  gRawEraseSize = 0;
}

int32_t mscRead(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufferSize) {
  if (gMscUsesFallback) {
    size_t address = (size_t)lba * MSC_FALLBACK_BLOCK_SIZE + offset;
    if (address + bufferSize > sizeof(gFallbackMscDisk)) return -1;
    memcpy(buffer, (uint8_t *)gFallbackMscDisk + address, bufferSize);
    return (int32_t)bufferSize;
  }
  if (gMscUsesImage) {
    uint32_t address = lba * LOADER_IMAGE_BLOCK_SIZE + offset;
    return readLoaderImageBytes(address, (uint8_t *)buffer, bufferSize) ? (int32_t)bufferSize : -1;
  }

  if (gRawHandle == WL_INVALID_HANDLE) return -1;
  size_t address = (size_t)lba * gMscBlockSize + offset;
  if (address + bufferSize > gRawSize) return -1;
  return wl_read(gRawHandle, address, buffer, bufferSize) == ESP_OK ? (int32_t)bufferSize : -1;
}

int32_t mscWrite(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufferSize) {
  if (bufferSize == 0) return 0;
  if (gMscUsesFallback) {
    size_t address = (size_t)lba * MSC_FALLBACK_BLOCK_SIZE + offset;
    if (address + bufferSize > sizeof(gFallbackMscDisk)) return -1;
    memcpy((uint8_t *)gFallbackMscDisk + address, buffer, bufferSize);
    return (int32_t)bufferSize;
  }
  if (gMscUsesImage) {
    uint32_t address = lba * LOADER_IMAGE_BLOCK_SIZE + offset;
    if (!writeLoaderImageBytes(address, buffer, bufferSize)) return -1;
    gMscDirty = true;
    return (int32_t)bufferSize;
  }
  if (gRawHandle == WL_INVALID_HANDLE || !gMscScratch) return -1;
  size_t address = (size_t)lba * gMscBlockSize + offset;
  if (address + bufferSize > gRawSize) return -1;

  size_t sourceOffset = 0;
  while (sourceOffset < bufferSize) {
    size_t currentAddress = address + sourceOffset;
    size_t sectorAddress = currentAddress - (currentAddress % gRawEraseSize);
    size_t withinSector = currentAddress - sectorAddress;
    size_t copyLength = gRawEraseSize - withinSector;
    if (copyLength > bufferSize - sourceOffset) copyLength = bufferSize - sourceOffset;

    bool writesWholeEraseSector = withinSector == 0 && copyLength == gRawEraseSize;
    uint8_t *writeBuffer = buffer + sourceOffset;
    if (!writesWholeEraseSector) {
      if (wl_read(gRawHandle, sectorAddress, gMscScratch, gRawEraseSize) != ESP_OK) return -1;
      memcpy(gMscScratch + withinSector, writeBuffer, copyLength);
      writeBuffer = gMscScratch;
    }
    if (wl_erase_range(gRawHandle, sectorAddress, gRawEraseSize) != ESP_OK ||
        wl_write(gRawHandle, sectorAddress, writeBuffer, gRawEraseSize) != ESP_OK) return -1;
    sourceOffset += copyLength;
  }

  gMscDirty = true;
  return (int32_t)bufferSize;
}

bool mscStartStop(uint8_t powerCondition, bool start, bool loadEject) {
  (void)powerCondition;
  if (!start && loadEject) gMscEjectRequested = true;
  return true;
}

bool readUsbLoadRequest() {
  Preferences preferences;
  bool requested = false;
  if (preferences.begin("fractureSong", true)) {
    requested = preferences.getBool("usb-load", false);
    preferences.end();
  }
  return requested;
}

bool readPostImportRequest() {
  Preferences preferences;
  bool requested = false;
  if (preferences.begin("fractureSong", true)) {
    requested = preferences.getBool("usb-imported", false);
    preferences.end();
  }
  return requested;
}

void writeUsbLoadRequest(bool requested) {
  Preferences preferences;
  if (preferences.begin("fractureSong", false)) {
    preferences.putBool("usb-load", requested);
    preferences.end();
  }
}

bool readRtcUsbLoadRequest() {
  return gRtcUsbLoadMagic == USB_LOAD_RTC_MAGIC &&
         gRtcUsbLoadMagicInverse == ~USB_LOAD_RTC_MAGIC;
}

void writeRtcUsbLoadRequest(bool requested) {
  if (requested) {
    gRtcUsbLoadMagic = USB_LOAD_RTC_MAGIC;
    gRtcUsbLoadMagicInverse = ~USB_LOAD_RTC_MAGIC;
  } else {
    gRtcUsbLoadMagic = 0;
    gRtcUsbLoadMagicInverse = 0;
  }
}

void writePostImportRequest(bool requested) {
  Preferences preferences;
  if (preferences.begin("fractureSong", false)) {
    preferences.putBool("usb-imported", requested);
    preferences.end();
  }
}

void writeLoaderResume() {
  Preferences preferences;
  if (preferences.begin("fractureSong", false)) {
    preferences.putULong("loader-coord", gCoordinatorId);
    preferences.putUChar("loader-session", gSession);
    preferences.putUChar("loader-target", gSongOwner);
    preferences.end();
  }
}

void readLoaderResume() {
  Preferences preferences;
  if (preferences.begin("fractureSong", true)) {
    gCoordinatorId = preferences.getULong("loader-coord", 0);
    gSession = preferences.getUChar("loader-session", 0);
    gSongOwner = preferences.getUChar("loader-target", 0xFF);
    preferences.end();
  }
}

void clearLoaderResume() {
  Preferences preferences;
  if (preferences.begin("fractureSong", false)) {
    preferences.remove("loader-coord");
    preferences.remove("loader-session");
    preferences.remove("loader-target");
    preferences.end();
  }
}

bool configureMsc(bool keepRawMounted, bool mediaPresent) {
  if (!gMsc) gMsc = new USBMSC();
  if (!gMsc) return false;
  gFatPartition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ffat");
  if (!gFatPartition) return false;

  bool hadStorageMounted = gStorageMounted;
  if (hadStorageMounted) unmountStorage();
  if (!mountRawStorage()) {
    if (hadStorageMounted) {
      mountStorage();
      scanLocalSongs();
    }
    return false;
  }
  size_t rawSize = gRawSize;
  uint16_t blockSize = gMscBlockSize;
  size_t eraseSize = gRawEraseSize;
  if (!gMscScratch) gMscScratch = static_cast<uint8_t *>(malloc(eraseSize));
  if (!gMscScratch) {
    unmountRawStorage();
    if (hadStorageMounted) {
      mountStorage();
      scanLocalSongs();
    }
    return false;
  }
  gMscUsesFallback = false;
  gMscUsesImage = false;
  gMsc->vendorID("Fracture");
  gMsc->productID("Fracture Songs");
  gMsc->productRevision("1.0");
  gMsc->onRead(mscRead);
  gMsc->onWrite(mscWrite);
  gMsc->onStartStop(mscStartStop);
  gMsc->mediaPresent(mediaPresent);
  gMsc->isWritable(true);
  gMscReady = gMsc->begin(rawSize / blockSize, blockSize);
  gMscMediaPresent = gMscReady && mediaPresent;
  if (!gMscReady) gMsc->mediaPresent(false);
  Serial.printf("MSC bridge: %u bytes, %u-byte USB blocks, %u-byte WL sectors\n",
                (unsigned int)rawSize, blockSize, (unsigned int)eraseSize);

  if (!keepRawMounted || !gMscReady) {
    unmountRawStorage();
    if (hadStorageMounted) {
      mountStorage();
      scanLocalSongs();
    }
  }
  return gMscReady;
}
#endif

void releaseGuideEvents() {
  if (!gEvents) return;
#if FRACTURE_HAS_HEAP_CAPS
  heap_caps_free(gEvents);
#else
  free(gEvents);
#endif
  gEvents = nullptr;
  gEventCapacity = 0;
}

void *resizeGuideEvents(size_t newCapacity) {
  size_t bytes = newCapacity * sizeof(GuideEvent);
#if FRACTURE_HAS_HEAP_CAPS
  void *memory = heap_caps_realloc(gEvents, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!memory) memory = heap_caps_realloc(gEvents, bytes, MALLOC_CAP_8BIT);
  return memory;
#else
  return realloc(gEvents, bytes);
#endif
}

StreamStats *streamFor(uint8_t id, bool create) {
  for (uint8_t i = 0; i < gStreamCount; ++i) {
    if (gStreams[i].id == id) return &gStreams[i];
  }
  if (!create || gStreamCount >= MAX_STREAMS) return nullptr;
  StreamStats &stream = gStreams[gStreamCount++];
  stream.id = id;
  stream.hand = HAND_UNKNOWN;
  stream.noteCount = 0;
  stream.noteSum = 0;
  stream.minNote = 127;
  stream.maxNote = 0;
  return &stream;
}

bool appendGuideEvent(const midi_event *event) {
  if (gEventCount >= MAX_GUIDE_EVENTS) {
    gEventOverflow = true;
    return false;
  }
  if (gEventCount == gEventCapacity) {
    size_t newCapacity = gEventCapacity == 0 ? 512 : gEventCapacity * 2;
    if (newCapacity > MAX_GUIDE_EVENTS) newCapacity = MAX_GUIDE_EVENTS;
    void *memory = resizeGuideEvents(newCapacity);
    if (!memory) {
      gEventOverflow = true;
      return false;
    }
    gEvents = static_cast<GuideEvent *>(memory);
    gEventCapacity = newCapacity;
  }

  uint8_t streamId = (uint8_t)((event->track << 4) | (event->channel & 0x0F));
  GuideEvent &guide = gEvents[gEventCount++];
  guide.tick = event->tick;
  guide.note = event->data[1] & 0x7F;
  guide.stream = streamId;
  guide.hand = HAND_UNKNOWN;

  StreamStats *stream = streamFor(streamId, true);
  if (stream) {
    stream->noteCount++;
    stream->noteSum += guide.note;
    if (guide.note < stream->minNote) stream->minNote = guide.note;
    if (guide.note > stream->maxNote) stream->maxNote = guide.note;
  } else {
    gStreamOverflow = true;
  }
  return true;
}

void midiEventCallback(midi_event *event) {
  if ((!gParsingMidi && !gAnalyzingMidi) || !event || event->size < 3) return;
  if ((event->data[0] & 0xF0) != 0x90 || event->data[2] == 0 || event->channel == 9) return;

  uint8_t note = event->data[1] & 0x7F;
  if (gAnalyzingMidi) {
    if (!gAnalysisHasNotes || note < gAnalysisMinNote) gAnalysisMinNote = note;
    if (!gAnalysisHasNotes || note > gAnalysisMaxNote) gAnalysisMaxNote = note;
    gAnalysisHasNotes = true;
    return;
  }

  appendGuideEvent(event);
}

bool analyzeMidiFile(const char *path, uint8_t &minNote, uint8_t &maxNote) {
  if (!gStorageMounted || !path || path[0] == '\0') return false;

  gAnalysisHasNotes = false;
  gAnalysisMinNote = 127;
  gAnalysisMaxNote = 0;

  gMidiFile.close();
  gMidiFile.begin(&FFat);
  gMidiFile.setMidiHandler(midiEventCallback);
  gMidiFile.setMetaHandler(midiMetaCallback);
  gAnalyzingMidi = true;
  int error = gMidiFile.load(path);
  if (error == MD_MIDIFile::E_OK) {
    uint32_t loops = 0;
    while (!gMidiFile.isEOF() && loops < 120000UL) {
      gMidiFile.processEvents(0xFFFF);
      if ((++loops & 0x0F) == 0) delay(0);
    }
    if (!gMidiFile.isEOF()) error = MD_MIDIFile::E_HEADER;
  }
  gAnalyzingMidi = false;
  gMidiFile.close();

  if (error != MD_MIDIFile::E_OK || !gAnalysisHasNotes) {
    minNote = 0;
    maxNote = 0;
    return false;
  }

  minNote = gAnalysisMinNote;
  maxNote = gAnalysisMaxNote;
  return true;
}

void midiMetaCallback(const meta_event *event) {
  if (!gParsingMidi || !event || event->type != 0x03 || event->track >= MAX_TRACKS) return;
  size_t length = event->size;
  if (length > 32) length = 32;
  memcpy(gTrackNames[event->track], event->chars, length);
  gTrackNames[event->track][length] = '\0';
}

bool tokenInText(const char *text, const char *token) {
  size_t tokenLength = strlen(token);
  if (tokenLength == 0) return false;
  for (size_t i = 0; text[i]; ++i) {
    size_t j = 0;
    while (j < tokenLength && text[i + j] &&
           lowerAscii(text[i + j]) == lowerAscii(token[j])) {
      ++j;
    }
    if (j != tokenLength) continue;
    char before = i == 0 ? ' ' : text[i - 1];
    char after = text[i + tokenLength];
    bool beforeBoundary = !isalnum((unsigned char)before);
    bool afterBoundary = !isalnum((unsigned char)after);
    if (beforeBoundary && afterBoundary) return true;
  }
  return false;
}

uint8_t namedHandForTrack(uint8_t track) {
  if (track >= MAX_TRACKS) return HAND_UNKNOWN;
  const char *name = gTrackNames[track];
  if (tokenInText(name, "left") || tokenInText(name, "lh") ||
      tokenInText(name, "bass")) {
    return HAND_LEFT;
  }
  if (tokenInText(name, "right") || tokenInText(name, "rh") ||
      tokenInText(name, "treble") || tokenInText(name, "melody")) {
    return HAND_RIGHT;
  }
  return HAND_UNKNOWN;
}

bool allStreamsHaveHands(uint8_t &handMask) {
  handMask = HAND_UNKNOWN;
  for (uint8_t i = 0; i < gStreamCount; ++i) {
    if (gStreams[i].noteCount == 0) continue;
    if (gStreams[i].hand == HAND_UNKNOWN) return false;
    handMask |= gStreams[i].hand;
  }
  return handMask != HAND_UNKNOWN;
}

void applyStreamHandsToEvents() {
  for (size_t e = 0; e < gEventCount; ++e) {
    StreamStats *stream = streamFor(gEvents[e].stream, false);
    gEvents[e].hand = stream ? stream->hand : HAND_UNKNOWN;
  }
}

bool inferStreamHandsByAverage() {
  if (gStreamCount < 2) return false;

  int32_t lowAverage = 127;
  int32_t highAverage = 0;
  for (uint8_t i = 0; i < gStreamCount; ++i) {
    if (gStreams[i].noteCount == 0) continue;
    int32_t average = gStreams[i].noteSum / gStreams[i].noteCount;
    if (average < lowAverage) lowAverage = average;
    if (average > highAverage) highAverage = average;
  }
  if (highAverage - lowAverage < 9) return false;

  int32_t split = (lowAverage + highAverage) / 2;
  for (uint8_t i = 0; i < gStreamCount; ++i) {
    if (gStreams[i].noteCount == 0 || gStreams[i].hand != HAND_UNKNOWN) continue;
    int32_t average = gStreams[i].noteSum / gStreams[i].noteCount;
    gStreams[i].hand = average <= split ? HAND_LEFT : HAND_RIGHT;
  }
  return true;
}

bool inferNoteSplit(uint8_t &splitNote) {
  if (gEventCount < 2) return false;

  uint8_t minNote = 127;
  uint8_t maxNote = 0;
  uint16_t votes[128];
  memset(votes, 0, sizeof(votes));

  for (size_t i = 0; i < gEventCount; ++i) {
    if (gEvents[i].note < minNote) minNote = gEvents[i].note;
    if (gEvents[i].note > maxNote) maxNote = gEvents[i].note;
  }
  if (maxNote <= minNote || (uint8_t)(maxNote - minNote) < 16) return false;

  size_t start = 0;
  while (start < gEventCount) {
    size_t end = start + 1;
    while (end < gEventCount && gEvents[end].tick == gEvents[start].tick) ++end;
    if (end - start >= 2) {
      uint8_t bestGap = 0;
      uint8_t gapLow = 0;
      uint8_t gapHigh = 0;
      for (size_t i = start + 1; i < end; ++i) {
        uint8_t low = gEvents[i - 1].note;
        uint8_t high = gEvents[i].note;
        uint8_t gap = high > low ? high - low : 0;
        if (gap > bestGap) {
          bestGap = gap;
          gapLow = low;
          gapHigh = high;
        }
      }
      if (bestGap >= 7) {
        uint8_t candidate = (uint8_t)(((uint16_t)gapLow + gapHigh) / 2);
        uint16_t weight = (uint16_t)(bestGap - 6) * (uint16_t)(end - start);
        uint16_t nextVotes = votes[candidate] + weight;
        votes[candidate] = nextVotes < votes[candidate] ? 0xFFFF : nextVotes;
      }
    }
    start = end;
  }

  uint16_t bestVotes = 0;
  uint8_t bestSplit = (uint8_t)(((uint16_t)minNote + maxNote) / 2);
  for (uint8_t note = minNote; note <= maxNote && note < 127; ++note) {
    if (votes[note] > bestVotes) {
      bestVotes = votes[note];
      bestSplit = note;
    }
  }

  splitNote = bestVotes > 0 ? bestSplit : (uint8_t)(((uint16_t)minNote + maxNote) / 2);
  return true;
}

uint8_t applyNoteSplitToUnknownEvents(uint8_t splitNote) {
  uint32_t leftCount = 0;
  uint32_t rightCount = 0;
  uint8_t handMask = HAND_UNKNOWN;

  for (size_t e = 0; e < gEventCount; ++e) {
    StreamStats *stream = streamFor(gEvents[e].stream, false);
    uint8_t hand = stream ? stream->hand : HAND_UNKNOWN;
    if (hand == HAND_UNKNOWN) {
      hand = gEvents[e].note <= splitNote ? HAND_LEFT : HAND_RIGHT;
    }
    gEvents[e].hand = hand;
    if (hand == HAND_LEFT) ++leftCount;
    else if (hand == HAND_RIGHT) ++rightCount;
    handMask |= hand;
  }

  uint32_t minimumSide = gEventCount / 25;
  if (minimumSide < 8) minimumSide = 8;
  if (leftCount < minimumSide || rightCount < minimumSide) {
    for (size_t e = 0; e < gEventCount; ++e) gEvents[e].hand = HAND_UNKNOWN;
    return HAND_UNKNOWN;
  }
  return handMask;
}

uint8_t assignHands() {
  if (gStreamOverflow) {
    for (size_t e = 0; e < gEventCount; ++e) gEvents[e].hand = HAND_UNKNOWN;
    return HAND_UNKNOWN;
  }

  for (uint8_t i = 0; i < gStreamCount; ++i) {
    gStreams[i].hand = namedHandForTrack(gStreams[i].id >> 4);
  }

  uint8_t handMask = HAND_UNKNOWN;
  if (allStreamsHaveHands(handMask)) {
    applyStreamHandsToEvents();
    return handMask;
  }

  uint8_t splitNote = 60;
  if (inferNoteSplit(splitNote)) {
    handMask = applyNoteSplitToUnknownEvents(splitNote);
    if (handMask != HAND_UNKNOWN) return handMask;
  }

  for (size_t e = 0; e < gEventCount; ++e) gEvents[e].hand = HAND_UNKNOWN;
  return HAND_UNKNOWN;
}

int compareGuideEvents(const void *leftValue, const void *rightValue) {
  const GuideEvent &left = *static_cast<const GuideEvent *>(leftValue);
  const GuideEvent &right = *static_cast<const GuideEvent *>(rightValue);
  if (left.tick < right.tick) return -1;
  if (left.tick > right.tick) return 1;
  if (left.note != right.note) return (int)left.note - (int)right.note;
  return (int)left.stream - (int)right.stream;
}

bool parseLocalSong(uint8_t localIndex, uint8_t &handMask) {
  if (!gStorageMounted || localIndex >= gLocalSongCount) return false;

  releaseGuideEvents();
  gEventCount = 0;
  gEventCapacity = 0;
  gEventOverflow = false;
  gStreamOverflow = false;
  gStreamCount = 0;
  memset(gTrackNames, 0, sizeof(gTrackNames));

  gMidiFile.close();
  gMidiFile.begin(&FFat);
  gMidiFile.setMidiHandler(midiEventCallback);
  gMidiFile.setMetaHandler(midiMetaCallback);
  gParsingMidi = true;
  int error = gMidiFile.load(gLocalSongs[localIndex].path);
  if (error == MD_MIDIFile::E_OK) {
    uint32_t loops = 0;
    while (!gMidiFile.isEOF() && loops < 200000UL) {
      gMidiFile.processEvents(0xFFFF);
      if ((++loops & 0x0F) == 0) delay(0);
    }
    if (!gMidiFile.isEOF()) error = MD_MIDIFile::E_HEADER;
  }
  gParsingMidi = false;
  gMidiFile.close();

  if (error != MD_MIDIFile::E_OK || gEventCount == 0) {
    Serial.printf("Song load failed (%d): %s\n", error, gLocalSongs[localIndex].path);
    clearLoadedSong();
    return false;
  }

  qsort(gEvents, gEventCount, sizeof(GuideEvent), compareGuideEvents);
  handMask = assignHands();
  if (gEventOverflow) {
    Serial.printf("Song truncated to %u note events: %s\n",
                  (unsigned)gEventCount, gLocalSongs[localIndex].path);
  } else {
    Serial.printf("Song ready: %s (%u note events)\n",
                  gLocalSongs[localIndex].title, (unsigned)gEventCount);
  }
  return true;
}

void sortNetworkSongs() {
  for (uint8_t i = 1; i < gNetworkSongCount; ++i) {
    NetworkSong value = gNetworkSongs[i];
    int8_t j = i - 1;
    while (j >= 0 && compareText(gNetworkSongs[j].title, value.title) > 0) {
      gNetworkSongs[j + 1] = gNetworkSongs[j];
      --j;
    }
    gNetworkSongs[j + 1] = value;
  }
}

uint8_t localSongCatalogFlags(const LocalSong &song) {
  if (!song.hasNotes) return 0;
  uint8_t flags = CATALOG_FLAG_HAS_NOTES;
  uint16_t supportedSpan = (uint16_t)(gBoardCount == 0 ? 1 : gBoardCount) * NOTES_PER_BOARD;
  uint16_t songSpan = song.maxNote >= song.minNote ?
      (uint16_t)song.maxNote - (uint16_t)song.minNote + 1 : 0;
  if (songSpan > supportedSpan) flags |= CATALOG_FLAG_SPAN_WIDE;
  return flags;
}

void addNetworkSong(uint8_t owner, uint8_t localIndex, uint8_t flags,
                    const char *title, size_t titleLength) {
  if (gNetworkSongCount >= MAX_NETWORK_SONGS || localIndex == 0xFF) return;
  for (uint8_t i = 0; i < gNetworkSongCount; ++i) {
    if (gNetworkSongs[i].ownerBoard == owner &&
        gNetworkSongs[i].localIndex == localIndex) return;
  }
  NetworkSong &song = gNetworkSongs[gNetworkSongCount++];
  song.ownerBoard = owner;
  song.localIndex = localIndex;
  song.flags = flags;
  if (titleLength > SONG_NAME_LENGTH) titleLength = SONG_NAME_LENGTH;
  memcpy(song.title, title, titleLength);
  song.title[titleLength] = '\0';
}

const char *browserSelectionTitle() {
  if (gBrowserSelection == 0) return "LOAD SONG";
  uint8_t index = gBrowserSelection - 1;
  return index < gNetworkSongCount ? gNetworkSongs[index].title : "LOAD SONG";
}

uint8_t browserSelectionFlags() {
  if (gBrowserSelection == 0) return TITLE_FLAG_LOAD_ITEM;
  uint8_t index = gBrowserSelection - 1;
  if (index >= gNetworkSongCount) return TITLE_FLAG_LOAD_ITEM;

  uint8_t flags = index & TITLE_COLOR_MASK;
  if (!(gNetworkSongs[index].flags & CATALOG_FLAG_HAS_NOTES) ||
      (gNetworkSongs[index].flags & CATALOG_FLAG_SPAN_WIDE)) {
    flags |= TITLE_FLAG_ATTENTION;
  }
  return flags;
}

void finishCatalogScan() {
  gCatalogScanning = false;
  gCatalogWaiting = false;
  sortNetworkSongs();
  uint8_t itemCount = gNetworkSongCount + 1;
  if (gPreferFirstSongAfterScan && gNetworkSongCount > 0) {
    gBrowserSelection = 1;
  } else if (gBrowserSelection >= itemCount) {
    gBrowserSelection = 0;
  }
  gPreferFirstSongAfterScan = false;
  broadcastTitle(browserSelectionTitle(), browserSelectionFlags());
}

void startCatalogScan() {
  if (!isCoordinator()) return;
  gNetworkSongCount = 0;
  gBrowserSelection = 0;
  gCatalogTarget = 0;
  gCatalogWaiting = false;
  gCatalogTargetComplete = false;
  gCatalogScanning = true;
  gCatalogNextActionMs = millis() + 30;
  broadcastTitle("SCANNING");
}

void addOwnCatalog() {
  for (uint8_t i = 0; i < gLocalSongCount; ++i) {
    addNetworkSong(gBoardIndex, i, localSongCatalogFlags(gLocalSongs[i]),
                   gLocalSongs[i].title, strlen(gLocalSongs[i].title));
  }
}

void sendCatalogRequest(uint8_t targetBoard) {
  if (!gSendPacket) return;
  uint8_t payload[6];
  encodeU32(payload, gCoordinatorId);
  payload[4] = gSession;
  payload[5] = targetBoard;
  gSendPacket(PACKET_CATALOG_REQUEST, payload, sizeof(payload));
}

void serviceCatalogScan(uint32_t nowMs) {
  if (!isCoordinator() || !gCatalogScanning || nowMs < gCatalogNextActionMs) return;
  if (gCatalogTarget >= gBoardCount) {
    finishCatalogScan();
    return;
  }

  if (gCatalogTarget == gBoardIndex) {
    addOwnCatalog();
    ++gCatalogTarget;
    gCatalogNextActionMs = nowMs + 12;
    return;
  }

  if (!gCatalogWaiting) {
    gCatalogTargetComplete = false;
    sendCatalogRequest(gCatalogTarget);
    gCatalogWaiting = true;
    gCatalogDeadlineMs = nowMs + CATALOG_TIMEOUT_MS;
    return;
  }

  if (gCatalogTargetComplete || nowMs >= gCatalogDeadlineMs) {
    gCatalogWaiting = false;
    ++gCatalogTarget;
    gCatalogNextActionMs = nowMs + 12;
  }
}

void serviceCatalogReply(uint32_t nowMs) {
  if (!gCatalogReplyPending || nowMs < gCatalogReplyNextMs || !gSendPacket) return;

  uint8_t payload[32];
  encodeU32(payload, gCatalogReplyCoordinator);
  payload[4] = gCatalogReplySession;
  payload[5] = gBoardIndex;
  payload[7] = gLocalSongCount;

  if (gLocalSongCount == 0) {
    if (gCatalogReplySentEmpty) {
      gCatalogReplyPending = false;
      return;
    }
    payload[6] = 0xFF;
    gSendPacket(PACKET_CATALOG_ITEM, payload, 8);
    gCatalogReplySentEmpty = true;
    gCatalogReplyNextMs = nowMs + CATALOG_REPLY_GAP_MS;
    return;
  }

  if (gCatalogReplyIndex >= gLocalSongCount) {
    gCatalogReplyPending = false;
    return;
  }

  LocalSong &song = gLocalSongs[gCatalogReplyIndex];
  payload[6] = (gCatalogReplyIndex & CATALOG_LOCAL_INDEX_MASK) |
               localSongCatalogFlags(song);
  uint8_t titleLength = strlen(song.title);
  if (titleLength > SONG_NAME_LENGTH) titleLength = SONG_NAME_LENGTH;
  memcpy(payload + 8, song.title, titleLength);
  gSendPacket(PACKET_CATALOG_ITEM, payload, 8 + titleLength);
  ++gCatalogReplyIndex;
  gCatalogReplyNextMs = nowMs + CATALOG_REPLY_GAP_MS;
}

void applyImportWave() {
  gImportWaveStartMs = millis() + 80;
}

void broadcastImportWave() {
  applyImportWave();
  if (!gSendPacket) return;
  uint8_t payload[4];
  encodeU32(payload, gNodeId);
  sendReliable(PACKET_WAVE, payload, sizeof(payload));
}

void sendCommand(uint8_t command, uint8_t first = 0, uint8_t second = 0) {
  if (!gSendPacket) return;
  uint8_t payload[8];
  encodeU32(payload, gCoordinatorId);
  payload[4] = gSession;
  payload[5] = command;
  payload[6] = first;
  payload[7] = second;
  gSendPacket(PACKET_COMMAND, payload, sizeof(payload));
}

void applyGuidePacket(const uint8_t *payload, uint8_t len);
void sendNextGuideStep();
void coordinatorConfirm(uint8_t requesterBoard);
void coordinatorBack();
void coordinatorNavigate(int8_t direction);
void coordinatorNextOrLoad(uint8_t requesterBoard);
void coordinatorImportComplete(uint8_t sourceBoard);
void prepareUsbLoad();
#if FRACTURE_SONG_MSC_ENABLED
void requestUsbExit();
#endif
void ownerCommand(uint8_t command, uint8_t first, uint8_t second);
void readyPacket(const uint8_t *payload, uint8_t len);
void sendReady(bool success, uint8_t handMask) {
  uint8_t payload[8];
  encodeU32(payload, gCoordinatorId);
  payload[4] = gSession;
  payload[5] = gBoardIndex;
  payload[6] = success ? 1 : 0;
  payload[7] = handMask;
  readyPacket(payload, sizeof(payload));
  sendReliable(PACKET_READY, payload, sizeof(payload));
}

void servicePendingLoad() {
  if (!gPendingLoad) return;
  gPendingLoad = false;
  uint8_t handMask = HAND_UNKNOWN;
  bool success = parseLocalSong(gPendingLocalSong, handMask);
  gSongOwner = success ? gBoardIndex : 0xFF;
  gAvailableHands = handMask;
  sendReady(success, handMask);
}

bool eventMatchesSelectedHand(const GuideEvent &event) {
  if (gLearningHand == HAND_BOTH) return true;
  return event.hand == gLearningHand;
}

void sendNextGuideStep() {
  if (gSongOwner != gBoardIndex || gEventCount == 0) return;

  uint8_t encodedNotes[MAX_GUIDE_NOTES];
  uint8_t noteCount = 0;
  while (gEventCursor < gEventCount && noteCount == 0) {
    size_t first = gEventCursor;
    while (first < gEventCount && !eventMatchesSelectedHand(gEvents[first])) ++first;
    if (first >= gEventCount) {
      gEventCursor = gEventCount;
      break;
    }

    uint32_t tick = gEvents[first].tick;
    size_t end = first;
    while (end < gEventCount && gEvents[end].tick == tick) ++end;
    gEventCursor = end;

    for (size_t i = first; i < end && noteCount < MAX_GUIDE_NOTES; ++i) {
      const GuideEvent &event = gEvents[i];
      if (!eventMatchesSelectedHand(event)) continue;
      bool duplicate = false;
      for (uint8_t n = 0; n < noteCount; ++n) {
        if ((encodedNotes[n] & 0x7F) == event.note) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        encodedNotes[noteCount++] = event.note |
            (event.hand == HAND_LEFT ? 0x80 : 0x00);
      }
    }
  }

  uint8_t payload[32];
  encodeU32(payload, gCoordinatorId);
  payload[4] = gSession;
  payload[5] = gBoardIndex;
  ++gGuideSequence;
  payload[6] = gGuideSequence & 0xFF;
  payload[7] = (gGuideSequence >> 8) & 0xFF;
  payload[8] = noteCount == 0 ? GUIDE_FLAG_END : 0;
  payload[9] = noteCount;
  if (noteCount > 0) memcpy(payload + 10, encodedNotes, noteCount);

  applyGuidePacket(payload, 10 + noteCount);
  sendReliable(PACKET_GUIDE_STEP, payload, 10 + noteCount);
}

void ownerCommand(uint8_t command, uint8_t first, uint8_t second) {
  if (command == COMMAND_ENTER_USB_LOADER && first == gBoardIndex) {
    prepareUsbLoad();
    return;
  }
  if (command == COMMAND_EXIT_USB_LOADER && first == gBoardIndex) {
#if FRACTURE_SONG_MSC_ENABLED
    if (gRestartPending && !gBootUsbLoad) {
      gRestartPending = false;
      writeUsbLoadRequest(false);
      writeRtcUsbLoadRequest(false);
      clearLoaderResume();
    } else {
      requestUsbExit();
    }
#endif
    return;
  }

  if (command == COMMAND_LOAD_SELECTED && first == gBoardIndex) {
    clearLoadedSong();
    gSongOwner = gBoardIndex;
    gPendingLocalSong = second;
    gPendingLoad = true;
    return;
  }

  if (command == COMMAND_START_LEARNING && first == gBoardIndex &&
      gSongOwner == gBoardIndex && gEventCount > 0) {
    gLearningHand = second;
    gEventCursor = 0;
    gGuideSequence = 0;
    gHaveGuideSequence = false;
    gAdvancePending = false;
    sendNextGuideStep();
    return;
  }

  if (command == COMMAND_CANCEL_OWNER &&
      (first == gBoardIndex || first == 0xFF)) {
    clearLoadedSong();
  }
}

void sendOwnerCommand(uint8_t command, uint8_t owner, uint8_t value = 0) {
  ownerCommand(command, owner, value);
  sendCommand(command, owner, value);
}

void startLearningFromCoordinator(uint8_t hand) {
  if (!isCoordinator() || gSongOwner == 0xFF) return;
  gHandChoice = hand;
  broadcastMode(MODE_LEARNING, gSongOwner);
  broadcastTitle(gSelectedSongTitle[0] ? gSelectedSongTitle : "PLAY");
  sendOwnerCommand(COMMAND_START_LEARNING, gSongOwner, hand);
}

#if FRACTURE_SONG_MSC_ENABLED
void requestUsbExit() {
  if (gMode != MODE_USB_LOAD || gUsbExitPending || !gBootUsbLoad || !gMscReady) return;
  if (gMsc && gMscReady && gMscMediaPresent) {
    gMsc->mediaPresent(false);
    gMscMediaPresent = false;
  }
  gUsbExitPending = true;
  gUsbExitAtMs = millis() + USB_SETTLE_MS;
}

void scheduleRestart(uint32_t delayMs) {
  gRestartPending = true;
  gRestartAtMs = millis() + delayMs;
}

void serviceRestart(uint32_t nowMs) {
  if (gRestartPending && (int32_t)(nowMs - gRestartAtMs) >= 0) {
    ESP.restart();
  }
}

void finishUsbExit() {
  bool wasDirty = gMscDirty;
  bool filesystemChanged = wasDirty && !gMscUsesFallback;
  gMscDirty = false;
  gMscEjectRequested = false;
  gBootUsbLoad = false;
  // Older firmware used a temporary staging image. Keep its import path for
  // compatibility, but normal loader boots now expose the live FFat library.
  if (gMscUsesImage && wasDirty) {
    filesystemChanged = importLoaderImage();
  }
  if (gMscUsesImage) {
    closeLoaderImage();
    if ((filesystemChanged || !wasDirty) && gStorageMounted) {
      FFat.remove(LOADER_IMAGE_PATH);
    }
  }
  writeUsbLoadRequest(false);
  writeRtcUsbLoadRequest(false);
  // A write through the live MSC volume can be an add, rename, or delete.
  // Rebooting after eject remounts and redistributes the current file catalog.
  writePostImportRequest(filesystemChanged);
  unmountRawStorage();
  gUsbExitPending = false;
  if (!filesystemChanged) {
    clearLoaderResume();
  }
  if (wasDirty && !filesystemChanged) broadcastTitle("FILES ERR");
  scheduleRestart(USB_EXIT_RESTART_DELAY_MS);
}

void prepareUsbLoad() {
  if (gRestartPending) return;
  // Validate the mounted song library before restarting into the USB MSC
  // personality. The next boot hands this live FFat volume to the computer.
  bool storageReady = mountStorage();
  gMscDirty = false;
  gMscEjectRequested = false;
  writeUsbLoadRequest(true);
  writeRtcUsbLoadRequest(true);
  writeLoaderResume();
  if (!storageReady) setTitleLocal("USB ERROR");
  scheduleRestart(USB_ENTER_RESTART_DELAY_MS);
}

void serviceUsb(uint32_t nowMs) {
  if (gMode == MODE_USB_LOAD && gBootUsbLoad && gMscEjectRequested &&
      !gUsbExitPending) {
    requestUsbExit();
  }
  if (gUsbExitPending && nowMs >= gUsbExitAtMs) finishUsbExit();
  serviceRestart(nowMs);
}
#else
void prepareUsbLoad() {
  broadcastTitle(gStorageAvailable ? "NO DRIVE" : "NO FAT");
}
#endif

void coordinatorNavigate(int8_t direction) {
  if (!isCoordinator() || direction == 0) return;
  direction = direction > 0 ? 1 : -1;

  if (gMode == MODE_BROWSER && !gCatalogScanning) {
    int16_t itemCount = (int16_t)gNetworkSongCount + 1;
    int16_t next = (int16_t)gBrowserSelection + direction;
    if (next < 0) next = itemCount - 1;
    if (next >= itemCount) next = 0;
    gBrowserSelection = (uint8_t)next;
    broadcastTitle(browserSelectionTitle(), browserSelectionFlags());
  } else if (gMode == MODE_HAND_SELECT) {
    int8_t choice = gHandChoice == HAND_BOTH ? 0 :
                    (gHandChoice == HAND_LEFT ? 1 : 2);
    choice += direction;
    if (choice < 0) choice = 2;
    if (choice > 2) choice = 0;
    gHandChoice = choice == 0 ? HAND_BOTH :
                  (choice == 1 ? HAND_LEFT : HAND_RIGHT);
    broadcastTitle(gHandChoice == HAND_BOTH ? "B" :
                   (gHandChoice == HAND_LEFT ? "L" : "R"));
  }
}

void coordinatorNextOrLoad(uint8_t requesterBoard) {
  if (!isCoordinator()) return;
  if (gMode == MODE_BROWSER && !gCatalogScanning && gBrowserSelection == 0) {
    coordinatorConfirm(requesterBoard);
  } else {
    coordinatorNavigate(1);
  }
}

void coordinatorConfirm(uint8_t requesterBoard) {
  if (!isCoordinator()) return;
  if (gMode == MODE_BROWSER) {
    if (gCatalogScanning) return;
    if (gBrowserSelection == 0) {
      uint8_t target = requesterBoard < gBoardCount ? requesterBoard : gBoardIndex;
      gSongOwner = target;
      broadcastMode(MODE_USB_LOAD, target);
      char title[18];
      snprintf(title, sizeof(title), "USB BOARD %u", (unsigned)(target + 1));
      broadcastTitle(title);
      sendOwnerCommand(COMMAND_ENTER_USB_LOADER, target);
      return;
    }

    uint8_t index = gBrowserSelection - 1;
    if (index >= gNetworkSongCount) return;
    NetworkSong &song = gNetworkSongs[index];
    gSongOwner = song.ownerBoard;
    gSelectedSongFlags = song.flags;
    strncpy(gSelectedSongTitle, song.title, SONG_NAME_LENGTH);
    gSelectedSongTitle[SONG_NAME_LENGTH] = '\0';
    broadcastMode(MODE_LOADING, gSongOwner);
    broadcastTitle("LOADING");
    sendOwnerCommand(COMMAND_LOAD_SELECTED, song.ownerBoard, song.localIndex);
  } else if (gMode == MODE_HAND_SELECT) {
    startLearningFromCoordinator(gHandChoice);
  } else if (gMode == MODE_COMPLETE) {
    broadcastMode(MODE_BROWSER);
    broadcastTitle(browserSelectionTitle(), browserSelectionFlags());
  }
}

void coordinatorBack() {
  if (!isCoordinator()) return;
#if FRACTURE_SONG_MSC_ENABLED
  if (gMode == MODE_USB_LOAD) {
    // Leave the shared UI immediately; the target will see the targeted
    // cancellation below if it is already in its loader boot.
    broadcastMode(MODE_BROWSER);
    broadcastTitle(browserSelectionTitle(), browserSelectionFlags());
    if (gBootUsbLoad && gMscReady) {
      requestUsbExit();
      return;
    }
    sendOwnerCommand(COMMAND_EXIT_USB_LOADER, gSongOwner);
    return;
  }
#endif

  if (gMode == MODE_BROWSER) {
    sendOwnerCommand(COMMAND_CANCEL_OWNER, gSongOwner);
    broadcastMode(MODE_NORMAL);
    setTitleLocal("");
    clearLoadedSong();
    return;
  }

  if (gMode != MODE_NORMAL) {
    sendOwnerCommand(COMMAND_CANCEL_OWNER, gSongOwner);
    broadcastMode(MODE_BROWSER);
    broadcastTitle(browserSelectionTitle(), browserSelectionFlags());
  }
}

void coordinatorImportComplete(uint8_t sourceBoard) {
  if (!isCoordinator() || gMode != MODE_USB_LOAD || sourceBoard != gSongOwner) return;
  gPreferFirstSongAfterScan = true;
  broadcastMode(MODE_BROWSER);
  broadcastImportWave();
  startCatalogScan();
}

void readyPacket(const uint8_t *payload, uint8_t len) {
  if (len != 8 || decodeU32(payload) != gCoordinatorId ||
      payload[4] != gSession || !isCoordinator() || gMode != MODE_LOADING ||
      payload[5] != gSongOwner) return;

  uint8_t owner = payload[5];
  bool success = payload[6] != 0;
  uint8_t handMask = payload[7];
  if (!success) {
    gSongOwner = 0xFF;
    broadcastMode(MODE_BROWSER);
    broadcastTitle("MIDI ERROR");
    return;
  }

  gSongOwner = owner;
  gAvailableHands = handMask;
  if ((handMask & HAND_BOTH) == HAND_BOTH) {
    gHandChoice = HAND_BOTH;
    broadcastMode(MODE_HAND_SELECT, handMask);
    broadcastTitle("B");
  } else {
    startLearningFromCoordinator(HAND_BOTH);
  }
}

void applyGuidePacket(const uint8_t *payload, uint8_t len) {
  if (len < 10 || decodeU32(payload) != gCoordinatorId || payload[4] != gSession) return;
  gLastModePacketMs = millis();
  uint8_t count = payload[9];
  if (count > MAX_GUIDE_NOTES || len != (uint8_t)(10 + count)) return;
  uint16_t sequence = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
  if (gHaveGuideSequence && sequence == gLastGuideSequence) return;
  gHaveGuideSequence = true;
  gLastGuideSequence = sequence;
  gSongOwner = payload[5];
  gGuideCount = count;
  gAdvancePending = false;

  for (uint8_t i = 0; i < count; ++i) {
    gGuideNotes[i] = payload[10 + i] & 0x7F;
    gGuideLeft[i] = (payload[10 + i] & 0x80) != 0;
    gGuideHit[i] = false;
  }

  if (payload[8] & GUIDE_FLAG_END) {
    gGuideCount = 0;
    applyMode(MODE_COMPLETE);
    setTitleLocal("DONE");
  }
}

void sendGuideHeartbeat() {
  if (gMode != MODE_LEARNING || gGuideCount == 0 || !gSendPacket) return;
  uint8_t encodedNotes[MAX_GUIDE_NOTES];
  for (uint8_t i = 0; i < gGuideCount; ++i) {
    encodedNotes[i] = gGuideNotes[i] | (gGuideLeft[i] ? 0x80 : 0x00);
  }

  uint8_t payload[32];
  encodeU32(payload, gCoordinatorId);
  payload[4] = gSession;
  payload[5] = gSongOwner;
  payload[6] = gLastGuideSequence & 0xFF;
  payload[7] = (gLastGuideSequence >> 8) & 0xFF;
  payload[8] = 0;
  payload[9] = gGuideCount;
  memcpy(payload + 10, encodedNotes, gGuideCount);
  sendReliable(PACKET_GUIDE_STEP, payload, 10 + gGuideCount);
}

void sendModeHeartbeat(uint32_t nowMs) {
  if (!isCoordinator() || gMode == MODE_NORMAL) return;
#if FRACTURE_SONG_MSC_ENABLED
  if (gRestartPending || gBootUsbLoad) return;
#endif
  if (nowMs - gLastModeHeartbeatMs < MODE_HEARTBEAT_MS) return;
  gLastModeHeartbeatMs = nowMs;
  uint8_t argument = gMode == MODE_HAND_SELECT ? gAvailableHands : gSongOwner;
  broadcastMode(gMode, argument);
  if (gTitle[0] != '\0') broadcastTitle(gTitle, gTitleFlags);
  sendGuideHeartbeat();
}

void handleCoordinatorCommand(uint8_t command, uint8_t first, uint8_t second) {
  (void)second;
  if (!isCoordinator()) return;
  if (command == COMMAND_NAVIGATE) coordinatorNavigate((int8_t)first);
  else if (command == COMMAND_CONFIRM) coordinatorConfirm(first);
  else if (command == COMMAND_BACK) coordinatorBack();
  else if (command == COMMAND_NEXT_OR_LOAD) coordinatorNextOrLoad(first);
  else if (command == COMMAND_IMPORT_COMPLETE) coordinatorImportComplete(first);
}

bool handleModePacket(const uint8_t *payload, uint8_t len) {
  if (len < 7) return true;
  uint32_t coordinator = decodeU32(payload);
  uint8_t session = payload[4];
  SongMode mode = (SongMode)payload[5];
  uint8_t argument = payload[6];
  uint16_t incomingAge = len >= 9 ?
      (uint16_t)payload[7] | ((uint16_t)payload[8] << 8) : 0;
  if (mode > MODE_COMPLETE) return true;

  uint32_t nowMs = millis();
  bool sameSession = coordinator == gCoordinatorId && session == gSession;
  bool localStale = gMode != MODE_NORMAL && !isCoordinator() &&
      (gLastModePacketMs == 0 || nowMs - gLastModePacketMs > MODE_STALE_MS);
  uint16_t localAge = modeAgeSeconds();
  bool incomingOlder = mode != MODE_NORMAL && incomingAge > localAge;
  uint16_t ageGap = incomingAge > localAge ? incomingAge - localAge : localAge - incomingAge;
  // Independent sessions created at nearly the same moment need a stable
  // answer instead of two coordinators waiting for a later heartbeat.
  bool authorityTie = mode != MODE_NORMAL && ageGap <= 1 &&
      gCoordinatorId != 0 && coordinator < gCoordinatorId;
  bool shouldAdopt = !sameSession &&
      (gMode == MODE_NORMAL || gCoordinatorId == 0 || localStale || incomingOlder || authorityTie);
  if (shouldAdopt) {
    clearLoadedSong();
    gCoordinatorId = coordinator;
    gSession = session;
    applyModeArgument(mode, argument);
    applyMode(mode);
    carryIncomingModeAge(incomingAge);
    gLastModePacketMs = nowMs;
    if (mode == MODE_BROWSER) setTitleLocal("SCANNING");
    if (mode == MODE_NORMAL) setTitleLocal("");
    return true;
  }

  if (!sameSession) return true;
  SongMode previousMode = gMode;
  applyModeArgument(mode, argument);
  applyMode(mode);
  if (previousMode != mode) carryIncomingModeAge(incomingAge);
  gLastModePacketMs = nowMs;
  if (mode == MODE_NORMAL) {
    setTitleLocal("");
    clearLoadedSong();
  }
  return true;
}

bool handleCatalogRequestPacket(const uint8_t *payload, uint8_t len) {
  if (len != 6 || payload[5] != gBoardIndex) return true;
  if (decodeU32(payload) != gCoordinatorId || payload[4] != gSession) return true;
  gCatalogReplyCoordinator = gCoordinatorId;
  gCatalogReplySession = gSession;
  gCatalogReplyIndex = 0;
  gCatalogReplySentEmpty = false;
  gCatalogReplyPending = true;
  gCatalogReplyNextMs = millis() + 4 + (gBoardIndex % 3) * 2;
  return true;
}

bool handleCatalogItemPacket(const uint8_t *payload, uint8_t len) {
  if (len < 8 || decodeU32(payload) != gNodeId || !isCoordinator() ||
      payload[4] != gSession) return true;
  uint8_t owner = payload[5];
  uint8_t packedIndex = payload[6];
  uint8_t total = payload[7];
  if (!gCatalogScanning || owner != gCatalogTarget) return true;
  if (packedIndex != 0xFF) {
    uint8_t localIndex = packedIndex & CATALOG_LOCAL_INDEX_MASK;
    uint8_t flags = packedIndex & (CATALOG_FLAG_HAS_NOTES | CATALOG_FLAG_SPAN_WIDE);
    addNetworkSong(owner, localIndex, flags, (const char *)(payload + 8), len - 8);
    if (localIndex + 1 >= total) gCatalogTargetComplete = true;
  }
  if (total == 0 || packedIndex == 0xFF) {
    gCatalogTargetComplete = true;
  }
  return true;
}

bool handleTitlePacket(const uint8_t *payload, uint8_t len) {
  if (len < 6 || decodeU32(payload) != gCoordinatorId || payload[4] != gSession) return true;
  char title[TITLE_LENGTH + 1];
  uint8_t flags = payload[5];
  uint8_t length = len - 6;
  if (length > TITLE_LENGTH) length = TITLE_LENGTH;
  memcpy(title, payload + 6, length);
  title[length] = '\0';
  setTitleLocal(title, flags);
  gLastModePacketMs = millis();
  return true;
}

bool handleCommandPacket(const uint8_t *payload, uint8_t len) {
  if (len != 8 || decodeU32(payload) != gCoordinatorId || payload[4] != gSession) return true;
  uint8_t command = payload[5];
  uint8_t first = payload[6];
  uint8_t second = payload[7];
  handleCoordinatorCommand(command, first, second);
  ownerCommand(command, first, second);
  return true;
}

}  // namespace

void beginBeforeUsb(PacketSender sender) {
  gSendPacket = sender;
#if FRACTURE_SONG_MSC_ENABLED
  gBootUsbLoad = readUsbLoadRequest() || readRtcUsbLoadRequest();
  gPostImportPending = !gBootUsbLoad && readPostImportRequest();
  if (gBootUsbLoad || gPostImportPending) readLoaderResume();
  if (gBootUsbLoad) {
    // Expose the actual FFat song library so the host can add, rename, and
    // delete files in its normal file explorer. FFat stays unmounted locally
    // until the host ejects and this board restarts.
    if (configureMsc(true, true)) {
      gStorageAvailable = true;
      gLocalSongCount = 0;
      gStorageSignature = 0;
    } else if (configureFallbackMsc("Song library could not be shared. Check FFat formatting.")) {
      gStorageAvailable = false;
      gLocalSongCount = 0;
      gStorageSignature = 0;
    } else {
      gBootUsbLoad = false;
      writeUsbLoadRequest(false);
      writeRtcUsbLoadRequest(false);
      mountStorage();
      scanLocalSongs();
    }
  } else {
    mountStorage();
    scanLocalSongs();
  }
  if (gBootUsbLoad) {
    // The request got us into this boot personality. Keep the in-RAM flag for
    // this USB session, but consume persistent intent so a reset/power loss
    // cannot strand a board in loader mode without the player.
    writeUsbLoadRequest(false);
    writeRtcUsbLoadRequest(false);
    applyMode(MODE_USB_LOAD);
    setTitleLocal(gMscUsesFallback ? "USB ERROR" : "FILES");
  }
  Serial.printf("Song storage: %s, %u MIDI files, USB loader: %s\n",
                gBootUsbLoad ? "shared with USB host" :
                               (gStorageMounted ? "ready" : "unavailable"),
                gLocalSongCount,
                gBootUsbLoad ? "active" : "standby");
#else
  mountStorage();
  scanLocalSongs();
  Serial.printf("Song storage: %s, %u MIDI files, USB loader: unavailable\n",
                gStorageMounted ? "ready" : "unavailable",
                gLocalSongCount);
#endif
}

bool isUsbLoaderBoot() {
#if FRACTURE_SONG_MSC_ENABLED
  return gBootUsbLoad && gMscReady;
#else
  return false;
#endif
}

void setTopology(uint32_t nodeId, uint8_t boardIndex, uint8_t boardCount) {
  gNodeId = nodeId;
  gBoardIndex = boardIndex;
  gBoardCount = boardCount == 0 ? 1 : boardCount;
#if FRACTURE_SONG_MSC_ENABLED
  if (gPostImportPending && gInitialTopologySeen) {
    gPostImportPending = false;
    writePostImportRequest(false);
    if (gCoordinatorId == 0 || isCoordinator()) {
      if (gCoordinatorId == 0) {
        ++gSessionCounter;
        if (gSessionCounter == 0) ++gSessionCounter;
        gSession = (uint8_t)(gSessionCounter ^ (uint8_t)nodeId ^ (uint8_t)millis());
        if (gSession == 0) gSession = 1;
        gCoordinatorId = nodeId;
      }
      gPreferFirstSongAfterScan = true;
      broadcastMode(MODE_BROWSER);
      broadcastImportWave();
      startCatalogScan();
    } else {
      sendCommand(COMMAND_IMPORT_COMPLETE, gBoardIndex);
    }
    clearLoaderResume();
  } else {
    gInitialTopologySeen = true;
  }
#endif
  if (isCoordinator() && gMode != MODE_NORMAL) {
    uint32_t nowMs = millis();
    gLastModeHeartbeatMs = nowMs > MODE_HEARTBEAT_MS ? nowMs - MODE_HEARTBEAT_MS : 0;
  }
#if FRACTURE_SONG_MSC_ENABLED
  if (gBootUsbLoad && !gUsbLoadModeAnnounced) {
    if (gCoordinatorId == 0 || gSession == 0) {
      ++gSessionCounter;
      if (gSessionCounter == 0) ++gSessionCounter;
      gSession = (uint8_t)(gSessionCounter ^ (uint8_t)gNodeId ^ (uint8_t)millis());
      if (gSession == 0) gSession = 1;
      gCoordinatorId = gNodeId;
      gSongOwner = gBoardIndex;
      broadcastMode(MODE_USB_LOAD, gSongOwner);
      broadcastTitle(gMscUsesFallback ? "USB ERROR" : "LOAD SONG");
    }
    gUsbLoadModeAnnounced = true;
  }
#endif
}

void update() {
  uint32_t nowMs = millis();
  serviceCatalogReply(nowMs);
  serviceCatalogScan(nowMs);
  servicePendingLoad();
  sendModeHeartbeat(nowMs);
#if FRACTURE_SONG_MSC_ENABLED
  serviceUsb(nowMs);
#endif
  if (gAdvancePending && nowMs >= gAdvanceAtMs &&
      gSongOwner == gBoardIndex && gMode == MODE_LEARNING) {
    gAdvancePending = false;
    sendNextGuideStep();
  }
}

bool handlePacket(uint8_t type, const uint8_t *payload, uint8_t len) {
  switch (type) {
    case PACKET_MODE:
      return handleModePacket(payload, len);
    case PACKET_CATALOG_REQUEST:
      return handleCatalogRequestPacket(payload, len);
    case PACKET_CATALOG_ITEM:
      return handleCatalogItemPacket(payload, len);
    case PACKET_TITLE:
      return handleTitlePacket(payload, len);
    case PACKET_COMMAND:
      return handleCommandPacket(payload, len);
    case PACKET_GUIDE_STEP:
      applyGuidePacket(payload, len);
      return true;
    case PACKET_WAVE:
      if (len == 4 && decodeU32(payload) != gNodeId) applyImportWave();
      return true;
    case PACKET_READY:
      readyPacket(payload, len);
      return true;
    default:
      return false;
  }
}

void enterBrowser() {
  if (gMode != MODE_NORMAL) return;
  clearLoadedSong();
  ++gSessionCounter;
  if (gSessionCounter == 0) ++gSessionCounter;
  gSession = (uint8_t)(gSessionCounter ^ (uint8_t)gNodeId ^ (uint8_t)millis());
  if (gSession == 0) gSession = 1;
  gCoordinatorId = gNodeId;
  broadcastMode(MODE_BROWSER);
  startCatalogScan();
}

void back() {
  if (gMode == MODE_NORMAL) return;
  if (isCoordinator()) coordinatorBack();
  else sendCommand(COMMAND_BACK);
}

void confirm() {
  if (gMode == MODE_NORMAL) return;
  if (isCoordinator()) coordinatorConfirm(gBoardIndex);
  else sendCommand(COMMAND_CONFIRM, gBoardIndex);
}

void encoderDelta(int8_t detents) {
  if (gMode == MODE_NORMAL || detents == 0) return;
  int8_t direction = detents > 0 ? 1 : -1;
  if (isCoordinator()) coordinatorNavigate(direction);
  else sendCommand(COMMAND_NAVIGATE, (uint8_t)direction);
}

void nextButton() {
  if (gMode == MODE_NORMAL) return;
  if (isCoordinator()) coordinatorNextOrLoad(gBoardIndex);
  else sendCommand(COMMAND_NEXT_OR_LOAD, gBoardIndex);
}

void showTitle() {
  if (!isActive()) return;
  if (gTitle[0] == '\0' && gSelectedSongTitle[0] != '\0') {
    setTitleLocal(gSelectedSongTitle);
  }
  gTitleEpochMs = millis();
  gTitleRevealUntilMs = millis() + 1800;
}

void noteOn(uint8_t midiNote) {
  if (gMode != MODE_LEARNING || gSongOwner != gBoardIndex || gGuideCount == 0) return;
  for (uint8_t i = 0; i < gGuideCount; ++i) {
    if (gGuideNotes[i] == midiNote) gGuideHit[i] = true;
  }
  for (uint8_t i = 0; i < gGuideCount; ++i) {
    if (!gGuideHit[i]) return;
  }
  if (!gAdvancePending) {
    gAdvancePending = true;
    gAdvanceAtMs = millis() + GUIDE_ADVANCE_DELAY_MS;
  }
}

void localNoteOn(uint8_t midiNote) {
  if (gMode == MODE_BROWSER) {
    if (!gCatalogScanning) {
      if (isCoordinator()) coordinatorConfirm(gBoardIndex);
      else sendCommand(COMMAND_CONFIRM, gBoardIndex);
    }
    return;
  }

  if (gMode == MODE_HAND_SELECT || gMode == MODE_COMPLETE) {
    if (isCoordinator()) coordinatorConfirm(gBoardIndex);
    else sendCommand(COMMAND_CONFIRM, gBoardIndex);
    return;
  }
  noteOn(midiNote);
}

bool isActive() {
  return gMode != MODE_NORMAL;
}

bool usesEncoderForMenu() {
  return gMode == MODE_BROWSER || gMode == MODE_HAND_SELECT;
}

bool usesEncoderForOctaves() {
  return !usesEncoderForMenu();
}

bool blocksNetworkMaintenance() {
  // A topology refresh must be allowed during the tutorial: otherwise a
  // hot-added board keeps its provisional index 0 and mirrors the first
  // board's octave instead of receiving its own 24-note range.
  return gMode == MODE_LOADING || gMode == MODE_USB_LOAD;
}

char statusGlyph() {
  uint32_t nowMs = millis();

  bool titleRevealed = gTitleRevealUntilMs != 0 &&
      (int32_t)(nowMs - gTitleRevealUntilMs) < 0;
  if (gMode == MODE_LEARNING && !titleRevealed) return 0;
  if (!isActive() || gTitle[0] == '\0') return 0;
  uint8_t titleLength = strlen(gTitle);
  uint8_t gap = gBoardCount == 0 ? 1 : gBoardCount;
  uint16_t cycle = titleLength + gap;
  uint32_t elapsed = nowMs >= gTitleEpochMs ? nowMs - gTitleEpochMs : 0;
  uint16_t frame = (elapsed / TITLE_FRAME_MS) % cycle;
  uint16_t index = (frame + gBoardIndex) % cycle;
  return index < titleLength ? gTitle[index] : ' ';
}

uint32_t elapsedSince(uint32_t nowMs, uint32_t epochMs) {
  return nowMs >= epochMs ? nowMs - epochMs : 0;
}

void addColor(uint8_t &red, uint8_t &green, uint8_t &blue,
              uint16_t addRed, uint16_t addGreen, uint16_t addBlue) {
  uint16_t nextRed = (uint16_t)red + addRed;
  uint16_t nextGreen = (uint16_t)green + addGreen;
  uint16_t nextBlue = (uint16_t)blue + addBlue;
  red = nextRed > 255 ? 255 : (uint8_t)nextRed;
  green = nextGreen > 255 ? 255 : (uint8_t)nextGreen;
  blue = nextBlue > 255 ? 255 : (uint8_t)nextBlue;
}

uint16_t circularDistance(uint16_t left, uint16_t right, uint16_t count) {
  if (count == 0) return 0;
  uint16_t forward = left >= right ? left - right : count + left - right;
  uint16_t backward = right >= left ? right - left : count + right - left;
  return forward < backward ? forward : backward;
}

uint16_t absoluteDistance(uint16_t left, uint16_t right) {
  return left > right ? left - right : right - left;
}

void browserPalette(uint8_t &baseRed, uint8_t &baseGreen, uint8_t &baseBlue,
                    uint8_t &leadRed, uint8_t &leadGreen, uint8_t &leadBlue) {
  if (gTitleFlags & TITLE_FLAG_LOAD_ITEM) {
    baseRed = 34; baseGreen = 12; baseBlue = 0;
    leadRed = 230; leadGreen = 65; leadBlue = 4;
    return;
  }

  uint8_t hue = (uint8_t)(85 + (gTitleFlags & TITLE_COLOR_MASK) * 97);
  uint8_t section = hue / 85;
  uint8_t fraction = hue % 85;
  uint8_t rising = (uint16_t)fraction * 255 / 84;
  uint8_t falling = 255 - rising;
  uint8_t red = 0, green = 0, blue = 0;
  if (section == 0) { red = falling; green = 255; blue = 0; }
  else if (section == 1) { red = 0; green = falling; blue = rising; }
  else { red = rising; green = 0; blue = falling; }
  leadRed = (uint16_t)red * 235 / 255;
  leadGreen = (uint16_t)green * 235 / 255;
  leadBlue = (uint16_t)blue * 235 / 255;
  baseRed = 1 + (uint16_t)red * 23 / 255;
  baseGreen = 1 + (uint16_t)green * 23 / 255;
  baseBlue = 1 + (uint16_t)blue * 23 / 255;
  if (gTitleFlags & TITLE_FLAG_ATTENTION && ((millis() / 260) & 1)) {
    baseRed = baseRed > 219 ? 255 : baseRed + 36;
    baseGreen = baseGreen > 241 ? 255 : baseGreen + 14;
  }
}

bool statusColor(uint8_t &red, uint8_t &green, uint8_t &blue) {
  if (!isActive()) return false;

  if (gMode == MODE_LEARNING) {
    red = 80; green = 24; blue = 8;
  } else if (gMode == MODE_BROWSER) {
    uint8_t baseRed, baseGreen, baseBlue, leadRed, leadGreen, leadBlue;
    browserPalette(baseRed, baseGreen, baseBlue, leadRed, leadGreen, leadBlue);
    uint8_t statusRed = leadRed / 3;
    uint8_t statusGreen = leadGreen / 3;
    uint8_t statusBlue = leadBlue / 3;
    red = baseRed > statusRed ? baseRed : statusRed;
    green = baseGreen > statusGreen ? baseGreen : statusGreen;
    blue = baseBlue > statusBlue ? baseBlue : statusBlue;
  } else if (gMode == MODE_USB_LOAD) {
    red = 110; green = 34; blue = 2;
  } else if (gMode == MODE_LOADING) {
    if (gSelectedSongFlags & CATALOG_FLAG_SPAN_WIDE) {
      red = 105; green = 50; blue = 2;
    } else {
      red = 75; green = 55; blue = 0;
    }
  } else if (gMode == MODE_HAND_SELECT) {
    red = 12; green = 70; blue = 95;
  } else {
    red = 4; green = 85; blue = 24;
  }
  return true;
}

bool keyOverlay(uint8_t midiNote, uint16_t globalPosition,
                uint8_t &red, uint8_t &green, uint8_t &blue) {
  bool applied = false;
  uint16_t keyCount = (uint16_t)(gBoardCount == 0 ? 1 : gBoardCount) * NOTES_PER_BOARD;
  uint16_t pos = keyCount == 0 ? 0 : globalPosition % keyCount;
  uint32_t nowMs = millis();
  uint32_t elapsed = elapsedSince(nowMs, gModeEpochMs);

  if (gMode == MODE_LEARNING) {
    for (uint8_t i = 0; i < gGuideCount; ++i) {
      if (gGuideNotes[i] == midiNote) {
        if (gGuideLeft[i]) {
          red = 5; green = 190; blue = 24;
        } else {
          red = 210; green = 12; blue = 5;
        }
        applied = true;
        break;
      }
    }

    if (!applied && gGuideCount > 0) {
      bool guideBelow = false;
      bool guideAbove = false;
      for (uint8_t i = 0; i < gGuideCount; ++i) {
        if (gGuideNotes[i] < midiNote) guideBelow = true;
        if (gGuideNotes[i] > midiNote) guideAbove = true;
      }
      bool leftEdge = globalPosition == 0 && guideBelow;
      bool rightEdge = keyCount > 0 && globalPosition + 1 == keyCount && guideAbove;
      if (leftEdge || rightEdge) {
        red = 125; green = 58; blue = 2;
        if (((elapsed / 180) & 1) == 0) addColor(red, green, blue, 55, 34, 4);
        applied = true;
      }
    }
  } else if (gMode == MODE_BROWSER) {
    uint8_t baseRed, baseGreen, baseBlue, leadRed, leadGreen, leadBlue;
    browserPalette(baseRed, baseGreen, baseBlue, leadRed, leadGreen, leadBlue);
    red = baseRed; green = baseGreen; blue = baseBlue;
    uint16_t phase = (elapsed / 32) % keyCount;
    uint16_t distance = (phase + keyCount - pos) % keyCount;
    if (distance <= 6) {
      uint8_t scale = 7 - distance;
      addColor(red, green, blue,
               (uint16_t)leadRed * scale / 7,
               (uint16_t)leadGreen * scale / 7,
               (uint16_t)leadBlue * scale / 7);
    }
    applied = true;
  } else if (gMode == MODE_USB_LOAD) {
    bool isUsbTarget = gSongOwner != 0xFF && gBoardIndex == gSongOwner;
    red = isUsbTarget ? 2 : 42;
    green = isUsbTarget ? 34 : 12;
    blue = isUsbTarget ? 12 : 0;
    uint16_t phase = (elapsed / 32) % keyCount;
    uint16_t distance = (pos + keyCount - phase) % keyCount;
    if (distance <= 6) {
      uint8_t scale = 7 - distance;
      if (isUsbTarget) addColor(red, green, blue, 3 * scale, 32 * scale, 12 * scale);
      else addColor(red, green, blue, 34 * scale, 11 * scale, scale);
    }
    applied = true;
  } else if (gMode == MODE_LOADING) {
    bool isLoadingOwner = gSongOwner != 0xFF && gBoardIndex == gSongOwner;
    if (isLoadingOwner) {
      red = 3; green = 26; blue = 34;
    } else if (gSelectedSongFlags & CATALOG_FLAG_SPAN_WIDE) {
      red = 42; green = 18; blue = 0;
    } else {
      red = 34; green = 22; blue = 0;
    }
    uint16_t phase = (elapsed / 34) % keyCount;
    uint16_t distance = circularDistance(pos, phase, keyCount);
    if (distance <= 4) {
      uint8_t scale = 5 - distance;
      if (isLoadingOwner) addColor(red, green, blue, 4 * scale, 28 * scale, 36 * scale);
      else addColor(red, green, blue, 32 * scale, 26 * scale, 4 * scale);
    }
    applied = true;
  } else if (gMode == MODE_HAND_SELECT) {
    red = 1; green = 18; blue = 24;
    uint16_t phase = (elapsed / 44) % keyCount;
    uint16_t distance = circularDistance(pos, phase, keyCount);
    if (distance <= 5) {
      uint8_t scale = 6 - distance;
      addColor(red, green, blue, 2 * scale, 25 * scale, 32 * scale);
    }
    applied = true;
  } else if (gMode == MODE_COMPLETE) {
    red = 3; green = 24; blue = 5;
    uint16_t phase = (elapsed / 58) % keyCount;
    uint16_t distance = circularDistance(pos, phase, keyCount);
    if (distance <= 7) {
      uint8_t scale = 8 - distance;
      addColor(red, green, blue, 12 * scale, 24 * scale, 3 * scale);
    }
    applied = true;
  }

  if (applied && elapsed < 420) {
    uint16_t front = (uint32_t)elapsed * keyCount / 420;
    uint16_t fromLeft = absoluteDistance(pos, front);
    uint16_t fromRight = absoluteDistance((uint16_t)(keyCount - 1 - pos), front);
    uint16_t distance = fromLeft < fromRight ? fromLeft : fromRight;
    if (distance <= 2) {
      uint8_t scale = 3 - distance;
      addColor(red, green, blue, 28 * scale, 26 * scale, 22 * scale);
    }
  }

  if (gImportWaveStartMs != 0 && nowMs >= gImportWaveStartMs) {
    uint32_t importElapsed = nowMs - gImportWaveStartMs;
    uint16_t front = importElapsed / 38;
    if (front > keyCount + 5) {
      gImportWaveStartMs = 0;
    } else {
      uint16_t distance = absoluteDistance(globalPosition, front);
      if (distance <= 3) {
        red = 2;
        green = 240 - distance * 45;
        blue = 18;
        applied = true;
      }
    }
  }
  return applied;
}

bool buttonOverlay(uint8_t buttonIndex, uint8_t &red, uint8_t &green, uint8_t &blue) {
  if (!isActive()) return false;
  uint32_t nowMs = millis();
  uint32_t elapsed = elapsedSince(nowMs, gModeEpochMs);
  uint8_t pos = buttonIndex & 0x07;
  if (gMode == MODE_USB_LOAD || gMode == MODE_LOADING) {
    bool isUsbTarget = gMode == MODE_USB_LOAD && gSongOwner != 0xFF &&
        gBoardIndex == gSongOwner;
    bool isLoadingOwner = gMode == MODE_LOADING && gSongOwner != 0xFF &&
        gBoardIndex == gSongOwner;
    red = isUsbTarget ? 2 : (isLoadingOwner ? 2 : 48);
    green = isUsbTarget ? 38 : (isLoadingOwner ? 32 : 13);
    blue = isUsbTarget ? 13 : (isLoadingOwner ? 38 : 0);
    uint8_t phase = (elapsed / 82) & 0x07;
    uint8_t distance = (pos + 8 - phase) & 0x07;
    if (distance <= 2) {
      if (isUsbTarget) addColor(red, green, blue, 3, 85 - distance * 26, 30 - distance * 9);
      else if (isLoadingOwner) addColor(red, green, blue, 3, 30 - distance * 8, 84 - distance * 25);
      else addColor(red, green, blue, 78 - distance * 24, 26 - distance * 8, 0);
    }
    if (buttonIndex == 0) addColor(red, green, blue, isUsbTarget ? 0 : 22,
                                     isUsbTarget ? 26 : 7, isUsbTarget ? 8 : 0);
    return true;
  }

  if (gMode == MODE_BROWSER) {
    uint8_t baseRed, baseGreen, baseBlue, leadRed, leadGreen, leadBlue;
    browserPalette(baseRed, baseGreen, baseBlue, leadRed, leadGreen, leadBlue);
    red = baseRed; green = baseGreen; blue = baseBlue;
    uint8_t phase = (elapsed / 88) & 0x07;
    uint8_t distance = (phase + 8 - pos) & 0x07;
    if (distance <= 2) {
      uint8_t scale = 3 - distance;
      addColor(red, green, blue,
               (uint16_t)leadRed * scale / 3,
               (uint16_t)leadGreen * scale / 3,
               (uint16_t)leadBlue * scale / 3);
    }
    return true;
  }

  if (gMode == MODE_HAND_SELECT) {
    red = 2; green = 24; blue = 32;
    if ((buttonIndex == 1 && gHandChoice != HAND_RIGHT) ||
        (buttonIndex == 2 && gHandChoice != HAND_LEFT)) {
      addColor(red, green, blue, 3, 44, 70);
    }
    return true;
  }

  red = 10; green = 30; blue = 8;
  return true;
}

bool topLedOverlay(uint8_t &red, uint8_t &green, uint8_t &blue) {
  if (!isActive()) return false;
  float phase = (millis() % 780) / 780.0f * 2.0f * PI;
  uint8_t pulse = (uint8_t)(45 + (sinf(phase) + 1.0f) * 42.0f);

  if (gMode == MODE_LEARNING) {
    red = pulse; green = 10; blue = 4;
  } else if (gMode == MODE_BROWSER) {
    uint8_t baseRed, baseGreen, baseBlue, leadRed, leadGreen, leadBlue;
    browserPalette(baseRed, baseGreen, baseBlue, leadRed, leadGreen, leadBlue);
    uint16_t scaledRed = (uint16_t)leadRed * pulse / 120;
    uint16_t scaledGreen = (uint16_t)leadGreen * pulse / 120;
    uint16_t scaledBlue = (uint16_t)leadBlue * pulse / 120;
    uint8_t pulseRed = scaledRed > 255 ? 255 : (uint8_t)scaledRed;
    uint8_t pulseGreen = scaledGreen > 255 ? 255 : (uint8_t)scaledGreen;
    uint8_t pulseBlue = scaledBlue > 255 ? 255 : (uint8_t)scaledBlue;
    red = baseRed > pulseRed ? baseRed : pulseRed;
    green = baseGreen > pulseGreen ? baseGreen : pulseGreen;
    blue = baseBlue > pulseBlue ? baseBlue : pulseBlue;
  } else if (gMode == MODE_USB_LOAD) {
    if (gSongOwner != 0xFF && gBoardIndex == gSongOwner) {
      red = 2;
      green = pulse > 90 ? pulse : 90;
      blue = 24;
    } else {
      uint16_t loaderRed = (uint16_t)pulse + 95;
      red = loaderRed > 255 ? 255 : (uint8_t)loaderRed;
      green = pulse / 2;
      blue = 2;
    }
  } else if (gMode == MODE_LOADING) {
    red = pulse; green = pulse / 2; blue = 0;
  } else if (gMode == MODE_HAND_SELECT) {
    red = 4; green = pulse / 2; blue = pulse;
  } else {
    red = 4; green = pulse; blue = 12;
  }
  return true;
}

}  // namespace FractureSongs
