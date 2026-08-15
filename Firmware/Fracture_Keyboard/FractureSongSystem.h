#pragma once

#include <Arduino.h>

namespace FractureSongs {

constexpr uint8_t PACKET_MODE = 0x08;
constexpr uint8_t PACKET_CATALOG_REQUEST = 0x09;
constexpr uint8_t PACKET_CATALOG_ITEM = 0x0A;
constexpr uint8_t PACKET_TITLE = 0x0B;
constexpr uint8_t PACKET_COMMAND = 0x0C;
constexpr uint8_t PACKET_GUIDE_STEP = 0x0D;
constexpr uint8_t PACKET_WAVE = 0x0E;
constexpr uint8_t PACKET_READY = 0x0F;

using PacketSender = void (*)(uint8_t type, const uint8_t *payload, uint8_t len);

// Call before USB.begin() to configure the selected USB personality.
void beginBeforeUsb(PacketSender sender);
bool isUsbLoaderBoot();
void setTopology(uint32_t nodeId, uint8_t boardIndex, uint8_t boardCount);
void update();
bool handlePacket(uint8_t type, const uint8_t *payload, uint8_t len);

void enterBrowser();
void back();
void confirm();
void encoderDelta(int8_t detents);
void nextButton();
void showTitle();
// Local presses may activate browser UI; remote presses may only advance a
// loaded guide on its owner board.
void localNoteOn(uint8_t midiNote);
void noteOn(uint8_t midiNote);

bool isActive();
bool usesEncoderForMenu();
bool usesEncoderForOctaves();
bool blocksNetworkMaintenance();
char statusGlyph();
bool statusColor(uint8_t &red, uint8_t &green, uint8_t &blue);
bool keyOverlay(uint8_t midiNote, uint16_t globalPosition,
                uint8_t &red, uint8_t &green, uint8_t &blue);
bool buttonOverlay(uint8_t buttonIndex, uint8_t &red, uint8_t &green, uint8_t &blue);
bool topLedOverlay(uint8_t &red, uint8_t &green, uint8_t &blue);

}  // namespace FractureSongs
