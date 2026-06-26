#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// Serial + LED Hall sensor scanner for Fracture.
// No MIDI, RS-485, I2S, or buzzer. Output says which PCB switch is active.

#define PIN_MUX_S0   6
#define PIN_MUX_S1   7
#define PIN_MUX_S2   8
#define PIN_MUX_S3   9
#define PIN_AM0      4
#define PIN_AM1      5
#define PIN_LED_DATA 14

const uint8_t SENSOR_COUNT = 32;
const uint16_t NUM_LEDS = 45;
const uint8_t CALIBRATION_SAMPLES = 64;
const uint8_t ADC_AVERAGE_SAMPLES = 4;
const uint16_t DEFAULT_PRESS_DELTA = 80;
const uint16_t FULL_RED_DELTA = 750;
const uint8_t PRESS_DEBOUNCE_SCANS = 3;
const uint8_t RELEASE_DEBOUNCE_SCANS = 10;
const uint32_t DOWN_PRINT_INTERVAL_MS = 350;

Adafruit_NeoPixel leds(NUM_LEDS, PIN_LED_DATA, NEO_GRB + NEO_KHZ800);

uint16_t baseline[SENSOR_COUNT];
uint16_t rawValue[SENSOR_COUNT];
bool pressed[SENSOR_COUNT];
uint8_t aboveCount[SENSOR_COUNT];
uint8_t belowCount[SENSOR_COUNT];
uint16_t peakAbs[SENSOR_COUNT];
int16_t peakDelta[SENSOR_COUNT];
uint16_t pressDelta = DEFAULT_PRESS_DELTA;
uint32_t lastDownPrintMs = 0;

void setMuxChannel(uint8_t channel) {
  digitalWrite(PIN_MUX_S0, channel & 0x01);
  digitalWrite(PIN_MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(PIN_MUX_S2, (channel >> 2) & 0x01);
  digitalWrite(PIN_MUX_S3, (channel >> 3) & 0x01);
  delayMicroseconds(80);
}

uint8_t sensorMuxIndex(uint8_t sensor) {
  return sensor < 16 ? 0 : 1;
}

uint8_t sensorMuxChannel(uint8_t sensor) {
  return sensor & 0x0F;
}

uint8_t sensorSwitchNumber(uint8_t sensor) {
  return sensor + 1;
}

const char *sensorRole(uint8_t sensor) {
  uint8_t sw = sensorSwitchNumber(sensor);
  if (sw <= 8) return "AUX";
  if (sw <= 18) return "BLACK";
  return "WHITE";
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

uint8_t physicalLedIndexForSensor(uint8_t sensor) {
  return physicalLedIndexForSwitch(sensorSwitchNumber(sensor));
}
uint8_t sensorAnalogPin(uint8_t sensor) {
  return sensorMuxIndex(sensor) == 0 ? PIN_AM0 : PIN_AM1;
}

uint16_t readSensorRaw(uint8_t sensor) {
  setMuxChannel(sensorMuxChannel(sensor));
  uint8_t pin = sensorAnalogPin(sensor);
  analogRead(pin);
  analogRead(pin);
  delayMicroseconds(20);

  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_AVERAGE_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(10);
  }
  return sum / ADC_AVERAGE_SAMPLES;
}

int16_t signedDeltaFor(uint8_t sensor) {
  return (int16_t)rawValue[sensor] - (int16_t)baseline[sensor];
}

uint16_t absDeltaFor(uint8_t sensor) {
  int16_t delta = signedDeltaFor(sensor);
  return delta < 0 ? (uint16_t)(-delta) : (uint16_t)delta;
}

uint8_t redLevelForDelta(uint16_t amount) {
  if (amount < 10) return 0;
  uint16_t bounded = amount > FULL_RED_DELTA ? FULL_RED_DELTA : amount;
  uint32_t scaled = (uint32_t)bounded * 255UL / FULL_RED_DELTA;
  // Exaggerate the lower half so travel changes are visible before full press.
  if (scaled < 96) scaled = scaled * 2;
  if (scaled > 255) scaled = 255;
  return (uint8_t)scaled;
}

void renderLeds() {
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    uint16_t amount = absDeltaFor(sensor);
    uint8_t red = redLevelForDelta(amount);

    if (amount >= pressDelta) {
      uint8_t ledIndex = physicalLedIndexForSensor(sensor);
      if (ledIndex < SENSOR_COUNT) leds.setPixelColor(ledIndex, leds.Color(red, 0, 0));
    } else if (amount >= pressDelta / 2) {
      uint8_t ledIndex = physicalLedIndexForSensor(sensor);
      if (ledIndex < SENSOR_COUNT) leds.setPixelColor(ledIndex, leds.Color(red / 2, 0, 0));
    } else {
      uint8_t ledIndex = physicalLedIndexForSensor(sensor);
      if (ledIndex < SENSOR_COUNT) leds.setPixelColor(ledIndex, 0);
    }
  }

  uint8_t heartbeat = (millis() / 250) & 0x01 ? 20 : 3;
  leds.setPixelColor(44, leds.Color(heartbeat, heartbeat, heartbeat));
  leds.show();
}

void printSensor(const char *eventName, uint8_t sensor) {
  uint8_t sw = sensorSwitchNumber(sensor);
  Serial.print(eventName);
  Serial.print(" S");
  if (sw < 10) Serial.print('0');
  Serial.print(sw);
  Serial.print(' ');
  Serial.print(sensorRole(sensor));
  Serial.print(" LED");
  Serial.print(physicalLedIndexForSensor(sensor) + 1);
  Serial.print(" AM");
  Serial.print(sensorMuxIndex(sensor));
  Serial.print(':');
  if (sensorMuxChannel(sensor) < 10) Serial.print('0');
  Serial.print(sensorMuxChannel(sensor));
  Serial.print(" raw=");
  Serial.print(rawValue[sensor]);
  Serial.print(" base=");
  Serial.print(baseline[sensor]);
  Serial.print(" delta=");
  Serial.print(signedDeltaFor(sensor));
  Serial.print(" abs=");
  Serial.print(absDeltaFor(sensor));
  Serial.print(" peakAbs=");
  Serial.print(peakAbs[sensor]);
  Serial.print(" peakDelta=");
  Serial.println(peakDelta[sensor]);
}

void printHelp() {
  Serial.println();
  Serial.println("Fracture key scanner with red range LEDs");
  Serial.println("Serial 115200. Commands: r=recalibrate, d=dump, +=threshold up, -=threshold down, h=help");
  Serial.println("Mapping: S01-S08 AUX, S09-S18 BLACK, S19-S32 WHITE.");
  Serial.println("LED map: S01-S15 -> LED01-15, S16-S30 -> LED30-16, S31-S32 -> LED31-32.");
  Serial.print("Threshold press=");
  Serial.print(pressDelta);
  Serial.print(" release=");
  Serial.print(pressDelta / 2);
  Serial.print(" fullRed=");
  Serial.println(FULL_RED_DELTA);
  Serial.println();
}

void readAllSensors() {
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    rawValue[sensor] = readSensorRaw(sensor);
  }
}

void dumpSensors() {
  readAllSensors();
  renderLeds();
  Serial.println("DUMP");
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    printSensor("RAW", sensor);
  }
  Serial.println("END DUMP");
}

void calibrateSensors() {
  Serial.println("CALIBRATE: release all keys");
  leds.clear();
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    leds.setPixelColor(i, leds.Color(12, 0, 0));
  }
  leds.show();
  delay(800);

  uint32_t sums[SENSOR_COUNT];
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    sums[sensor] = 0;
    pressed[sensor] = false;
    aboveCount[sensor] = 0;
    belowCount[sensor] = 0;
    peakAbs[sensor] = 0;
    peakDelta[sensor] = 0;
  }

  for (uint8_t sample = 0; sample < CALIBRATION_SAMPLES; sample++) {
    for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
      sums[sensor] += readSensorRaw(sensor);
    }
  }

  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    baseline[sensor] = sums[sensor] / CALIBRATION_SAMPLES;
  }

  leds.clear();
  leds.show();
  Serial.println("READY");
  dumpSensors();
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      calibrateSensors();
    } else if (c == 'd' || c == 'D') {
      dumpSensors();
    } else if (c == '+') {
      pressDelta += 20;
      printHelp();
    } else if (c == '-') {
      if (pressDelta > 30) pressDelta -= 20;
      printHelp();
    } else if (c == 'h' || c == 'H' || c == '?') {
      printHelp();
    }
  }
}

void scanTransitions() {
  uint16_t releaseDelta = pressDelta / 2;

  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    uint16_t amount = absDeltaFor(sensor);
    int16_t delta = signedDeltaFor(sensor);

    if (amount > peakAbs[sensor]) {
      peakAbs[sensor] = amount;
      peakDelta[sensor] = delta;
    }

    if (amount >= pressDelta) {
      if (aboveCount[sensor] < 255) aboveCount[sensor]++;
      belowCount[sensor] = 0;
    } else if (amount <= releaseDelta) {
      if (belowCount[sensor] < 255) belowCount[sensor]++;
      aboveCount[sensor] = 0;
    }

    if (!pressed[sensor] && aboveCount[sensor] >= PRESS_DEBOUNCE_SCANS) {
      pressed[sensor] = true;
      peakAbs[sensor] = amount;
      peakDelta[sensor] = delta;
      printSensor("PRESS", sensor);
    }

    if (pressed[sensor] && belowCount[sensor] >= RELEASE_DEBOUNCE_SCANS) {
      printSensor("RELEASE", sensor);
      pressed[sensor] = false;
      peakAbs[sensor] = 0;
      peakDelta[sensor] = 0;
    }
  }
}

void printDownSensorsIfNeeded() {
  uint32_t now = millis();
  if (now - lastDownPrintMs < DOWN_PRINT_INTERVAL_MS) return;
  lastDownPrintMs = now;

  bool any = false;
  for (uint8_t sensor = 0; sensor < SENSOR_COUNT; sensor++) {
    if (pressed[sensor] && absDeltaFor(sensor) >= pressDelta / 2) {
      any = true;
      printSensor("DOWN", sensor);
    }
  }

  if (any) Serial.println("--");
}

void setup() {
  Serial.begin(115200);
  uint32_t startMs = millis();
  while (!Serial && millis() - startMs < 2500) {
    delay(10);
  }

  pinMode(PIN_MUX_S0, OUTPUT);
  pinMode(PIN_MUX_S1, OUTPUT);
  pinMode(PIN_MUX_S2, OUTPUT);
  pinMode(PIN_MUX_S3, OUTPUT);
  analogReadResolution(12);

  leds.begin();
  leds.clear();
  leds.show();

  Serial.println();
  Serial.println("BOOT Fracture key scanner with red range LEDs");
  printHelp();
  calibrateSensors();
}

void loop() {
  handleSerial();
  readAllSensors();
  scanTransitions();
  renderLeds();
  printDownSensorsIfNeeded();
  delay(10);
}