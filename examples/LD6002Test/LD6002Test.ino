/*
LD6002 Radar Sensor + ESP32-S3-DevKitM-1 Connection Guide
Author: Henry Li
Date: 2026-03-16
Target Board: ESP32-S3-DevKitM-1

Hardware connections:
  - LD6002 TX  -> ESP32 GPIO18 (Serial1 RX)
  - LD6002 RX  -> ESP32 GPIO17 (Serial1 TX)
  - LD6002 GND -> ESP32 GND
  - LD6002 VCC -> ESP32 3.3V or 5V (follow module requirements)

Notes:
  1. Make sure LD6002.h / LD6002.cpp are included in your project.
  2. Set Serial Monitor baud rate to 115200.
*/

#include <Arduino.h>
#include "LD6002.h"

// ============ Pin Definitions ============
#define SERIAL1_RX_PIN 18  // Connected to LD6002 TX
#define SERIAL1_TX_PIN 17  // Connected to LD6002 RX

// ============ Debug Options ============
// Print raw 0A04 frame (HEX) or not
static constexpr bool kPrintRaw0A04 = false;

// ============ Global Variables ============
// Use pointer to avoid constructing before Serial1 is initialized
LD6002 *radar = nullptr;

// Data cache to avoid printing duplicate values
float lastHeartRate = 0.0f;
float lastBreathRate = 0.0f;
float lastDistance = 0.0f;
uint16_t lastHumanPresence = 0xFFFF;
bool hasLastAz = false;
float lastAzDeg = 0.0f;

// LED blink control
unsigned long lastLEDToggle = 0;
bool ledState = false;

// ============ Function Declarations ============
void printStatus();
void toggleLED();
void queryAndPrintFirmwareVersion();

// ============ Setup ============
void setup()
{
  // 1) Give the system a short time to stabilize
  delay(500);

  // 2) Initialize built-in LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // 3) Initialize USB serial for debug output
  Serial.begin(115200);

  // Wait for USB serial connection (up to 3 seconds)
  unsigned long startAttempt = millis();
  while (!Serial) {
    delay(10);
    if (millis() - startAttempt > 3000) {
      break;
    }
  }

  Serial.println("\n=================================");
  Serial.println("ESP32-S3-DevKitM-1 LD6002 Radar Test");
  Serial.println("=================================");

  // 4) Initialize hardware serial Serial1 (connected to LD6002)
  Serial.println("Initializing Serial1...");
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);

  // 5) Wait for serial stabilization
  delay(100);

  // 6) Create radar object
  Serial.println("Initializing LD6002 radar...");
  radar = new LD6002(Serial1);

  // 7) Wait for module power-up stabilization
  Serial.println("Waiting for radar module to stabilize...");
  delay(500);
  queryAndPrintFirmwareVersion();

  Serial.println("LD6002 initialization complete!");
  Serial.printf("Serial1 config: RX=%d, TX=%d, baud=115200\n",
                SERIAL1_RX_PIN, SERIAL1_TX_PIN);
  Serial.println("Start receiving data...\n");
}

// ============ Main Loop ============
void loop()
{
  // 1) Basic guard
  if (radar == nullptr) {
    Serial.println("Error: radar is not initialized");
    delay(1000);
    return;
  }

  // 2) Update radar data (serial frame parsing)
  radar->update();

  // 3) LED running indicator
  toggleLED();

  // 4) Heart rate
  if (radar->hasNewHeartRate()) {
    float heartRateMain = radar->getHeartRate();
    if ( (heartRateMain > 0)) {
      Serial.printf("[Heart Rate] %.2f bpm\n", heartRateMain);
      lastHeartRate = heartRateMain;
    }
    radar->clearHeartRateFlag();
  }

  // 5) Breath rate
  if (radar->hasNewBreathRate()) {
    float breathRateMain = radar->getBreathRate();
    if ( (breathRateMain > 0)) {
      Serial.printf("[Breath Rate] %.2f bpm\n", breathRateMain);
      lastBreathRate = breathRateMain;
    }
    radar->clearBreathRateFlag();
  }

  // 6) Distance
  if (radar->hasNewDistance()) {
    float distanceMain = radar->getDistance();
    if ( (distanceMain > 0)) {
      Serial.printf("[Distance] %.2f cm\n", distanceMain);
      lastDistance = distanceMain;
    }
    radar->clearDistanceFlag();
  }

  // 7) Human presence
  if (radar->hasNewHumanPresence()) {
    uint16_t humanPresence = radar->getHumanPresenceRaw();
    if (true) {
      Serial.printf("[0F09] is_human(raw)=0x%04X, present=%s\n",
                    humanPresence,
                    radar->isHumanPresent() ? "YES" : "NO");
      lastHumanPresence = humanPresence;
    }
    radar->clearHumanPresenceFlag();
  }

  // 8) 0A04 raw frame (optional)
  if (kPrintRaw0A04 && radar->hasNewRaw0A04Frame()) {
    const uint16_t rawLen = radar->getRaw0A04FrameLen();
    const uint8_t *rawFrame = radar->getRaw0A04Frame();
    Serial.printf("[0A04 RAW] len=%u, frame=", rawLen);
    for (uint16_t i = 0; i < rawLen; i++) {
      if (rawFrame[i] < 0x10) {
        Serial.print("0");
      }
      Serial.print(rawFrame[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    radar->clearRaw0A04FrameFlag();
  }

  // 9) 0A04 parsed result (ignore Z per protocol)
  if (radar->hasNewPersonPosition()) {
    const LD6002::PersonPositionData &position = radar->getPersonPosition();
    Serial.printf("[0A04 PARSE] target_num=%ld, x=%.3f m, y=%.3f m (z/dop_idx ignored)\n",
                  static_cast<long>(position.targetNum),
                  position.x,
                  position.y);
    if (position.hasFirstTarget) {
      /*
      if (position.hasClusterId) {
        Serial.printf("[0A04 PARSE] has_cluster_id=YES, cluster_id=%ld\n",
                      static_cast<long>(position.clusterId));
      } else {
        Serial.println("[0A04 PARSE] has_cluster_id=NO");
      }
      */

      if (radar->hasValidAngle()) {
        const LD6002::AngleData &angle = radar->getLatestAngle();
        hasLastAz = true;
        lastAzDeg = angle.azimuthDeg;
        Serial.printf("[ANGLE] from 0A04: az=%.1f deg, el=%.1f deg, range=%.2f m\n",
                      angle.azimuthDeg, angle.elevationDeg, angle.rangeM);
      }
    } else {
      Serial.println("[0A04] no valid first-target payload");
    }
    radar->clearPersonPositionFlag();
  }

  // 10) 0A13 clear flag only (no print)
  if (radar->hasNewPhase()) {
    radar->clearPhaseFlag();
  }

  // 11) 0A17 parsed result
  if (radar->hasNewTrackPosition()) {
    const LD6002::TrackPositionData &track = radar->getTrackPosition();
    Serial.printf("[0A17] x=%.3f m, y=%.3f m, z=%.3f m\n",
                  track.x, track.y, track.z);
    if (radar->hasValidAngle()) {
      const LD6002::AngleData &angle = radar->getLatestAngle();
      hasLastAz = true;
      lastAzDeg = angle.azimuthDeg;
      Serial.printf("[ANGLE] from 0A17: az=%.1f deg, el=%.1f deg, range=%.2f m\n",
                    angle.azimuthDeg, angle.elevationDeg, angle.rangeM);
    }
    radar->clearTrackPositionFlag();
  }

  // 12) Print status every 5 seconds (current presence state)
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint > 5000) {
    printStatus();
    lastStatusPrint = millis();
  }
}

// ============ Helper Functions ============

/**
 * Query and print firmware version on startup (0xFFFF)
 */
void queryAndPrintFirmwareVersion()
{
  if (radar == nullptr) {
    return;
  }

  delay(300);
  Serial.println("[0xFFFF] query firmware info...");
  if (!radar->requestFirmwareInfo()) {
    Serial.println("[0xFFFF] request send failed");
    return;
  }

  unsigned long start = millis();
  while (millis() - start < 1500) {
    radar->update();
    if (radar->hasNewFirmwareInfo()) {
      const LD6002::FirmwareInfoData &fw = radar->getFirmwareInfo();
      const char *projectName = "Unknown";
      switch (fw.projectName) {
        case 1: projectName = "BreathDetect"; break;
        case 2: projectName = "GestureDetect"; break;
        case 3: projectName = "RangeDetect"; break;
        case 4: projectName = "PeopleCount"; break;
        case 5: projectName = "PointCloud3D"; break;
        case 6: projectName = "BreathHeart"; break;
        default: break;
      }

      Serial.printf("[0xFFFF] project_name=%u (%s), version=%u.%u.%u\n",
                    fw.projectName,
                    projectName,
                    fw.majorVersion,
                    fw.subVersion,
                    fw.modifiedVersion);
      Serial.printf("[BOOT] Firmware Version: %u.%u.%u\n",
                    fw.majorVersion,
                    fw.subVersion,
                    fw.modifiedVersion);
      radar->clearFirmwareInfoFlag();
      return;
    }
    delay(10);
  }

  Serial.println("[0xFFFF] firmware info timeout");
}

/**
 * Print status: only current presence marker
 */
void printStatus()
{
  Serial.println("------------------------");
  Serial.printf("uptime: %lu s\n", millis() / 1000);
  if (lastHumanPresence == 0xFFFF) {
    Serial.println("human_presence: unknown");
  } else if (lastHumanPresence == 0x0001) {
    Serial.printf("human_presence: present  ");
    if (hasLastAz) {
      Serial.printf("last_az: %.1f deg\n", lastAzDeg);
    } else {
      Serial.println("last_az: unknown");
    }
  } else {
    Serial.printf("human_presence: absent (raw=0x%04X)\n", lastHumanPresence);
  }
  Serial.println("------------------------");
}

/**
 * LED blink control
 */
void toggleLED()
{
  static unsigned long lastDataTime = 0;

  // Update last receive timestamp when new data arrives
  if (radar->hasNewHeartRate() ||
      radar->hasNewBreathRate() ||
      radar->hasNewDistance() ||
      radar->hasNewHumanPresence() ||
      radar->hasNewPersonPosition() ||
      radar->hasNewPhase() ||
      radar->hasNewTrackPosition()) {
    lastDataTime = millis();
  }

  // Data available: fast blink; no data: slow blink
  unsigned long period = (millis() - lastDataTime < 2000) ? 500 : 1000;

  if (millis() - lastLEDToggle >= period) {
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    lastLEDToggle = millis();
  }
}

/**
 * Error handling (optional)
 */
void handleError(const char *errorMsg)
{
  Serial.printf("Error: %s\n", errorMsg);
  // Fast LED blink to indicate error
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}
