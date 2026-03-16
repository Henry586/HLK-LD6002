// LD6002.cpp
#include "LD6002.h"
#include <math.h>

LD6002::LD6002(HardwareSerial &serial) : serial(serial) {}

void LD6002::update()
{
  while (serial.available()) {
    uint8_t byte = serial.read();
    if (!syncing) {
      if (byte == 0x01) {
        frame[0] = byte;
        pos = 1;
        syncing = true;
        expectedFrameLen = 0;
      }
      continue;
    }

    if (pos >= MAX_FRAME_LEN) {
      syncing = false;
      pos = 0;
      expectedFrameLen = 0;
      continue;
    }

    frame[pos++] = byte;

    if (pos == 7) {
      uint16_t dataLen = (static_cast<uint16_t>(frame[3]) << 8) | frame[4];
      uint32_t totalFrameLen = static_cast<uint32_t>(8) + dataLen + ((dataLen > 0) ? 1 : 0);
      if (totalFrameLen > MAX_FRAME_LEN) {
        syncing = false;
        pos = 0;
        expectedFrameLen = 0;
        continue;
      }
      expectedFrameLen = static_cast<uint16_t>(totalFrameLen);
    }

    if (expectedFrameLen > 0 && pos >= expectedFrameLen) {
      const uint16_t type = (static_cast<uint16_t>(frame[5]) << 8) | frame[6];
      if (type == static_cast<uint16_t>(FrameType::PersonPosition)) {
        cacheRaw0A04Frame(frame, expectedFrameLen);
      }
      parseFrame(frame);
      syncing = false;
      pos = 0;
      expectedFrameLen = 0;
    }
  }
}

bool LD6002::requestFirmwareInfo(uint16_t frameId)
{
  return sendFrame(static_cast<uint16_t>(FrameType::FirmwareInfo), frameId, nullptr, 0);
}

bool LD6002::hasNewFirmwareInfo() const { return newFirmwareInfo; }
bool LD6002::hasNewRaw0A04Frame() const { return newRaw0A04Frame; }
bool LD6002::hasNewHumanPresence() const { return newHumanPresence; }
bool LD6002::hasNewPersonPosition() const { return newPersonPosition; }
bool LD6002::hasNewPhase() const { return newPhase; }
bool LD6002::hasNewHeartRate() const { return newHeartRate; }
bool LD6002::hasNewBreathRate() const { return newBreathRate; }
bool LD6002::hasNewDistance() const { return newDistance; }
bool LD6002::hasNewTrackPosition() const { return newTrackPosition; }

const LD6002::FirmwareInfoData &LD6002::getFirmwareInfo() const { return firmwareInfo; }
const uint8_t *LD6002::getRaw0A04Frame() const { return raw0A04Frame; }
uint16_t LD6002::getRaw0A04FrameLen() const { return raw0A04FrameLen; }
const LD6002::AngleData &LD6002::getLatestAngle() const { return latestAngle; }
bool LD6002::hasValidAngle() const { return latestAngle.valid; }
uint16_t LD6002::getHumanPresenceRaw() const { return humanPresenceRaw; }
bool LD6002::isHumanPresent() const { return humanPresenceRaw == 1; }
const LD6002::PersonPositionData &LD6002::getPersonPosition() const { return personPosition; }
const LD6002::PhaseData &LD6002::getPhase() const { return phaseData; }
float LD6002::getHeartRate() const { return heartRate; }
float LD6002::getBreathRate() const { return breathRate; }
float LD6002::getDistance() const { return distance; }
const LD6002::TrackPositionData &LD6002::getTrackPosition() const { return trackPosition; }

void LD6002::clearFirmwareInfoFlag() { newFirmwareInfo = false; }
void LD6002::clearRaw0A04FrameFlag() { newRaw0A04Frame = false; }
void LD6002::clearHumanPresenceFlag() { newHumanPresence = false; }
void LD6002::clearPersonPositionFlag() { newPersonPosition = false; }
void LD6002::clearPhaseFlag() { newPhase = false; }
void LD6002::clearHeartRateFlag() { newHeartRate = false; }
void LD6002::clearBreathRateFlag() { newBreathRate = false; }
void LD6002::clearDistanceFlag() { newDistance = false; }
void LD6002::clearTrackPositionFlag() { newTrackPosition = false; }

float LD6002::bytesToFloat(const uint8_t *data) const
{
  float f;
  memcpy(&f, data, sizeof(float));
  return f;
}

uint16_t LD6002::bytesToUInt16(const uint8_t *data) const
{
  uint16_t val;
  memcpy(&val, data, sizeof(uint16_t));
  return val;
}

uint32_t LD6002::bytesToUInt32(const uint8_t *data) const
{
  uint32_t val;
  memcpy(&val, data, sizeof(uint32_t));
  return val;
}

int32_t LD6002::bytesToInt32(const uint8_t *data) const
{
  int32_t val;
  memcpy(&val, data, sizeof(int32_t));
  return val;
}

uint8_t LD6002::calcXorInverse(const uint8_t *data, uint16_t len) const
{
  uint8_t cksum = 0;
  for (uint16_t i = 0; i < len; i++) {
    cksum ^= data[i];
  }
  return static_cast<uint8_t>(~cksum);
}

void LD6002::cacheRaw0A04Frame(const uint8_t *frame, uint16_t frameLen)
{
  if (frameLen > MAX_FRAME_LEN) {
    frameLen = MAX_FRAME_LEN;
  }
  memcpy(raw0A04Frame, frame, frameLen);
  raw0A04FrameLen = frameLen;
  newRaw0A04Frame = true;
}

void LD6002::updateAngleFromXYZ(float x, float y, float z)
{
  const float horizontal = sqrtf(x * x + y * y);
  const float range = sqrtf(horizontal * horizontal + z * z);
  if (range < 1e-6f) {
    latestAngle.valid = false;
    latestAngle.azimuthDeg = 0;
    latestAngle.elevationDeg = 0;
    latestAngle.rangeM = 0;
    return;
  }

  static constexpr float RAD_TO_DEG_F = 57.2957795f;
  latestAngle.valid = true;
  latestAngle.azimuthDeg = atan2f(x, y) * RAD_TO_DEG_F;
  latestAngle.elevationDeg = atan2f(z, horizontal) * RAD_TO_DEG_F;
  latestAngle.rangeM = range;
}

bool LD6002::sendFrame(uint16_t type, uint16_t frameId, const uint8_t *data, uint16_t dataLen)
{
  uint16_t totalLen = static_cast<uint16_t>(8 + dataLen + ((dataLen > 0) ? 1 : 0));
  if (totalLen > MAX_FRAME_LEN) return false;

  uint8_t out[MAX_FRAME_LEN] = {0};
  out[0] = 0x01;
  out[1] = static_cast<uint8_t>((frameId >> 8) & 0xFF);
  out[2] = static_cast<uint8_t>(frameId & 0xFF);
  out[3] = static_cast<uint8_t>((dataLen >> 8) & 0xFF);
  out[4] = static_cast<uint8_t>(dataLen & 0xFF);
  out[5] = static_cast<uint8_t>((type >> 8) & 0xFF);
  out[6] = static_cast<uint8_t>(type & 0xFF);
  out[7] = calcXorInverse(out, 7);

  if (dataLen > 0) {
    if (data == nullptr) return false;
    memcpy(out + 8, data, dataLen);
    out[8 + dataLen] = calcXorInverse(out + 8, dataLen);
  }

  return serial.write(out, totalLen) == totalLen;
}

LD6002::FrameType LD6002::getFrameType(const uint8_t *frame) const
{
  uint16_t type = (static_cast<uint16_t>(frame[5]) << 8) | frame[6];
  switch (type) {
    case 0xFFFF: return FrameType::FirmwareInfo;
    case 0x0F09: return FrameType::HumanPresence;
    case 0x0A04: return FrameType::PersonPosition;
    case 0x0A13: return FrameType::Phase;
    case 0x0A14: return FrameType::BreathRate;
    case 0x0A15: return FrameType::HeartRate;
    case 0x0A16: return FrameType::Distance;
    case 0x0A17: return FrameType::TrackPosition;
    default: return FrameType::Unknown;
  }
}

void LD6002::parseFrame(const uint8_t *frame)
{
  if (calcXorInverse(frame, 7) != frame[7]) return;

  const uint16_t dataLen = (static_cast<uint16_t>(frame[3]) << 8) | frame[4];
  const uint16_t totalLen = static_cast<uint16_t>(8 + dataLen + ((dataLen > 0) ? 1 : 0));
  if (totalLen > MAX_FRAME_LEN) return;
  if (dataLen > 0 && calcXorInverse(frame + 8, dataLen) != frame[8 + dataLen]) return;

  const uint8_t *data = frame + 8;
  FrameType type = getFrameType(frame);

  switch (type) {
    case FrameType::FirmwareInfo:
    {
      if (dataLen < 4) return;
      firmwareInfo.projectName = data[0];
      firmwareInfo.majorVersion = data[1];
      firmwareInfo.subVersion = data[2];
      firmwareInfo.modifiedVersion = data[3];
      newFirmwareInfo = true;
      break;
    }
    case FrameType::HumanPresence:
    {
      if (dataLen < 2) return;
      humanPresenceRaw = bytesToUInt16(data);
      newHumanPresence = true;
      break;
    }
    case FrameType::PersonPosition:
    {
      if (dataLen < 4) return;
      personPosition.targetNum = bytesToInt32(data);
      personPosition.hasFirstTarget = false;
      personPosition.hasClusterId = false;
      personPosition.zReliable = false;
      personPosition.zRawBits = 0;
      personPosition.x = 0;
      personPosition.y = 0;
      personPosition.z = 0;
      personPosition.dopplerIndex = 0;
      personPosition.clusterId = 0;

      if (personPosition.targetNum <= 0) {
        latestAngle.valid = false;
        latestAngle.azimuthDeg = 0;
        latestAngle.elevationDeg = 0;
        latestAngle.rangeM = 0;
      } else if (dataLen >= 20) {
        personPosition.x = bytesToFloat(data + 4);
        personPosition.y = bytesToFloat(data + 8);
        personPosition.zRawBits = bytesToUInt32(data + 12);
        // 0A04 protocol note: Z axis is not meaningful, ignore decoded value.
        personPosition.z = 0.0f;
        personPosition.zReliable = false;
        // 0A04 protocol note: dop_idx can be ignored for this application.
        personPosition.dopplerIndex = 0;

        if (dataLen >= 24) {
          personPosition.clusterId = bytesToInt32(data + 20);
          personPosition.hasClusterId = true;
        }
        personPosition.hasFirstTarget = true;
        updateAngleFromXYZ(personPosition.x, personPosition.y, personPosition.z);
      } else {
        latestAngle.valid = false;
        latestAngle.azimuthDeg = 0;
        latestAngle.elevationDeg = 0;
        latestAngle.rangeM = 0;
      }
      newPersonPosition = true;
      break;
    }
    case FrameType::Phase:
    {
      if (dataLen < 12) return;
      phaseData.totalPhase = bytesToFloat(data);
      phaseData.breathPhase = bytesToFloat(data + 4);
      phaseData.heartPhase = bytesToFloat(data + 8);
      newPhase = true;
      break;
    }
    case FrameType::BreathRate:
    {
      if (dataLen < 4) return;
      breathRate = bytesToFloat(data);
      newBreathRate = true;
      break;
    }
    case FrameType::HeartRate:
    {
      if (dataLen < 4) return;
      heartRate = bytesToFloat(data);
      newHeartRate = true;
      break;
    }
    case FrameType::Distance:
    {
      if (dataLen < 8) return;
      uint32_t flag = bytesToUInt32(data);
      if (flag == 1) {
        distance = bytesToFloat(data + 4);
        newDistance = true;
      }
      break;
    }
    case FrameType::TrackPosition:
    {
      if (dataLen < 12) return;
      trackPosition.x = bytesToFloat(data);
      trackPosition.y = bytesToFloat(data + 4);
      trackPosition.z = bytesToFloat(data + 8);
      updateAngleFromXYZ(trackPosition.x, trackPosition.y, trackPosition.z);
      newTrackPosition = true;
      break;
    }
    default:
    {
      break;
    }
  }
}

void LD6002::printHex(const uint8_t *data, int len) const
{
  for (int i = 0; i < len; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
}
