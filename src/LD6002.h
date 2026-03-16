#pragma once
#include <Arduino.h>

class LD6002
{
public:
  enum class FrameType : uint16_t
  {
    FirmwareInfo = 0xFFFF,
    HumanPresence = 0x0F09,
    PersonPosition = 0x0A04,
    Phase = 0x0A13,
    BreathRate = 0x0A14,
    HeartRate = 0x0A15,
    Distance = 0x0A16,
    TrackPosition = 0x0A17,
    Unknown = 0xFFFE
  };

  struct PersonPositionData
  {
    int32_t targetNum = 0;
    bool hasFirstTarget = false;
    bool hasClusterId = false;
    bool zReliable = true;
    uint32_t zRawBits = 0;
    float x = 0;
    float y = 0;
    float z = 0;
    int32_t dopplerIndex = 0;
    int32_t clusterId = 0;
  };

  struct PhaseData
  {
    float totalPhase = 0;
    float breathPhase = 0;
    float heartPhase = 0;
  };

  struct TrackPositionData
  {
    float x = 0;
    float y = 0;
    float z = 0;
  };

  struct FirmwareInfoData
  {
    uint8_t projectName = 0;
    uint8_t majorVersion = 0;
    uint8_t subVersion = 0;
    uint8_t modifiedVersion = 0;
  };

  struct AngleData
  {
    bool valid = false;
    float azimuthDeg = 0;     // Horizontal angle, positive to +X, relative to +Y axis
    float elevationDeg = 0;   // Vertical angle, positive upwards
    float rangeM = 0;
  };

  explicit LD6002(HardwareSerial &serial);
  void update();
  bool requestFirmwareInfo(uint16_t frameId = 0x0001);

  bool hasNewFirmwareInfo() const;
  bool hasNewRaw0A04Frame() const;
  bool hasNewHumanPresence() const;
  bool hasNewPersonPosition() const;
  bool hasNewPhase() const;
  bool hasNewHeartRate() const;
  bool hasNewBreathRate() const;
  bool hasNewDistance() const;
  bool hasNewTrackPosition() const;

  const FirmwareInfoData &getFirmwareInfo() const;
  const uint8_t *getRaw0A04Frame() const;
  uint16_t getRaw0A04FrameLen() const;
  const AngleData &getLatestAngle() const;
  bool hasValidAngle() const;
  uint16_t getHumanPresenceRaw() const;
  bool isHumanPresent() const;
  const PersonPositionData &getPersonPosition() const;
  const PhaseData &getPhase() const;
  float getHeartRate() const;
  float getBreathRate() const;
  float getDistance() const;
  const TrackPositionData &getTrackPosition() const;

  void clearFirmwareInfoFlag();
  void clearRaw0A04FrameFlag();
  void clearHumanPresenceFlag();
  void clearPersonPositionFlag();
  void clearPhaseFlag();
  void clearHeartRateFlag();
  void clearBreathRateFlag();
  void clearDistanceFlag();
  void clearTrackPositionFlag();

private:
  static constexpr uint16_t MAX_FRAME_LEN = 256;

  HardwareSerial &serial;
  uint8_t frame[MAX_FRAME_LEN];
  uint16_t pos = 0;
  bool syncing = false;
  uint16_t expectedFrameLen = 0;

  FirmwareInfoData firmwareInfo;
  uint8_t raw0A04Frame[MAX_FRAME_LEN] = {0};
  uint16_t raw0A04FrameLen = 0;
  AngleData latestAngle;
  uint16_t humanPresenceRaw = 0;
  PersonPositionData personPosition;
  PhaseData phaseData;
  float heartRate = 0;
  float breathRate = 0;
  float distance = 0;
  TrackPositionData trackPosition;

  bool newFirmwareInfo = false;
  bool newRaw0A04Frame = false;
  bool newHumanPresence = false;
  bool newPersonPosition = false;
  bool newPhase = false;
  bool newHeartRate = false;
  bool newBreathRate = false;
  bool newDistance = false;
  bool newTrackPosition = false;

  float bytesToFloat(const uint8_t *data) const;
  uint16_t bytesToUInt16(const uint8_t *data) const;
  uint32_t bytesToUInt32(const uint8_t *data) const;
  int32_t bytesToInt32(const uint8_t *data) const;
  uint8_t calcXorInverse(const uint8_t *data, uint16_t len) const;
  void cacheRaw0A04Frame(const uint8_t *frame, uint16_t frameLen);
  void updateAngleFromXYZ(float x, float y, float z);
  bool sendFrame(uint16_t type, uint16_t frameId, const uint8_t *data, uint16_t dataLen);
  FrameType getFrameType(const uint8_t *frame) const;
  void parseFrame(const uint8_t *frame);
  void printHex(const uint8_t *data, int len) const;
};
