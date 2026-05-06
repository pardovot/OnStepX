#pragma once
#include "../../../Common.h"

#if ABSOLUTE_MOTOR_POSITION == ON

#pragma pack(1)
typedef struct AmpSettings {
  float driftThreshold;  // radians
  float eastLimit;       // radians east of home (Deg90 - eastLimit = min absoluteMotorPos1)
  float westLimit;       // radians west of home (Deg90 + westLimit = max absoluteMotorPos1)
  float horizonLimit;    // radians, altitude minimum
} AmpSettings;
#pragma pack()

#define AmpSettingsSize 16

class AbsoluteMotorPosition {
public:
  void init();
  void savePosition();
  void resetOnHome();
  void applyDriftCorrection();
  void checkLimits();
  bool errorEast = false;
  bool errorWest = false;
  bool errorHorizon = false;

  bool command(char *reply, char *command, char *parameter,
               bool *supressFrame, bool *numericReply, CommandError *commandError);

  double getAbsoluteMotorPos1();
  double getAbsoluteMotorPos2();

private:
  double absoluteOffset1 = 0.0;
  double absoluteOffset2 = 0.0;

  AmpSettings settings = {
    degToRadF(AMP_DRIFT_THRESHOLD_DEG),
    degToRadF(AMP_RA_EAST_LIMIT_DEG),
    degToRadF(AMP_RA_WEST_LIMIT_DEG),
    degToRadF(AMP_HORIZON_LIMIT_DEG)
  };

  bool homed = false;

  // gates resetOnHome() until init() has loaded NV state.
  // boot-time home.reset() (from Mount::begin) calls resetOnHome before init runs.
  // skipping it preserves persisted AMP state across reboots.
  bool initialized = false;

  // altitude at the most recent horizon-trip stop; used by checkLimits() to
  // detect when the mount is being driven further into the violation
  double lastStopAltitude = 0.0;
};

extern AbsoluteMotorPosition amp;

#endif
