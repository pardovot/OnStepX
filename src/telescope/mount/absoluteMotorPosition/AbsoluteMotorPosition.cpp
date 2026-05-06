//--------------------------------------------------------------------------------------------------
// absolute motor position safety system

#include "AbsoluteMotorPosition.h"

#if ABSOLUTE_MOTOR_POSITION == ON

#include "../Mount.h"
#include "../home/Home.h"
#include "../limits/Limits.h"
#include "../coordinates/Transform.h"
#include "../../../lib/axis/Axis.h"

// horizon re-entry hysteresis (deg): how far below the most recent stop's
// altitude the mount can drift before checkLimits() fires limits.stop() again.
// small enough to keep cumulative drift bounded if the user keeps slewing into
// the violation; large enough that minor fluctuations during recovery slews
// don't false-trigger another stop.
static const float HORIZON_REENTRY_HYST_DEG = 0.02F;

void AbsoluteMotorPosition::init() {
  if (AmpSettingsSize < sizeof(AmpSettings)) {
    nv.initError = true;
    DL("ERR: AbsoluteMotorPosition::init(), AmpSettingsSize error");
  }

  if (!nv.hasValidKey() || nv.isNull(NV_AMP_SETTINGS_BASE, sizeof(AmpSettings))) {
    VLF("MSG: AMP, writing defaults to NV");
    nv.writeBytes(NV_AMP_SETTINGS_BASE, &settings, sizeof(AmpSettings));
    nv.write(NV_AMP_POSITION_BASE,     (float)axis1.getInstrumentCoordinate());
    nv.write(NV_AMP_POSITION_BASE + 4, (float)axis2.getInstrumentCoordinate());
    nv.write(NV_AMP_HOMED_BASE,        (uint8_t)1);
  }

  nv.readBytes(NV_AMP_SETTINGS_BASE, &settings, sizeof(AmpSettings));
  absoluteOffset1 = nv.readF(NV_AMP_POSITION_BASE);
  absoluteOffset2 = nv.readF(NV_AMP_POSITION_BASE + 4);
  homed = (nv.readUC(NV_AMP_HOMED_BASE) == 1);
  initialized = true;
}

void AbsoluteMotorPosition::savePosition() {
  nv.ignoreCache(true);
  nv.write(NV_AMP_POSITION_BASE,     (float)getAbsoluteMotorPos1());
  nv.write(NV_AMP_POSITION_BASE + 4, (float)getAbsoluteMotorPos2());
  nv.ignoreCache(false);
}

// called from Home::reset() after resetPosition(0) and setInstrumentCoordinate()
// at this point: motorSteps=0, getInstrumentCoordinate()=Deg90 for GEM axis1
void AbsoluteMotorPosition::resetOnHome() {
  if (!initialized) return;
  absoluteOffset1 = axis1.getInstrumentCoordinate();
  absoluteOffset2 = axis2.getInstrumentCoordinate();
  homed = true;
  nv.ignoreCache(true);
  nv.write(NV_AMP_POSITION_BASE,     (float)absoluteOffset1);
  nv.write(NV_AMP_POSITION_BASE + 4, (float)absoluteOffset2);
  nv.write(NV_AMP_HOMED_BASE,        (uint8_t)1);
  nv.ignoreCache(false);
}

void AbsoluteMotorPosition::applyDriftCorrection() {
  if (!homed) return;

  // getIndexPosition() = indexSteps/stepsPerMeasure, only changes on sync - never from motor
  // movement - so the subtraction stays small and never wraps
  double drift1 = fabs(axis1.getIndexPosition() - absoluteOffset1);
  double drift2 = fabs(axis2.getIndexPosition() - absoluteOffset2);

  if (drift1 >= settings.driftThreshold || drift2 >= settings.driftThreshold) {
    VLF("MSG: AMP, drift threshold exceeded, correcting virtual position to absolute motor position");
    axis1.setInstrumentCoordinate(getAbsoluteMotorPos1());
    axis2.setInstrumentCoordinate(getAbsoluteMotorPos2());
  }
}

void AbsoluteMotorPosition::checkLimits() {
  // capture previous-poll state before reset so we can edge-trigger limits.stop()
  bool lastErrorHorizon = errorHorizon;

  errorEast = false;
  errorWest = false;
  errorHorizon = false;
  if (!homed) return;

  double absPos1 = getAbsoluteMotorPos1();
  double absPos2 = getAbsoluteMotorPos2();

  // RA east: absoluteMotorPos1 below (Deg90 - eastLimit) → too far east
  if (absPos1 < (Deg90 - settings.eastLimit)) {
    VLF("WRN: AMP, axis1 east limit");
    limits.stopAxis1(GA_REVERSE);
    errorEast = true;
  }

  // RA west: absoluteMotorPos1 above (Deg90 + westLimit) → too far west
  if (absPos1 > (Deg90 + settings.westLimit)) {
    VLF("WRN: AMP, axis1 west limit");
    limits.stopAxis1(GA_FORWARD);
    errorWest = true;
  }

  // halt fires on the rising edge of the trip, and again any time the mount
  // drifts more than the hysteresis below the last-stopped altitude (i.e. the
  // user is driving deeper into the violation). recovery slews where alt
  // rises don't trigger.
  Coordinate absCoord = transform.instrumentToMount(absPos1, absPos2);
  transform.equToHor(&absCoord);
  if (absCoord.a < settings.horizonLimit) {
    bool worsening = absCoord.a < lastStopAltitude - degToRadF(HORIZON_REENTRY_HYST_DEG);
    if (!lastErrorHorizon) {
      VLF("WRN: AMP, altitude below horizon limit");
      lastStopAltitude = absCoord.a;
    } else if (worsening) {
      limits.stop();
      lastStopAltitude = absCoord.a;
    }
    errorHorizon = true;
  }
}

double AbsoluteMotorPosition::getAbsoluteMotorPos1() {
  return axis1.getMotorPosition() + absoluteOffset1;
}

double AbsoluteMotorPosition::getAbsoluteMotorPos2() {
  return axis2.getMotorPosition() + absoluteOffset2;
}

AbsoluteMotorPosition amp;

#endif
