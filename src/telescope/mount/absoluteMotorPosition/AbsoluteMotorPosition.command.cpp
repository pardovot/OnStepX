//--------------------------------------------------------------------------------------------------
// absolute motor position safety system, commands
//
// Command interface (:PA prefix):
//   :PAGp#         Get absolute motor positions          → "+aaa.aa,+bbb.bb"  (axis1,axis2 deg)
//   :PAGt#         Get drift threshold                   → "ddd.dd"           (deg)
//   :PAGe#         Get RA east limit (radius from home)  → "ddd.dd"           (deg)
//   :PAGw#         Get RA west limit (radius from home)  → "ddd.dd"           (deg)
//   :PAGh#         Get horizon (altitude min) limit      → "ddd.dd"           (deg)
//   :PAGd#         Get current drift per axis            → "+a.aaaa,+b.bbbb"  (deg, signed)
//   :PAGr#         Get status                            → "h,e,w,r"          (homed,east,west,horizon; 0/1)
//
//   :PASt,[f]#     Set drift threshold        (1..90 deg)
//   :PASe,[f]#     Set RA east limit          (1..180 deg)
//   :PASw,[f]#     Set RA west limit          (1..180 deg)
//   :PASh,[f]#     Set horizon limit          (-30..30 deg)
//   :PASz#         Manual re-anchor: snap absoluteOffset to current virtual coord, mark homed,
//                  persist to NV. Use after a sync that drifts by more than the threshold,
//                  i.e. when applyDriftCorrection() would otherwise undo the sync.

#include "AbsoluteMotorPosition.h"

#if ABSOLUTE_MOTOR_POSITION == ON

#include "../Mount.h"
#include "../../../lib/axis/Axis.h"

bool AbsoluteMotorPosition::command(char *reply, char *command, char *parameter,
                                    bool *supressFrame, bool *numericReply, CommandError *commandError) {
  if (command[0] != 'P' || command[1] != 'A') return false;

  // :PAG[x]# — getters
  if (parameter[0] == 'G' && parameter[2] == 0) {
    *numericReply = false;
    switch (parameter[1]) {
      // absolute motor positions (sync-immune): motorPosition + absoluteOffset
      case 'p':
        sprintf(reply, "%+.2f,%+.2f", radToDeg(getAbsoluteMotorPos1()), radToDeg(getAbsoluteMotorPos2()));
      break;

      // settings
      case 't': sprintf(reply, "%.2f", radToDeg(settings.driftThreshold)); break;
      case 'e': sprintf(reply, "%.2f", radToDeg(settings.eastLimit));      break;
      case 'w': sprintf(reply, "%.2f", radToDeg(settings.westLimit));      break;
      case 'h': sprintf(reply, "%.2f", radToDeg(settings.horizonLimit));   break;

      // drift per axis: instrumentCoord = motorPos + indexPos; absoluteMotorPos = motorPos + offset
      // diff = indexPos - offset, the same quantity applyDriftCorrection() compares to threshold
      case 'd': {
        double drift1 = axis1.getInstrumentCoordinate() - getAbsoluteMotorPos1();
        double drift2 = axis2.getInstrumentCoordinate() - getAbsoluteMotorPos2();
        sprintf(reply, "%+.4f,%+.4f", radToDeg(drift1), radToDeg(drift2));
      } break;

      // status: homed flag + per-limit error flags
      case 'r':
        sprintf(reply, "%d,%d,%d,%d", homed ? 1 : 0, errorEast ? 1 : 0, errorWest ? 1 : 0, errorHorizon ? 1 : 0);
      break;

      default: return false;
    }
    return true;
  }

  // :PAS… — setters and special re-anchor
  if (parameter[0] == 'S') {
    // :PASz# — manual re-anchor: absoluteOffset := instrumentCoord - motorPosition.
    // sets homed=true and persists. ignoreCache used because position writes must hit FRAM
    // immediately (consistent with savePosition()), settings can ride the cache.
    if (parameter[1] == 'z' && parameter[2] == 0) {
      absoluteOffset1 = axis1.getInstrumentCoordinate() - axis1.getMotorPosition();
      absoluteOffset2 = axis2.getInstrumentCoordinate() - axis2.getMotorPosition();
      homed = true;
      nv.ignoreCache(true);
      nv.write(NV_AMP_POSITION_BASE,     (float)absoluteOffset1);
      nv.write(NV_AMP_POSITION_BASE + 4, (float)absoluteOffset2);
      nv.write(NV_AMP_HOMED_BASE,        (uint8_t)1);
      nv.ignoreCache(false);
      VLF("MSG: AMP, manual offset reset");
      return true;
    }

    // all other setters take ":PAS[x],[val]#"
    if (parameter[2] != ',') return false;
    char *conv_end;
    float val = strtof(&parameter[3], &conv_end);
    if (conv_end == &parameter[3]) { *commandError = CE_PARAM_FORM; return true; }

    switch (parameter[1]) {
      case 't':
        if (val < 1.0f || val > 90.0f) { *commandError = CE_PARAM_RANGE; return true; }
        settings.driftThreshold = degToRadF(val);
      break;
      case 'e':
        if (val < 1.0f || val > 180.0f) { *commandError = CE_PARAM_RANGE; return true; }
        settings.eastLimit = degToRadF(val);
      break;
      case 'w':
        if (val < 1.0f || val > 180.0f) { *commandError = CE_PARAM_RANGE; return true; }
        settings.westLimit = degToRadF(val);
      break;
      case 'h':
        if (val < -30.0f || val > 30.0f) { *commandError = CE_PARAM_RANGE; return true; }
        settings.horizonLimit = degToRadF(val);
      break;
      default: return false;
    }
    nv.updateBytes(NV_AMP_SETTINGS_BASE, &settings, sizeof(AmpSettings));
    return true;
  }

  return false;
}

#endif
