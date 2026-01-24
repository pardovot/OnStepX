//--------------------------------------------------------------------------------------------------
// telescope mount control - supervised features

#include "Supervised.h"

#ifdef MOUNT_PRESENT
#ifdef SUPERVISED_FEATURES

#include "../../../lib/tasks/OnTask.h"
#include "../Mount.h"
#include "../site/Site.h"

// Initialize supervised features from NV storage
void Supervised::init() {
  // Confirm the data structure size
  if (SupervisedSettingsSize < sizeof(SupervisedSettings)) {
    nv.initError = true;
    DL("ERR: Supervised::init(), SupervisedSettingsSize error");
    return;
  }

  // Write default settings to NV if not valid or null
  if (!nv.hasValidKey() || nv.isNull(NV_SUPERVISED_BASE, sizeof(SupervisedSettings))) {
    VLF("MSG: Mount, supervised writing defaults to NV");
    
    // Set default values
    supervisedSettings.axis1TruePosition = 0.0f;
    supervisedSettings.axis2TruePosition = 0.0f;
    supervisedSettings.raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    supervisedSettings.raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    supervisedSettings.homeEnable = SUPERVISED_ENABLED;       // Default enabled
    supervisedSettings.raLimitEnable = SUPERVISED_DISABLED;   // Default disabled
    supervisedSettings.gotoEnable = 0;                        // Default disabled
    supervisedSettings.syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    supervisedSettings.memoryEnable = SUPERVISED_DISABLED;    // Default disabled
    supervisedSettings.mountTypeValidation = MOUNT_SUBTYPE;
    
    nv.writeBytes(NV_SUPERVISED_BASE, &supervisedSettings, sizeof(SupervisedSettings));
  }

  // Read settings from NV
  nv.readBytes(NV_SUPERVISED_BASE, &supervisedSettings, sizeof(SupervisedSettings));

  // Validate and apply defaults if needed
  validateSettings();

  VLF("MSG: Mount, supervised features initialized");
  VF("MSG: Mount, supervised home="); VL(homeEnabled() ? "ON" : "OFF");
  VF("MSG: Mount, supervised RA limits="); VL(raLimitEnabled() ? "ON" : "OFF");
  VF("MSG: Mount, supervised GOTO="); VL(gotoEnabled() ? "ON" : "OFF");
  VF("MSG: Mount, supervised memory="); VL(memoryEnabled() ? "ON" : "OFF");
}

// Validate settings and apply defaults if invalid
void Supervised::validateSettings() {
  bool needsWrite = false;

  // Validate RA limit east (1-180 degrees)
  if (supervisedSettings.raLimitEast < SUPERVISED_RA_LIMIT_MIN || 
      supervisedSettings.raLimitEast > SUPERVISED_RA_LIMIT_MAX) {
    DLF("WRN: Supervised::validateSettings(), raLimitEast invalid, applying default");
    supervisedSettings.raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    needsWrite = true;
  }

  // Validate RA limit west (1-180 degrees)
  if (supervisedSettings.raLimitWest < SUPERVISED_RA_LIMIT_MIN || 
      supervisedSettings.raLimitWest > SUPERVISED_RA_LIMIT_MAX) {
    DLF("WRN: Supervised::validateSettings(), raLimitWest invalid, applying default");
    supervisedSettings.raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    needsWrite = true;
  }

  // Validate sync threshold (5-30 degrees)
  if (supervisedSettings.syncThreshold < SUPERVISED_SYNC_THRESHOLD_MIN || 
      supervisedSettings.syncThreshold > SUPERVISED_SYNC_THRESHOLD_MAX) {
    DLF("WRN: Supervised::validateSettings(), syncThreshold invalid, applying default");
    supervisedSettings.syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    needsWrite = true;
  }

  // Validate home enable (must be SUPERVISED_ENABLED or SUPERVISED_DISABLED)
  if (supervisedSettings.homeEnable != SUPERVISED_ENABLED && 
      supervisedSettings.homeEnable != SUPERVISED_DISABLED) {
    DLF("WRN: Supervised::validateSettings(), homeEnable invalid, applying default");
    supervisedSettings.homeEnable = SUPERVISED_ENABLED;
    needsWrite = true;
  }

  // Validate RA limit enable (must be SUPERVISED_ENABLED or SUPERVISED_DISABLED)
  if (supervisedSettings.raLimitEnable != SUPERVISED_ENABLED && 
      supervisedSettings.raLimitEnable != SUPERVISED_DISABLED) {
    DLF("WRN: Supervised::validateSettings(), raLimitEnable invalid, applying default");
    supervisedSettings.raLimitEnable = SUPERVISED_DISABLED;
    needsWrite = true;
  }

  // Validate memory enable (must be SUPERVISED_ENABLED or SUPERVISED_DISABLED)
  if (supervisedSettings.memoryEnable != SUPERVISED_ENABLED && 
      supervisedSettings.memoryEnable != SUPERVISED_DISABLED) {
    DLF("WRN: Supervised::validateSettings(), memoryEnable invalid, applying default");
    supervisedSettings.memoryEnable = SUPERVISED_DISABLED;
    needsWrite = true;
  }

  if (needsWrite) {
    writeSettings();
  }
}

// Write settings to NV storage
void Supervised::writeSettings() {
  nv.writeBytes(NV_SUPERVISED_BASE, &supervisedSettings, sizeof(SupervisedSettings));
}

// Poll for periodic tasks
void Supervised::poll() {
  // Position saving every second when memory is enabled
  if (memoryEnabled()) {
    unsigned long now = millis();
    if (now - lastSaveTime >= 1000) {
      lastSaveTime = now;
      savePositions();
    }
  }
}

// Save true positions to NV storage
void Supervised::savePositions() {
  #ifdef SUPERVISED_FEATURES
  // Calculate true positions (motorPosition + absoluteIndex)
  float axis1True = (float)axis1.getTruePosition();
  float axis2True = (float)axis2.getTruePosition();

  // Only write if values have changed significantly
  if (fabs(axis1True - supervisedSettings.axis1TruePosition) > 0.0001f ||
      fabs(axis2True - supervisedSettings.axis2TruePosition) > 0.0001f) {
    supervisedSettings.axis1TruePosition = axis1True;
    supervisedSettings.axis2TruePosition = axis2True;
    supervisedSettings.mountTypeValidation = MOUNT_SUBTYPE;
    
    // Write with ignoreCache for immediate persistence
    nv.writeBytes(NV_SUPERVISED_BASE, &supervisedSettings, sizeof(SupervisedSettings));
  }
  #endif
}

// Restore positions from NV storage
void Supervised::restorePositions() {
  // Check mount type validation
  if (supervisedSettings.mountTypeValidation != MOUNT_SUBTYPE) {
    VLF("MSG: Mount, supervised position restore skipped - mount type mismatch");
    return;
  }

  // Restore positions to axes
  VF("MSG: Mount, supervised restoring positions: axis1=");
  V(radToDeg(supervisedSettings.axis1TruePosition));
  VF(" axis2=");
  VL(radToDeg(supervisedSettings.axis2TruePosition));

  // Set the absolute index to restore true position
  #ifdef SUPERVISED_FEATURES
  axis1.setAbsoluteIndex(supervisedSettings.axis1TruePosition);
  axis2.setAbsoluteIndex(supervisedSettings.axis2TruePosition);
  #endif
}

// Feature enables - Supervised Home
bool Supervised::homeEnabled() {
  return supervisedSettings.homeEnable == SUPERVISED_ENABLED;
}

void Supervised::setHomeEnabled(bool state) {
  supervisedSettings.homeEnable = state ? SUPERVISED_ENABLED : SUPERVISED_DISABLED;
  nv.write(NV_SUPERVISED_HOME_ENABLE, supervisedSettings.homeEnable);
}

// Feature enables - Supervised RA Limits
bool Supervised::raLimitEnabled() {
  return supervisedSettings.raLimitEnable == SUPERVISED_ENABLED;
}

void Supervised::setRaLimitEnabled(bool state) {
  supervisedSettings.raLimitEnable = state ? SUPERVISED_ENABLED : SUPERVISED_DISABLED;
  nv.write(NV_SUPERVISED_RA_LIMIT_ENABLE, supervisedSettings.raLimitEnable);
}

// Feature enables - Supervised GOTO
bool Supervised::gotoEnabled() {
  return supervisedSettings.gotoEnable != 0;
}

void Supervised::setGotoEnabled(bool state) {
  supervisedSettings.gotoEnable = state ? 1 : 0;
  nv.write(NV_SUPERVISED_GOTO_ENABLE, supervisedSettings.gotoEnable);
}

// Feature enables - Power-off Memory
bool Supervised::memoryEnabled() {
  return supervisedSettings.memoryEnable == SUPERVISED_ENABLED;
}

void Supervised::setMemoryEnabled(bool state) {
  supervisedSettings.memoryEnable = state ? SUPERVISED_ENABLED : SUPERVISED_DISABLED;
  nv.write(NV_SUPERVISED_MEMORY_ENABLE, supervisedSettings.memoryEnable);
}

// RA Limits configuration
uint8_t Supervised::getRaLimitEast() {
  return supervisedSettings.raLimitEast;
}

uint8_t Supervised::getRaLimitWest() {
  return supervisedSettings.raLimitWest;
}

void Supervised::setRaLimitEast(uint8_t degrees) {
  if (degrees >= SUPERVISED_RA_LIMIT_MIN && degrees <= SUPERVISED_RA_LIMIT_MAX) {
    supervisedSettings.raLimitEast = degrees;
    nv.write(NV_SUPERVISED_RA_LIMIT_EAST, supervisedSettings.raLimitEast);
  }
}

void Supervised::setRaLimitWest(uint8_t degrees) {
  if (degrees >= SUPERVISED_RA_LIMIT_MIN && degrees <= SUPERVISED_RA_LIMIT_MAX) {
    supervisedSettings.raLimitWest = degrees;
    nv.write(NV_SUPERVISED_RA_LIMIT_WEST, supervisedSettings.raLimitWest);
  }
}

// GOTO sync threshold configuration
uint8_t Supervised::getSyncThreshold() {
  return supervisedSettings.syncThreshold;
}

void Supervised::setSyncThreshold(uint8_t degrees) {
  if (degrees >= SUPERVISED_SYNC_THRESHOLD_MIN && degrees <= SUPERVISED_SYNC_THRESHOLD_MAX) {
    supervisedSettings.syncThreshold = degrees;
    nv.write(NV_SUPERVISED_SYNC_THRESHOLD, supervisedSettings.syncThreshold);
  }
}

// Get the angular distance between true and virtual positions in radians
// Requirements: 5.1, 5.7, 9.2, 9.3, 9.4
float Supervised::getDistanceBetweenTruePosAndVirtualPos() {
  #ifdef SUPERVISED_FEATURES
  // Get true motor positions (motorPosition + absoluteIndex)
  double trueAxis1 = axis1.getTruePosition();
  double trueAxis2 = axis2.getTruePosition();
  
  // Convert true motor position to mount coordinates (equatorial for GEM/FORK)
  // The instrumentToMount function handles the conversion from instrument coordinates
  // to mount coordinates (h, d for equatorial mounts)
  Coordinate trueCoord = transform.instrumentToMount(trueAxis1, trueAxis2);
  
  // Get current virtual position (instrument coordinates)
  double virtualAxis1 = axis1.getInstrumentCoordinate();
  double virtualAxis2 = axis2.getInstrumentCoordinate();
  
  // Convert virtual position to mount coordinates
  Coordinate virtualCoord = transform.instrumentToMount(virtualAxis1, virtualAxis2);
  
  // Calculate angular separation between true and virtual positions
  // For equatorial mounts, use hour angle (h) and declination (d)
  // For altazm mounts, use azimuth (z) and altitude (a)
  double delta1, delta2;
  
  if (transform.mountType == ALTAZM) {
    delta1 = trueCoord.z - virtualCoord.z;
    delta2 = trueCoord.a - virtualCoord.a;
  } else {
    // Equatorial mount (GEM, FORK)
    delta1 = trueCoord.h - virtualCoord.h;
    delta2 = trueCoord.d - virtualCoord.d;
  }
  
  // Calculate angular separation using spherical geometry approximation
  // For small angles, this is approximately sqrt(delta1^2 + delta2^2)
  // For larger angles, use the haversine formula or great circle distance
  // Using simplified formula: sqrt(delta1^2 * cos^2(dec) + delta2^2)
  // where dec is the average declination
  double avgDec;
  if (transform.mountType == ALTAZM) {
    avgDec = (trueCoord.a + virtualCoord.a) / 2.0;
  } else {
    avgDec = (trueCoord.d + virtualCoord.d) / 2.0;
  }
  
  double cosDec = cos(avgDec);
  double distance = sqrt((delta1 * delta1 * cosDec * cosDec) + (delta2 * delta2));
  
  // Ensure non-negative result
  return (float)fabs(distance);
  #else
  return 0.0f;
  #endif
}

// Synchronize true position to virtual position before GOTO
// Requirements: 5.2, 5.3
void Supervised::syncTruePosToVirtualPos(Coordinate& gotoTarget) {
  #ifdef SUPERVISED_FEATURES
  // Check if supervised GOTO is enabled
  if (!gotoEnabled()) return;
  
  // Centering detection: check if target is within 10 degrees of last target
  // If within centering tolerance, skip synchronization (user is centering on an object)
  if (lastTargetValid) {
    // Calculate distance between current target and last target
    double deltaH = gotoTarget.h - lastGotoTarget.h;
    double deltaD = gotoTarget.d - lastGotoTarget.d;
    
    // Use average declination for proper angular distance calculation
    double avgDec = (gotoTarget.d + lastGotoTarget.d) / 2.0;
    double cosDec = cos(avgDec);
    double targetDistance = sqrt((deltaH * deltaH * cosDec * cosDec) + (deltaD * deltaD));
    
    // Convert centering tolerance from degrees to radians
    double centeringToleranceRad = degToRad(SUPERVISED_CENTERING_TOLERANCE);
    
    if (targetDistance < centeringToleranceRad) {
      // Target is within centering tolerance of last target - skip sync
      VLF("MSG: Mount, supervised GOTO sync skipped - centering detection");
      // Update last target even when skipping
      lastGotoTarget = gotoTarget;
      return;
    }
  }
  
  // Calculate drift distance between true and virtual positions
  float driftDistance = getDistanceBetweenTruePosAndVirtualPos();
  
  // Convert sync threshold from degrees to radians
  double syncThresholdRad = degToRad((double)supervisedSettings.syncThreshold);
  
  // Check if drift exceeds threshold
  if (driftDistance > syncThresholdRad) {
    VF("MSG: Mount, supervised GOTO sync triggered - drift=");
    V(radToDeg(driftDistance));
    VF(" deg, threshold=");
    VL(supervisedSettings.syncThreshold);
    
    // Get true positions
    double trueAxis1 = axis1.getTruePosition();
    double trueAxis2 = axis2.getTruePosition();
    
    // Synchronize instrument coordinates to true position
    // This aligns the virtual position with the true motor position
    axis1.setInstrumentCoordinate(trueAxis1);
    axis2.setInstrumentCoordinate(trueAxis2);
    
    VLF("MSG: Mount, supervised GOTO sync complete - instrument coordinates updated");
  } else {
    VF("MSG: Mount, supervised GOTO sync not needed - drift=");
    V(radToDeg(driftDistance));
    VF(" deg < threshold=");
    VL(settings.syncThreshold);
  }
  
  // Update last target
  lastGotoTarget = gotoTarget;
  lastTargetValid = true;
  #else
  UNUSED(gotoTarget);
  #endif
}

// Check supervised RA limits
// Returns: 0 = no violation, 1 = east limit exceeded, 2 = west limit exceeded
int Supervised::checkRaLimits() {
  #ifdef SUPERVISED_FEATURES
  // Only check for GEM mounts
  if (transform.mountType != GEM) return 0;
  
  // Get true RA position in radians
  // True position = motorPosition + absoluteIndex
  double trueRaPosition = axis1.getTruePosition();
  
  // Convert limits from degrees to radians
  double eastLimitRad = degToRad((double)supervisedSettings.raLimitEast);
  double westLimitRad = degToRad((double)supervisedSettings.raLimitWest);
  
  // Handle hemisphere swap for southern latitudes
  // In southern hemisphere, east and west are swapped
  bool southernHemisphere = site.location.latitude < 0;
  
  double effectiveEastLimit, effectiveWestLimit;
  if (southernHemisphere) {
    // Swap limits for southern hemisphere
    effectiveEastLimit = westLimitRad;
    effectiveWestLimit = eastLimitRad;
  } else {
    effectiveEastLimit = eastLimitRad;
    effectiveWestLimit = westLimitRad;
  }
  
  // Check limits
  // East limit: true position should not go below -eastLimit (reverse direction)
  // West limit: true position should not go above +westLimit (forward direction)
  if (trueRaPosition < -effectiveEastLimit) {
    VF("MSG: Mount, supervised RA east limit exceeded: ");
    V(radToDeg(trueRaPosition));
    VF(" < -");
    VL(radToDeg(effectiveEastLimit));
    return 1; // East limit exceeded
  }
  
  if (trueRaPosition > effectiveWestLimit) {
    VF("MSG: Mount, supervised RA west limit exceeded: ");
    V(radToDeg(trueRaPosition));
    VF(" > ");
    VL(radToDeg(effectiveWestLimit));
    return 2; // West limit exceeded
  }
  
  #endif
  return 0; // No violation
}

// Command processing is implemented in Supervised.command.cpp

Supervised supervised;

#endif // SUPERVISED_FEATURES
#endif // MOUNT_PRESENT
