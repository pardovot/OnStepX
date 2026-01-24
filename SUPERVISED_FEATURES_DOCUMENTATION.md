# Supervised Features Documentation

## Overview

The "Supervised" system provides enhanced position tracking and safety features for telescope mounts without absolute encoders. It maintains a "true position" (actual motor position) separate from the "virtual position" (coordinate system position), enabling:

1. Power-off position memory
2. Motor-based RA limits (independent of coordinate system)
3. Automatic position synchronization during GOTO operations
4. Home position calibration persistence

---

## Core Concept: True Position vs Virtual Position

### The Problem

Standard OnStepX tracks position using `instrumentCoordinate` which can drift from actual motor position due to:

- Alignment model corrections
- Sync operations
- Coordinate transformations

### The Solution

Supervised features maintain a separate "Absolute Index" (`Absolute_index`) that tracks the true motor position offset from home. This allows:

```
True Position = motorPosition + Absolute_index
Virtual Position = instrumentCoordinate - 90°
```

The 90° offset accounts for the home position being at the celestial pole.

---

## Feature 1: Power-Off Position Memory

### Purpose

Remembers mount position across power cycles without requiring re-homing or parking.

### How It Works

**On Startup (Mount.begin):**

```cpp
// Read stored motor positions from FRAM
float axis1_pos = nv.readF(NV_AXIS1_POSITION);
float axis2_pos = nv.readF(NV_AXIS2_POSITION);

// Restore the absolute index
axis1.absoluteIndex = axis1_pos;
axis2.absoluteIndex = axis2_pos;

// Set instrument coordinates based on true position
axis1.setInstrumentCoordinate(axis1_pos + 90°);
axis2.setInstrumentCoordinate(axis2_pos + latitude_sign * 90°);
```

**During Operation (Mount.poll - called every second):**

```cpp
// Calculate and store current true position
float truePos1 = axis1.getMotorPosition() + axis1.absoluteIndex;
float truePos2 = axis2.getMotorPosition() + axis2.absoluteIndex;

nv.write(NV_AXIS1_POSITION, truePos1);
nv.write(NV_AXIS2_POSITION, truePos2);
```

### Implementation Requirements

- Two NV addresses for storing axis positions (4 bytes each, float)
- One NV address for mount type validation (1 byte)
- FRAM storage (high write endurance)
- Feature enable flag
- User preference storage (enable/disable)

### Comparison with MOUNT_COORDS_MEMORY

| Aspect     | MOUNT_COORDS_MEMORY        | Supervised Memory                    |
| ---------- | -------------------------- | ------------------------------------ |
| Stores     | Instrument coordinates     | Motor positions + absolute index     |
| Accuracy   | Subject to alignment drift | Tracks true motor position           |
| Use case   | Quick approximate restore  | Precise position tracking            |
| Dependency | None                       | Requires supervised home calibration |

**Recommendation:** Use only ONE of these systems. Supervised memory is more accurate but requires initial home calibration. MOUNT_COORDS_MEMORY is simpler but less precise.

---

## Feature 2: Supervised Home

### Purpose

Establishes and maintains the absolute position reference when homing the mount.

### How It Works

When the mount performs a home operation (parking at known position):

```cpp
void setInstrumentCoordinatePark(long value, int modulo) {
    // Standard index calculation
    indexSteps = calculateAlignedSteps(value, modulo);

    // If supervised home is enabled, also set absolute index
    if (supervisedHomeEnabled) {
        absoluteIndexSteps = indexSteps;
    }
}
```

This synchronizes the absolute index with the standard index at the home position, establishing the reference point for all supervised features.

### Implementation Requirements

- Boolean flag: `supervisedHomeEnabled`
- NV storage for user preference (1 byte, inverted logic for default-on)
- Integration with existing home/park routines

---

## Feature 3: Supervised RA Limits

### Purpose

Enforces RA axis limits based on true motor position rather than coordinate system position. Prevents cable wrap and mechanical interference regardless of alignment corrections.

### How It Works

**Limit Check (in Limits.poll):**

```cpp
if (supervisedRaLimitEnabled && mountType == GEM) {
    float trueRaPosition = axis1.getMotorPosition() + axis1.absoluteIndex;

    // Check east limit (tracking direction)
    if (trueRaPosition > raLimitWest * DEG_TO_RAD) {
        stopAxis1(FORWARD);
        error.limit.axis1.max = true;
    }

    // Check west limit (opposite tracking)
    if (trueRaPosition < -raLimitEast * DEG_TO_RAD) {
        stopAxis1(REVERSE);
        error.limit.axis1.min = true;
    }
}
```

### Configuration

- `raLimitEast`: Degrees past meridian allowed (east/right side)
- `raLimitWest`: Degrees past meridian allowed (west/left side)
- Default: 95° each side
- Range: 1-180°
- Hemisphere-aware: Limits swap for southern hemisphere

### Implementation Requirements

- Two NV addresses for limit values (1 byte each)
- Boolean flag: `supervisedRaLimitEnabled`
- NV storage for user preference
- Integration with existing limit polling

---

## Feature 4: Supervised GOTO

### Purpose

Automatically synchronizes virtual position to true position before large slews, preventing accumulated drift from causing pointing errors.

### How It Works

**Before GOTO execution:**

```cpp
void syncTruePosToVirtualPos(Coordinate gotoTarget) {
    // Skip if target is close to previous target (likely centering)
    float targetDistance = angularDistance(gotoTarget, lastTarget);
    if (targetDistance <= 10.0°) return;

    lastTarget = gotoTarget;

    // Calculate distance between true and virtual positions
    float drift = getDistanceBetweenTruePosAndVirtualPos();

    // If drift exceeds threshold, synchronize
    if (drift >= syncThreshold) {
        axis1.setInstrumentCoordinate(axis1.getMotorPosition() + axis1.absoluteIndex + 90°);
        axis2.setInstrumentCoordinate(axis2.getMotorPosition() + axis2.absoluteIndex + 90°);
    }
}
```

**Distance Calculation:**

```cpp
float getDistanceBetweenTruePosAndVirtualPos() {
    // Convert true motor position to equatorial coordinates
    Coordinate trueCoord = instrumentToMount(
        axis1.getMotorPosition() + axis1.absoluteIndex + 90°,
        axis2.getMotorPosition() + axis2.absoluteIndex + 90°
    );

    // Get current virtual position
    Coordinate virtualCoord = getPosition();

    // Return angular separation
    return angularDistance(trueCoord, virtualCoord);
}
```

### Configuration

- `syncThreshold`: Degrees of drift before auto-sync (default: 15°, range: 5-30°)
- Centering tolerance: 10° (targets closer than this skip sync)

### Implementation Requirements

- NV address for threshold value (1 byte)
- NV address for enable flag (1 byte)
- Boolean flag: `supervisedGotoEnabled`
- Last target storage for centering detection
- Angular distance calculation function

---

## NV Memory Layout

| Address | Size | Purpose                               |
| ------- | ---- | ------------------------------------- |
| BASE+0  | 4    | Axis 1 true position (float)          |
| BASE+4  | 4    | Axis 2 true position (float)          |
| BASE+8  | 1    | RA limit east (degrees)               |
| BASE+9  | 1    | RA limit west (degrees)               |
| BASE+10 | 1    | Supervised home enable (inverted)     |
| BASE+11 | 1    | Supervised RA limit enable (inverted) |
| BASE+12 | 1    | Supervised GOTO enable                |
| BASE+13 | 1    | GOTO sync threshold (degrees)         |
| BASE+14 | 1    | Power-off memory enable (inverted)    |

---

## Implementation Checklist

### Axis/Motor Layer

- [ ] Add `absoluteIndex` variable to Axis class
- [ ] Add `absoluteIndexSteps` variable to Motor class
- [ ] Add getter/setter methods for absolute index
- [ ] Modify `setInstrumentCoordinatePark()` to update absolute index

### Mount Layer

- [ ] Add position restore logic in `begin()`
- [ ] Add position save logic in `poll()`
- [ ] Add `getDistanceBetweenTruePosAndVirtualPos()` method
- [ ] Add `syncTruePosToVirtualPos()` method
- [ ] Add feature enable flags

### Limits Layer

- [ ] Add supervised RA limit variables
- [ ] Add limit check logic in `poll()`
- [ ] Add `setRaLimit()` method

### Command Interface

- [ ] Add commands to get/set supervised home enable
- [ ] Add commands to get/set supervised RA limit enable
- [ ] Add commands to get/set RA limit values
- [ ] Add commands to get/set supervised GOTO enable
- [ ] Add commands to get/set GOTO sync threshold
- [ ] Add commands to get/set power-off memory enable
- [ ] Add diagnostic commands for true vs virtual position

### Configuration

- [ ] Add master enable flag for all supervised features
- [ ] Add NV address definitions
- [ ] Add default value handling with validation

---

## Command Protocol Example

```
:SHg#     - Get supervised home status (0=off, 1=on)
:SHs[n]#  - Set supervised home (0=off, 1=on)
:SRg#     - Get supervised RA limit status
:SRs[n]#  - Set supervised RA limit status
:SRl#     - Get RA limits (returns "left|right")
:SRlL[n]# - Set left RA limit (degrees)
:SRlR[n]# - Set right RA limit (degrees)
:SGg#     - Get supervised GOTO status
:SGs[n]#  - Set supervised GOTO status
:SGt#     - Get GOTO sync threshold
:SGt[n]#  - Set GOTO sync threshold (5-30 degrees)
:SMg#     - Get power-off memory status
:SMs[n]#  - Set power-off memory status
:SDp#     - Get true vs virtual position diagnostic
```

---

## Key Design Decisions

1. **Inverted Storage Logic**: User preferences stored as "don't use" flags (1=disabled) so default NV value (0xFF or 0x00) enables features.

2. **Hemisphere Handling**: RA limits swap east/west for southern hemisphere to maintain consistent "left/right" user interface.

3. **Centering Detection**: GOTO sync skips if target is within 10° of previous target, assuming user is centering same object.

4. **Validation Ranges**: All stored values validated on read with sensible defaults applied if out of range.

5. **Mount Type Check**: Position restore validates stored mount type matches current configuration.
