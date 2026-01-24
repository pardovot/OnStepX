/**
 * Integration Tests for Supervised Features
 * 
 * Feature: supervised-features
 * 
 * These tests verify the integration of supervised features:
 * - 14.1: Supervised home cycle test
 * - 14.2: Power cycle simulation test
 * - 14.3: RA limit enforcement test
 * 
 * Validates: Requirements 2.1, 3.1-3.6, 4.1-4.3
 */

#include <unity.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

// ============================================================================
// Simulated Mount Types (from Constants.h)
// ============================================================================
#define GEM                         1
#define FORK                        2
#define ALTAZM                      3
#define ALTALT                      4

// Current mount type for testing
static uint8_t MOUNT_SUBTYPE = GEM;

// ============================================================================
// Simulated NV Storage
// ============================================================================
#define NV_SUPERVISED_BASE          1056

// NV storage uses 0xFF for enabled (erased EEPROM default) and 0x00 for disabled
#define SUPERVISED_ENABLED          0xFF
#define SUPERVISED_DISABLED         0x00

// Default values
#define SUPERVISED_RA_LIMIT_DEFAULT     95
#define SUPERVISED_SYNC_THRESHOLD_DEFAULT 15
#define SUPERVISED_RA_LIMIT_MIN         1
#define SUPERVISED_RA_LIMIT_MAX         180

// Tolerance for floating point comparison
#define FLOAT_TOLERANCE 0.0001f

// ============================================================================
// Simulated NV Storage Structure
// ============================================================================
typedef struct {
    float axis1TruePosition;      // BASE+0: 4 bytes
    float axis2TruePosition;      // BASE+4: 4 bytes
    uint8_t raLimitEast;          // BASE+8: 1 byte
    uint8_t raLimitWest;          // BASE+9: 1 byte
    uint8_t homeEnable;           // BASE+10: 1 byte
    uint8_t raLimitEnable;        // BASE+11: 1 byte
    uint8_t gotoEnable;           // BASE+12: 1 byte
    uint8_t syncThreshold;        // BASE+13: 1 byte
    uint8_t memoryEnable;         // BASE+14: 1 byte
    uint8_t mountTypeValidation;  // BASE+15: 1 byte
} SimulatedNVStorage;

static SimulatedNVStorage nvStorage;

// ============================================================================
// Simulated Motor State
// ============================================================================
typedef struct {
    long motorSteps;              // Current motor position in steps
    long indexSteps;              // Index offset in steps
    long absoluteIndexSteps;      // Absolute index for supervised features
    long targetSteps;             // Target position in steps
} SimulatedMotor;

static SimulatedMotor motor1;  // RA/Azimuth axis
static SimulatedMotor motor2;  // Dec/Altitude axis

// Steps per radian (typical value for telescope mounts)
#define STEPS_PER_RADIAN 206264.806  // ~1 arcsec per step

// ============================================================================
// Simulated Axis State
// ============================================================================
typedef struct {
    double absoluteIndex;         // Absolute index in radians
    double motorPosition;         // Motor position in radians
} SimulatedAxis;

static SimulatedAxis axis1;  // RA/Azimuth axis
static SimulatedAxis axis2;  // Dec/Altitude axis

// ============================================================================
// Simulated Site State
// ============================================================================
static double siteLatitude = 45.0;  // Degrees, positive = north

// ============================================================================
// Simulated Supervised Features State
// ============================================================================
static bool supervisedHomeEnabled = true;
static bool supervisedRaLimitEnabled = false;
static bool supervisedMemoryEnabled = false;
static uint8_t raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
static uint8_t raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;

// Error flags for RA limits
static bool eastLimitError = false;
static bool westLimitError = false;
static bool forwardMotionStopped = false;
static bool reverseMotionStopped = false;

// ============================================================================
// Helper Functions
// ============================================================================
double degToRad(double deg) {
    return deg * M_PI / 180.0;
}

double radToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

// ============================================================================
// Simulated Motor Methods
// ============================================================================

// Get motor position in steps
long getMotorPositionSteps(SimulatedMotor* motor) {
    return motor->motorSteps;
}

// Get index position in steps
long getIndexPositionSteps(SimulatedMotor* motor) {
    return motor->indexSteps;
}

// Get absolute index in steps
long getAbsoluteIndexSteps(SimulatedMotor* motor) {
    return motor->absoluteIndexSteps;
}

// Set absolute index in steps
void setAbsoluteIndexSteps(SimulatedMotor* motor, long value) {
    motor->absoluteIndexSteps = value;
}

// Get instrument coordinate in steps
long getInstrumentCoordinateSteps(SimulatedMotor* motor) {
    return motor->motorSteps + motor->indexSteps;
}

// Set instrument coordinate in steps
void setInstrumentCoordinateSteps(SimulatedMotor* motor, long value) {
    motor->indexSteps = value - motor->motorSteps;
}

/**
 * Simulates setInstrumentCoordinateParkSteps with supervised home logic
 * This is called during home operation
 */
void setInstrumentCoordinateParkSteps(SimulatedMotor* motor, long value, int modulo) {
    // Simplified version - just set the index
    motor->indexSteps = value - motor->motorSteps;
    
    // If supervised home is enabled, set absoluteIndexSteps to indexSteps
    if (supervisedHomeEnabled) {
        motor->absoluteIndexSteps = motor->indexSteps;
    }
}

// ============================================================================
// Simulated Axis Methods
// ============================================================================

// Get absolute index in radians
double getAbsoluteIndex(SimulatedAxis* axis) {
    return axis->absoluteIndex;
}

// Set absolute index in radians
void setAbsoluteIndex(SimulatedAxis* axis, double value) {
    axis->absoluteIndex = value;
}

// Get motor position in radians
double getMotorPosition(SimulatedAxis* axis) {
    return axis->motorPosition;
}

// Get true position (motorPosition + absoluteIndex)
double getTruePosition(SimulatedAxis* axis) {
    return axis->motorPosition + axis->absoluteIndex;
}

// ============================================================================
// Simulated Supervised Class Methods
// ============================================================================

// Initialize NV storage with defaults
void initNVStorage() {
    nvStorage.axis1TruePosition = 0.0f;
    nvStorage.axis2TruePosition = 0.0f;
    nvStorage.raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    nvStorage.raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    nvStorage.homeEnable = SUPERVISED_ENABLED;       // Default enabled
    nvStorage.raLimitEnable = SUPERVISED_DISABLED;   // Default disabled
    nvStorage.gotoEnable = 0;
    nvStorage.syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    nvStorage.memoryEnable = SUPERVISED_DISABLED;    // Default disabled
    nvStorage.mountTypeValidation = MOUNT_SUBTYPE;
}

// Save positions to NV storage
void savePositions() {
    if (!supervisedMemoryEnabled) return;
    
    nvStorage.axis1TruePosition = (float)getTruePosition(&axis1);
    nvStorage.axis2TruePosition = (float)getTruePosition(&axis2);
    nvStorage.mountTypeValidation = MOUNT_SUBTYPE;
}

// Restore positions from NV storage
bool restorePositions() {
    // Check mount type validation
    if (nvStorage.mountTypeValidation != MOUNT_SUBTYPE) {
        return false;  // Mount type mismatch - skip restoration
    }
    
    // Restore positions to axes
    setAbsoluteIndex(&axis1, nvStorage.axis1TruePosition);
    setAbsoluteIndex(&axis2, nvStorage.axis2TruePosition);
    return true;
}

// Check supervised RA limits
// Returns: 0 = no violation, 1 = east limit exceeded, 2 = west limit exceeded
int checkRaLimits() {
    // Only check for GEM mounts
    if (MOUNT_SUBTYPE != GEM) return 0;
    
    // Get true RA position in radians
    double trueRaPosition = getTruePosition(&axis1);
    
    // Convert limits from degrees to radians
    double eastLimitRad = degToRad((double)raLimitEast);
    double westLimitRad = degToRad((double)raLimitWest);
    
    // Handle hemisphere swap for southern latitudes
    bool southernHemisphere = siteLatitude < 0;
    
    double effectiveEastLimit, effectiveWestLimit;
    if (southernHemisphere) {
        effectiveEastLimit = westLimitRad;
        effectiveWestLimit = eastLimitRad;
    } else {
        effectiveEastLimit = eastLimitRad;
        effectiveWestLimit = westLimitRad;
    }
    
    // Check limits
    if (trueRaPosition < -effectiveEastLimit) {
        return 1; // East limit exceeded
    }
    
    if (trueRaPosition > effectiveWestLimit) {
        return 2; // West limit exceeded
    }
    
    return 0; // No violation
}

// Check and enforce supervised RA limits
void checkSupervisedRaLimits() {
    if (!supervisedRaLimitEnabled) {
        eastLimitError = false;
        westLimitError = false;
        return;
    }
    
    if (MOUNT_SUBTYPE != GEM) {
        eastLimitError = false;
        westLimitError = false;
        return;
    }
    
    bool lastEastError = eastLimitError;
    bool lastWestError = westLimitError;
    
    int limitResult = checkRaLimits();
    
    if (limitResult == 1) {
        // East limit exceeded - stop reverse motion
        eastLimitError = true;
        westLimitError = false;
        if (!lastEastError) {
            reverseMotionStopped = true;
        }
    } else if (limitResult == 2) {
        // West limit exceeded - stop forward motion
        eastLimitError = false;
        westLimitError = true;
        if (!lastWestError) {
            forwardMotionStopped = true;
        }
    } else {
        eastLimitError = false;
        westLimitError = false;
    }
}

/**
 * Simulate a home operation
 * This sets the motor to home position and calls setInstrumentCoordinateParkSteps
 */
void performHomeOperation(SimulatedMotor* motor, long homeIndexSteps) {
    // Reset motor to home position (0)
    motor->motorSteps = 0;
    motor->targetSteps = 0;
    
    // Set instrument coordinate park steps (this triggers supervised home logic)
    setInstrumentCoordinateParkSteps(motor, homeIndexSteps, 1);
}

// ============================================================================
// Reset Functions
// ============================================================================

void resetMotor(SimulatedMotor* motor) {
    motor->motorSteps = 0;
    motor->indexSteps = 0;
    motor->absoluteIndexSteps = 0;
    motor->targetSteps = 0;
}

void resetAxis(SimulatedAxis* axis) {
    axis->absoluteIndex = 0.0;
    axis->motorPosition = 0.0;
}

void resetAllState() {
    resetMotor(&motor1);
    resetMotor(&motor2);
    resetAxis(&axis1);
    resetAxis(&axis2);
    
    initNVStorage();
    
    supervisedHomeEnabled = true;
    supervisedRaLimitEnabled = false;
    supervisedMemoryEnabled = false;
    raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    
    eastLimitError = false;
    westLimitError = false;
    forwardMotionStopped = false;
    reverseMotionStopped = false;
    
    siteLatitude = 45.0;
    MOUNT_SUBTYPE = GEM;
}

// ============================================================================
// Integration Test 14.1: Supervised Home Cycle Test
// ============================================================================

/**
 * Test 14.1: Supervised Home Cycle Test
 * 
 * Verifies that when supervised home is enabled and a home operation is
 * performed, the absoluteIndexSteps equals the indexSteps.
 * 
 * Validates: Requirements 2.1
 */
void test_supervised_home_cycle() {
    resetAllState();
    
    // Enable supervised home
    supervisedHomeEnabled = true;
    nvStorage.homeEnable = SUPERVISED_ENABLED;
    
    // Set up initial motor state (simulating mount at some position)
    motor1.motorSteps = 10000;
    motor1.indexSteps = 5000;
    motor1.absoluteIndexSteps = 0;  // Not yet homed
    
    // Define home index position (where home sensor triggers)
    long homeIndexSteps = 12345;
    
    // Perform home operation
    performHomeOperation(&motor1, homeIndexSteps);
    
    // Verify: absoluteIndexSteps should equal indexSteps after home
    TEST_ASSERT_EQUAL_MESSAGE(motor1.indexSteps, motor1.absoluteIndexSteps,
        "After supervised home, absoluteIndexSteps should equal indexSteps");
    
    // Verify the index was set correctly
    TEST_ASSERT_EQUAL_MESSAGE(homeIndexSteps, motor1.indexSteps,
        "Index steps should be set to home index value");
}

/**
 * Test 14.1 (continued): Supervised Home Disabled
 * 
 * Verifies that when supervised home is disabled, the absoluteIndexSteps
 * is NOT set during home operation.
 */
void test_supervised_home_disabled() {
    resetAllState();
    
    // Disable supervised home
    supervisedHomeEnabled = false;
    nvStorage.homeEnable = SUPERVISED_DISABLED;
    
    // Set up initial motor state
    motor1.motorSteps = 10000;
    motor1.indexSteps = 5000;
    motor1.absoluteIndexSteps = 999;  // Some existing value
    
    long homeIndexSteps = 12345;
    
    // Perform home operation
    performHomeOperation(&motor1, homeIndexSteps);
    
    // Verify: absoluteIndexSteps should NOT be changed when disabled
    TEST_ASSERT_EQUAL_MESSAGE(999, motor1.absoluteIndexSteps,
        "When supervised home is disabled, absoluteIndexSteps should not change");
}

/**
 * Test 14.1 (continued): Multiple Home Cycles
 * 
 * Verifies that multiple home operations correctly update absoluteIndexSteps
 */
void test_supervised_home_multiple_cycles() {
    resetAllState();
    
    supervisedHomeEnabled = true;
    
    // First home cycle
    motor1.motorSteps = 5000;
    performHomeOperation(&motor1, 10000);
    TEST_ASSERT_EQUAL(motor1.indexSteps, motor1.absoluteIndexSteps);
    long firstAbsIndex = motor1.absoluteIndexSteps;
    
    // Simulate mount movement
    motor1.motorSteps = 20000;
    
    // Second home cycle with different index
    performHomeOperation(&motor1, 15000);
    TEST_ASSERT_EQUAL(motor1.indexSteps, motor1.absoluteIndexSteps);
    
    // Verify the absolute index was updated
    TEST_ASSERT_NOT_EQUAL_MESSAGE(firstAbsIndex, motor1.absoluteIndexSteps,
        "Absolute index should be updated on subsequent home operations");
}

// ============================================================================
// Integration Test 14.2: Power Cycle Simulation Test
// ============================================================================

/**
 * Test 14.2: Power Cycle Simulation Test
 * 
 * Verifies that positions are correctly saved and restored across a
 * simulated power cycle when memory is enabled.
 * 
 * Validates: Requirements 3.1-3.6
 */
void test_power_cycle_simulation() {
    resetAllState();
    
    // Enable memory feature
    supervisedMemoryEnabled = true;
    nvStorage.memoryEnable = SUPERVISED_ENABLED;
    
    // Set up axis positions (simulating mount at some position)
    axis1.motorPosition = 0.0;
    axis1.absoluteIndex = degToRad(45.0);  // 45 degrees
    axis2.motorPosition = 0.0;
    axis2.absoluteIndex = degToRad(30.0);  // 30 degrees
    
    // Save positions (simulates periodic save during operation)
    savePositions();
    
    // Verify positions were saved
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 
        (float)getTruePosition(&axis1), nvStorage.axis1TruePosition);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 
        (float)getTruePosition(&axis2), nvStorage.axis2TruePosition);
    TEST_ASSERT_EQUAL_UINT8(MOUNT_SUBTYPE, nvStorage.mountTypeValidation);
    
    // Simulate power cycle - reset axis state
    axis1.motorPosition = 0.0;
    axis1.absoluteIndex = 0.0;
    axis2.motorPosition = 0.0;
    axis2.absoluteIndex = 0.0;
    
    // Restore positions (simulates startup restore)
    bool restored = restorePositions();
    
    // Verify restoration succeeded
    TEST_ASSERT_TRUE_MESSAGE(restored, "Position restoration should succeed");
    
    // Verify positions were restored correctly
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 
        degToRad(45.0), (float)getTruePosition(&axis1));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 
        degToRad(30.0), (float)getTruePosition(&axis2));
}

/**
 * Test 14.2 (continued): Mount Type Mismatch
 * 
 * Verifies that position restoration is skipped when mount type doesn't match.
 */
void test_power_cycle_mount_type_mismatch() {
    resetAllState();
    
    supervisedMemoryEnabled = true;
    
    // Set up and save positions with GEM mount type
    MOUNT_SUBTYPE = GEM;
    axis1.absoluteIndex = degToRad(45.0);
    axis2.absoluteIndex = degToRad(30.0);
    savePositions();
    
    // Simulate power cycle with different mount type
    axis1.absoluteIndex = 0.0;
    axis2.absoluteIndex = 0.0;
    MOUNT_SUBTYPE = FORK;  // Different mount type
    
    // Attempt to restore
    bool restored = restorePositions();
    
    // Verify restoration was skipped
    TEST_ASSERT_FALSE_MESSAGE(restored, 
        "Position restoration should be skipped for mount type mismatch");
    
    // Verify positions remain unchanged (at 0)
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0, (float)axis1.absoluteIndex);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0, (float)axis2.absoluteIndex);
}

/**
 * Test 14.2 (continued): Memory Disabled
 * 
 * Verifies that positions are not saved when memory is disabled.
 */
void test_power_cycle_memory_disabled() {
    resetAllState();
    
    // Disable memory feature
    supervisedMemoryEnabled = false;
    nvStorage.memoryEnable = SUPERVISED_DISABLED;
    
    // Set up axis positions
    axis1.absoluteIndex = degToRad(45.0);
    axis2.absoluteIndex = degToRad(30.0);
    
    // Clear NV storage
    nvStorage.axis1TruePosition = 0.0f;
    nvStorage.axis2TruePosition = 0.0f;
    
    // Attempt to save positions
    savePositions();
    
    // Verify positions were NOT saved (NV should still be 0)
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, nvStorage.axis1TruePosition);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, nvStorage.axis2TruePosition);
}

/**
 * Test 14.2 (continued): Extreme Position Values
 * 
 * Verifies that extreme position values are correctly saved and restored.
 */
void test_power_cycle_extreme_positions() {
    resetAllState();
    
    supervisedMemoryEnabled = true;
    
    // Set up extreme positions (near full rotation)
    axis1.absoluteIndex = degToRad(350.0);   // Near full rotation
    axis2.absoluteIndex = degToRad(-85.0);   // Near pole
    
    savePositions();
    
    // Simulate power cycle
    axis1.absoluteIndex = 0.0;
    axis2.absoluteIndex = 0.0;
    
    bool restored = restorePositions();
    
    TEST_ASSERT_TRUE(restored);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 
        degToRad(350.0), (float)getTruePosition(&axis1));
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 
        degToRad(-85.0), (float)getTruePosition(&axis2));
}

// ============================================================================
// Integration Test 14.3: RA Limit Enforcement Test
// ============================================================================

/**
 * Test 14.3: RA Limit Enforcement Test
 * 
 * Verifies that when supervised RA limits are enabled and the mount moves
 * past the limit, motion is stopped and error flags are set.
 * 
 * Validates: Requirements 4.1-4.3
 */
void test_ra_limit_enforcement_west() {
    resetAllState();
    
    // Enable supervised RA limits
    supervisedRaLimitEnabled = true;
    nvStorage.raLimitEnable = SUPERVISED_ENABLED;
    
    // Set RA limits
    raLimitEast = 90;
    raLimitWest = 90;
    
    // Set mount position within limits
    axis1.motorPosition = degToRad(80.0);  // 80 degrees west - within limit
    axis1.absoluteIndex = 0.0;
    
    // Check limits - should be OK
    checkSupervisedRaLimits();
    TEST_ASSERT_FALSE_MESSAGE(westLimitError, "Should not trigger west limit at 80 deg");
    TEST_ASSERT_FALSE_MESSAGE(forwardMotionStopped, "Forward motion should not be stopped");
    
    // Move past west limit
    axis1.motorPosition = degToRad(95.0);  // 95 degrees west - past limit
    
    // Check limits - should trigger west limit
    checkSupervisedRaLimits();
    TEST_ASSERT_TRUE_MESSAGE(westLimitError, "Should trigger west limit at 95 deg");
    TEST_ASSERT_TRUE_MESSAGE(forwardMotionStopped, "Forward motion should be stopped");
    TEST_ASSERT_FALSE_MESSAGE(eastLimitError, "East limit should not be set");
}

/**
 * Test 14.3 (continued): East Limit Enforcement
 */
void test_ra_limit_enforcement_east() {
    resetAllState();
    
    supervisedRaLimitEnabled = true;
    raLimitEast = 90;
    raLimitWest = 90;
    
    // Set mount position within limits
    axis1.motorPosition = degToRad(-80.0);  // 80 degrees east - within limit
    axis1.absoluteIndex = 0.0;
    
    checkSupervisedRaLimits();
    TEST_ASSERT_FALSE_MESSAGE(eastLimitError, "Should not trigger east limit at -80 deg");
    
    // Move past east limit
    axis1.motorPosition = degToRad(-95.0);  // 95 degrees east - past limit
    
    checkSupervisedRaLimits();
    TEST_ASSERT_TRUE_MESSAGE(eastLimitError, "Should trigger east limit at -95 deg");
    TEST_ASSERT_TRUE_MESSAGE(reverseMotionStopped, "Reverse motion should be stopped");
    TEST_ASSERT_FALSE_MESSAGE(westLimitError, "West limit should not be set");
}

/**
 * Test 14.3 (continued): RA Limits Disabled
 */
void test_ra_limit_enforcement_disabled() {
    resetAllState();
    
    // Disable supervised RA limits
    supervisedRaLimitEnabled = false;
    nvStorage.raLimitEnable = SUPERVISED_DISABLED;
    
    raLimitEast = 90;
    raLimitWest = 90;
    
    // Move way past limits
    axis1.motorPosition = degToRad(180.0);  // Way past west limit
    
    checkSupervisedRaLimits();
    
    // Should not trigger any errors when disabled
    TEST_ASSERT_FALSE_MESSAGE(westLimitError, "West limit should not trigger when disabled");
    TEST_ASSERT_FALSE_MESSAGE(eastLimitError, "East limit should not trigger when disabled");
    TEST_ASSERT_FALSE_MESSAGE(forwardMotionStopped, "Motion should not be stopped when disabled");
}

/**
 * Test 14.3 (continued): Non-GEM Mount
 */
void test_ra_limit_enforcement_non_gem() {
    resetAllState();
    
    supervisedRaLimitEnabled = true;
    MOUNT_SUBTYPE = FORK;  // Non-GEM mount
    
    raLimitEast = 90;
    raLimitWest = 90;
    
    // Move past limits
    axis1.motorPosition = degToRad(180.0);
    
    checkSupervisedRaLimits();
    
    // Should not trigger for non-GEM mounts
    TEST_ASSERT_FALSE_MESSAGE(westLimitError, "RA limits should not apply to non-GEM mounts");
    TEST_ASSERT_FALSE_MESSAGE(eastLimitError, "RA limits should not apply to non-GEM mounts");
}

/**
 * Test 14.3 (continued): Southern Hemisphere Limit Swap
 */
void test_ra_limit_enforcement_southern_hemisphere() {
    resetAllState();
    
    supervisedRaLimitEnabled = true;
    siteLatitude = -45.0;  // Southern hemisphere
    
    // Set asymmetric limits to detect swap
    raLimitEast = 60;   // Smaller east limit
    raLimitWest = 120;  // Larger west limit
    
    // In southern hemisphere, limits are swapped
    // So effective east limit is 120, effective west limit is 60
    
    // Position at 70 degrees west - should trigger west limit in south
    // (because effective west limit is 60)
    axis1.motorPosition = degToRad(70.0);
    
    checkSupervisedRaLimits();
    TEST_ASSERT_TRUE_MESSAGE(westLimitError, 
        "Southern hemisphere should swap limits - 70 deg should exceed swapped west limit of 60");
    
    // Reset and test east side
    eastLimitError = false;
    westLimitError = false;
    forwardMotionStopped = false;
    reverseMotionStopped = false;
    
    // Position at -70 degrees east - should NOT trigger east limit in south
    // (because effective east limit is 120)
    axis1.motorPosition = degToRad(-70.0);
    
    checkSupervisedRaLimits();
    TEST_ASSERT_FALSE_MESSAGE(eastLimitError, 
        "Southern hemisphere: -70 deg should be within swapped east limit of 120");
}

/**
 * Test 14.3 (continued): Boundary Conditions
 */
void test_ra_limit_boundary_conditions() {
    resetAllState();
    
    supervisedRaLimitEnabled = true;
    raLimitEast = 90;
    raLimitWest = 90;
    
    // Test exactly at west limit (should NOT trigger)
    axis1.motorPosition = degToRad(90.0);
    checkSupervisedRaLimits();
    TEST_ASSERT_FALSE_MESSAGE(westLimitError, "Exactly at west limit should not trigger");
    
    // Reset flags
    westLimitError = false;
    forwardMotionStopped = false;
    
    // Test just past west limit (should trigger)
    axis1.motorPosition = degToRad(90.001);
    checkSupervisedRaLimits();
    TEST_ASSERT_TRUE_MESSAGE(westLimitError, "Just past west limit should trigger");
    
    // Reset and test east boundary
    eastLimitError = false;
    westLimitError = false;
    reverseMotionStopped = false;
    
    // Test exactly at east limit (should NOT trigger)
    axis1.motorPosition = degToRad(-90.0);
    checkSupervisedRaLimits();
    TEST_ASSERT_FALSE_MESSAGE(eastLimitError, "Exactly at east limit should not trigger");
    
    // Test just past east limit (should trigger)
    axis1.motorPosition = degToRad(-90.001);
    checkSupervisedRaLimits();
    TEST_ASSERT_TRUE_MESSAGE(eastLimitError, "Just past east limit should trigger");
}

// ============================================================================
// Test Setup and Teardown
// ============================================================================

void setUp(void) {
    resetAllState();
}

void tearDown(void) {
    // Nothing to clean up
}

// ============================================================================
// Main Test Runner
// ============================================================================

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Integration Test 14.1: Supervised Home Cycle
    RUN_TEST(test_supervised_home_cycle);
    RUN_TEST(test_supervised_home_disabled);
    RUN_TEST(test_supervised_home_multiple_cycles);
    
    // Integration Test 14.2: Power Cycle Simulation
    RUN_TEST(test_power_cycle_simulation);
    RUN_TEST(test_power_cycle_mount_type_mismatch);
    RUN_TEST(test_power_cycle_memory_disabled);
    RUN_TEST(test_power_cycle_extreme_positions);
    
    // Integration Test 14.3: RA Limit Enforcement
    RUN_TEST(test_ra_limit_enforcement_west);
    RUN_TEST(test_ra_limit_enforcement_east);
    RUN_TEST(test_ra_limit_enforcement_disabled);
    RUN_TEST(test_ra_limit_enforcement_non_gem);
    RUN_TEST(test_ra_limit_enforcement_southern_hemisphere);
    RUN_TEST(test_ra_limit_boundary_conditions);
    
    return UNITY_END();
}
