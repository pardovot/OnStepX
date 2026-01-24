/**
 * Property-Based Tests for Supervised GOTO Synchronization
 * 
 * Feature: supervised-features
 * Property 10: GOTO Sync Decision
 * Property 12: Angular Distance Calculation
 * 
 * Property 10: For any GOTO target more than 10 degrees from the previous target,
 * if the drift between true and virtual positions exceeds the sync threshold,
 * synchronization SHALL occur. If the target is within 10 degrees of the previous
 * target, synchronization SHALL be skipped.
 * Validates: Requirements 5.2, 5.3
 * 
 * Property 12: For any two coordinate positions, the calculated angular distance
 * SHALL be non-negative and SHALL equal zero if and only if the positions are identical.
 * Validates: Requirements 5.7, 9.2
 */

#include <unity.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>

// Minimum iterations for property-based tests
#define PBT_ITERATIONS 100

// Constants from design
#define SUPERVISED_CENTERING_TOLERANCE  10.0  // degrees
#define SUPERVISED_SYNC_THRESHOLD_MIN   5     // degrees
#define SUPERVISED_SYNC_THRESHOLD_MAX   30    // degrees
#define SUPERVISED_SYNC_THRESHOLD_DEFAULT 15  // degrees

// Tolerance for floating point comparison
#define FLOAT_TOLERANCE 0.0001f
#define DOUBLE_TOLERANCE 0.000001

// Helper: Convert degrees to radians
double degToRadTest(double deg) {
    return deg * M_PI / 180.0;
}

// Helper: Convert radians to degrees
double radToDegTest(double rad) {
    return rad * 180.0 / M_PI;
}

// Simulated coordinate structure (mirrors OnStepX Coordinate)
typedef struct {
    double h;   // Hour angle (radians)
    double d;   // Declination (radians)
    double r;   // Right ascension (radians)
    double a;   // Altitude (radians)
    double z;   // Azimuth (radians)
} SimCoordinate;

// Simulated axis state
static double axis1TruePosition = 0.0;      // True position (motor + absoluteIndex)
static double axis2TruePosition = 0.0;
static double axis1InstrumentCoord = 0.0;   // Virtual position (instrument coordinate)
static double axis2InstrumentCoord = 0.0;

// Simulated supervised settings
static uint8_t syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
static bool gotoEnabled = true;

// Last GOTO target for centering detection
static SimCoordinate lastGotoTarget;
static bool lastTargetValid = false;

// Track if sync occurred
static bool syncOccurred = false;

// Mount type simulation (0 = equatorial, 1 = altazm)
static int mountType = 0;  // 0 = equatorial (GEM/FORK)

/**
 * Simulated getDistanceBetweenTruePosAndVirtualPos()
 * Calculates angular distance between true and virtual positions
 */
float getDistanceBetweenTruePosAndVirtualPos() {
    // Get true and virtual positions
    double trueAxis1 = axis1TruePosition;
    double trueAxis2 = axis2TruePosition;
    double virtualAxis1 = axis1InstrumentCoord;
    double virtualAxis2 = axis2InstrumentCoord;
    
    // Calculate deltas
    double delta1 = trueAxis1 - virtualAxis1;
    double delta2 = trueAxis2 - virtualAxis2;
    
    // For equatorial mounts, use spherical geometry approximation
    // Using simplified formula: sqrt(delta1^2 * cos^2(dec) + delta2^2)
    double avgDec = (trueAxis2 + virtualAxis2) / 2.0;
    double cosDec = cos(avgDec);
    double distance = sqrt((delta1 * delta1 * cosDec * cosDec) + (delta2 * delta2));
    
    // Ensure non-negative result
    return (float)fabs(distance);
}

/**
 * Calculate angular distance between two coordinates
 * Uses spherical geometry for proper angular separation
 */
double calculateAngularDistance(SimCoordinate* coord1, SimCoordinate* coord2) {
    double deltaH = coord1->h - coord2->h;
    double deltaD = coord1->d - coord2->d;
    
    // Use average declination for proper angular distance calculation
    double avgDec = (coord1->d + coord2->d) / 2.0;
    double cosDec = cos(avgDec);
    double distance = sqrt((deltaH * deltaH * cosDec * cosDec) + (deltaD * deltaD));
    
    return fabs(distance);
}

/**
 * Simulated syncTruePosToVirtualPos()
 * Mirrors the Supervised class implementation
 */
void syncTruePosToVirtualPos(SimCoordinate* gotoTarget) {
    syncOccurred = false;
    
    // Check if supervised GOTO is enabled
    if (!gotoEnabled) return;
    
    // Centering detection: check if target is within 10 degrees of last target
    if (lastTargetValid) {
        double targetDistance = calculateAngularDistance(gotoTarget, &lastGotoTarget);
        double centeringToleranceRad = degToRadTest(SUPERVISED_CENTERING_TOLERANCE);
        
        if (targetDistance < centeringToleranceRad) {
            // Target is within centering tolerance - skip sync
            lastGotoTarget = *gotoTarget;
            return;
        }
    }
    
    // Calculate drift distance between true and virtual positions
    float driftDistance = getDistanceBetweenTruePosAndVirtualPos();
    
    // Convert sync threshold from degrees to radians
    double syncThresholdRad = degToRadTest((double)syncThreshold);
    
    // Check if drift exceeds threshold
    if (driftDistance > syncThresholdRad) {
        // Synchronize instrument coordinates to true position
        axis1InstrumentCoord = axis1TruePosition;
        axis2InstrumentCoord = axis2TruePosition;
        syncOccurred = true;
    }
    
    // Update last target
    lastGotoTarget = *gotoTarget;
    lastTargetValid = true;
}

/**
 * Reset all simulated state
 */
void resetState() {
    axis1TruePosition = 0.0;
    axis2TruePosition = 0.0;
    axis1InstrumentCoord = 0.0;
    axis2InstrumentCoord = 0.0;
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    gotoEnabled = true;
    lastTargetValid = false;
    lastGotoTarget.h = 0.0;
    lastGotoTarget.d = 0.0;
    syncOccurred = false;
    mountType = 0;
}

/**
 * Generate a random double in range [min, max]
 */
double generateRandomDouble(double min, double max) {
    return min + ((double)rand() / RAND_MAX) * (max - min);
}

/**
 * Generate a random coordinate within valid ranges
 * Hour angle: -12h to +12h (-PI to +PI radians)
 * Declination: -90 to +90 degrees (-PI/2 to +PI/2 radians)
 */
void generateRandomCoordinate(SimCoordinate* coord) {
    coord->h = generateRandomDouble(-M_PI, M_PI);
    coord->d = generateRandomDouble(-M_PI/2.0, M_PI/2.0);
    coord->r = 0.0;
    coord->a = 0.0;
    coord->z = 0.0;
}

/**
 * Property 12: Angular Distance Calculation
 * 
 * For any two coordinate positions, the calculated angular distance
 * SHALL be non-negative and SHALL equal zero if and only if the positions
 * are identical.
 * 
 * Feature: supervised-features, Property 12: Angular Distance Calculation
 * Validates: Requirements 5.7, 9.2
 */
void test_property_angular_distance_non_negative() {
    srand((unsigned int)time(NULL));
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetState();
        
        // Generate two random coordinates
        SimCoordinate coord1, coord2;
        generateRandomCoordinate(&coord1);
        generateRandomCoordinate(&coord2);
        
        // Calculate angular distance
        double distance = calculateAngularDistance(&coord1, &coord2);
        
        // Property: distance must be non-negative
        TEST_ASSERT_TRUE_MESSAGE(distance >= 0.0, 
            "Angular distance must be non-negative");
    }
}

/**
 * Property 12 (continued): Angular Distance Zero for Identical Positions
 */
void test_property_angular_distance_zero_for_identical() {
    srand((unsigned int)time(NULL) + 1);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetState();
        
        // Generate a random coordinate
        SimCoordinate coord1;
        generateRandomCoordinate(&coord1);
        
        // Create identical coordinate
        SimCoordinate coord2 = coord1;
        
        // Calculate angular distance
        double distance = calculateAngularDistance(&coord1, &coord2);
        
        // Property: distance must be zero for identical positions
        TEST_ASSERT_FLOAT_WITHIN_MESSAGE(DOUBLE_TOLERANCE, 0.0, distance,
            "Angular distance must be zero for identical positions");
    }
}

/**
 * Property 12 (continued): Angular Distance Positive for Different Positions
 */
void test_property_angular_distance_positive_for_different() {
    srand((unsigned int)time(NULL) + 2);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetState();
        
        // Generate two different coordinates
        SimCoordinate coord1, coord2;
        generateRandomCoordinate(&coord1);
        
        // Ensure coord2 is different by adding a small offset
        coord2.h = coord1.h + degToRadTest(1.0);  // Add 1 degree
        coord2.d = coord1.d + degToRadTest(1.0);
        
        // Calculate angular distance
        double distance = calculateAngularDistance(&coord1, &coord2);
        
        // Property: distance must be positive for different positions
        TEST_ASSERT_TRUE_MESSAGE(distance > 0.0,
            "Angular distance must be positive for different positions");
    }
}

/**
 * Property 10: GOTO Sync Decision - Sync Occurs When Drift Exceeds Threshold
 * 
 * For any GOTO target more than 10 degrees from the previous target,
 * if the drift between true and virtual positions exceeds the sync threshold,
 * synchronization SHALL occur.
 * 
 * Feature: supervised-features, Property 10: GOTO Sync Decision
 * Validates: Requirements 5.2, 5.3
 */
void test_property_goto_sync_when_drift_exceeds_threshold() {
    srand((unsigned int)time(NULL) + 3);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetState();
        
        // Generate random sync threshold (5-30 degrees)
        syncThreshold = (uint8_t)(SUPERVISED_SYNC_THRESHOLD_MIN + 
            rand() % (SUPERVISED_SYNC_THRESHOLD_MAX - SUPERVISED_SYNC_THRESHOLD_MIN + 1));
        double thresholdRad = degToRadTest((double)syncThreshold);
        
        // Create drift that exceeds threshold
        double driftAmount = thresholdRad + degToRadTest(5.0);  // threshold + 5 degrees
        axis1TruePosition = driftAmount;
        axis1InstrumentCoord = 0.0;
        axis2TruePosition = 0.0;
        axis2InstrumentCoord = 0.0;
        
        // Set up a previous target
        lastGotoTarget.h = 0.0;
        lastGotoTarget.d = 0.0;
        lastTargetValid = true;
        
        // Create a new target more than 10 degrees away from previous
        SimCoordinate newTarget;
        newTarget.h = degToRadTest(20.0);  // 20 degrees away
        newTarget.d = 0.0;
        
        // Perform sync check
        syncTruePosToVirtualPos(&newTarget);
        
        // Property: sync should occur when drift exceeds threshold
        TEST_ASSERT_TRUE_MESSAGE(syncOccurred,
            "Sync should occur when drift exceeds threshold and target is far from previous");
        
        // Verify instrument coordinates were updated to true position
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, (float)axis1TruePosition, (float)axis1InstrumentCoord);
    }
}

/**
 * Property 10 (continued): No Sync When Drift Below Threshold
 */
void test_property_goto_no_sync_when_drift_below_threshold() {
    srand((unsigned int)time(NULL) + 4);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetState();
        
        // Generate random sync threshold (5-30 degrees)
        syncThreshold = (uint8_t)(SUPERVISED_SYNC_THRESHOLD_MIN + 
            rand() % (SUPERVISED_SYNC_THRESHOLD_MAX - SUPERVISED_SYNC_THRESHOLD_MIN + 1));
        double thresholdRad = degToRadTest((double)syncThreshold);
        
        // Create drift that is below threshold
        double driftAmount = thresholdRad * 0.5;  // Half of threshold
        axis1TruePosition = driftAmount;
        axis1InstrumentCoord = 0.0;
        axis2TruePosition = 0.0;
        axis2InstrumentCoord = 0.0;
        
        // Store original instrument coordinate
        double originalInstrumentCoord = axis1InstrumentCoord;
        
        // Set up a previous target
        lastGotoTarget.h = 0.0;
        lastGotoTarget.d = 0.0;
        lastTargetValid = true;
        
        // Create a new target more than 10 degrees away from previous
        SimCoordinate newTarget;
        newTarget.h = degToRadTest(20.0);  // 20 degrees away
        newTarget.d = 0.0;
        
        // Perform sync check
        syncTruePosToVirtualPos(&newTarget);
        
        // Property: sync should NOT occur when drift is below threshold
        TEST_ASSERT_FALSE_MESSAGE(syncOccurred,
            "Sync should NOT occur when drift is below threshold");
        
        // Verify instrument coordinates were NOT changed
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, (float)originalInstrumentCoord, (float)axis1InstrumentCoord);
    }
}

/**
 * Property 10 (continued): Centering Detection - Skip Sync for Nearby Targets
 * 
 * If the target is within 10 degrees of the previous target,
 * synchronization SHALL be skipped (centering detection).
 */
void test_property_goto_centering_detection_skips_sync() {
    srand((unsigned int)time(NULL) + 5);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetState();
        
        // Set threshold to minimum so drift would normally trigger sync
        syncThreshold = SUPERVISED_SYNC_THRESHOLD_MIN;
        double thresholdRad = degToRadTest((double)syncThreshold);
        
        // Create drift that exceeds threshold
        double driftAmount = thresholdRad + degToRadTest(10.0);  // Well above threshold
        axis1TruePosition = driftAmount;
        axis1InstrumentCoord = 0.0;
        axis2TruePosition = 0.0;
        axis2InstrumentCoord = 0.0;
        
        // Store original instrument coordinate
        double originalInstrumentCoord = axis1InstrumentCoord;
        
        // Set up a previous target
        lastGotoTarget.h = degToRadTest(5.0);
        lastGotoTarget.d = 0.0;
        lastTargetValid = true;
        
        // Create a new target within 10 degrees of previous (centering)
        // Generate random offset less than 10 degrees
        double offsetDeg = generateRandomDouble(0.1, 9.0);  // 0.1 to 9 degrees
        SimCoordinate newTarget;
        newTarget.h = lastGotoTarget.h + degToRadTest(offsetDeg);
        newTarget.d = lastGotoTarget.d;
        
        // Perform sync check
        syncTruePosToVirtualPos(&newTarget);
        
        // Property: sync should be skipped due to centering detection
        TEST_ASSERT_FALSE_MESSAGE(syncOccurred,
            "Sync should be skipped when target is within centering tolerance of previous target");
        
        // Verify instrument coordinates were NOT changed
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, (float)originalInstrumentCoord, (float)axis1InstrumentCoord);
    }
}

/**
 * Property 10 (continued): First GOTO Has No Previous Target
 * 
 * When there is no previous target (first GOTO), centering detection
 * should not apply, and sync should occur if drift exceeds threshold.
 */
void test_property_goto_first_target_no_centering() {
    srand((unsigned int)time(NULL) + 6);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetState();
        
        // Ensure no previous target
        lastTargetValid = false;
        
        // Set threshold
        syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
        double thresholdRad = degToRadTest((double)syncThreshold);
        
        // Create drift that exceeds threshold
        double driftAmount = thresholdRad + degToRadTest(5.0);
        axis1TruePosition = driftAmount;
        axis1InstrumentCoord = 0.0;
        axis2TruePosition = 0.0;
        axis2InstrumentCoord = 0.0;
        
        // Create any target
        SimCoordinate newTarget;
        generateRandomCoordinate(&newTarget);
        
        // Perform sync check
        syncTruePosToVirtualPos(&newTarget);
        
        // Property: sync should occur for first GOTO when drift exceeds threshold
        TEST_ASSERT_TRUE_MESSAGE(syncOccurred,
            "Sync should occur for first GOTO when drift exceeds threshold");
        
        // Verify last target is now valid
        TEST_ASSERT_TRUE(lastTargetValid);
    }
}

/**
 * Unit test: GOTO sync disabled
 */
void test_goto_sync_disabled() {
    resetState();
    
    // Disable GOTO sync
    gotoEnabled = false;
    
    // Create drift that would normally trigger sync
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_MIN;
    axis1TruePosition = degToRadTest(30.0);  // Large drift
    axis1InstrumentCoord = 0.0;
    
    // Store original
    double originalInstrumentCoord = axis1InstrumentCoord;
    
    // Create target
    SimCoordinate newTarget;
    newTarget.h = degToRadTest(20.0);
    newTarget.d = 0.0;
    
    // Perform sync check
    syncTruePosToVirtualPos(&newTarget);
    
    // Should not sync when disabled
    TEST_ASSERT_FALSE(syncOccurred);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, (float)originalInstrumentCoord, (float)axis1InstrumentCoord);
}

/**
 * Unit test: Angular distance symmetry
 */
void test_angular_distance_symmetry() {
    SimCoordinate coord1, coord2;
    
    coord1.h = degToRadTest(10.0);
    coord1.d = degToRadTest(45.0);
    coord2.h = degToRadTest(20.0);
    coord2.d = degToRadTest(50.0);
    
    double dist1to2 = calculateAngularDistance(&coord1, &coord2);
    double dist2to1 = calculateAngularDistance(&coord2, &coord1);
    
    // Distance should be symmetric
    TEST_ASSERT_FLOAT_WITHIN(DOUBLE_TOLERANCE, dist1to2, dist2to1);
}

/**
 * Unit test: getDistanceBetweenTruePosAndVirtualPos basic functionality
 */
void test_distance_between_true_and_virtual_basic() {
    resetState();
    
    // Set true and virtual positions to same value
    axis1TruePosition = degToRadTest(45.0);
    axis2TruePosition = degToRadTest(30.0);
    axis1InstrumentCoord = degToRadTest(45.0);
    axis2InstrumentCoord = degToRadTest(30.0);
    
    float distance = getDistanceBetweenTruePosAndVirtualPos();
    
    // Distance should be zero when positions are identical
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, distance);
    
    // Now create a drift
    axis1InstrumentCoord = degToRadTest(40.0);  // 5 degree drift in axis1
    
    distance = getDistanceBetweenTruePosAndVirtualPos();
    
    // Distance should be positive
    TEST_ASSERT_TRUE(distance > 0.0f);
}

/**
 * Unit test: Sync threshold boundary - exactly at threshold
 */
void test_sync_threshold_boundary() {
    resetState();
    
    syncThreshold = 15;  // 15 degrees
    double thresholdRad = degToRadTest(15.0);
    
    // Set drift slightly below threshold to ensure no sync
    axis1TruePosition = thresholdRad - degToRadTest(0.01);  // Just below threshold
    axis1InstrumentCoord = 0.0;
    axis2TruePosition = 0.0;
    axis2InstrumentCoord = 0.0;
    
    // Set up previous target far away
    lastGotoTarget.h = 0.0;
    lastGotoTarget.d = 0.0;
    lastTargetValid = true;
    
    SimCoordinate newTarget;
    newTarget.h = degToRadTest(20.0);
    newTarget.d = 0.0;
    
    // Below threshold, should NOT sync
    syncTruePosToVirtualPos(&newTarget);
    TEST_ASSERT_FALSE_MESSAGE(syncOccurred, 
        "Sync should NOT occur when drift is below threshold");
    
    // Now set drift clearly above threshold
    resetState();
    syncThreshold = 15;
    axis1TruePosition = thresholdRad + degToRadTest(1.0);  // 1 degree above
    axis1InstrumentCoord = 0.0;
    lastGotoTarget.h = 0.0;
    lastGotoTarget.d = 0.0;
    lastTargetValid = true;
    
    syncTruePosToVirtualPos(&newTarget);
    TEST_ASSERT_TRUE_MESSAGE(syncOccurred,
        "Sync should occur when drift exceeds threshold");
}

void setUp(void) {
    resetState();
}

void tearDown(void) {
    // Nothing to clean up
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Property 12: Angular Distance Calculation
    RUN_TEST(test_property_angular_distance_non_negative);
    RUN_TEST(test_property_angular_distance_zero_for_identical);
    RUN_TEST(test_property_angular_distance_positive_for_different);
    
    // Property 10: GOTO Sync Decision
    RUN_TEST(test_property_goto_sync_when_drift_exceeds_threshold);
    RUN_TEST(test_property_goto_no_sync_when_drift_below_threshold);
    RUN_TEST(test_property_goto_centering_detection_skips_sync);
    RUN_TEST(test_property_goto_first_target_no_centering);
    
    // Unit tests
    RUN_TEST(test_goto_sync_disabled);
    RUN_TEST(test_angular_distance_symmetry);
    RUN_TEST(test_distance_between_true_and_virtual_basic);
    RUN_TEST(test_sync_threshold_boundary);
    
    return UNITY_END();
}
