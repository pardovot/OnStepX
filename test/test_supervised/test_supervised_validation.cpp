/**
 * Property-Based Tests for Supervised Features Validation
 * 
 * Feature: supervised-features
 * Property 6: RA Limit Validation
 * Property 7: RA Limit Enforcement
 * Property 8: Hemisphere Limit Swap
 * Property 9: Sync Threshold Validation
 * 
 * Property 6: For any RA limit value outside the 1-180 degree range, 
 * the system SHALL reject the value or apply the default of 95 degrees.
 * Validates: Requirements 4.5, 4.6
 * 
 * Property 7: For any true RA position that exceeds the configured limit 
 * (west for forward, east for reverse), the system SHALL stop motion in 
 * that direction and set the appropriate error flag.
 * Validates: Requirements 4.2, 4.3
 * 
 * Property 8: For any site with negative latitude (southern hemisphere), 
 * the east and west RA limit applications SHALL be swapped compared to 
 * northern hemisphere.
 * Validates: Requirements 4.7
 * 
 * Property 9: For any sync threshold value outside the 5-30 degree range,
 * the system SHALL reject the value or apply the default of 15 degrees.
 * Validates: Requirements 5.5, 5.6
 */

#include <unity.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>

// Minimum iterations for property-based tests
#define PBT_ITERATIONS 100

// Validation ranges from design
#define SUPERVISED_RA_LIMIT_MIN         1
#define SUPERVISED_RA_LIMIT_MAX         180
#define SUPERVISED_RA_LIMIT_DEFAULT     95
#define SUPERVISED_SYNC_THRESHOLD_MIN   5
#define SUPERVISED_SYNC_THRESHOLD_MAX   30
#define SUPERVISED_SYNC_THRESHOLD_DEFAULT 15

// Simulated settings storage
static uint8_t raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
static uint8_t raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
static uint8_t syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;

// Getter/setter functions that mirror the Supervised class behavior
uint8_t getRaLimitEast() { return raLimitEast; }
uint8_t getRaLimitWest() { return raLimitWest; }
uint8_t getSyncThreshold() { return syncThreshold; }

void setRaLimitEast(uint8_t degrees) {
    if (degrees >= SUPERVISED_RA_LIMIT_MIN && degrees <= SUPERVISED_RA_LIMIT_MAX) {
        raLimitEast = degrees;
    }
    // Invalid values are rejected (not stored)
}

void setRaLimitWest(uint8_t degrees) {
    if (degrees >= SUPERVISED_RA_LIMIT_MIN && degrees <= SUPERVISED_RA_LIMIT_MAX) {
        raLimitWest = degrees;
    }
    // Invalid values are rejected (not stored)
}

void setSyncThreshold(uint8_t degrees) {
    if (degrees >= SUPERVISED_SYNC_THRESHOLD_MIN && degrees <= SUPERVISED_SYNC_THRESHOLD_MAX) {
        syncThreshold = degrees;
    }
    // Invalid values are rejected (not stored)
}

// Validation function that applies defaults (mirrors validateSettings behavior)
void validateRaLimitEast() {
    if (raLimitEast < SUPERVISED_RA_LIMIT_MIN || raLimitEast > SUPERVISED_RA_LIMIT_MAX) {
        raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    }
}

void validateRaLimitWest() {
    if (raLimitWest < SUPERVISED_RA_LIMIT_MIN || raLimitWest > SUPERVISED_RA_LIMIT_MAX) {
        raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    }
}

void validateSyncThreshold() {
    if (syncThreshold < SUPERVISED_SYNC_THRESHOLD_MIN || syncThreshold > SUPERVISED_SYNC_THRESHOLD_MAX) {
        syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    }
}

// ============================================================================
// RA Limit Enforcement Simulation (Property 7 & 8)
// ============================================================================

// Simulated mount state for RA limit enforcement tests
static double trueRaPosition = 0.0;  // True RA position in radians
static double siteLatitude = 45.0;   // Site latitude in degrees (positive = north)
static bool raLimitEnabled = true;   // Whether supervised RA limits are enabled
static bool isGemMount = true;       // Whether mount is GEM type

// Error flags
static bool eastLimitError = false;
static bool westLimitError = false;

// Motion state
static bool forwardMotionStopped = false;
static bool reverseMotionStopped = false;

// Helper: Convert degrees to radians
double degToRadTest(double deg) {
    return deg * M_PI / 180.0;
}

// Helper: Convert radians to degrees
double radToDegTest(double rad) {
    return rad * 180.0 / M_PI;
}

/**
 * Simulates the checkRaLimits() method from Supervised class
 * Returns: 0 = no violation, 1 = east limit exceeded, 2 = west limit exceeded
 */
int checkRaLimitsSimulated() {
    // Only check for GEM mounts
    if (!isGemMount) return 0;
    
    // Convert limits from degrees to radians
    double eastLimitRad = degToRadTest((double)raLimitEast);
    double westLimitRad = degToRadTest((double)raLimitWest);
    
    // Handle hemisphere swap for southern latitudes
    bool southernHemisphere = siteLatitude < 0;
    
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
        return 1; // East limit exceeded
    }
    
    if (trueRaPosition > effectiveWestLimit) {
        return 2; // West limit exceeded
    }
    
    return 0; // No violation
}

/**
 * Simulates the checkSupervisedRaLimits() method from Limits class
 */
void checkSupervisedRaLimitsSimulated() {
    // Only check if supervised RA limits are enabled
    if (!raLimitEnabled) {
        eastLimitError = false;
        westLimitError = false;
        return;
    }
    
    // Only check for GEM mounts
    if (!isGemMount) {
        eastLimitError = false;
        westLimitError = false;
        return;
    }
    
    bool lastEastError = eastLimitError;
    bool lastWestError = westLimitError;
    
    // Call the check function
    int limitResult = checkRaLimitsSimulated();
    
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
 * Reset RA limit enforcement state
 */
void resetRaLimitState() {
    trueRaPosition = 0.0;
    siteLatitude = 45.0;
    raLimitEnabled = true;
    isGemMount = true;
    eastLimitError = false;
    westLimitError = false;
    forwardMotionStopped = false;
    reverseMotionStopped = false;
    raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
}

/**
 * Generate a random double in range [min, max]
 */
double generateRandomDouble(double min, double max) {
    return min + ((double)rand() / RAND_MAX) * (max - min);
}

/**
 * Generate a random uint8_t value
 */
uint8_t generateRandomUint8() {
    return (uint8_t)(rand() % 256);
}

/**
 * Property 6: RA Limit Validation
 * 
 * For any RA limit value outside the 1-180 degree range,
 * the system SHALL reject the value or apply the default of 95 degrees.
 * 
 * Feature: supervised-features, Property 6: RA Limit Validation
 * Validates: Requirements 4.5, 4.6
 */
void test_property_ra_limit_validation() {
    srand((unsigned int)time(NULL));
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Generate random test value
        uint8_t testValue = generateRandomUint8();
        
        // Store original value to check rejection
        uint8_t originalEast = raLimitEast;
        uint8_t originalWest = raLimitWest;
        
        // Attempt to set the value
        setRaLimitEast(testValue);
        setRaLimitWest(testValue);
        
        // Check the property
        if (testValue >= SUPERVISED_RA_LIMIT_MIN && testValue <= SUPERVISED_RA_LIMIT_MAX) {
            // Valid value should be accepted
            TEST_ASSERT_EQUAL_UINT8(testValue, getRaLimitEast());
            TEST_ASSERT_EQUAL_UINT8(testValue, getRaLimitWest());
        } else {
            // Invalid value should be rejected (original value preserved)
            TEST_ASSERT_EQUAL_UINT8(originalEast, getRaLimitEast());
            TEST_ASSERT_EQUAL_UINT8(originalWest, getRaLimitWest());
        }
        
        // Reset for next iteration
        raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
        raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    }
}

/**
 * Property 6 (continued): RA Limit Validation with defaults
 * 
 * Test that validation applies defaults for invalid stored values.
 */
void test_property_ra_limit_validation_defaults() {
    srand((unsigned int)time(NULL) + 1);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Generate random test value
        uint8_t testValue = generateRandomUint8();
        
        // Directly set the storage (simulating corrupted NV data)
        raLimitEast = testValue;
        raLimitWest = testValue;
        
        // Run validation
        validateRaLimitEast();
        validateRaLimitWest();
        
        // Check the property
        if (testValue >= SUPERVISED_RA_LIMIT_MIN && testValue <= SUPERVISED_RA_LIMIT_MAX) {
            // Valid value should be preserved
            TEST_ASSERT_EQUAL_UINT8(testValue, raLimitEast);
            TEST_ASSERT_EQUAL_UINT8(testValue, raLimitWest);
        } else {
            // Invalid value should be replaced with default
            TEST_ASSERT_EQUAL_UINT8(SUPERVISED_RA_LIMIT_DEFAULT, raLimitEast);
            TEST_ASSERT_EQUAL_UINT8(SUPERVISED_RA_LIMIT_DEFAULT, raLimitWest);
        }
    }
}

/**
 * Property 9: Sync Threshold Validation
 * 
 * For any sync threshold value outside the 5-30 degree range,
 * the system SHALL reject the value or apply the default of 15 degrees.
 * 
 * Feature: supervised-features, Property 9: Sync Threshold Validation
 * Validates: Requirements 5.5, 5.6
 */
void test_property_sync_threshold_validation() {
    srand((unsigned int)time(NULL) + 2);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Generate random test value
        uint8_t testValue = generateRandomUint8();
        
        // Store original value to check rejection
        uint8_t original = syncThreshold;
        
        // Attempt to set the value
        setSyncThreshold(testValue);
        
        // Check the property
        if (testValue >= SUPERVISED_SYNC_THRESHOLD_MIN && testValue <= SUPERVISED_SYNC_THRESHOLD_MAX) {
            // Valid value should be accepted
            TEST_ASSERT_EQUAL_UINT8(testValue, getSyncThreshold());
        } else {
            // Invalid value should be rejected (original value preserved)
            TEST_ASSERT_EQUAL_UINT8(original, getSyncThreshold());
        }
        
        // Reset for next iteration
        syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    }
}

/**
 * Property 9 (continued): Sync Threshold Validation with defaults
 * 
 * Test that validation applies defaults for invalid stored values.
 */
void test_property_sync_threshold_validation_defaults() {
    srand((unsigned int)time(NULL) + 3);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Generate random test value
        uint8_t testValue = generateRandomUint8();
        
        // Directly set the storage (simulating corrupted NV data)
        syncThreshold = testValue;
        
        // Run validation
        validateSyncThreshold();
        
        // Check the property
        if (testValue >= SUPERVISED_SYNC_THRESHOLD_MIN && testValue <= SUPERVISED_SYNC_THRESHOLD_MAX) {
            // Valid value should be preserved
            TEST_ASSERT_EQUAL_UINT8(testValue, syncThreshold);
        } else {
            // Invalid value should be replaced with default
            TEST_ASSERT_EQUAL_UINT8(SUPERVISED_SYNC_THRESHOLD_DEFAULT, syncThreshold);
        }
    }
}

/**
 * Unit test: RA limit boundary values
 */
void test_ra_limit_boundary_values() {
    // Reset to defaults
    raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    
    // Test minimum valid value (1)
    setRaLimitEast(1);
    TEST_ASSERT_EQUAL_UINT8(1, getRaLimitEast());
    
    // Test maximum valid value (180)
    setRaLimitEast(180);
    TEST_ASSERT_EQUAL_UINT8(180, getRaLimitEast());
    
    // Test just below minimum (0) - should be rejected
    raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    setRaLimitEast(0);
    TEST_ASSERT_EQUAL_UINT8(SUPERVISED_RA_LIMIT_DEFAULT, getRaLimitEast());
    
    // Test just above maximum (181) - should be rejected
    raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    setRaLimitEast(181);
    TEST_ASSERT_EQUAL_UINT8(SUPERVISED_RA_LIMIT_DEFAULT, getRaLimitEast());
    
    // Test max uint8_t (255) - should be rejected
    raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    setRaLimitEast(255);
    TEST_ASSERT_EQUAL_UINT8(SUPERVISED_RA_LIMIT_DEFAULT, getRaLimitEast());
}

/**
 * Unit test: Sync threshold boundary values
 */
void test_sync_threshold_boundary_values() {
    // Reset to default
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    
    // Test minimum valid value (5)
    setSyncThreshold(5);
    TEST_ASSERT_EQUAL_UINT8(5, getSyncThreshold());
    
    // Test maximum valid value (30)
    setSyncThreshold(30);
    TEST_ASSERT_EQUAL_UINT8(30, getSyncThreshold());
    
    // Test just below minimum (4) - should be rejected
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    setSyncThreshold(4);
    TEST_ASSERT_EQUAL_UINT8(SUPERVISED_SYNC_THRESHOLD_DEFAULT, getSyncThreshold());
    
    // Test just above maximum (31) - should be rejected
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    setSyncThreshold(31);
    TEST_ASSERT_EQUAL_UINT8(SUPERVISED_SYNC_THRESHOLD_DEFAULT, getSyncThreshold());
    
    // Test zero - should be rejected
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    setSyncThreshold(0);
    TEST_ASSERT_EQUAL_UINT8(SUPERVISED_SYNC_THRESHOLD_DEFAULT, getSyncThreshold());
    
    // Test max uint8_t (255) - should be rejected
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    setSyncThreshold(255);
    TEST_ASSERT_EQUAL_UINT8(SUPERVISED_SYNC_THRESHOLD_DEFAULT, getSyncThreshold());
}

/**
 * Property 7: RA Limit Enforcement
 * 
 * For any true RA position that exceeds the configured limit (west for forward, 
 * east for reverse), the system SHALL stop motion in that direction and set 
 * the appropriate error flag.
 * 
 * Feature: supervised-features, Property 7: RA Limit Enforcement
 * Validates: Requirements 4.2, 4.3
 */
void test_property_ra_limit_enforcement() {
    srand((unsigned int)time(NULL) + 4);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        resetRaLimitState();
        
        // Generate random RA limits (valid range 1-180 degrees)
        uint8_t eastLimit = (uint8_t)(1 + rand() % 180);
        uint8_t westLimit = (uint8_t)(1 + rand() % 180);
        raLimitEast = eastLimit;
        raLimitWest = westLimit;
        
        // Generate random true RA position in range [-200, 200] degrees
        double positionDeg = generateRandomDouble(-200.0, 200.0);
        trueRaPosition = degToRadTest(positionDeg);
        
        // Northern hemisphere for this test
        siteLatitude = 45.0;
        
        // Check limits
        checkSupervisedRaLimitsSimulated();
        
        // Verify the property
        if (positionDeg < -(double)eastLimit) {
            // East limit exceeded - should set east error and stop reverse motion
            TEST_ASSERT_TRUE_MESSAGE(eastLimitError, "East limit error should be set when position exceeds east limit");
            TEST_ASSERT_FALSE_MESSAGE(westLimitError, "West limit error should not be set when east limit exceeded");
            TEST_ASSERT_TRUE_MESSAGE(reverseMotionStopped, "Reverse motion should be stopped when east limit exceeded");
        } else if (positionDeg > (double)westLimit) {
            // West limit exceeded - should set west error and stop forward motion
            TEST_ASSERT_FALSE_MESSAGE(eastLimitError, "East limit error should not be set when west limit exceeded");
            TEST_ASSERT_TRUE_MESSAGE(westLimitError, "West limit error should be set when position exceeds west limit");
            TEST_ASSERT_TRUE_MESSAGE(forwardMotionStopped, "Forward motion should be stopped when west limit exceeded");
        } else {
            // Within limits - no errors
            TEST_ASSERT_FALSE_MESSAGE(eastLimitError, "East limit error should not be set when within limits");
            TEST_ASSERT_FALSE_MESSAGE(westLimitError, "West limit error should not be set when within limits");
        }
    }
}

/**
 * Property 8: Hemisphere Limit Swap
 * 
 * For any site with negative latitude (southern hemisphere), the east and west 
 * RA limit applications SHALL be swapped compared to northern hemisphere.
 * 
 * Feature: supervised-features, Property 8: Hemisphere Limit Swap
 * Validates: Requirements 4.7
 */
void test_property_hemisphere_limit_swap() {
    srand((unsigned int)time(NULL) + 5);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Generate random but different east and west limits to make swap detectable
        uint8_t eastLimit = (uint8_t)(1 + rand() % 90);   // 1-90 degrees
        uint8_t westLimit = (uint8_t)(91 + rand() % 90);  // 91-180 degrees
        
        // Generate a position that would exceed east limit in north but not west
        // Position between -westLimit and -eastLimit (in degrees)
        double positionDeg = generateRandomDouble(-(double)westLimit + 1, -(double)eastLimit - 1);
        
        // Test in northern hemisphere
        resetRaLimitState();
        raLimitEast = eastLimit;
        raLimitWest = westLimit;
        siteLatitude = 45.0;  // Northern hemisphere
        trueRaPosition = degToRadTest(positionDeg);
        
        checkSupervisedRaLimitsSimulated();
        bool northEastError = eastLimitError;
        bool northWestError = westLimitError;
        
        // Test in southern hemisphere with same position
        resetRaLimitState();
        raLimitEast = eastLimit;
        raLimitWest = westLimit;
        siteLatitude = -45.0;  // Southern hemisphere
        trueRaPosition = degToRadTest(positionDeg);
        
        checkSupervisedRaLimitsSimulated();
        bool southEastError = eastLimitError;
        bool southWestError = westLimitError;
        
        // In southern hemisphere, limits are swapped
        // So a position that exceeds east limit in north should exceed west limit in south
        // (because the effective east limit in south is the configured west limit)
        
        // The position is between -westLimit and -eastLimit
        // In north: effective east limit is eastLimit, so position < -eastLimit triggers east error
        // In south: effective east limit is westLimit, so position < -westLimit triggers east error
        
        // Since position is > -westLimit (closer to 0), it won't trigger east error in south
        // But since position is < -eastLimit, it will trigger east error in north
        
        if (positionDeg < -(double)eastLimit) {
            // Should trigger east error in north
            TEST_ASSERT_TRUE_MESSAGE(northEastError, "North: East limit should be exceeded");
        }
        
        if (positionDeg < -(double)westLimit) {
            // Should trigger east error in south (swapped)
            TEST_ASSERT_TRUE_MESSAGE(southEastError, "South: East limit (swapped) should be exceeded");
        } else if (positionDeg >= -(double)westLimit && positionDeg < -(double)eastLimit) {
            // Position is between -westLimit and -eastLimit
            // In south, effective east limit is westLimit, so no east error
            TEST_ASSERT_FALSE_MESSAGE(southEastError, "South: Position within swapped limits should not trigger east error");
        }
    }
}

/**
 * Unit test: RA limit enforcement disabled
 */
void test_ra_limit_enforcement_disabled() {
    resetRaLimitState();
    
    // Disable RA limits
    raLimitEnabled = false;
    
    // Set position way beyond limits
    trueRaPosition = degToRadTest(200.0);  // Way past any limit
    
    // Check limits
    checkSupervisedRaLimitsSimulated();
    
    // Should not set any errors when disabled
    TEST_ASSERT_FALSE(eastLimitError);
    TEST_ASSERT_FALSE(westLimitError);
    TEST_ASSERT_FALSE(forwardMotionStopped);
    TEST_ASSERT_FALSE(reverseMotionStopped);
}

/**
 * Unit test: RA limit enforcement non-GEM mount
 */
void test_ra_limit_enforcement_non_gem() {
    resetRaLimitState();
    
    // Set to non-GEM mount
    isGemMount = false;
    
    // Set position way beyond limits
    trueRaPosition = degToRadTest(200.0);  // Way past any limit
    
    // Check limits
    checkSupervisedRaLimitsSimulated();
    
    // Should not set any errors for non-GEM mounts
    TEST_ASSERT_FALSE(eastLimitError);
    TEST_ASSERT_FALSE(westLimitError);
    TEST_ASSERT_FALSE(forwardMotionStopped);
    TEST_ASSERT_FALSE(reverseMotionStopped);
}

/**
 * Unit test: RA limit boundary conditions
 */
void test_ra_limit_boundary_conditions() {
    resetRaLimitState();
    
    // Set specific limits
    raLimitEast = 90;
    raLimitWest = 90;
    
    // Test exactly at east limit (should not trigger)
    trueRaPosition = degToRadTest(-90.0);
    checkSupervisedRaLimitsSimulated();
    TEST_ASSERT_FALSE_MESSAGE(eastLimitError, "Position exactly at east limit should not trigger error");
    
    // Test just past east limit (should trigger)
    resetRaLimitState();
    raLimitEast = 90;
    raLimitWest = 90;
    trueRaPosition = degToRadTest(-90.001);
    checkSupervisedRaLimitsSimulated();
    TEST_ASSERT_TRUE_MESSAGE(eastLimitError, "Position just past east limit should trigger error");
    
    // Test exactly at west limit (should not trigger)
    resetRaLimitState();
    raLimitEast = 90;
    raLimitWest = 90;
    trueRaPosition = degToRadTest(90.0);
    checkSupervisedRaLimitsSimulated();
    TEST_ASSERT_FALSE_MESSAGE(westLimitError, "Position exactly at west limit should not trigger error");
    
    // Test just past west limit (should trigger)
    resetRaLimitState();
    raLimitEast = 90;
    raLimitWest = 90;
    trueRaPosition = degToRadTest(90.001);
    checkSupervisedRaLimitsSimulated();
    TEST_ASSERT_TRUE_MESSAGE(westLimitError, "Position just past west limit should trigger error");
}

void setUp(void) {
    // Reset all values to defaults before each test
    raLimitEast = SUPERVISED_RA_LIMIT_DEFAULT;
    raLimitWest = SUPERVISED_RA_LIMIT_DEFAULT;
    syncThreshold = SUPERVISED_SYNC_THRESHOLD_DEFAULT;
    
    // Reset RA limit enforcement state
    resetRaLimitState();
}

void tearDown(void) {
    // Nothing to clean up
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Property-based tests for validation (Property 6 & 9)
    RUN_TEST(test_property_ra_limit_validation);
    RUN_TEST(test_property_ra_limit_validation_defaults);
    RUN_TEST(test_property_sync_threshold_validation);
    RUN_TEST(test_property_sync_threshold_validation_defaults);
    
    // Property-based tests for RA limit enforcement (Property 7 & 8)
    RUN_TEST(test_property_ra_limit_enforcement);
    RUN_TEST(test_property_hemisphere_limit_swap);
    
    // Unit tests for boundary values
    RUN_TEST(test_ra_limit_boundary_values);
    RUN_TEST(test_sync_threshold_boundary_values);
    
    // Unit tests for RA limit enforcement
    RUN_TEST(test_ra_limit_enforcement_disabled);
    RUN_TEST(test_ra_limit_enforcement_non_gem);
    RUN_TEST(test_ra_limit_boundary_conditions);
    
    return UNITY_END();
}
