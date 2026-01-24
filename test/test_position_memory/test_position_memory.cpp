/**
 * Property-Based Tests for Supervised Features Position Memory
 * 
 * Feature: supervised-features
 * Property 4: Position Memory Round-Trip
 * Property 5: Mount Type Mismatch Skips Restoration
 * 
 * Property 4: For any valid axis positions stored to NV when power-off memory is enabled,
 * restoring from NV with matching mount type SHALL restore the same positions.
 * Validates: Requirements 3.2, 3.3
 * 
 * Property 5: For any stored position with a mount type that differs from the current
 * configuration, position restoration SHALL be skipped and positions SHALL remain unchanged.
 * Validates: Requirements 3.6
 */

#include <unity.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <math.h>

// Minimum iterations for property-based tests
#define PBT_ITERATIONS 100

// Simulated mount type (from Constants.h)
#define GEM                         1
#define FORK                        2
#define ALTAZM                      3
#define ALTALT                      4

// Current mount type for testing
static uint8_t CURRENT_MOUNT_TYPE = GEM;

// Tolerance for floating point comparison
#define FLOAT_TOLERANCE 0.0001f

// Simulated NV storage for position memory
typedef struct {
    float axis1TruePosition;
    float axis2TruePosition;
    uint8_t mountTypeValidation;
} SimulatedNVStorage;

static SimulatedNVStorage nvStorage;

// Simulated axis state
static double axis1AbsoluteIndex = 0.0;
static double axis2AbsoluteIndex = 0.0;
static double axis1MotorPosition = 0.0;
static double axis2MotorPosition = 0.0;

// Simulated axis methods
double getAxis1TruePosition() {
    return axis1MotorPosition + axis1AbsoluteIndex;
}

double getAxis2TruePosition() {
    return axis2MotorPosition + axis2AbsoluteIndex;
}

void setAxis1AbsoluteIndex(double value) {
    axis1AbsoluteIndex = value;
}

void setAxis2AbsoluteIndex(double value) {
    axis2AbsoluteIndex = value;
}


// Simulated savePositions() - mirrors Supervised::savePositions()
void savePositions() {
    nvStorage.axis1TruePosition = (float)getAxis1TruePosition();
    nvStorage.axis2TruePosition = (float)getAxis2TruePosition();
    nvStorage.mountTypeValidation = CURRENT_MOUNT_TYPE;
}

// Simulated restorePositions() - mirrors Supervised::restorePositions()
bool restorePositions() {
    // Check mount type validation
    if (nvStorage.mountTypeValidation != CURRENT_MOUNT_TYPE) {
        // Mount type mismatch - skip restoration
        return false;
    }

    // Restore positions to axes
    setAxis1AbsoluteIndex(nvStorage.axis1TruePosition);
    setAxis2AbsoluteIndex(nvStorage.axis2TruePosition);
    return true;
}

/**
 * Generate a random float in a reasonable range for position values
 * Range: -2*PI to 2*PI radians (full rotation range)
 */
float generateRandomPosition() {
    // Generate random value between -2*PI and 2*PI
    float range = 2.0f * 3.14159265f;
    return ((float)rand() / (float)RAND_MAX) * 2.0f * range - range;
}

/**
 * Generate a random mount type
 */
uint8_t generateRandomMountType() {
    return (uint8_t)((rand() % 4) + 1); // 1-4 (GEM, FORK, ALTAZM, ALTALT)
}

/**
 * Reset all simulated state
 */
void resetState() {
    axis1AbsoluteIndex = 0.0;
    axis2AbsoluteIndex = 0.0;
    axis1MotorPosition = 0.0;
    axis2MotorPosition = 0.0;
    nvStorage.axis1TruePosition = 0.0f;
    nvStorage.axis2TruePosition = 0.0f;
    nvStorage.mountTypeValidation = 0;
}

/**
 * Property 4: Position Memory Round-Trip
 * 
 * For any valid axis positions stored to NV when power-off memory is enabled,
 * restoring from NV with matching mount type SHALL restore the same positions.
 * 
 * Feature: supervised-features, Property 4: Position Memory Round-Trip
 * Validates: Requirements 3.2, 3.3
 */
void test_property_position_memory_round_trip() {
    srand((unsigned int)time(NULL));
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Reset state
        resetState();
        
        // Generate random positions
        float originalAxis1 = generateRandomPosition();
        float originalAxis2 = generateRandomPosition();
        
        // Set up initial state with random positions
        // Simulate motor at some position with absolute index
        axis1MotorPosition = originalAxis1 * 0.3;  // Split between motor and index
        axis1AbsoluteIndex = originalAxis1 * 0.7;
        axis2MotorPosition = originalAxis2 * 0.3;
        axis2AbsoluteIndex = originalAxis2 * 0.7;
        
        // Save positions (simulates power-off save)
        savePositions();
        
        // Verify mount type was saved
        TEST_ASSERT_EQUAL_UINT8(CURRENT_MOUNT_TYPE, nvStorage.mountTypeValidation);
        
        // Simulate power cycle - motor position resets to 0
        axis1MotorPosition = 0.0;
        axis2MotorPosition = 0.0;
        axis1AbsoluteIndex = 0.0;
        axis2AbsoluteIndex = 0.0;
        
        // Restore positions (simulates power-on restore)
        bool restored = restorePositions();
        
        // Verify restoration succeeded
        TEST_ASSERT_TRUE(restored);
        
        // Verify positions were restored correctly
        // After restore, getTruePosition() should return the original saved value
        float restoredAxis1 = (float)getAxis1TruePosition();
        float restoredAxis2 = (float)getAxis2TruePosition();
        
        // Check that restored positions match original (within tolerance)
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, nvStorage.axis1TruePosition, restoredAxis1);
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, nvStorage.axis2TruePosition, restoredAxis2);
    }
}


/**
 * Property 5: Mount Type Mismatch Skips Restoration
 * 
 * For any stored position with a mount type that differs from the current
 * configuration, position restoration SHALL be skipped and positions SHALL
 * remain unchanged.
 * 
 * Feature: supervised-features, Property 5: Mount Type Mismatch Skips Restoration
 * Validates: Requirements 3.6
 */
void test_property_mount_type_mismatch_skips_restoration() {
    srand((unsigned int)time(NULL) + 1);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Reset state
        resetState();
        
        // Generate random positions for storage
        float storedAxis1 = generateRandomPosition();
        float storedAxis2 = generateRandomPosition();
        
        // Generate a mount type that differs from current
        uint8_t differentMountType;
        do {
            differentMountType = generateRandomMountType();
        } while (differentMountType == CURRENT_MOUNT_TYPE);
        
        // Manually set NV storage with different mount type
        nvStorage.axis1TruePosition = storedAxis1;
        nvStorage.axis2TruePosition = storedAxis2;
        nvStorage.mountTypeValidation = differentMountType;
        
        // Set initial axis state (should remain unchanged after failed restore)
        double initialAxis1Index = 0.0;
        double initialAxis2Index = 0.0;
        axis1AbsoluteIndex = initialAxis1Index;
        axis2AbsoluteIndex = initialAxis2Index;
        
        // Attempt to restore positions
        bool restored = restorePositions();
        
        // Verify restoration was skipped
        TEST_ASSERT_FALSE(restored);
        
        // Verify positions remain unchanged
        TEST_ASSERT_EQUAL_DOUBLE(initialAxis1Index, axis1AbsoluteIndex);
        TEST_ASSERT_EQUAL_DOUBLE(initialAxis2Index, axis2AbsoluteIndex);
    }
}

/**
 * Unit test: Position memory with zero positions
 */
void test_position_memory_zero_positions() {
    resetState();
    
    // Set positions to zero
    axis1MotorPosition = 0.0;
    axis1AbsoluteIndex = 0.0;
    axis2MotorPosition = 0.0;
    axis2AbsoluteIndex = 0.0;
    
    // Save positions
    savePositions();
    
    // Verify saved values
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, nvStorage.axis1TruePosition);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, nvStorage.axis2TruePosition);
    
    // Restore and verify
    bool restored = restorePositions();
    TEST_ASSERT_TRUE(restored);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, (float)getAxis1TruePosition());
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 0.0f, (float)getAxis2TruePosition());
}

/**
 * Unit test: Position memory with extreme positions
 */
void test_position_memory_extreme_positions() {
    resetState();
    
    // Set positions to extreme values (near float limits but reasonable for telescope)
    float extremePos = 100.0f;  // ~5730 degrees, way beyond normal range
    axis1AbsoluteIndex = extremePos;
    axis2AbsoluteIndex = -extremePos;
    
    // Save positions
    savePositions();
    
    // Simulate power cycle
    axis1AbsoluteIndex = 0.0;
    axis2AbsoluteIndex = 0.0;
    
    // Restore and verify
    bool restored = restorePositions();
    TEST_ASSERT_TRUE(restored);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, extremePos, (float)getAxis1TruePosition());
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, -extremePos, (float)getAxis2TruePosition());
}

/**
 * Unit test: Mount type validation with all mount types
 */
void test_mount_type_validation_all_types() {
    uint8_t mountTypes[] = {GEM, FORK, ALTAZM, ALTALT};
    
    for (int i = 0; i < 4; i++) {
        resetState();
        
        // Set current mount type
        CURRENT_MOUNT_TYPE = mountTypes[i];
        
        // Set and save position
        axis1AbsoluteIndex = 1.5;
        axis2AbsoluteIndex = 0.5;
        savePositions();
        
        // Verify mount type was saved correctly
        TEST_ASSERT_EQUAL_UINT8(mountTypes[i], nvStorage.mountTypeValidation);
        
        // Simulate power cycle
        axis1AbsoluteIndex = 0.0;
        axis2AbsoluteIndex = 0.0;
        
        // Restore should succeed with matching mount type
        bool restored = restorePositions();
        TEST_ASSERT_TRUE(restored);
        TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOLERANCE, 1.5f, (float)getAxis1TruePosition());
    }
    
    // Reset to default
    CURRENT_MOUNT_TYPE = GEM;
}

void setUp(void) {
    resetState();
    CURRENT_MOUNT_TYPE = GEM;
}

void tearDown(void) {
    // Nothing to clean up
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Property-based tests
    RUN_TEST(test_property_position_memory_round_trip);
    RUN_TEST(test_property_mount_type_mismatch_skips_restoration);
    
    // Unit tests
    RUN_TEST(test_position_memory_zero_positions);
    RUN_TEST(test_position_memory_extreme_positions);
    RUN_TEST(test_mount_type_validation_all_types);
    
    return UNITY_END();
}
