/**
 * Property-Based Tests for Motor Absolute Index
 * 
 * Feature: supervised-features
 * Property 3: Supervised Home Sets Absolute Index
 * 
 * For any home operation when supervised home is enabled, after the operation 
 * completes, absoluteIndexSteps SHALL equal indexSteps.
 * 
 * Validates: Requirements 2.1
 */

#include <unity.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

// Minimum iterations for property-based tests
#define PBT_ITERATIONS 100

// Simulate Motor class variables for native testing
// Since we can't include the full Motor class in native tests,
// we simulate the relevant behavior
static volatile long motorSteps = 0;
static volatile long indexSteps = 0;
static volatile long absoluteIndexSteps = 0;
static bool supervisedHomeEnabled = true;

// NV storage uses 0xFF for enabled (erased EEPROM default) and 0x00 for disabled
#define SUPERVISED_ENABLED   0xFF
#define SUPERVISED_DISABLED  0x00

// Simulate NV storage for supervised home enable
static uint8_t homeEnable = SUPERVISED_ENABLED;

// Getter/setter for absoluteIndexSteps
long getAbsoluteIndexSteps() { return absoluteIndexSteps; }
void setAbsoluteIndexSteps(long value) { absoluteIndexSteps = value; }

// Getter/setter for indexSteps
long getIndexSteps() { return indexSteps; }

// Simulate setInstrumentCoordinateParkSteps behavior
// This is the key function that sets absoluteIndexSteps = indexSteps when supervised home is enabled
void setInstrumentCoordinateParkSteps(long value, int modulo) {
    // Simplified version of the actual implementation
    if (modulo == 0) modulo = 1;
    long steps = value - motorSteps;
    steps -= modulo * 2L;
    for (int l = 0; l < modulo * 4; l++) { 
        if (steps % (modulo * 4L) == 0) break; 
        steps++; 
    }
    indexSteps = steps;
    
    // Supervised features logic
    if (homeEnable == SUPERVISED_ENABLED) {
        absoluteIndexSteps = indexSteps;
    }
}

// Simulate enabling/disabling supervised home
void setSupervisedHomeEnabled(bool enabled) {
    homeEnable = enabled ? SUPERVISED_ENABLED : SUPERVISED_DISABLED;
}

bool isSupervisedHomeEnabled() {
    return homeEnable == SUPERVISED_ENABLED;
}

/**
 * Generate a random long value for step positions
 */
long generateRandomSteps() {
    // Generate random steps in a reasonable range
    // Range: -1,000,000 to 1,000,000 steps
    return (rand() % 2000001) - 1000000;
}

/**
 * Generate a random modulo value (typically 1, 2, 4, 8, etc.)
 */
int generateRandomModulo() {
    int moduloValues[] = {1, 2, 4, 8, 16, 32};
    return moduloValues[rand() % 6];
}

/**
 * Property 3: Supervised Home Sets Absolute Index
 * 
 * For any home operation when supervised home is enabled, after the operation
 * completes, absoluteIndexSteps SHALL equal indexSteps.
 * 
 * Feature: supervised-features, Property 3: Supervised Home Sets Absolute Index
 * Validates: Requirements 2.1
 */
void test_property_supervised_home_sets_absolute_index() {
    srand((unsigned int)time(NULL));
    
    // Ensure supervised home is enabled
    setSupervisedHomeEnabled(true);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Generate random motor position and park value
        motorSteps = generateRandomSteps();
        long parkValue = generateRandomSteps();
        int modulo = generateRandomModulo();
        
        // Reset absoluteIndexSteps to a different value to ensure it gets updated
        absoluteIndexSteps = generateRandomSteps();
        
        // Perform the park operation (simulates home operation)
        setInstrumentCoordinateParkSteps(parkValue, modulo);
        
        // Property: absoluteIndexSteps SHALL equal indexSteps
        TEST_ASSERT_EQUAL_INT32(indexSteps, absoluteIndexSteps);
    }
}

/**
 * Test that absoluteIndexSteps is NOT set when supervised home is disabled
 */
void test_supervised_home_disabled_does_not_set_absolute_index() {
    srand((unsigned int)time(NULL));
    
    // Disable supervised home
    setSupervisedHomeEnabled(false);
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Generate random motor position and park value
        motorSteps = generateRandomSteps();
        long parkValue = generateRandomSteps();
        int modulo = generateRandomModulo();
        
        // Set absoluteIndexSteps to a known value
        long originalAbsoluteIndex = generateRandomSteps();
        absoluteIndexSteps = originalAbsoluteIndex;
        
        // Perform the park operation
        setInstrumentCoordinateParkSteps(parkValue, modulo);
        
        // Property: absoluteIndexSteps SHALL remain unchanged when supervised home is disabled
        TEST_ASSERT_EQUAL_INT32(originalAbsoluteIndex, absoluteIndexSteps);
    }
}

/**
 * Test boundary values for supervised home absolute index
 */
void test_supervised_home_boundary_values() {
    setSupervisedHomeEnabled(true);
    
    // Test with zero motor position
    motorSteps = 0;
    setInstrumentCoordinateParkSteps(0, 1);
    TEST_ASSERT_EQUAL_INT32(indexSteps, absoluteIndexSteps);
    
    // Test with large positive values
    motorSteps = 1000000;
    setInstrumentCoordinateParkSteps(2000000, 4);
    TEST_ASSERT_EQUAL_INT32(indexSteps, absoluteIndexSteps);
    
    // Test with large negative values
    motorSteps = -1000000;
    setInstrumentCoordinateParkSteps(-500000, 8);
    TEST_ASSERT_EQUAL_INT32(indexSteps, absoluteIndexSteps);
    
    // Test with mixed signs
    motorSteps = -500000;
    setInstrumentCoordinateParkSteps(500000, 16);
    TEST_ASSERT_EQUAL_INT32(indexSteps, absoluteIndexSteps);
}

/**
 * Test round-trip for absoluteIndexSteps getter/setter
 */
void test_absolute_index_steps_round_trip() {
    srand((unsigned int)time(NULL));
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        long testValue = generateRandomSteps();
        
        setAbsoluteIndexSteps(testValue);
        long retrievedValue = getAbsoluteIndexSteps();
        
        TEST_ASSERT_EQUAL_INT32(testValue, retrievedValue);
    }
}

void setUp(void) {
    // Reset all variables before each test
    motorSteps = 0;
    indexSteps = 0;
    absoluteIndexSteps = 0;
    homeEnable = SUPERVISED_ENABLED;  // Default to enabled
}

void tearDown(void) {
    // Nothing to clean up
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Property-based tests
    RUN_TEST(test_property_supervised_home_sets_absolute_index);
    RUN_TEST(test_supervised_home_disabled_does_not_set_absolute_index);
    RUN_TEST(test_absolute_index_steps_round_trip);
    
    // Unit tests for specific cases
    RUN_TEST(test_supervised_home_boundary_values);
    
    return UNITY_END();
}
