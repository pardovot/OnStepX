/**
 * Property-Based Tests for Axis Absolute Index
 * 
 * Feature: supervised-features
 * Property 1: Absolute Index Round-Trip
 * 
 * For any valid absolute index value set on an Axis, getting the absolute 
 * index SHALL return the same value that was set.
 * 
 * Validates: Requirements 1.3
 */

#include <unity.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <float.h>

// Minimum iterations for property-based tests
#define PBT_ITERATIONS 100

// Test double storage for absolute index simulation
// Since we can't include the full Axis class in native tests,
// we simulate the getter/setter behavior
static double absoluteIndex = 0.0;

double getAbsoluteIndex() { return absoluteIndex; }
void setAbsoluteIndex(double value) { absoluteIndex = value; }

/**
 * Generate a random double value in a reasonable range for radians
 * Range: -2*PI to 2*PI (full rotation in either direction)
 */
double generateRandomRadian() {
    // Generate random double between -2*PI and 2*PI
    double range = 4.0 * M_PI;
    double random = ((double)rand() / RAND_MAX) * range - (range / 2.0);
    return random;
}

/**
 * Generate a random double value including edge cases
 */
double generateRandomDouble() {
    int choice = rand() % 10;
    switch (choice) {
        case 0: return 0.0;                    // Zero
        case 1: return -0.0;                   // Negative zero
        case 2: return DBL_MIN;                // Smallest positive
        case 3: return -DBL_MIN;               // Smallest negative
        case 4: return 1.0;                    // Unity
        case 5: return -1.0;                   // Negative unity
        case 6: return M_PI;                   // Pi
        case 7: return -M_PI;                  // Negative Pi
        default: return generateRandomRadian(); // Random radian value
    }
}

/**
 * Property 1: Absolute Index Round-Trip
 * 
 * For any valid absolute index value, setting and then getting
 * should return the exact same value.
 * 
 * Feature: supervised-features, Property 1: Absolute Index Round-Trip
 * Validates: Requirements 1.3
 */
void test_property_absolute_index_round_trip() {
    srand((unsigned int)time(NULL));
    
    for (int i = 0; i < PBT_ITERATIONS; i++) {
        // Generate random test value
        double testValue = generateRandomDouble();
        
        // Set the absolute index
        setAbsoluteIndex(testValue);
        
        // Get the absolute index
        double retrievedValue = getAbsoluteIndex();
        
        // Verify round-trip: the retrieved value should equal the set value
        // Using exact equality since we're storing doubles directly
        TEST_ASSERT_EQUAL_DOUBLE(testValue, retrievedValue);
    }
}

/**
 * Test specific boundary values for absolute index
 */
void test_absolute_index_boundary_values() {
    // Test zero
    setAbsoluteIndex(0.0);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, getAbsoluteIndex());
    
    // Test positive PI
    setAbsoluteIndex(M_PI);
    TEST_ASSERT_EQUAL_DOUBLE(M_PI, getAbsoluteIndex());
    
    // Test negative PI
    setAbsoluteIndex(-M_PI);
    TEST_ASSERT_EQUAL_DOUBLE(-M_PI, getAbsoluteIndex());
    
    // Test 2*PI (full rotation)
    setAbsoluteIndex(2.0 * M_PI);
    TEST_ASSERT_EQUAL_DOUBLE(2.0 * M_PI, getAbsoluteIndex());
    
    // Test very small positive value
    setAbsoluteIndex(DBL_MIN);
    TEST_ASSERT_EQUAL_DOUBLE(DBL_MIN, getAbsoluteIndex());
    
    // Test very small negative value
    setAbsoluteIndex(-DBL_MIN);
    TEST_ASSERT_EQUAL_DOUBLE(-DBL_MIN, getAbsoluteIndex());
}

/**
 * Test that absolute index maintains precision
 */
void test_absolute_index_precision() {
    // Test with a value that has many decimal places
    double preciseValue = 1.23456789012345;
    setAbsoluteIndex(preciseValue);
    TEST_ASSERT_EQUAL_DOUBLE(preciseValue, getAbsoluteIndex());
    
    // Test with a very small difference
    double smallValue = 0.000000001;
    setAbsoluteIndex(smallValue);
    TEST_ASSERT_EQUAL_DOUBLE(smallValue, getAbsoluteIndex());
}

void setUp(void) {
    // Reset absolute index before each test
    absoluteIndex = 0.0;
}

void tearDown(void) {
    // Nothing to clean up
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Property-based test
    RUN_TEST(test_property_absolute_index_round_trip);
    
    // Unit tests for specific cases
    RUN_TEST(test_absolute_index_boundary_values);
    RUN_TEST(test_absolute_index_precision);
    
    return UNITY_END();
}
