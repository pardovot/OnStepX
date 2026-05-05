// AMP getAbsoluteMotorPos1/2 round-trip property tests.
//
// Production formula (AbsoluteMotorPosition.cpp):
//   getAbsoluteMotorPos1() = axis1.getMotorPosition() + absoluteOffset1
//   getAbsoluteMotorPos2() = axis2.getMotorPosition() + absoluteOffset2
//
// Both motorPosition and absoluteOffset are radians. Test reimplements the
// formula in pure native code (no firmware deps) and verifies invariants.

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

#define PBT_ITERATIONS 500

static double motorPos1, motorPos2;
static double offset1, offset2;

static double getAbsoluteMotorPos1() { return motorPos1 + offset1; }
static double getAbsoluteMotorPos2() { return motorPos2 + offset2; }

void setUp(void) {
  motorPos1 = motorPos2 = 0.0;
  offset1 = offset2 = 0.0;
}
void tearDown(void) {}

void test_zero_state_returns_zero() {
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, 0.0, getAbsoluteMotorPos1());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, 0.0, getAbsoluteMotorPos2());
}

void test_offset_only() {
  offset1 = Deg90;
  offset2 = Deg90;
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, getAbsoluteMotorPos1());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, getAbsoluteMotorPos2());
}

void test_motor_only() {
  motorPos1 = degToRad(45.0);
  motorPos2 = degToRad(-30.0);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(45.0),  getAbsoluteMotorPos1());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(-30.0), getAbsoluteMotorPos2());
}

void test_both_components_sum() {
  motorPos1 = degToRad(10.0);  offset1 = Deg90;
  motorPos2 = degToRad(-5.0);  offset2 = Deg90;
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(100.0), getAbsoluteMotorPos1());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad( 85.0), getAbsoluteMotorPos2());
}

// P1: getAbsoluteMotorPos == motorPos + offset for any inputs
void test_property_sum_invariant() {
  srand(1);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    motorPos1 = randomDouble(-Deg180, Deg180);
    offset1   = randomDouble(-Deg180, Deg180);
    motorPos2 = randomDouble(-Deg90, Deg90);
    offset2   = randomDouble(-Deg90, Deg90);
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, motorPos1 + offset1, getAbsoluteMotorPos1());
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, motorPos2 + offset2, getAbsoluteMotorPos2());
  }
}

// P2: motor steps move absolute position by the same amount (offset frozen)
void test_property_motor_delta_propagates() {
  srand(2);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    motorPos1 = randomDouble(-Deg90, Deg90);
    offset1   = randomDouble(-Deg90, Deg90);
    double before = getAbsoluteMotorPos1();
    double delta  = randomDouble(-degToRad(10.0), degToRad(10.0));
    motorPos1 += delta;
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, before + delta, getAbsoluteMotorPos1());
  }
}

// P3: offset shift moves absolute position by the same amount (motor frozen)
void test_property_offset_delta_propagates() {
  srand(3);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    motorPos2 = randomDouble(-Deg90, Deg90);
    offset2   = randomDouble(-Deg90, Deg90);
    double before = getAbsoluteMotorPos2();
    double delta  = randomDouble(-degToRad(10.0), degToRad(10.0));
    offset2 += delta;
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, before + delta, getAbsoluteMotorPos2());
  }
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_zero_state_returns_zero);
  RUN_TEST(test_offset_only);
  RUN_TEST(test_motor_only);
  RUN_TEST(test_both_components_sum);
  RUN_TEST(test_property_sum_invariant);
  RUN_TEST(test_property_motor_delta_propagates);
  RUN_TEST(test_property_offset_delta_propagates);
  return UNITY_END();
}
