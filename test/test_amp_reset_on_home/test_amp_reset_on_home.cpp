// AMP::resetOnHome() tests.
//
// Production (AbsoluteMotorPosition.cpp):
//   resetOnHome():
//     absoluteOffset1 = axis1.getInstrumentCoordinate();
//     absoluteOffset2 = axis2.getInstrumentCoordinate();
//     homed = true;
//
// Called from Home::reset() AFTER resetPosition(0) and setInstrumentCoordinate(),
// so motorPosition == 0 and instrumentCoordinate == home (Deg90 for GEM axis1).
// Post-condition: getAbsoluteMotorPosN() == instrumentCoordinate at home time.

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

#define PBT_ITERATIONS 300

static double motorPos1, motorPos2;
static double instrumentCoord1, instrumentCoord2;
static double offset1, offset2;
static bool   homed;

static double getAbsoluteMotorPos1() { return motorPos1 + offset1; }
static double getAbsoluteMotorPos2() { return motorPos2 + offset2; }

// mirrors Home::reset() preamble + AMP::resetOnHome()
static void simulateHome(double homePos1, double homePos2) {
  motorPos1 = 0.0;
  motorPos2 = 0.0;
  instrumentCoord1 = homePos1;
  instrumentCoord2 = homePos2;
  // resetOnHome():
  offset1 = instrumentCoord1;
  offset2 = instrumentCoord2;
  homed = true;
}

void setUp(void) {
  motorPos1 = motorPos2 = 0.0;
  instrumentCoord1 = instrumentCoord2 = 0.0;
  offset1 = offset2 = 0.0;
  homed = false;
}
void tearDown(void) {}

void test_homed_flag_set() {
  TEST_ASSERT_FALSE(homed);
  simulateHome(Deg90, Deg90);
  TEST_ASSERT_TRUE(homed);
}

// GEM home: axis1 instrumentCoord = Deg90, axis2 = Deg90
void test_gem_home_offsets_equal_deg90() {
  simulateHome(Deg90, Deg90);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, offset1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, offset2);
}

// post-home: getAbsoluteMotorPos == instrumentCoord (motor=0)
void test_absolute_pos_equals_home_after_reset() {
  simulateHome(Deg90, Deg90);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, getAbsoluteMotorPos1());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, getAbsoluteMotorPos2());
}

// after home, motor moves but offset stays - absolute tracks motor delta
void test_motor_movement_after_home() {
  simulateHome(Deg90, Deg90);
  motorPos1 = degToRad(20.0);
  motorPos2 = degToRad(-15.0);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90 + degToRad(20.0),  getAbsoluteMotorPos1());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90 + degToRad(-15.0), getAbsoluteMotorPos2());
}

// re-home wipes prior offset cleanly
void test_rehome_overrides_previous_offset() {
  simulateHome(Deg90, Deg90);
  motorPos1 = degToRad(45.0);
  // simulate a second home
  simulateHome(Deg90, Deg90);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, 0.0, motorPos1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, offset1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, getAbsoluteMotorPos1());
}

// P1: for any home position, post-reset absolute pos == that home position
void test_property_post_reset_absolute_equals_home() {
  srand(1);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    double h1 = randomDouble(-Deg180, Deg180);
    double h2 = randomDouble(-Deg90,  Deg90);
    simulateHome(h1, h2);
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, h1, getAbsoluteMotorPos1());
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, h2, getAbsoluteMotorPos2());
  }
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_homed_flag_set);
  RUN_TEST(test_gem_home_offsets_equal_deg90);
  RUN_TEST(test_absolute_pos_equals_home_after_reset);
  RUN_TEST(test_motor_movement_after_home);
  RUN_TEST(test_rehome_overrides_previous_offset);
  RUN_TEST(test_property_post_reset_absolute_equals_home);
  return UNITY_END();
}
