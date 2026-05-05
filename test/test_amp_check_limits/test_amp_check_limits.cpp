// AMP::checkLimits() east/west/horizon enforcement.
//
// Production (AbsoluteMotorPosition.cpp):
//   void checkLimits() {
//     errorEast = false; errorWest = false; errorHorizon = false;
//     if (!homed) return;
//     double absPos1 = getAbsoluteMotorPos1();
//     if (absPos1 < (Deg90 - settings.eastLimit)) { stopAxis1(REVERSE); errorEast = true; }
//     if (absPos1 > (Deg90 + settings.westLimit)) { stopAxis1(FORWARD); errorWest = true; }
//     Coordinate absCoord = transform.instrumentToMount(absPos1, absPos2);
//     transform.equToHor(&absCoord);
//     if (absCoord.a < settings.horizonLimit) { stop(); errorHorizon = true; }
//   }
//
// Convention: home is at absoluteMotorPos1 == Deg90. East side is "below"
// Deg90 (smaller absPos), west side is "above" Deg90 (larger absPos).
// All three error flags reset at function entry, so they clear on the next
// call when conditions return in-range. East/west/horizon checks are independent
// `if`s, not `else-if` - both can fire simultaneously.
//
// Horizon: real production computes altitude via transform.equToHor from
// absoluteMotorPos. Unit test models altitude as a direct input (the transform
// itself is not under test here - that path is exercised in the serial suite).

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

#define PBT_ITERATIONS 300

static double motorPos1, motorPos2;
static double offset1,   offset2;
static double eastLimit, westLimit;     // radians
static double horizonLimit;             // radians, altitude minimum
static double altitude;                 // simulated altitude (radians)
static bool   homed;

static bool errorEast    = false;
static bool errorWest    = false;
static bool errorHorizon = false;

static double getAbsoluteMotorPos1() { return motorPos1 + offset1; }

static void checkLimits() {
  errorEast    = false;
  errorWest    = false;
  errorHorizon = false;
  if (!homed) return;
  double absPos1 = getAbsoluteMotorPos1();
  if (absPos1 < (Deg90 - eastLimit)) errorEast = true;
  if (absPos1 > (Deg90 + westLimit)) errorWest = true;
  if (altitude < horizonLimit) errorHorizon = true;
}

static void simulateHome() {
  motorPos1 = 0.0; motorPos2 = 0.0;
  offset1 = Deg90; offset2 = Deg90;
  altitude = degToRad(45.0);  // safely above horizon
  homed = true;
}

void setUp(void) {
  motorPos1 = motorPos2 = 0.0;
  offset1 = offset2 = 0.0;
  eastLimit = degToRad(95.0);
  westLimit = degToRad(95.0);
  horizonLimit = degToRad(-10.0);
  altitude = degToRad(45.0);
  homed = false;
  errorEast = errorWest = errorHorizon = false;
}
void tearDown(void) {}

void test_not_homed_no_errors() {
  homed = false;
  motorPos1 = -degToRad(170.0);  // way past either limit
  offset1 = 0.0;
  altitude = degToRad(-45.0);    // also below horizon
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_FALSE(errorWest);
  TEST_ASSERT_FALSE(errorHorizon);
}

void test_at_home_no_errors() {
  simulateHome();
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_FALSE(errorWest);
}

// just inside east limit: absPos1 = (Deg90 - eastLimit) + epsilon
void test_inside_east_limit_no_error() {
  simulateHome();
  motorPos1 = -eastLimit + degToRad(0.5);  // absPos = Deg90 - eastLimit + 0.5°
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_FALSE(errorWest);
}

// outside east limit: absPos1 < (Deg90 - eastLimit)
void test_outside_east_limit_triggers_east() {
  simulateHome();
  motorPos1 = -eastLimit - degToRad(1.0);  // absPos = Deg90 - eastLimit - 1°
  checkLimits();
  TEST_ASSERT_TRUE(errorEast);
  TEST_ASSERT_FALSE(errorWest);
}

void test_inside_west_limit_no_error() {
  simulateHome();
  motorPos1 = westLimit - degToRad(0.5);
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_FALSE(errorWest);
}

void test_outside_west_limit_triggers_west() {
  simulateHome();
  motorPos1 = westLimit + degToRad(1.0);
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_TRUE(errorWest);
}

// boundary: absPos1 == (Deg90 - eastLimit) → no error (strict <)
void test_at_east_boundary_no_error() {
  simulateHome();
  motorPos1 = -eastLimit;  // absPos = Deg90 - eastLimit exactly
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
}

void test_at_west_boundary_no_error() {
  simulateHome();
  motorPos1 = westLimit;
  checkLimits();
  TEST_ASSERT_FALSE(errorWest);
}

// recovery: trip a limit, then return in-range - flag clears on next call
void test_east_error_clears_on_recovery() {
  simulateHome();
  motorPos1 = -eastLimit - degToRad(5.0);
  checkLimits();
  TEST_ASSERT_TRUE(errorEast);

  motorPos1 = 0.0;  // back at home
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
}

void test_west_error_clears_on_recovery() {
  simulateHome();
  motorPos1 = westLimit + degToRad(5.0);
  checkLimits();
  TEST_ASSERT_TRUE(errorWest);

  motorPos1 = 0.0;
  checkLimits();
  TEST_ASSERT_FALSE(errorWest);
}

// asymmetric limits: east 30°, west 100°
void test_asymmetric_limits() {
  simulateHome();
  eastLimit = degToRad(30.0);
  westLimit = degToRad(100.0);

  motorPos1 = -degToRad(31.0);   // outside east
  checkLimits();
  TEST_ASSERT_TRUE(errorEast);
  TEST_ASSERT_FALSE(errorWest);

  motorPos1 = degToRad(50.0);    // inside both
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_FALSE(errorWest);

  motorPos1 = degToRad(101.0);   // outside west
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_TRUE(errorWest);
}

// P1: absPos within (Deg90 - east, Deg90 + west) → no errors
void test_property_inside_limits_no_error() {
  srand(1);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    simulateHome();
    eastLimit = degToRad(randomFloat(EAST_LIMIT_MIN_DEG + 1.0f, EAST_LIMIT_MAX_DEG - 1.0f));
    westLimit = degToRad(randomFloat(WEST_LIMIT_MIN_DEG + 1.0f, WEST_LIMIT_MAX_DEG - 1.0f));
    motorPos1 = randomDouble(-eastLimit + 1e-3, westLimit - 1e-3);
    checkLimits();
    TEST_ASSERT_FALSE(errorEast);
    TEST_ASSERT_FALSE(errorWest);
  }
}

// P2: absPos < (Deg90 - east) → errorEast set, errorWest clear
void test_property_outside_east_triggers_east_only() {
  srand(2);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    simulateHome();
    eastLimit = degToRad(randomFloat(10.0f, 80.0f));
    westLimit = degToRad(randomFloat(10.0f, 80.0f));
    motorPos1 = -eastLimit - randomDouble(0.001, degToRad(5.0));
    checkLimits();
    TEST_ASSERT_TRUE(errorEast);
    TEST_ASSERT_FALSE(errorWest);
  }
}

// P3: error flags ALWAYS reset at function entry (idempotency under change)
void test_property_flags_reset_each_call() {
  srand(3);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    simulateHome();
    motorPos1 = -eastLimit - degToRad(5.0);
    altitude  = degToRad(-30.0);  // also below horizon
    checkLimits();
    TEST_ASSERT_TRUE(errorEast);
    TEST_ASSERT_TRUE(errorHorizon);

    motorPos1 = randomDouble(-eastLimit + 1e-3, westLimit - 1e-3);
    altitude  = degToRad(45.0);
    checkLimits();
    TEST_ASSERT_FALSE(errorEast);
    TEST_ASSERT_FALSE(errorWest);
    TEST_ASSERT_FALSE(errorHorizon);
  }
}

// ── horizon ──────────────────────────────────────────────────────────────────

void test_inside_horizon_no_error() {
  simulateHome();
  altitude = degToRad(45.0);
  checkLimits();
  TEST_ASSERT_FALSE(errorHorizon);
}

void test_below_horizon_triggers_horizon() {
  simulateHome();
  altitude = degToRad(-15.0);  // below default horizon (-10°)
  checkLimits();
  TEST_ASSERT_TRUE(errorHorizon);
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_FALSE(errorWest);
}

void test_at_horizon_boundary_no_error() {
  simulateHome();
  altitude = horizonLimit;     // strict <
  checkLimits();
  TEST_ASSERT_FALSE(errorHorizon);
}

void test_horizon_error_clears_on_recovery() {
  simulateHome();
  altitude = degToRad(-30.0);
  checkLimits();
  TEST_ASSERT_TRUE(errorHorizon);

  altitude = degToRad(45.0);   // back above horizon
  checkLimits();
  TEST_ASSERT_FALSE(errorHorizon);
}

void test_horizon_blocked_when_not_homed() {
  homed = false;
  altitude = degToRad(-45.0);  // way below horizon
  checkLimits();
  TEST_ASSERT_FALSE(errorHorizon);
}

// east + horizon are independent `if`s in production, both can fire
void test_east_and_horizon_simultaneous() {
  simulateHome();
  motorPos1 = -eastLimit - degToRad(5.0);  // east trip
  altitude  = degToRad(-30.0);             // horizon trip
  checkLimits();
  TEST_ASSERT_TRUE(errorEast);
  TEST_ASSERT_FALSE(errorWest);
  TEST_ASSERT_TRUE(errorHorizon);
}

void test_west_and_horizon_simultaneous() {
  simulateHome();
  motorPos1 = westLimit + degToRad(5.0);
  altitude  = degToRad(-30.0);
  checkLimits();
  TEST_ASSERT_FALSE(errorEast);
  TEST_ASSERT_TRUE(errorWest);
  TEST_ASSERT_TRUE(errorHorizon);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_not_homed_no_errors);
  RUN_TEST(test_at_home_no_errors);
  RUN_TEST(test_inside_east_limit_no_error);
  RUN_TEST(test_outside_east_limit_triggers_east);
  RUN_TEST(test_inside_west_limit_no_error);
  RUN_TEST(test_outside_west_limit_triggers_west);
  RUN_TEST(test_at_east_boundary_no_error);
  RUN_TEST(test_at_west_boundary_no_error);
  RUN_TEST(test_east_error_clears_on_recovery);
  RUN_TEST(test_west_error_clears_on_recovery);
  RUN_TEST(test_asymmetric_limits);
  RUN_TEST(test_property_inside_limits_no_error);
  RUN_TEST(test_property_outside_east_triggers_east_only);
  RUN_TEST(test_property_flags_reset_each_call);
  RUN_TEST(test_inside_horizon_no_error);
  RUN_TEST(test_below_horizon_triggers_horizon);
  RUN_TEST(test_at_horizon_boundary_no_error);
  RUN_TEST(test_horizon_error_clears_on_recovery);
  RUN_TEST(test_horizon_blocked_when_not_homed);
  RUN_TEST(test_east_and_horizon_simultaneous);
  RUN_TEST(test_west_and_horizon_simultaneous);
  return UNITY_END();
}
