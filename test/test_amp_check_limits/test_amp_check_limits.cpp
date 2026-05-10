// AMP::checkLimits() east/west/horizon enforcement.
//
// Production (AbsoluteMotorPosition.cpp):
//   void checkLimits() {
//     bool lastErrorHorizon = errorHorizon;
//     errorEast = false; errorWest = false; errorHorizon = false;
//     if (!homed) return;
//     double absPos1 = getAbsoluteMotorPos1();
//     if (absPos1 < (Deg90 - settings.eastLimit)) { limits.stopAxis1(REVERSE); errorEast = true; }
//     if (absPos1 > (Deg90 + settings.westLimit)) { limits.stopAxis1(FORWARD); errorWest = true; }
//     Coordinate absCoord = transform.instrumentToMount(absPos1, absPos2);
//     transform.equToHor(&absCoord);
//     if (absCoord.a < settings.horizonLimit) {
//       bool worsening = absCoord.a < lastStopAltitude - HORIZON_REENTRY_HYST;
//       if (!lastErrorHorizon)        lastStopAltitude = absCoord.a;     // rising edge: no direct stop
//       else if (worsening) { limits.stop(); lastStopAltitude = absCoord.a; }
//       errorHorizon = true;
//     }
//   }
//
// East/west: direct stopAxis1() on every cycle the flag is set; flag cleared at
// entry. Independent `if`s, not else-if. Both can fire with horizon.
//
// Horizon: rising-edge stop fires via Limits::poll propagation
// (errorHorizon -> error.altitude.min -> rising-edge stop()), NOT here.
// checkLimits() calls limits.stop() directly only when worsening past
// (lastStopAltitude - hysteresis), to halt user driving deeper into violation.
// Recovery slews (alt rising while flag still set) must NOT trigger another stop.
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

// horizon edge-trigger state (mirrors AbsoluteMotorPosition.cpp)
static const double HORIZON_REENTRY_HYST = 0.02 * (M_PI / 180.0);  // 0.02 deg
static double lastStopAltitude = 0.0;

// production-call counters: increment when production would call limits.stop()
// or limits.stopAxis1(). Tests assert these to verify the contract that the
// rising horizon edge does NOT call stop() directly.
static int stopCount      = 0;
static int stopAxis1Count = 0;

static double getAbsoluteMotorPos1() { return motorPos1 + offset1; }

static void checkLimits() {
  bool lastErrorHorizon = errorHorizon;

  errorEast    = false;
  errorWest    = false;
  errorHorizon = false;
  if (!homed) return;
  double absPos1 = getAbsoluteMotorPos1();
  if (absPos1 < (Deg90 - eastLimit)) { stopAxis1Count++; errorEast = true; }
  if (absPos1 > (Deg90 + westLimit)) { stopAxis1Count++; errorWest = true; }

  if (altitude < horizonLimit) {
    bool worsening = altitude < lastStopAltitude - HORIZON_REENTRY_HYST;
    if (!lastErrorHorizon) {
      lastStopAltitude = altitude;
    } else if (worsening) {
      stopCount++;
      lastStopAltitude = altitude;
    }
    errorHorizon = true;
  }
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
  lastStopAltitude = 0.0;
  stopCount = 0;
  stopAxis1Count = 0;
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

// horizon rising-edge contract: lastErrorHorizon=false on entry sets
// lastStopAltitude but does NOT call limits.stop() directly. The stop arrives
// via Limits::poll propagation (errorHorizon -> error.altitude.min ->
// rising-edge stop()), not modeled here - this test asserts only the contract
// owned by checkLimits().
void test_horizon_rising_edge_no_direct_stop() {
  simulateHome();
  altitude = degToRad(-15.0);
  checkLimits();
  TEST_ASSERT_TRUE(errorHorizon);
  TEST_ASSERT_EQUAL_INT(0, stopCount);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(-15.0), lastStopAltitude);
}

// east/west: direct stopAxis1() on every cycle the flag is set (every poll
// while in violation). Distinct from horizon, which only stops on edges.
void test_east_calls_stop_axis1_every_poll() {
  simulateHome();
  motorPos1 = -eastLimit - degToRad(5.0);
  checkLimits();
  TEST_ASSERT_TRUE(errorEast);
  TEST_ASSERT_EQUAL_INT(1, stopAxis1Count);
  checkLimits();
  TEST_ASSERT_EQUAL_INT(2, stopAxis1Count);
  checkLimits();
  TEST_ASSERT_EQUAL_INT(3, stopAxis1Count);
}

// once in violation, recovery slews where alt rises must NOT call stop() again.
// The flag stays set until alt clears the limit; counter must remain 0.
void test_horizon_recovery_no_extra_stop() {
  simulateHome();
  altitude = degToRad(-15.0);
  checkLimits();                              // rising edge, stopCount=0
  TEST_ASSERT_EQUAL_INT(0, stopCount);

  altitude = degToRad(-13.0);                 // alt rising (recovery), still below
  checkLimits();
  TEST_ASSERT_TRUE(errorHorizon);
  TEST_ASSERT_EQUAL_INT(0, stopCount);

  altitude = degToRad(-11.0);                 // closer to horizon
  checkLimits();
  TEST_ASSERT_TRUE(errorHorizon);
  TEST_ASSERT_EQUAL_INT(0, stopCount);
}

// alt drifting within hysteresis of the last stop (jitter) must NOT call stop().
// HORIZON_REENTRY_HYST = 0.02 deg.
void test_horizon_within_hysteresis_no_stop() {
  simulateHome();
  altitude = degToRad(-15.0);
  checkLimits();                              // anchor lastStopAltitude = -15
  TEST_ASSERT_EQUAL_INT(0, stopCount);

  // worsen by less than hyst: -15.01 deg, still > (-15 - 0.02) = -15.02
  altitude = degToRad(-15.01);
  checkLimits();
  TEST_ASSERT_TRUE(errorHorizon);
  TEST_ASSERT_EQUAL_INT(0, stopCount);
}

// alt drops past (lastStopAltitude - hysteresis): worsening, stop() fires and
// lastStopAltitude updates to the new (worse) altitude.
void test_horizon_worsening_calls_stop() {
  simulateHome();
  altitude = degToRad(-15.0);
  checkLimits();                              // anchor lastStopAltitude = -15
  TEST_ASSERT_EQUAL_INT(0, stopCount);

  altitude = degToRad(-15.5);                 // 0.5 deg below anchor, > 0.02 hyst
  checkLimits();
  TEST_ASSERT_TRUE(errorHorizon);
  TEST_ASSERT_EQUAL_INT(1, stopCount);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(-15.5), lastStopAltitude);

  altitude = degToRad(-16.0);                 // worsens further
  checkLimits();
  TEST_ASSERT_EQUAL_INT(2, stopCount);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(-16.0), lastStopAltitude);
}

// after clearing (alt back above horizon), errorHorizon flips false. A second
// excursion below horizon is a fresh rising edge: lastStopAltitude resets to
// the new altitude and stop() is NOT called again.
void test_horizon_clear_then_retrip_is_new_rising_edge() {
  simulateHome();
  altitude = degToRad(-15.0);
  checkLimits();                              // rising edge, anchor = -15
  TEST_ASSERT_EQUAL_INT(0, stopCount);
  TEST_ASSERT_TRUE(errorHorizon);

  altitude = degToRad(45.0);                  // recover above horizon
  checkLimits();
  TEST_ASSERT_FALSE(errorHorizon);

  altitude = degToRad(-12.0);                 // re-trip, but ABOVE old anchor (-15)
  checkLimits();
  TEST_ASSERT_TRUE(errorHorizon);
  TEST_ASSERT_EQUAL_INT(0, stopCount);        // fresh rising edge, no stop
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(-12.0), lastStopAltitude);
}

// across many random polls under sustained violation, stop() may fire (on each
// worsening past hyst) but stopCount must always be < pollCount - i.e.
// in-violation polls do NOT each generate a stop call.
void test_property_horizon_polls_dont_each_stop() {
  srand(4);
  simulateHome();
  altitude = degToRad(-15.0);
  checkLimits();                              // anchor

  int polls = 100;
  for (int i = 0; i < polls; i++) {
    // random walk within +-0.5 deg of anchor (sometimes worsening, sometimes recovering)
    altitude = degToRad(-15.0) + degToRad(randomFloat(-0.5f, 0.5f));
    checkLimits();
    TEST_ASSERT_TRUE(errorHorizon || altitude >= horizonLimit);
  }
  TEST_ASSERT_TRUE_MESSAGE(stopCount < polls, "stop() must not fire on every poll");
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
  RUN_TEST(test_horizon_rising_edge_no_direct_stop);
  RUN_TEST(test_east_calls_stop_axis1_every_poll);
  RUN_TEST(test_horizon_recovery_no_extra_stop);
  RUN_TEST(test_horizon_within_hysteresis_no_stop);
  RUN_TEST(test_horizon_worsening_calls_stop);
  RUN_TEST(test_horizon_clear_then_retrip_is_new_rising_edge);
  RUN_TEST(test_property_horizon_polls_dont_each_stop);
  return UNITY_END();
}
