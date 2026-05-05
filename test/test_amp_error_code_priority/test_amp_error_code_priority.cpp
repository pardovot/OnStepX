// Limits::errorCode() priority chain - verifies AMP errors slot between
// hardware limit-sense and altitude in the if-chain.
//
// Production (Limits.cpp):
//   if (motorFault())                            return ERR_MOTOR_FAULT;
//   if (limitSense.axis1.min/max || axis2.min/max) return ERR_LIMIT_SENSE;
//   #if ABSOLUTE_MOTOR_POSITION == ON
//     if (amp.errorEast)    return ERR_AMP_EAST;
//     if (amp.errorWest)    return ERR_AMP_WEST;
//     if (amp.errorHorizon) return ERR_AMP_HORIZON;
//   #endif
//   if (altitude.min) return ERR_ALT_MIN;
//   if (altitude.max) return ERR_ALT_MAX;
//   ... // axis limits, meridian, NV init etc.
//
// Why this matters: errorHorizon and altitude.min can both be set
// simultaneously (Limits.cpp aggregates errorHorizon → altitude.min).
// The chain must return ERR_AMP_HORIZON, not ERR_ALT_MIN - the more
// specific error wins.

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

// error codes - match Limits.cpp ordering
enum GeneralErrors : uint8_t {
  ERR_NONE, ERR_MOTOR_FAULT, ERR_ALT_MIN, ERR_LIMIT_SENSE, ERR_DEC, ERR_AZM,
  ERR_UNDER_POLE, ERR_MERIDIAN, ERR_SYNC, ERR_PARK, ERR_GOTO_SYNC, ERR_UNSPECIFIED,
  ERR_ALT_MAX, ERR_WEATHER_INIT, ERR_SITE_INIT, ERR_NV_INIT,
  ERR_AMP_EAST, ERR_AMP_WEST, ERR_AMP_HORIZON
};

// state mirroring Limits members exercised by errorCode()
static bool motorFault;
static bool limitSenseAxis1Min, limitSenseAxis1Max;
static bool limitSenseAxis2Min, limitSenseAxis2Max;
static bool ampErrorEast, ampErrorWest, ampErrorHorizon;
static bool altitudeMin, altitudeMax;
static bool limitAxis1Min, limitAxis1Max;
static bool limitAxis2Min, limitAxis2Max;
static bool meridianEast, meridianWest;
static bool initErrorNv, initErrorValue, initErrorTls, initErrorWeather;
static bool isEquatorial;

static uint8_t errorCode() {
  if (motorFault) return ERR_MOTOR_FAULT;
  if (limitSenseAxis1Min || limitSenseAxis1Max ||
      limitSenseAxis2Min || limitSenseAxis2Max) return ERR_LIMIT_SENSE;
  if (ampErrorEast)    return ERR_AMP_EAST;
  if (ampErrorWest)    return ERR_AMP_WEST;
  if (ampErrorHorizon) return ERR_AMP_HORIZON;
  if (altitudeMin) return ERR_ALT_MIN;
  if (altitudeMax) return ERR_ALT_MAX;
  if (isEquatorial) {
    if (limitAxis1Min || limitAxis1Max) return ERR_UNDER_POLE;
    if (limitAxis2Min || limitAxis2Max) return ERR_DEC;
  } else {
    if (limitAxis1Min || limitAxis1Max) return ERR_AZM;
    if (limitAxis2Min) return ERR_ALT_MIN;
    if (limitAxis2Max) return ERR_ALT_MAX;
  }
  if (meridianEast || meridianWest) return ERR_MERIDIAN;
  if (initErrorNv || initErrorValue) return ERR_NV_INIT;
  if (initErrorTls) return ERR_SITE_INIT;
  if (initErrorWeather) return ERR_WEATHER_INIT;
  return ERR_NONE;
}

void setUp(void) {
  motorFault = false;
  limitSenseAxis1Min = limitSenseAxis1Max = false;
  limitSenseAxis2Min = limitSenseAxis2Max = false;
  ampErrorEast = ampErrorWest = ampErrorHorizon = false;
  altitudeMin = altitudeMax = false;
  limitAxis1Min = limitAxis1Max = false;
  limitAxis2Min = limitAxis2Max = false;
  meridianEast = meridianWest = false;
  initErrorNv = initErrorValue = initErrorTls = initErrorWeather = false;
  isEquatorial = true;
}
void tearDown(void) {}

// ── single-flag baseline ─────────────────────────────────────────────────────

void test_no_error_returns_err_none() {
  TEST_ASSERT_EQUAL(ERR_NONE, errorCode());
}

void test_amp_east_alone() {
  ampErrorEast = true;
  TEST_ASSERT_EQUAL(ERR_AMP_EAST, errorCode());
}

void test_amp_west_alone() {
  ampErrorWest = true;
  TEST_ASSERT_EQUAL(ERR_AMP_WEST, errorCode());
}

void test_amp_horizon_alone() {
  ampErrorHorizon = true;
  TEST_ASSERT_EQUAL(ERR_AMP_HORIZON, errorCode());
}

// ── priority: motor fault beats everything ──────────────────────────────────

void test_motor_fault_beats_amp_east() {
  motorFault = true;
  ampErrorEast = true;
  TEST_ASSERT_EQUAL(ERR_MOTOR_FAULT, errorCode());
}

void test_motor_fault_beats_all_amp() {
  motorFault = true;
  ampErrorEast = ampErrorWest = ampErrorHorizon = true;
  TEST_ASSERT_EQUAL(ERR_MOTOR_FAULT, errorCode());
}

// ── priority: hw limit sense beats AMP ──────────────────────────────────────

void test_limit_sense_beats_amp_east() {
  limitSenseAxis1Min = true;
  ampErrorEast = true;
  TEST_ASSERT_EQUAL(ERR_LIMIT_SENSE, errorCode());
}

void test_limit_sense_beats_amp_horizon() {
  limitSenseAxis2Max = true;
  ampErrorHorizon = true;
  TEST_ASSERT_EQUAL(ERR_LIMIT_SENSE, errorCode());
}

// ── priority within AMP: east > west > horizon ──────────────────────────────

void test_amp_east_beats_west() {
  ampErrorEast = true;
  ampErrorWest = true;
  TEST_ASSERT_EQUAL(ERR_AMP_EAST, errorCode());
}

void test_amp_east_beats_horizon() {
  ampErrorEast = true;
  ampErrorHorizon = true;
  TEST_ASSERT_EQUAL(ERR_AMP_EAST, errorCode());
}

void test_amp_west_beats_horizon() {
  ampErrorWest = true;
  ampErrorHorizon = true;
  TEST_ASSERT_EQUAL(ERR_AMP_WEST, errorCode());
}

// ── priority: AMP beats altitude ─────────────────────────────────────────────
// This is the load-bearing case: errorHorizon also sets altitude.min via
// Limits.cpp aggregation. errorCode() must return AMP_HORIZON (specific),
// not ALT_MIN (generic).

void test_amp_horizon_beats_altitude_min() {
  ampErrorHorizon = true;
  altitudeMin = true;       // both set, as production aggregation does
  TEST_ASSERT_EQUAL(ERR_AMP_HORIZON, errorCode());
}

void test_amp_east_beats_altitude_min() {
  ampErrorEast = true;
  altitudeMin = true;
  TEST_ASSERT_EQUAL(ERR_AMP_EAST, errorCode());
}

void test_amp_west_beats_altitude_max() {
  ampErrorWest = true;
  altitudeMax = true;
  TEST_ASSERT_EQUAL(ERR_AMP_WEST, errorCode());
}

// ── altitude beats axis-limit / meridian ────────────────────────────────────

void test_altitude_min_beats_axis1_limit() {
  altitudeMin = true;
  limitAxis1Min = true;
  TEST_ASSERT_EQUAL(ERR_ALT_MIN, errorCode());
}

// ── full cascade: every higher-priority flag set, lower flags ignored ───────

void test_full_cascade_motor_fault_wins() {
  motorFault = true;
  limitSenseAxis1Min = true;
  ampErrorEast = ampErrorWest = ampErrorHorizon = true;
  altitudeMin = altitudeMax = true;
  limitAxis1Min = limitAxis1Max = true;
  meridianEast = true;
  TEST_ASSERT_EQUAL(ERR_MOTOR_FAULT, errorCode());
}

void test_amp_east_with_lower_priority_noise() {
  ampErrorEast = true;
  altitudeMin = true;
  limitAxis2Min = true;
  meridianEast = true;
  initErrorNv = true;
  TEST_ASSERT_EQUAL(ERR_AMP_EAST, errorCode());
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_no_error_returns_err_none);
  RUN_TEST(test_amp_east_alone);
  RUN_TEST(test_amp_west_alone);
  RUN_TEST(test_amp_horizon_alone);
  RUN_TEST(test_motor_fault_beats_amp_east);
  RUN_TEST(test_motor_fault_beats_all_amp);
  RUN_TEST(test_limit_sense_beats_amp_east);
  RUN_TEST(test_limit_sense_beats_amp_horizon);
  RUN_TEST(test_amp_east_beats_west);
  RUN_TEST(test_amp_east_beats_horizon);
  RUN_TEST(test_amp_west_beats_horizon);
  RUN_TEST(test_amp_horizon_beats_altitude_min);
  RUN_TEST(test_amp_east_beats_altitude_min);
  RUN_TEST(test_amp_west_beats_altitude_max);
  RUN_TEST(test_altitude_min_beats_axis1_limit);
  RUN_TEST(test_full_cascade_motor_fault_wins);
  RUN_TEST(test_amp_east_with_lower_priority_noise);
  return UNITY_END();
}
