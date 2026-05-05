// AMP settings range validation (pure arithmetic).
//
// Mirrors AbsoluteMotorPosition.command.cpp setter ranges:
//   :PASt   driftThreshold  1..90 deg
//   :PASe   eastLimit       1..180 deg
//   :PASw   westLimit       1..180 deg
//   :PASh   horizonLimit   -30..30 deg
//
// Out-of-range values cause CE_PARAM_RANGE; valid stays unchanged.

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

#define PBT_ITERATIONS 200

static float driftThresholdDeg;
static float eastLimitDeg;
static float westLimitDeg;
static float horizonDeg;

static bool setDriftThreshold(float v) {
  if (v < DRIFT_THRESH_MIN_DEG || v > DRIFT_THRESH_MAX_DEG) return false;
  driftThresholdDeg = v; return true;
}
static bool setEastLimit(float v) {
  if (v < EAST_LIMIT_MIN_DEG || v > EAST_LIMIT_MAX_DEG) return false;
  eastLimitDeg = v; return true;
}
static bool setWestLimit(float v) {
  if (v < WEST_LIMIT_MIN_DEG || v > WEST_LIMIT_MAX_DEG) return false;
  westLimitDeg = v; return true;
}
static bool setHorizon(float v) {
  if (v < HORIZON_MIN_DEG || v > HORIZON_MAX_DEG) return false;
  horizonDeg = v; return true;
}

void setUp(void) {
  driftThresholdDeg = DRIFT_THRESH_DEFAULT_DEG;
  eastLimitDeg      = EAST_LIMIT_DEFAULT_DEG;
  westLimitDeg      = WEST_LIMIT_DEFAULT_DEG;
  horizonDeg        = HORIZON_DEFAULT_DEG;
}
void tearDown(void) {}

// ── drift threshold ───────────────────────────────────────────────────────────

void test_drift_min_accepted()  { TEST_ASSERT_TRUE(setDriftThreshold(1.0f));   TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 1.0f, driftThresholdDeg); }
void test_drift_max_accepted()  { TEST_ASSERT_TRUE(setDriftThreshold(90.0f));  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 90.0f, driftThresholdDeg); }
void test_drift_below_min()     { TEST_ASSERT_FALSE(setDriftThreshold(0.99f)); TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, DRIFT_THRESH_DEFAULT_DEG, driftThresholdDeg); }
void test_drift_above_max()     { TEST_ASSERT_FALSE(setDriftThreshold(90.01f));TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, DRIFT_THRESH_DEFAULT_DEG, driftThresholdDeg); }
void test_drift_negative()      { TEST_ASSERT_FALSE(setDriftThreshold(-5.0f)); }

// ── east limit ────────────────────────────────────────────────────────────────

void test_east_min_accepted()   { TEST_ASSERT_TRUE(setEastLimit(1.0f)); }
void test_east_max_accepted()   { TEST_ASSERT_TRUE(setEastLimit(180.0f)); }
void test_east_below_min()      { TEST_ASSERT_FALSE(setEastLimit(0.99f)); }
void test_east_above_max()      { TEST_ASSERT_FALSE(setEastLimit(180.01f)); }

// ── west limit ────────────────────────────────────────────────────────────────

void test_west_min_accepted()   { TEST_ASSERT_TRUE(setWestLimit(1.0f)); }
void test_west_max_accepted()   { TEST_ASSERT_TRUE(setWestLimit(180.0f)); }
void test_west_below_min()      { TEST_ASSERT_FALSE(setWestLimit(0.5f)); }
void test_west_above_max()      { TEST_ASSERT_FALSE(setWestLimit(181.0f)); }

// ── horizon ───────────────────────────────────────────────────────────────────

void test_horizon_min_accepted()  { TEST_ASSERT_TRUE(setHorizon(-30.0f)); }
void test_horizon_max_accepted()  { TEST_ASSERT_TRUE(setHorizon(30.0f)); }
void test_horizon_zero_accepted() { TEST_ASSERT_TRUE(setHorizon(0.0f)); }
void test_horizon_below_min()     { TEST_ASSERT_FALSE(setHorizon(-30.01f)); }
void test_horizon_above_max()     { TEST_ASSERT_FALSE(setHorizon(30.01f)); }

// ── rejected value preserves prior ────────────────────────────────────────────

void test_rejected_does_not_change_state() {
  TEST_ASSERT_TRUE(setDriftThreshold(20.0f));
  TEST_ASSERT_FALSE(setDriftThreshold(200.0f));
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 20.0f, driftThresholdDeg);
}

// P1: every value in valid range accepted; every value outside rejected
void test_property_drift_range() {
  srand(1);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    float v = randomFloat(DRIFT_THRESH_MIN_DEG, DRIFT_THRESH_MAX_DEG);
    TEST_ASSERT_TRUE(setDriftThreshold(v));
  }
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    float v = DRIFT_THRESH_MAX_DEG + 0.01f + randomFloat(0.0f, 100.0f);
    TEST_ASSERT_FALSE(setDriftThreshold(v));
  }
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    float v = DRIFT_THRESH_MIN_DEG - 0.01f - randomFloat(0.0f, 100.0f);
    TEST_ASSERT_FALSE(setDriftThreshold(v));
  }
}

void test_property_east_range() {
  srand(2);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    TEST_ASSERT_TRUE(setEastLimit(randomFloat(EAST_LIMIT_MIN_DEG, EAST_LIMIT_MAX_DEG)));
  }
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    TEST_ASSERT_FALSE(setEastLimit(EAST_LIMIT_MAX_DEG + 0.01f + randomFloat(0.0f, 100.0f)));
  }
}

void test_property_horizon_range() {
  srand(3);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    TEST_ASSERT_TRUE(setHorizon(randomFloat(HORIZON_MIN_DEG, HORIZON_MAX_DEG)));
  }
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    TEST_ASSERT_FALSE(setHorizon(HORIZON_MAX_DEG + 0.01f + randomFloat(0.0f, 100.0f)));
    TEST_ASSERT_FALSE(setHorizon(HORIZON_MIN_DEG - 0.01f - randomFloat(0.0f, 100.0f)));
  }
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_drift_min_accepted);
  RUN_TEST(test_drift_max_accepted);
  RUN_TEST(test_drift_below_min);
  RUN_TEST(test_drift_above_max);
  RUN_TEST(test_drift_negative);
  RUN_TEST(test_east_min_accepted);
  RUN_TEST(test_east_max_accepted);
  RUN_TEST(test_east_below_min);
  RUN_TEST(test_east_above_max);
  RUN_TEST(test_west_min_accepted);
  RUN_TEST(test_west_max_accepted);
  RUN_TEST(test_west_below_min);
  RUN_TEST(test_west_above_max);
  RUN_TEST(test_horizon_min_accepted);
  RUN_TEST(test_horizon_max_accepted);
  RUN_TEST(test_horizon_zero_accepted);
  RUN_TEST(test_horizon_below_min);
  RUN_TEST(test_horizon_above_max);
  RUN_TEST(test_rejected_does_not_change_state);
  RUN_TEST(test_property_drift_range);
  RUN_TEST(test_property_east_range);
  RUN_TEST(test_property_horizon_range);
  return UNITY_END();
}
