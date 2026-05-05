// AMP::applyDriftCorrection() tests.
//
// Production (AbsoluteMotorPosition.cpp):
//   void applyDriftCorrection() {
//     if (!homed) return;
//     double drift1 = fabs(axis1.getIndexPosition() - absoluteOffset1);
//     double drift2 = fabs(axis2.getIndexPosition() - absoluteOffset2);
//     if (drift1 >= driftThreshold || drift2 >= driftThreshold) {
//       axis1.setInstrumentCoordinate(getAbsoluteMotorPos1());
//       axis2.setInstrumentCoordinate(getAbsoluteMotorPos2());
//     }
//   }
//
// Model:
//   instrumentCoord = motorPos + indexPos        (firmware identity)
//   getAbsoluteMotorPos = motorPos + offset
//   drift = indexPos - offset                    (signed; applyDriftCorrection thresholds fabs)
//   sync(target): indexPos := target - motorPos  (motor and offset untouched)
//   correction:   indexPos := offset             (so drift becomes 0)

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

#define PBT_ITERATIONS 300

static double motorPos1, motorPos2;
static double indexPos1, indexPos2;
static double offset1,   offset2;
static double driftThreshold;
static bool   homed;

static double getAbsoluteMotorPos1() { return motorPos1 + offset1; }
static double getAbsoluteMotorPos2() { return motorPos2 + offset2; }
static double getInstrumentCoord1() { return motorPos1 + indexPos1; }
static double getInstrumentCoord2() { return motorPos2 + indexPos2; }

// what :PAGd# reports - signed
static double getDrift1() { return getInstrumentCoord1() - getAbsoluteMotorPos1(); }
static double getDrift2() { return getInstrumentCoord2() - getAbsoluteMotorPos2(); }

static void simulateHome(double homePos1, double homePos2) {
  motorPos1 = 0.0; motorPos2 = 0.0;
  indexPos1 = homePos1; indexPos2 = homePos2;
  offset1 = homePos1; offset2 = homePos2;
  homed = true;
}

// sync sets instrumentCoord, which means indexPos := target - motorPos
static void simulateSync(double target1, double target2) {
  indexPos1 = target1 - motorPos1;
  indexPos2 = target2 - motorPos2;
}

// returns true if correction was applied
static bool applyDriftCorrection() {
  if (!homed) return false;
  double d1 = fabs(indexPos1 - offset1);
  double d2 = fabs(indexPos2 - offset2);
  if (d1 >= driftThreshold || d2 >= driftThreshold) {
    indexPos1 = (motorPos1 + offset1) - motorPos1;  // = offset1
    indexPos2 = (motorPos2 + offset2) - motorPos2;
    return true;
  }
  return false;
}

void setUp(void) {
  motorPos1 = motorPos2 = 0.0;
  indexPos1 = indexPos2 = 0.0;
  offset1 = offset2 = 0.0;
  driftThreshold = degToRad(15.0);
  homed = false;
}
void tearDown(void) {}

void test_not_homed_no_op() {
  homed = false;
  indexPos1 = degToRad(45.0);  // huge "drift"
  TEST_ASSERT_FALSE(applyDriftCorrection());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(45.0), indexPos1);
}

void test_zero_drift_no_op() {
  simulateHome(Deg90, Deg90);
  TEST_ASSERT_FALSE(applyDriftCorrection());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, 0.0, getDrift1());
}

// boundary: |drift| just above threshold → correction fires (>= comparison)
// Note: testing exact equality would be FP-fragile (degToRad imprecision),
// so we nudge above threshold by a small epsilon to make the test deterministic.
void test_drift_just_above_threshold_corrects() {
  simulateHome(Deg90, Deg90);
  simulateSync(getAbsoluteMotorPos1() + driftThreshold * 1.0001, getAbsoluteMotorPos2());
  TEST_ASSERT_TRUE(applyDriftCorrection());
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, 0.0, getDrift1());
}

void test_drift_below_threshold_preserved() {
  simulateHome(Deg90, Deg90);
  double smallDrift = driftThreshold * 0.5;
  simulateSync(getAbsoluteMotorPos1() + smallDrift, getAbsoluteMotorPos2());
  double driftBefore = getDrift1();
  TEST_ASSERT_FALSE(applyDriftCorrection());
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, driftBefore, getDrift1());
}

void test_drift_above_threshold_corrects() {
  simulateHome(Deg90, Deg90);
  double bigDrift = driftThreshold * 2.0;
  simulateSync(getAbsoluteMotorPos1() + bigDrift, getAbsoluteMotorPos2());
  TEST_ASSERT_TRUE(applyDriftCorrection());
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, 0.0, getDrift1());
}

// negative drift (sync target below absolute position) above threshold
void test_negative_drift_above_threshold_corrects() {
  simulateHome(Deg90, Deg90);
  simulateSync(getAbsoluteMotorPos1() - driftThreshold * 2.0, getAbsoluteMotorPos2());
  TEST_ASSERT_TRUE(applyDriftCorrection());
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, 0.0, getDrift1());
}

// only axis2 drifts → correction still fires (OR-trigger on either axis)
void test_axis2_drift_triggers_both_axes_correction() {
  simulateHome(Deg90, Deg90);
  simulateSync(getAbsoluteMotorPos1(), getAbsoluteMotorPos2() + driftThreshold * 2.0);
  TEST_ASSERT_TRUE(applyDriftCorrection());
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, 0.0, getDrift1());
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, 0.0, getDrift2());
}

void test_correction_idempotent() {
  simulateHome(Deg90, Deg90);
  simulateSync(getAbsoluteMotorPos1() + driftThreshold * 2.0, getAbsoluteMotorPos2());
  applyDriftCorrection();
  TEST_ASSERT_FALSE(applyDriftCorrection());
}

// after correction: instrumentCoord == absoluteMotorPos
void test_correction_aligns_instrument_to_absolute() {
  simulateHome(Deg90, Deg90);
  simulateSync(getAbsoluteMotorPos1() + driftThreshold * 2.0, getAbsoluteMotorPos2());
  applyDriftCorrection();
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, getAbsoluteMotorPos1(), getInstrumentCoord1());
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, getAbsoluteMotorPos2(), getInstrumentCoord2());
}

// P1: when |drift| < threshold on both axes → no correction
void test_property_below_threshold_never_corrects() {
  srand(1);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    simulateHome(Deg90, Deg90);
    double d1 = randomDouble(-driftThreshold * 0.99, driftThreshold * 0.99);
    double d2 = randomDouble(-driftThreshold * 0.99, driftThreshold * 0.99);
    simulateSync(getAbsoluteMotorPos1() + d1, getAbsoluteMotorPos2() + d2);
    TEST_ASSERT_FALSE(applyDriftCorrection());
  }
}

// P2: when |drift| > threshold on either axis → correction fires and zeros drift
void test_property_above_threshold_always_corrects() {
  srand(2);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    simulateHome(Deg90, Deg90);
    double dExcess = randomDouble(driftThreshold * 1.01, driftThreshold * 3.0);
    if (rand() % 2) dExcess = -dExcess;
    if (rand() % 2) {
      simulateSync(getAbsoluteMotorPos1() + dExcess, getAbsoluteMotorPos2());
    } else {
      simulateSync(getAbsoluteMotorPos1(), getAbsoluteMotorPos2() + dExcess);
    }
    TEST_ASSERT_TRUE(applyDriftCorrection());
    TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, 0.0, getDrift1());
    TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, 0.0, getDrift2());
  }
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_not_homed_no_op);
  RUN_TEST(test_zero_drift_no_op);
  RUN_TEST(test_drift_just_above_threshold_corrects);
  RUN_TEST(test_drift_below_threshold_preserved);
  RUN_TEST(test_drift_above_threshold_corrects);
  RUN_TEST(test_negative_drift_above_threshold_corrects);
  RUN_TEST(test_axis2_drift_triggers_both_axes_correction);
  RUN_TEST(test_correction_idempotent);
  RUN_TEST(test_correction_aligns_instrument_to_absolute);
  RUN_TEST(test_property_below_threshold_never_corrects);
  RUN_TEST(test_property_above_threshold_always_corrects);
  return UNITY_END();
}
