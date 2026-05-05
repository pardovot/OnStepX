// AMP undoes sync deltas above driftThreshold - intentional safety behavior.
//
// See memory: project_amp_drift_undoes_sync.md
//   "applyDriftCorrection wiping syncs >driftThreshold (15deg default) is
//    intentional safety behavior"
//
// Workflow:
//   user syncs to a target far from absolute pos → indexPos shifts
//   next GOTO triggers applyDriftCorrection → snaps indexPos back to offset
//   net effect: the sync gets undone if it was beyond driftThreshold
//
// Within threshold: small alignment syncs survive.

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

#define PBT_ITERATIONS 200

static double motorPos1, motorPos2;
static double indexPos1, indexPos2;
static double offset1,   offset2;
static double driftThreshold;
static bool   homed;

static double getAbsoluteMotorPos1() { return motorPos1 + offset1; }
static double getAbsoluteMotorPos2() { return motorPos2 + offset2; }
static double getInstrumentCoord1() { return motorPos1 + indexPos1; }
static double getInstrumentCoord2() { return motorPos2 + indexPos2; }

static void simulateHome(double homePos1, double homePos2) {
  motorPos1 = 0.0; motorPos2 = 0.0;
  indexPos1 = homePos1; indexPos2 = homePos2;
  offset1 = homePos1; offset2 = homePos2;
  homed = true;
}

// sync: writes only indexPos
static void simulateSync(double target1, double target2) {
  indexPos1 = target1 - motorPos1;
  indexPos2 = target2 - motorPos2;
}

static void applyDriftCorrection() {
  if (!homed) return;
  double d1 = fabs(indexPos1 - offset1);
  double d2 = fabs(indexPos2 - offset2);
  if (d1 >= driftThreshold || d2 >= driftThreshold) {
    indexPos1 = offset1;
    indexPos2 = offset2;
  }
}

void setUp(void) {
  motorPos1 = motorPos2 = 0.0;
  indexPos1 = indexPos2 = 0.0;
  offset1 = offset2 = 0.0;
  driftThreshold = degToRad(15.0);
  homed = false;
}
void tearDown(void) {}

// large sync gets undone - instrumentCoord snaps back to absolute
void test_large_sync_undone() {
  simulateHome(Deg90, Deg90);
  double targetBeforeSync = getInstrumentCoord1();
  simulateSync(targetBeforeSync + degToRad(30.0), getAbsoluteMotorPos2());

  // sync took effect
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, targetBeforeSync + degToRad(30.0), getInstrumentCoord1());

  applyDriftCorrection();

  // sync undone - instrumentCoord back to absolute pos
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, getAbsoluteMotorPos1(), getInstrumentCoord1());
}

// small sync survives - within threshold
void test_small_sync_survives() {
  simulateHome(Deg90, Deg90);
  double smallDelta = driftThreshold * 0.5;
  double newCoord = getInstrumentCoord1() + smallDelta;
  simulateSync(newCoord, getAbsoluteMotorPos2());

  applyDriftCorrection();

  // sync intact
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, newCoord, getInstrumentCoord1());
}

// just-above-threshold sync gets undone (>= comparison)
// Note: exact-threshold equality is FP-fragile (degToRad imprecision), so we
// nudge slightly above threshold to make the test deterministic.
void test_just_above_threshold_sync_undone() {
  simulateHome(Deg90, Deg90);
  double newCoord = getInstrumentCoord1() + driftThreshold * 1.0001;
  simulateSync(newCoord, getAbsoluteMotorPos2());

  applyDriftCorrection();

  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, getAbsoluteMotorPos1(), getInstrumentCoord1());
}

// :PASz# manual re-anchor: when user wants the sync to stick beyond threshold
// they call PASz which sets offset := instrumentCoord - motorPos, so future
// applyDriftCorrection becomes a no-op for that target
void test_manual_reanchor_makes_sync_stick() {
  simulateHome(Deg90, Deg90);
  double newCoord = getInstrumentCoord1() + degToRad(30.0);  // big sync
  simulateSync(newCoord, getAbsoluteMotorPos2());

  // simulate :PASz#
  offset1 = getInstrumentCoord1() - motorPos1;
  offset2 = getInstrumentCoord2() - motorPos2;

  applyDriftCorrection();

  // sync intact even though delta was way beyond threshold
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, newCoord, getInstrumentCoord1());
}

// P1: any sync delta with |d| < threshold survives applyDriftCorrection
void test_property_below_threshold_sync_survives() {
  srand(1);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    simulateHome(Deg90, Deg90);
    double d = randomDouble(-driftThreshold * 0.99, driftThreshold * 0.99);
    double targetBeforeSync = getInstrumentCoord1();
    simulateSync(targetBeforeSync + d, getAbsoluteMotorPos2());
    double afterSync = getInstrumentCoord1();
    applyDriftCorrection();
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, afterSync, getInstrumentCoord1());
  }
}

// P2: any sync delta with |d| >= threshold gets undone
void test_property_above_threshold_sync_undone() {
  srand(2);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    simulateHome(Deg90, Deg90);
    double d = randomDouble(driftThreshold, driftThreshold * 3.0);
    if (rand() % 2) d = -d;
    simulateSync(getInstrumentCoord1() + d, getAbsoluteMotorPos2());
    applyDriftCorrection();
    TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, getAbsoluteMotorPos1(), getInstrumentCoord1());
  }
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_large_sync_undone);
  RUN_TEST(test_small_sync_survives);
  RUN_TEST(test_just_above_threshold_sync_undone);
  RUN_TEST(test_manual_reanchor_makes_sync_stick);
  RUN_TEST(test_property_below_threshold_sync_survives);
  RUN_TEST(test_property_above_threshold_sync_undone);
  return UNITY_END();
}
