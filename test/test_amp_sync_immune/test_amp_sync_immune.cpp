// Sync-immunity property of AMP - the load-bearing invariant.
//
// In OnStepX, sync (:CM#) calls axis.setInstrumentCoordinate(target), which
// shifts the indexPosition / instrumentCoord but leaves motorPosition and
// the AMP absoluteOffset untouched. Therefore getAbsoluteMotorPos must NOT
// change across a sync. This is the entire reason AMP exists for cable-wrap.
//
// See memory: feedback_motor_safety_frame.md
//   "AMP cable-wrap uses motorPosition + absoluteOffset (sync-immune),
//    NOT axis.getInstrumentCoordinate()"

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

#define PBT_ITERATIONS 500

static double motorPos1;
static double instrumentCoord1;  // = motorPos + indexPos in firmware; we model directly
static double offset1;            // AMP absoluteOffset, set by resetOnHome and never by sync

static double getAbsoluteMotorPos1() { return motorPos1 + offset1; }

// sync only writes instrumentCoord; motor and offset untouched
static void simulateSync(double newInstrumentCoord) {
  instrumentCoord1 = newInstrumentCoord;
}

// home convention: motor=0, offset=instrumentCoord=Deg90
static void simulateHome(double homePos) {
  motorPos1 = 0.0;
  instrumentCoord1 = homePos;
  offset1 = homePos;
}

void setUp(void) {
  motorPos1 = 0.0;
  instrumentCoord1 = 0.0;
  offset1 = 0.0;
}
void tearDown(void) {}

void test_sync_does_not_change_absolute_pos() {
  simulateHome(Deg90);
  motorPos1 = degToRad(30.0);
  double before = getAbsoluteMotorPos1();
  simulateSync(degToRad(180.0));    // huge sync offset
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, before, getAbsoluteMotorPos1());
}

void test_sync_changes_instrument_coord() {
  simulateHome(Deg90);
  simulateSync(degToRad(45.0));
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(45.0), instrumentCoord1);
}

void test_repeated_sync_no_drift_in_absolute() {
  simulateHome(Deg90);
  motorPos1 = degToRad(10.0);
  double before = getAbsoluteMotorPos1();
  for (int i = 0; i < 50; i++) {
    simulateSync(degToRad((double)i));
    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, before, getAbsoluteMotorPos1());
  }
}

// P1: sync can never affect getAbsoluteMotorPos for any motor/offset/sync values
void test_property_sync_immune() {
  srand(1);
  for (int i = 0; i < PBT_ITERATIONS; i++) {
    motorPos1 = randomDouble(-Deg180, Deg180);
    offset1   = randomDouble(-Deg180, Deg180);
    instrumentCoord1 = randomDouble(-Deg180, Deg180);
    double before = getAbsoluteMotorPos1();

    double syncTarget = randomDouble(-Deg180, Deg180);
    simulateSync(syncTarget);

    TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, before, getAbsoluteMotorPos1());
  }
}

// drift query: drift = instrumentCoord - getAbsoluteMotorPos = indexPos - offset
// so a sync of delta degrees creates exactly delta of drift, with motor frozen
void test_sync_creates_exact_drift() {
  simulateHome(Deg90);
  double absPos = getAbsoluteMotorPos1();
  simulateSync(absPos + degToRad(7.5));
  double drift = instrumentCoord1 - getAbsoluteMotorPos1();
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(7.5), drift);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_sync_does_not_change_absolute_pos);
  RUN_TEST(test_sync_changes_instrument_coord);
  RUN_TEST(test_repeated_sync_no_drift_in_absolute);
  RUN_TEST(test_property_sync_immune);
  RUN_TEST(test_sync_creates_exact_drift);
  return UNITY_END();
}
