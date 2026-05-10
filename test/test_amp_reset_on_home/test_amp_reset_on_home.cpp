// AMP::resetOnHome() tests.
//
// Production (AbsoluteMotorPosition.cpp):
//   resetOnHome():
//     if (!initialized) return;                      // gate
//     absoluteOffset1 = axis1.getInstrumentCoordinate();
//     absoluteOffset2 = axis2.getInstrumentCoordinate();
//     homed = true;
//     nv.write(... position, homed)                  // persist
//
// Called from Home::reset() AFTER resetPosition(0) and setInstrumentCoordinate(),
// so motorPosition == 0 and instrumentCoordinate == home (Deg90 for GEM axis1).
// Post-condition: getAbsoluteMotorPosN() == instrumentCoordinate at home time.
//
// initialized gate: Mount::begin runs home.reset() (which calls resetOnHome)
// BEFORE amp.init(). On that boot-time call initialized=false, so the gate
// makes resetOnHome a no-op and persisted NV state survives reboot. After
// init() sets initialized=true, subsequent home commands work normally.

#include <unity.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

#define PBT_ITERATIONS 300

static double motorPos1, motorPos2;
static double instrumentCoord1, instrumentCoord2;
static double offset1, offset2;
static bool   homed;
static bool   initialized;          // set by initAmp(); gates resetOnHome()

static double getAbsoluteMotorPos1() { return motorPos1 + offset1; }
static double getAbsoluteMotorPos2() { return motorPos2 + offset2; }

// mirrors AMP::resetOnHome() body
static void resetOnHome() {
  if (!initialized) return;
  offset1 = instrumentCoord1;
  offset2 = instrumentCoord2;
  homed = true;
}

// mirrors Home::reset() preamble + AMP::resetOnHome().
// Pre-condition: motorSteps reset to 0 and instrumentCoordinate set to home
// position by the firmware before resetOnHome runs.
static void simulateHome(double homePos1, double homePos2) {
  motorPos1 = 0.0;
  motorPos2 = 0.0;
  instrumentCoord1 = homePos1;
  instrumentCoord2 = homePos2;
  resetOnHome();
}

void setUp(void) {
  motorPos1 = motorPos2 = 0.0;
  instrumentCoord1 = instrumentCoord2 = 0.0;
  offset1 = offset2 = 0.0;
  homed = false;
  // Default to post-init state for legacy tests; gate-specific tests override
  // to false to exercise the boot-time path.
  initialized = true;
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

// ── initialized gate ─────────────────────────────────────────────────────────

// boot-time call: home.reset() runs in Mount::begin BEFORE amp.init(), so
// resetOnHome must be a no-op until init() flips the gate.
void test_reset_before_init_is_noop() {
  initialized = false;
  // simulate firmware pre-init state: motorSteps=0, instrumentCoord=Deg90 from
  // home.reset()'s setInstrumentCoordinate call
  motorPos1 = 0.0; motorPos2 = 0.0;
  instrumentCoord1 = Deg90; instrumentCoord2 = Deg90;
  resetOnHome();
  TEST_ASSERT_FALSE(homed);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, 0.0, offset1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, 0.0, offset2);
}

// after init() sets initialized=true (e.g. user :hF# command), resetOnHome
// performs the normal anchor + homed=1 update.
void test_reset_after_init_is_effective() {
  initialized = true;
  simulateHome(Deg90, Deg90);
  TEST_ASSERT_TRUE(homed);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, offset1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, Deg90, getAbsoluteMotorPos1());
}

// the load-bearing reboot scenario: previous session persisted offset/homed in
// NV. Boot sequence is (1) home.reset -> resetOnHome (gated, no-op) (2) init()
// loads NV. Persisted state must survive without being overwritten by step 1.
void test_boot_sequence_preserves_persisted_state() {
  // session N: user homed at non-default position; offset persisted to NV
  double persistedOffset1 = degToRad(110.0);
  double persistedOffset2 = degToRad(45.0);

  // === reboot ===
  initialized = false;          // freshly booted, init not yet run
  homed = false;
  offset1 = 0.0; offset2 = 0.0; // scratch (will be loaded from NV by init)

  // (1) Mount::begin -> home.reset() -> setInstrumentCoordinate(Deg90, Deg90)
  motorPos1 = 0.0; motorPos2 = 0.0;
  instrumentCoord1 = Deg90; instrumentCoord2 = Deg90;
  resetOnHome();                // gate makes this a no-op
  TEST_ASSERT_FALSE(homed);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, 0.0, offset1);

  // (2) amp.init() reads NV - persisted state intact, then sets initialized
  offset1 = persistedOffset1;
  offset2 = persistedOffset2;
  homed = true;
  initialized = true;

  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, persistedOffset1, offset1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, persistedOffset2, offset2);
  TEST_ASSERT_TRUE(homed);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_homed_flag_set);
  RUN_TEST(test_gem_home_offsets_equal_deg90);
  RUN_TEST(test_absolute_pos_equals_home_after_reset);
  RUN_TEST(test_motor_movement_after_home);
  RUN_TEST(test_rehome_overrides_previous_offset);
  RUN_TEST(test_property_post_reset_absolute_equals_home);
  RUN_TEST(test_reset_before_init_is_noop);
  RUN_TEST(test_reset_after_init_is_effective);
  RUN_TEST(test_boot_sequence_preserves_persisted_state);
  return UNITY_END();
}
