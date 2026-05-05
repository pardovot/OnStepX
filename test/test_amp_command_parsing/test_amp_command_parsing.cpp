// AMP :PA* command parser tests.
//
// Mirrors AbsoluteMotorPosition.command.cpp routing:
//   :PAGp / :PAGt / :PAGe / :PAGw / :PAGh / :PAGd / :PAGr   getters
//   :PASt,[f] / :PASe,[f] / :PASw,[f] / :PASh,[f]            setters
//   :PASz                                                    manual re-anchor
//
// Behaviors verified:
//   - prefix mismatch (command != "PA") → return false
//   - getter format: parameter[0]='G' && parameter[2]==0
//   - setter format: parameter[0]='S' && parameter[2]==',' (except z)
//   - strtof failure on non-numeric value → CE_PARAM_FORM
//   - out-of-range value → CE_PARAM_RANGE
//   - unknown sub-letter → return false (in either G or S branch)
//   - getters produce non-numeric reply (numericReply set false)
//   - :PAGd format "+a.aaaa,+b.bbbb"; :PAGr format "h,e,w,r" with 0/1 fields

#include <unity.h>
#include <string.h>
#include <stdio.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

// firmware-equivalent error codes
enum CommandError { CE_NONE, CE_PARAM_FORM, CE_PARAM_RANGE };

// minimal model of the parser's outcomes
enum RouteResult { ROUTE_UNHANDLED, ROUTE_OK, ROUTE_FORM_ERR, ROUTE_RANGE_ERR };

// settings (deg) - match production defaults
static float driftThresholdDeg, eastLimitDeg, westLimitDeg, horizonDeg;
// state for getter format checks
static bool homed;
static bool errorEast, errorWest, errorHorizon;
static double absPos1Deg, absPos2Deg;
static double drift1Deg, drift2Deg;
// state used by :PASz# re-anchor (mirrors AMP private state + axis getters)
static double motorPos1, motorPos2;            // radians
static double instrumentCoord1, instrumentCoord2;  // radians
static double absoluteOffset1, absoluteOffset2;    // radians

// reply buffer + flags
static char replyBuf[64];
static bool numericReply;

// parser model - mirrors structure of AbsoluteMotorPosition::command
static RouteResult amp_command(const char *command, const char *parameter, CommandError *err) {
  *err = CE_NONE;
  replyBuf[0] = 0;
  numericReply = true;  // firmware default; getters set false

  if (command[0] != 'P' || command[1] != 'A') return ROUTE_UNHANDLED;

  if (parameter[0] == 'G' && parameter[2] == 0) {
    numericReply = false;
    switch (parameter[1]) {
      case 'p': sprintf(replyBuf, "%+.2f,%+.2f", absPos1Deg, absPos2Deg); return ROUTE_OK;
      case 't': sprintf(replyBuf, "%.2f", driftThresholdDeg); return ROUTE_OK;
      case 'e': sprintf(replyBuf, "%.2f", eastLimitDeg);      return ROUTE_OK;
      case 'w': sprintf(replyBuf, "%.2f", westLimitDeg);      return ROUTE_OK;
      case 'h': sprintf(replyBuf, "%.2f", horizonDeg);        return ROUTE_OK;
      case 'd': sprintf(replyBuf, "%+.4f,%+.4f", drift1Deg, drift2Deg); return ROUTE_OK;
      case 'r': sprintf(replyBuf, "%d,%d,%d,%d",
                        homed ? 1 : 0, errorEast ? 1 : 0, errorWest ? 1 : 0, errorHorizon ? 1 : 0);
                return ROUTE_OK;
      default: return ROUTE_UNHANDLED;
    }
  }

  if (parameter[0] == 'S') {
    if (parameter[1] == 'z' && parameter[2] == 0) {
      // production: absoluteOffset := instrumentCoord - motorPos; homed := true
      absoluteOffset1 = instrumentCoord1 - motorPos1;
      absoluteOffset2 = instrumentCoord2 - motorPos2;
      homed = true;
      return ROUTE_OK;
    }
    if (parameter[2] != ',') return ROUTE_UNHANDLED;

    char *conv_end;
    float val = strtof(&parameter[3], &conv_end);
    if (conv_end == &parameter[3]) { *err = CE_PARAM_FORM; return ROUTE_FORM_ERR; }

    switch (parameter[1]) {
      case 't':
        if (val < DRIFT_THRESH_MIN_DEG || val > DRIFT_THRESH_MAX_DEG) { *err = CE_PARAM_RANGE; return ROUTE_RANGE_ERR; }
        driftThresholdDeg = val; return ROUTE_OK;
      case 'e':
        if (val < EAST_LIMIT_MIN_DEG || val > EAST_LIMIT_MAX_DEG) { *err = CE_PARAM_RANGE; return ROUTE_RANGE_ERR; }
        eastLimitDeg = val; return ROUTE_OK;
      case 'w':
        if (val < WEST_LIMIT_MIN_DEG || val > WEST_LIMIT_MAX_DEG) { *err = CE_PARAM_RANGE; return ROUTE_RANGE_ERR; }
        westLimitDeg = val; return ROUTE_OK;
      case 'h':
        if (val < HORIZON_MIN_DEG || val > HORIZON_MAX_DEG) { *err = CE_PARAM_RANGE; return ROUTE_RANGE_ERR; }
        horizonDeg = val; return ROUTE_OK;
      default: return ROUTE_UNHANDLED;
    }
  }

  return ROUTE_UNHANDLED;
}

void setUp(void) {
  driftThresholdDeg = DRIFT_THRESH_DEFAULT_DEG;
  eastLimitDeg      = EAST_LIMIT_DEFAULT_DEG;
  westLimitDeg      = WEST_LIMIT_DEFAULT_DEG;
  horizonDeg        = HORIZON_DEFAULT_DEG;
  homed = false;
  errorEast = errorWest = errorHorizon = false;
  absPos1Deg = 90.0; absPos2Deg = 90.0;
  drift1Deg = 0.0; drift2Deg = 0.0;
  motorPos1 = motorPos2 = 0.0;
  instrumentCoord1 = instrumentCoord2 = 0.0;
  absoluteOffset1 = absoluteOffset2 = 0.0;
}
void tearDown(void) {}

// ── prefix routing ────────────────────────────────────────────────────────────

void test_wrong_prefix_unhandled() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("XX", "Gp", &e));
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("GX", "Gp", &e));
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("PB", "Gp", &e));
}

void test_pa_prefix_routes() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gp", &e));
}

// ── getters ───────────────────────────────────────────────────────────────────

void test_get_position_format() {
  CommandError e;
  absPos1Deg = 92.34;
  absPos2Deg = -45.67;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gp", &e));
  TEST_ASSERT_FALSE(numericReply);
  TEST_ASSERT_EQUAL_STRING("+92.34,-45.67", replyBuf);
}

void test_get_drift_threshold() {
  CommandError e;
  driftThresholdDeg = 15.0f;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gt", &e));
  TEST_ASSERT_EQUAL_STRING("15.00", replyBuf);
}

void test_get_drift_query_format() {
  CommandError e;
  drift1Deg =  1.2345;
  drift2Deg = -2.3456;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gd", &e));
  TEST_ASSERT_EQUAL_STRING("+1.2345,-2.3456", replyBuf);
}

void test_get_status_all_zero() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gr", &e));
  TEST_ASSERT_EQUAL_STRING("0,0,0,0", replyBuf);
}

void test_get_status_homed_only() {
  CommandError e;
  homed = true;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gr", &e));
  TEST_ASSERT_EQUAL_STRING("1,0,0,0", replyBuf);
}

void test_get_status_all_flags() {
  CommandError e;
  homed = true; errorEast = true; errorWest = true; errorHorizon = true;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gr", &e));
  TEST_ASSERT_EQUAL_STRING("1,1,1,1", replyBuf);
}

void test_get_unknown_subletter_unhandled() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("PA", "Gx", &e));
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("PA", "Gz", &e));
}

// G with trailing junk (parameter[2] != 0) is not a getter
void test_get_with_trailing_junk_unhandled() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("PA", "Gp,15", &e));
}

// ── setters ───────────────────────────────────────────────────────────────────

void test_set_drift_valid_accepts_and_persists() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "St,20.5", &e));
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, 20.5f, driftThresholdDeg);
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gt", &e));
  TEST_ASSERT_EQUAL_STRING("20.50", replyBuf);
}

void test_set_drift_below_range() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_RANGE_ERR, amp_command("PA", "St,0.5", &e));
  TEST_ASSERT_EQUAL(CE_PARAM_RANGE, e);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, DRIFT_THRESH_DEFAULT_DEG, driftThresholdDeg);
}

void test_set_drift_above_range() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_RANGE_ERR, amp_command("PA", "St,91", &e));
}

void test_set_drift_non_numeric() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_FORM_ERR, amp_command("PA", "St,abc", &e));
  TEST_ASSERT_EQUAL(CE_PARAM_FORM, e);
}

// ":PASt,#" - empty value. strtof on empty string yields conv_end == start
// → CE_PARAM_FORM. Same path as "St,abc" but with no leading char.
void test_set_drift_empty_value() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_FORM_ERR, amp_command("PA", "St,", &e));
  TEST_ASSERT_EQUAL(CE_PARAM_FORM, e);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, DRIFT_THRESH_DEFAULT_DEG, driftThresholdDeg);
}

void test_set_east_empty_value() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_FORM_ERR, amp_command("PA", "Se,", &e));
}

void test_set_east_min_max() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Se,1", &e));
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Se,180", &e));
  TEST_ASSERT_EQUAL(ROUTE_RANGE_ERR, amp_command("PA", "Se,0", &e));
  TEST_ASSERT_EQUAL(ROUTE_RANGE_ERR, amp_command("PA", "Se,181", &e));
}

void test_set_horizon_negative_valid() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sh,-10", &e));
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, -10.0f, horizonDeg);
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sh,-30", &e));
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sh,30", &e));
  TEST_ASSERT_EQUAL(ROUTE_RANGE_ERR, amp_command("PA", "Sh,-31", &e));
  TEST_ASSERT_EQUAL(ROUTE_RANGE_ERR, amp_command("PA", "Sh,31", &e));
}

void test_set_unknown_subletter_unhandled() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("PA", "Sx,10", &e));
}

void test_set_missing_comma_unhandled() {
  CommandError e;
  // ":PASt15" - no comma between letter and value
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("PA", "St15", &e));
}

// ── re-anchor ─────────────────────────────────────────────────────────────────

void test_reanchor_sets_homed() {
  CommandError e;
  TEST_ASSERT_FALSE(homed);
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sz", &e));
  TEST_ASSERT_TRUE(homed);
}

// :PASz with junk after z is rejected (parameter[2] != 0)
void test_reanchor_with_junk_unhandled() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_UNHANDLED, amp_command("PA", "Sz,5", &e));
}

// :PASz recomputes absoluteOffset := instrumentCoord - motorPos
// (production behavior - covered indirectly elsewhere, made explicit here)
void test_reanchor_recomputes_offset_at_home() {
  CommandError e;
  // mount is at home: motor=0, instrumentCoord=Deg90
  motorPos1 = motorPos2 = 0.0;
  instrumentCoord1 = degToRad(90.0);
  instrumentCoord2 = degToRad(90.0);

  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sz", &e));

  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(90.0), absoluteOffset1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(90.0), absoluteOffset2);
  TEST_ASSERT_TRUE(homed);
}

void test_reanchor_recomputes_offset_after_motor_move() {
  CommandError e;
  // motor moved 30° east of home; instrumentCoord follows
  motorPos1 = degToRad(-30.0);
  motorPos2 = degToRad(0.0);
  instrumentCoord1 = degToRad(60.0);   // = home (90) + motor (-30)
  instrumentCoord2 = degToRad(90.0);

  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sz", &e));

  // offset := instrumentCoord - motor → 60 - (-30) = 90 (still anchored at home)
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(90.0), absoluteOffset1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(90.0), absoluteOffset2);
}

// after a sync the instrumentCoord drifts away from motor+offset.
// :PASz "absorbs" that drift into the offset itself.
void test_reanchor_absorbs_sync_drift() {
  CommandError e;
  // pretend sync moved instrumentCoord1 by +20° (motor unchanged)
  motorPos1 = degToRad(0.0);
  instrumentCoord1 = degToRad(110.0);  // 90 home + 20 sync drift
  motorPos2 = degToRad(0.0);
  instrumentCoord2 = degToRad(90.0);

  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sz", &e));

  // offset1 := 110 - 0 = 110; the 20° drift is now baked into the anchor
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(110.0), absoluteOffset1);
  TEST_ASSERT_DOUBLE_WITHIN(DOUBLE_TOL, degToRad(90.0),  absoluteOffset2);
}

// ── round-trip ────────────────────────────────────────────────────────────────

void test_set_then_get_round_trip() {
  CommandError e;
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Se,75", &e));
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sw,80", &e));
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Sh,5", &e));

  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Ge", &e));
  TEST_ASSERT_EQUAL_STRING("75.00", replyBuf);
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gw", &e));
  TEST_ASSERT_EQUAL_STRING("80.00", replyBuf);
  TEST_ASSERT_EQUAL(ROUTE_OK, amp_command("PA", "Gh", &e));
  TEST_ASSERT_EQUAL_STRING("5.00", replyBuf);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_wrong_prefix_unhandled);
  RUN_TEST(test_pa_prefix_routes);
  RUN_TEST(test_get_position_format);
  RUN_TEST(test_get_drift_threshold);
  RUN_TEST(test_get_drift_query_format);
  RUN_TEST(test_get_status_all_zero);
  RUN_TEST(test_get_status_homed_only);
  RUN_TEST(test_get_status_all_flags);
  RUN_TEST(test_get_unknown_subletter_unhandled);
  RUN_TEST(test_get_with_trailing_junk_unhandled);
  RUN_TEST(test_set_drift_valid_accepts_and_persists);
  RUN_TEST(test_set_drift_below_range);
  RUN_TEST(test_set_drift_above_range);
  RUN_TEST(test_set_drift_non_numeric);
  RUN_TEST(test_set_drift_empty_value);
  RUN_TEST(test_set_east_empty_value);
  RUN_TEST(test_set_east_min_max);
  RUN_TEST(test_set_horizon_negative_valid);
  RUN_TEST(test_set_unknown_subletter_unhandled);
  RUN_TEST(test_set_missing_comma_unhandled);
  RUN_TEST(test_reanchor_sets_homed);
  RUN_TEST(test_reanchor_with_junk_unhandled);
  RUN_TEST(test_reanchor_recomputes_offset_at_home);
  RUN_TEST(test_reanchor_recomputes_offset_after_motor_move);
  RUN_TEST(test_reanchor_absorbs_sync_drift);
  RUN_TEST(test_set_then_get_round_trip);
  return UNITY_END();
}
