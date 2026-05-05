// AMP::init() NV first-run / second-run behavior.
//
// Production (AbsoluteMotorPosition.cpp):
//   void init() {
//     if (!nv.hasValidKey() || nv.isNull(NV_AMP_SETTINGS_BASE, sizeof(AmpSettings))) {
//       nv.writeBytes(NV_AMP_SETTINGS_BASE, &settings, sizeof(AmpSettings));
//       // Mount::begin ran home.reset() before us; seed offset from axis state.
//       nv.write(NV_AMP_POSITION_BASE,     (float)axis1.getInstrumentCoordinate());
//       nv.write(NV_AMP_POSITION_BASE + 4, (float)axis2.getInstrumentCoordinate());
//       nv.write(NV_AMP_HOMED_BASE,        (uint8_t)1);
//     }
//     nv.readBytes(NV_AMP_SETTINGS_BASE, &settings, sizeof(AmpSettings));
//     absoluteOffset1 = nv.readF(NV_AMP_POSITION_BASE);
//     absoluteOffset2 = nv.readF(NV_AMP_POSITION_BASE + 4);
//     homed = (nv.readUC(NV_AMP_HOMED_BASE) == 1);
//   }
//
// Behaviors verified:
//   - first run (no valid key OR settings null): write defaults seeded from
//     axis instrument coord (Deg90 at home for GEM), homed=1, then read back
//   - second run (valid data): leave NV alone, read existing values
//   - homed flag round-trip via uint8 0/1
//   - position offsets round-trip via float

#include <unity.h>
#include <string.h>
#include "../common/test_fixtures.h"

using namespace AmpTestFixtures;

struct AmpSettings {
  float driftThreshold;
  float eastLimit;
  float westLimit;
  float horizonLimit;
};

// NV slot constants (from Constants.h)
#define NV_AMP_SETTINGS_BASE  886
#define NV_AMP_POSITION_BASE  902
#define NV_AMP_HOMED_BASE     910

// minimal MockNV - backs reads/writes with a flat byte array, plus a
// hasValidKey() flag that gets toggled by tests to simulate first-run.
class MockNV {
public:
  uint8_t bytes[1024];
  bool valid_key;
  int writeBytesCount;
  int writeFloatCount;
  int writeUCCount;

  MockNV() { reset(); }

  void reset() {
    memset(bytes, 0, sizeof(bytes));
    valid_key = true;
    writeBytesCount = 0;
    writeFloatCount = 0;
    writeUCCount = 0;
  }

  bool hasValidKey() const { return valid_key; }

  // production behavior: returns true if all bytes in [base, base+len) are zero
  bool isNull(int base, int len) const {
    for (int i = 0; i < len; i++) {
      if (bytes[base + i] != 0) return false;
    }
    return true;
  }

  void writeBytes(int base, const void *src, int len) {
    memcpy(&bytes[base], src, len);
    writeBytesCount++;
  }

  void readBytes(int base, void *dst, int len) const {
    memcpy(dst, &bytes[base], len);
  }

  void write(int base, float v) {
    memcpy(&bytes[base], &v, sizeof(float));
    writeFloatCount++;
  }

  void write(int base, uint8_t v) {
    bytes[base] = v;
    writeUCCount++;
  }

  float readF(int base) const {
    float v;
    memcpy(&v, &bytes[base], sizeof(float));
    return v;
  }

  uint8_t readUC(int base) const { return bytes[base]; }
};

static MockNV nv;

// production-like AMP state under test
static AmpSettings settings;
static double absoluteOffset1, absoluteOffset2;
static bool   homed;

// fake axis state - mount is at home when amp.init() runs (home.reset()
// in Mount::begin runs first), so axes report Deg90 instrument coord on GEM.
static double axis1InstrumentCoord, axis2InstrumentCoord;

// the compile-time defaults (Config.defaults.h)
static AmpSettings makeDefaults() {
  AmpSettings s;
  s.driftThreshold = (float)degToRad(DRIFT_THRESH_DEFAULT_DEG);
  s.eastLimit      = (float)degToRad(EAST_LIMIT_DEFAULT_DEG);
  s.westLimit      = (float)degToRad(WEST_LIMIT_DEFAULT_DEG);
  s.horizonLimit   = (float)degToRad(HORIZON_DEFAULT_DEG);
  return s;
}

// mirrors AMP::init()
static void initAmp() {
  if (!nv.hasValidKey() || nv.isNull(NV_AMP_SETTINGS_BASE, sizeof(AmpSettings))) {
    nv.writeBytes(NV_AMP_SETTINGS_BASE, &settings, sizeof(AmpSettings));
    nv.write(NV_AMP_POSITION_BASE,     (float)axis1InstrumentCoord);
    nv.write(NV_AMP_POSITION_BASE + 4, (float)axis2InstrumentCoord);
    nv.write(NV_AMP_HOMED_BASE,        (uint8_t)1);
  }
  nv.readBytes(NV_AMP_SETTINGS_BASE, &settings, sizeof(AmpSettings));
  absoluteOffset1 = nv.readF(NV_AMP_POSITION_BASE);
  absoluteOffset2 = nv.readF(NV_AMP_POSITION_BASE + 4);
  homed = (nv.readUC(NV_AMP_HOMED_BASE) == 1);
}

void setUp(void) {
  nv.reset();
  settings = makeDefaults();
  absoluteOffset1 = absoluteOffset2 = 0.0;
  homed = false;
  // GEM home convention: motorSteps=0, instrumentCoordinate=Deg90.
  axis1InstrumentCoord = Deg90;
  axis2InstrumentCoord = Deg90;
}
void tearDown(void) {}

// ── first run ────────────────────────────────────────────────────────────────

void test_first_run_no_valid_key_writes_defaults() {
  nv.valid_key = false;
  // pre-populate settings with the defaults we expect to write
  AmpSettings before = makeDefaults();
  settings = before;

  initAmp();

  TEST_ASSERT_EQUAL(1, nv.writeBytesCount);
  TEST_ASSERT_EQUAL(2, nv.writeFloatCount);  // position 1 + position 2
  TEST_ASSERT_EQUAL(1, nv.writeUCCount);     // homed flag
}

void test_first_run_settings_null_writes_defaults() {
  // valid key but NV is fresh (all zeros) → isNull returns true
  TEST_ASSERT_TRUE(nv.hasValidKey());
  TEST_ASSERT_TRUE(nv.isNull(NV_AMP_SETTINGS_BASE, sizeof(AmpSettings)));

  initAmp();

  TEST_ASSERT_EQUAL(1, nv.writeBytesCount);
}

void test_first_run_homed_flag_set_true() {
  nv.valid_key = false;
  initAmp();
  TEST_ASSERT_TRUE(homed);
  TEST_ASSERT_EQUAL_UINT8(1, nv.readUC(NV_AMP_HOMED_BASE));
}

void test_first_run_position_seeded_from_axis() {
  nv.valid_key = false;
  initAmp();
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, Deg90, absoluteOffset1);
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, Deg90, absoluteOffset2);
}

void test_first_run_settings_match_defaults() {
  nv.valid_key = false;
  AmpSettings before = makeDefaults();
  settings = before;
  initAmp();

  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, before.driftThreshold, settings.driftThreshold);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, before.eastLimit,      settings.eastLimit);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, before.westLimit,      settings.westLimit);
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, before.horizonLimit,   settings.horizonLimit);
}

// ── second run ───────────────────────────────────────────────────────────────

void test_second_run_does_not_overwrite_settings() {
  // first-run init seeds NV
  nv.valid_key = false;
  initAmp();
  int writesAfterFirst = nv.writeBytesCount;
  nv.valid_key = true;  // post-first-run state

  // user changed a setting via :PASt,17# - simulate by writing 17° directly
  AmpSettings custom = makeDefaults();
  custom.driftThreshold = (float)degToRad(17.0);
  nv.writeBytes(NV_AMP_SETTINGS_BASE, &custom, sizeof(AmpSettings));
  int writesBeforeSecondRun = nv.writeBytesCount;

  // simulate a reboot
  settings = makeDefaults();  // C++ struct in fresh process
  initAmp();

  TEST_ASSERT_EQUAL(writesBeforeSecondRun, nv.writeBytesCount);  // no new writes
  TEST_ASSERT_FLOAT_WITHIN(FLOAT_TOL, (float)degToRad(17.0), settings.driftThreshold);
  (void)writesAfterFirst;
}

void test_second_run_loads_persisted_position() {
  // simulate a previous session that saved positions
  nv.valid_key = true;
  AmpSettings def = makeDefaults();
  nv.writeBytes(NV_AMP_SETTINGS_BASE, &def, sizeof(AmpSettings));
  nv.write(NV_AMP_POSITION_BASE,     (float)degToRad(123.0));
  nv.write(NV_AMP_POSITION_BASE + 4, (float)degToRad(-45.0));
  nv.write(NV_AMP_HOMED_BASE,        (uint8_t)1);

  initAmp();

  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, degToRad(123.0), absoluteOffset1);
  TEST_ASSERT_DOUBLE_WITHIN(FLOAT_TOL, degToRad(-45.0), absoluteOffset2);
  TEST_ASSERT_TRUE(homed);
}

// ── homed flag round-trip ────────────────────────────────────────────────────

void test_homed_flag_round_trip_one() {
  nv.valid_key = true;
  AmpSettings def = makeDefaults();
  nv.writeBytes(NV_AMP_SETTINGS_BASE, &def, sizeof(AmpSettings));
  nv.write(NV_AMP_HOMED_BASE, (uint8_t)1);
  initAmp();
  TEST_ASSERT_TRUE(homed);
}

void test_homed_flag_round_trip_zero() {
  nv.valid_key = true;
  AmpSettings def = makeDefaults();
  nv.writeBytes(NV_AMP_SETTINGS_BASE, &def, sizeof(AmpSettings));
  nv.write(NV_AMP_HOMED_BASE, (uint8_t)0);
  initAmp();
  TEST_ASSERT_FALSE(homed);
}

// non-{0,1} reads as not-homed (== 1 comparison is strict)
void test_homed_flag_garbage_treated_as_unhomed() {
  nv.valid_key = true;
  AmpSettings def = makeDefaults();
  nv.writeBytes(NV_AMP_SETTINGS_BASE, &def, sizeof(AmpSettings));
  nv.write(NV_AMP_HOMED_BASE, (uint8_t)42);
  initAmp();
  TEST_ASSERT_FALSE(homed);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_first_run_no_valid_key_writes_defaults);
  RUN_TEST(test_first_run_settings_null_writes_defaults);
  RUN_TEST(test_first_run_homed_flag_set_true);
  RUN_TEST(test_first_run_position_seeded_from_axis);
  RUN_TEST(test_first_run_settings_match_defaults);
  RUN_TEST(test_second_run_does_not_overwrite_settings);
  RUN_TEST(test_second_run_loads_persisted_position);
  RUN_TEST(test_homed_flag_round_trip_one);
  RUN_TEST(test_homed_flag_round_trip_zero);
  RUN_TEST(test_homed_flag_garbage_treated_as_unhomed);
  return UNITY_END();
}
