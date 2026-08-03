// Shared constants and helpers for AMP native unit tests.
// All values mirror src/telescope/mount/absoluteMotorPosition/AbsoluteMotorPosition.command.cpp
// validation ranges and src/Config.defaults.h compile-time defaults.

#ifndef AMP_TEST_FIXTURES_H
#define AMP_TEST_FIXTURES_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace AmpTestFixtures {

// command setter ranges (from AbsoluteMotorPosition.command.cpp)
const float DRIFT_THRESH_MIN_DEG =   1.0f;
const float DRIFT_THRESH_MAX_DEG =  90.0f;
const float EAST_LIMIT_MIN_DEG   =   1.0f;
const float EAST_LIMIT_MAX_DEG   = 180.0f;
const float WEST_LIMIT_MIN_DEG   =   1.0f;
const float WEST_LIMIT_MAX_DEG   = 180.0f;
const float HORIZON_MIN_DEG      = -30.0f;
const float HORIZON_MAX_DEG      =  30.0f;

// compile-time defaults (from Config.defaults.h)
const float DRIFT_THRESH_DEFAULT_DEG = 15.0f;
const float EAST_LIMIT_DEFAULT_DEG   = 100.0f;
const float WEST_LIMIT_DEFAULT_DEG   = 100.0f;
const float HORIZON_DEFAULT_DEG      = -10.0f;

const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;
const double Deg90      = M_PI / 2.0;
const double Deg180     = M_PI;
const double Deg360     = 2.0 * M_PI;

const float  FLOAT_TOL  = 1e-4f;
const double DOUBLE_TOL = 1e-9;

inline double degToRad(double d) { return d * DEG_TO_RAD; }
inline double radToDeg(double r) { return r * RAD_TO_DEG; }

inline double randomDouble(double lo, double hi) {
  return lo + ((double)rand() / (double)RAND_MAX) * (hi - lo);
}
inline float randomFloat(float lo, float hi) {
  return lo + ((float)rand() / (float)RAND_MAX) * (hi - lo);
}

} // namespace AmpTestFixtures

#endif
