# AMP Test Suite

Native unit/property-based tests + serial tests for the AbsoluteMotorPosition (AMP) feature.

## Native (no hardware)

```bash
pio test -e native                       # all suites
pio test -e native -f test_amp_drift_correction
pio test -e native -v
```

| Suite | Covers |
|---|---|
| `test_amp_get_absolute_pos` | `getAbsoluteMotorPosN = motorPos + offset` round-trip + sum invariant |
| `test_amp_reset_on_home` | `resetOnHome()` sets `offset=instrumentCoord`, `homed=true` |
| `test_amp_sync_immune` | sync changes `instrumentCoord` but not `getAbsoluteMotorPos` |
| `test_amp_drift_correction` | `applyDriftCorrection()` threshold (>=), sign, `homed` gate, idempotency |
| `test_amp_drift_undoes_sync` | sync delta ≥ threshold gets undone; below threshold survives |
| `test_amp_check_limits` | east/west thresholds, `homed` gate, error flags reset each call |
| `test_amp_settings_validation` | range arithmetic for drift / east / west / horizon |
| `test_amp_command_parsing` | `:PA*` parser routing, form/range errors, getter formats |

Tests are pure native - they re-implement production formulas locally; no firmware headers, no Arduino. Property-based tests use `PBT_ITERATIONS` (200–500) over random inputs.

## Serial (connected mount)

```bash
python test/automated_amp_tests.py --port COM5                      # all (default)
python test/automated_amp_tests.py --port COM5 --suite basic        # no movement
python test/automated_amp_tests.py --port COM5 --suite movement     # mount must be free
python test/automated_amp_tests.py --port COM5 --test set_drift_threshold
python test/automated_amp_tests.py --port COM5 --last-failed        # rerun last run's failures
python test/automated_amp_tests.py --port COM5 --list-tests
python test/automated_amp_tests.py --port COM5 --no-wait-confirm    # skip movement prompts
python test/automated_amp_tests.py --port COM5 --quiet-serial       # less verbose
```

`--last-failed` reads `test/.amp_last_failed.json` (auto-saved after each run; cleared when no failures remain).

Requires: `pip install pyserial colorama`

Mount prereqs: GEM + stepper, `ABSOLUTE_MOTOR_POSITION = ON`, mount free to home and slew across meridian (movement suite).

## :PA* command reference

| Cmd | Effect |
|---|---|
| `:PAGp#` | absolute motor positions: `+aaa.aa,+bbb.bb` (axis1,axis2 deg) |
| `:PAGt#` | drift threshold (deg) |
| `:PAGe#` / `:PAGw#` | RA east / west limit (deg) |
| `:PAGh#` | horizon (altitude min) limit (deg) |
| `:PAGd#` | drift per axis: `+a.aaaa,+b.bbbb` (signed deg) |
| `:PAGr#` | status: `homed,errorEast,errorWest,errorHorizon` (each 0/1) |
| `:PASt,[f]#` | set drift threshold (1..90 deg) |
| `:PASe,[f]#` | set east limit (1..180 deg) |
| `:PASw,[f]#` | set west limit (1..180 deg) |
| `:PASh,[f]#` | set horizon (-30..30 deg) |
| `:PASz#` | manual re-anchor: snap `absoluteOffset` to current virtual coord |
