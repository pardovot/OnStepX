#!/usr/bin/env python3
"""
Automated serial tests for AbsoluteMotorPosition (AMP).

Verifies the :PA* command interface and the AMP runtime behavior on a real
OnStepX mount: settings persistence, sync-immune absolute position, drift
correction at the configured threshold, motor limit enforcement, and the
manual re-anchor (:PASz#).

Suites:
  basic    - no mount movement (settings, parsing, getters)
  movement - slews and homing required (excluding limit tests)
  limits   - east/west/horizon limit trip + recovery tests (slow, geometry-heavy)
  all      - everything

Prereq:
  - GEM mount, stepper, no encoders (project default)
  - ABSOLUTE_MOTOR_POSITION = ON
  - Mount free to home + slew across meridian (movement suite)

Usage:
  python test/automated_amp_tests.py --port COM5                      # all (default)
  python test/automated_amp_tests.py --port COM5 --suite basic        # no movement
  python test/automated_amp_tests.py --port COM5 --suite movement     # non-limit movement
  python test/automated_amp_tests.py --port COM5 --suite limits       # limit trip + recovery
  python test/automated_amp_tests.py --port COM5 --test set_drift_threshold
  python test/automated_amp_tests.py --port COM5 --last-failed        # rerun failures
  python test/automated_amp_tests.py --port COM5 --list-tests
"""

import serial
import time
import argparse
import sys
import json
import os
from typing import List
from colorama import init, Fore
init(autoreset=True)


LAST_FAILED_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '.amp_last_failed.json'
)


TEST_DEC_NORTH = "+45:00:00"
DEFAULT_LAT = "+30*00"
DEFAULT_LON = "-034*00"
SLEW_TIMEOUT = 90
HOME_TIMEOUT = 120

# Default AMP settings (from Config.defaults.h) - used to restore after tests
AMP_DEFAULTS = {
    't': 15.0,    # drift threshold (deg)
    'e': 95.0,    # east limit (deg)
    'w': 95.0,    # west limit (deg)
    'h': -10.0,   # horizon (deg)
}


class AmpTester:
    def __init__(self, port, baud=9600, timeout=2.0, auto_confirm=False, verbose_serial=True):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.auto_confirm = auto_confirm
        self.verbose_serial = verbose_serial
        self.ser = None
        self.results = []
        self.lst = 0.0
        self._current_test_key = None
        # 3-tuple: key -> (name, method, suite)
        self.available_tests = {
            # ── basic (no movement) ──────────────────────────────────────────
            'get_all_getters_format':      ('Get formats: PAGp/Gt/Ge/Gw/Gh/Gd/Gr', self.test_get_all_getters_format,    'basic'),
            'set_drift_threshold':         ('Set drift threshold range 1..90',    self.test_set_drift_threshold,         'basic'),
            'set_east_limit':              ('Set east limit range 1..180',        self.test_set_east_limit,              'basic'),
            'set_west_limit':              ('Set west limit range 1..180',        self.test_set_west_limit,              'basic'),
            'set_horizon_limit':           ('Set horizon range -30..30',          self.test_set_horizon_limit,           'basic'),
            'non_numeric_value_rejected':  ('PASt,abc -> form error',              self.test_non_numeric_value_rejected,  'basic'),
            'unknown_subcommand_rejected': ('PAGx / PASx unhandled',              self.test_unknown_subcommand_rejected, 'basic'),
            'set_get_round_trip':          ('Set then Get returns same value',    self.test_set_get_round_trip,          'basic'),
            # ── movement ────────────────────────────────────────────────────
            'home_resets_drift':           ('Home: PAGd ~ 0,0 and PAGp ~ 90,90',  self.test_home_resets_drift,           'movement'),
            'sync_immune':                 ('Sync does not move PAGp',            self.test_sync_immune,                 'movement'),
            'sync_creates_drift':          ('Sync delta visible in PAGd',         self.test_sync_creates_drift,          'movement'),
            'drift_above_threshold_corrected': ('GOTO with drift > threshold -> corrected', self.test_drift_above_threshold_corrected, 'movement'),
            'drift_below_threshold_preserved': ('GOTO with drift < threshold -> kept',      self.test_drift_below_threshold_preserved, 'movement'),
            'east_limit_stops_motor':      ('Slew east into limit: stop / tracking off / recovery / tracking re-enable', self.test_east_limit_stops_motor, 'limits'),
            'west_limit_stops_motor':      ('Slew west into limit: stop / tracking off / recovery / tracking re-enable', self.test_west_limit_stops_motor, 'limits'),
            'horizon_trip_axis2_south':    ('Horizon trip via Ms (HA=0 Dec=-30): trip + 4 probes in-limit + MS-rejected + Mn recover', self.test_horizon_trip_axis2_south, 'limits'),
            'horizon_trip_axis1_east':     ('Horizon trip via Me: GOTO HA=-6h alt~19, slew east, recover Mw',           self.test_horizon_trip_axis1_east,  'limits'),
            'horizon_trip_axis1_west':     ('Horizon trip via Mw: GOTO HA=+3h alt~20.7, slew west, recover Me',     self.test_horizon_trip_axis1_west,  'limits'),
            'horizon_trip_axis2_north':    ('Horizon trip 4-dir probes (HA=-6h Dec=+63): trip Ms + 4 probes in-limit + Mn recover', self.test_horizon_trip_axis2_north, 'limits'),
            'manual_reanchor_zeros_drift': ('PASz after big sync -> PAGd ~ 0',     self.test_manual_reanchor_zeros_drift, 'movement'),
        }

    # ── serial plumbing ─────────────────────────────────────────────────────
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(2)
            print(f"{Fore.GREEN}✓ Connected {self.port} @ {self.baud}")
            return True
        except serial.SerialException as e:
            print(f"{Fore.RED}✗ Connect failed: {e}")
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"{Fore.GREEN}✓ Disconnected")

    def send(self, cmd, wait_ms=0, expect_reply=True, quiet=False):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("serial not open")
        self.ser.reset_input_buffer()
        self.ser.write(cmd.encode('ascii'))
        self.ser.flush()
        if wait_ms:
            time.sleep(wait_ms / 1000.0)
        if not expect_reply:
            if self.verbose_serial and not quiet:
                print(f"  {Fore.BLUE}-> {cmd}  {Fore.CYAN}(no reply expected)")
            return ""
        resp = ""
        t0 = time.time()
        while time.time() - t0 < self.timeout:
            if self.ser.in_waiting:
                c = self.ser.read(1).decode('ascii', errors='ignore')
                resp += c
                if c == '#':
                    break
        reply = resp.rstrip('#')
        if self.verbose_serial and not quiet:
            print(f"  {Fore.BLUE}-> {cmd}  {Fore.CYAN}← '{reply}'")
        return reply

    # ── logging ─────────────────────────────────────────────────────────────
    def log(self, name, ok, details=""):
        tag = f"{Fore.GREEN}PASS" if ok else f"{Fore.RED}FAIL"
        print(f"  {tag} - {name}")
        if details:
            print(f"       {Fore.CYAN}{details}")
        self.results.append({
            'name': name, 'ok': ok, 'details': details,
            'test_key': self._current_test_key,
        })

    def info(self, msg):  print(f"  {Fore.CYAN}ℹ {msg}")
    def warn(self, msg):  print(f"  {Fore.YELLOW}⚠ {msg}")
    def step(self, msg):  print(f"  {Fore.YELLOW}⏳ {msg}")

    def log_state(self, label="state"):
        """Dump RA, Dec, HA, pier, status, plus AMP-specific PAGp/PAGd/PAGr."""
        print(f"  {Fore.MAGENTA}── mount state [{label}] ──")
        ra   = self.send(":GR#",  quiet=True)
        dec  = self.send(":GD#",  quiet=True)
        pier = self.send(":Gm#",  quiet=True)
        lst  = self.send(":GS#",  quiet=True)
        st   = self.send(":GU#",  quiet=True)
        pp   = self.send(":PAGp#", quiet=True)
        pd   = self.send(":PAGd#", quiet=True)
        pr   = self.send(":PAGr#", quiet=True)
        ha_str = "?"
        try:
            ha = (self._parse_hms(lst) - self._parse_hms(ra) + 12.0) % 24.0 - 12.0
            ha_str = f"{ha:+.3f}h"
        except Exception:
            pass
        print(f"    {Fore.MAGENTA}RA={ra} Dec={dec} HA={ha_str} pier={pier} status={st}")
        print(f"    {Fore.MAGENTA}PAGp={pp} PAGd={pd} PAGr={pr} (h,e,w,r)")

    @staticmethod
    def _parse_hms(s):
        p = s.split(':')
        return int(p[0]) + int(p[1]) / 60.0 + float(p[2]) / 3600.0

    # ── helpers ─────────────────────────────────────────────────────────────
    def get_lst_hours(self):
        return self._parse_hms(self.send(":GS#", quiet=True))

    def ha_to_ra_str(self, ha_hours):
        ra = (self.lst - ha_hours) % 24.0
        h = int(ra)
        m = int((ra - h) * 60)
        s = int(((ra - h) * 60 - m) * 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    def setup_site(self, lat=DEFAULT_LAT):
        self.step(f"Setting site lat={lat}, lon={DEFAULT_LON}")
        self.send(f":St{lat}#")
        self.send(f":Sg{DEFAULT_LON}#")
        self.lst = self.get_lst_hours()
        self.info(f"Cached LST = {self.lst:.4f}h")

    def home(self):
        self.step("Tracking off then home (:hF#)")
        self.send(":Td#", wait_ms=100)
        self.send(":hF#")
        ok = self.wait_for_slew(HOME_TIMEOUT)
        self.info(f"Home complete: {ok}")
        self.log_state("after-home")

    def wait_for_slew(self, max_wait=SLEW_TIMEOUT):
        self.step(f"Waiting for slew (up to {max_wait}s)...")
        time.sleep(0.5)
        t0 = time.time()
        last = ""
        while time.time() - t0 < max_wait:
            st = self.send(":GU#", quiet=True)
            if st != last:
                print(f"    {Fore.CYAN}status: '{st}'  (t+{time.time()-t0:.1f}s)")
                last = st
            if st and 'G' not in st and 'h' not in st and ('N' in st or 'n' in st):
                print(f"  {Fore.GREEN}✓ Slew done in {time.time()-t0:.1f}s")
                return True
            time.sleep(0.4)
        print(f"  {Fore.RED}✗ Slew timeout after {max_wait}s [last='{last}']")
        return False

    def get_pagd(self):
        """Return (drift1_deg, drift2_deg) signed."""
        r = self.send(":PAGd#", quiet=True)
        a, b = r.split(',')
        return float(a), float(b)

    def get_pagp(self):
        """Return (absPos1_deg, absPos2_deg) signed."""
        r = self.send(":PAGp#", quiet=True)
        a, b = r.split(',')
        return float(a), float(b)

    def get_pagr(self):
        """Return (homed, errorEast, errorWest, errorHorizon) bools."""
        r = self.send(":PAGr#", quiet=True)
        h, e, w, hz = r.split(',')
        return bool(int(h)), bool(int(e)), bool(int(w)), bool(int(hz))

    def is_tracking(self):
        """:GW# 2nd char: 'T' tracking, 'N' not."""
        r = self.send(":GW#", quiet=True)
        return len(r) >= 2 and r[1] == 'T'

    def assert_te_ms_blocked(self, error_label, expect_te_blocked):
        """At a limit: :MS# must be rejected. :Te# behavior depends on whether
        sidereal tracking worsens the limit:
          - expect_te_blocked=True : tracking pushes axis further past limit;
            Limits::poll re-trips within 100ms -> is_tracking=False
          - expect_te_blocked=False: tracking moves axis away from / parallel to
            the limit -> tracking stays on (firmware does not gate :Te# on
            error state)
        Caller must already be in-limit."""
        self.send(":Te#", wait_ms=500)
        tracking = self.is_tracking()
        if expect_te_blocked:
            self.log(f":Te# re-tripped while {error_label} active (tracking worsens limit)",
                     not tracking, f"is_tracking={tracking}")
        else:
            self.log(f":Te# enables tracking while {error_label} active (tracking does not worsen limit)",
                     tracking, f"is_tracking={tracking}")
        self.send(":Sr12:00:00#")
        self.send(":Sd+45:00:00#")
        rc = self.send(":MS#")
        self.log(f":MS# rejected with {error_label} active",
                 rc != '0', f":MS# rc='{rc}'")

    def probe_direction(self, dir_char, axis_idx, expect_blocked,
                        pulse_ms=1000, threshold=0.1, label=""):
        """Pulse :M{dir}# for pulse_ms then :Q{dir}#. Compare PAGp[axis_idx] delta
        to threshold. expect_blocked=True asserts delta<threshold (worsening
        detection stopped motor); False asserts delta>threshold (motion allowed).
        Caller should set rate (e.g. :R7#) appropriate for pulse/threshold."""
        move_cmd = f":M{dir_char}#"
        stop_cmd = f":Q{dir_char}#"
        before = self.get_pagp()[axis_idx]
        self.send(move_cmd, expect_reply=False)
        time.sleep(pulse_ms / 1000.0)
        self.send(stop_cmd, expect_reply=False)
        time.sleep(0.5)
        after = self.get_pagp()[axis_idx]
        delta = abs(after - before)
        ok = (delta < threshold) if expect_blocked else (delta > threshold)
        kind = "blocked" if expect_blocked else "allowed"
        suffix = f" ({label})" if label else ""
        self.log(f"Probe M{dir_char.upper()} {kind} (Δ {'<' if expect_blocked else '>'} {threshold}°){suffix}",
                 ok,
                 f"before={before:+.2f}° after={after:+.2f}° Δ={delta:.3f}°")
        return delta

    def restore_amp_defaults(self):
        self.step("Restoring AMP defaults")
        for k, v in AMP_DEFAULTS.items():
            self.send(f":PAS{k},{v}#", quiet=True)

    def slew_until_horizon_clears(self, direction, axis_idx, rate_cmd=":R9#",
                                  max_wait=30):
        """Slew :M{direction}# at rate_cmd while polling PAGr; stop on errorHorizon
        clear. Use when the horizon clearance window is narrow (e.g. tight
        horizon margin near meridian) and a fixed-duration recovery would
        overshoot from below-horizon to past-meridian-below-horizon."""
        self.send(rate_cmd)
        p_before = self.get_pagp()[axis_idx]
        self.send(f":M{direction}#", expect_reply=False)
        t0 = time.time()
        cleared = False
        while time.time() - t0 < max_wait:
            time.sleep(0.2)
            _, _, _, hz = self.get_pagr()
            if not hz:
                cleared = True
                break
        self.send(f":Q{direction}#", expect_reply=False)
        time.sleep(0.5)
        p_after = self.get_pagp()[axis_idx]
        return p_before, p_after, cleared

    def goto_ha(self, ha_hours, dec=TEST_DEC_NORTH, label=""):
        self.step(f"GOTO HA={ha_hours:+.3f}h Dec={dec} [{label}]")
        ra = self.ha_to_ra_str(ha_hours)
        self.send(f":Sr{ra}#")
        self.send(f":Sd{dec}#")
        self.log_state("pre-MS")
        rc = self.send(":MS#")
        self.info(f":MS# reply='{rc}' ({'ACCEPTED' if rc == '0' else 'REJECTED'})")
        if rc != '0':
            return None, rc
        self.wait_for_slew()
        self.log_state(f"post-GOTO HA={ha_hours:+.2f}h")
        return self.send(":Gm#", quiet=True), rc

    def sync_to_offset_ra(self, ra_offset_deg, dec=TEST_DEC_NORTH):
        """Issue a sync that shifts RA by ra_offset_deg from current position."""
        ra_now = self.send(":GR#")
        h, m, s = ra_now.split(':')
        ra_h = int(h) + int(m)/60.0 + float(s)/3600.0
        new_ra_h = (ra_h + ra_offset_deg / 15.0) % 24.0
        nh = int(new_ra_h)
        nm = int((new_ra_h - nh) * 60)
        ns = int(((new_ra_h - nh) * 60 - nm) * 60)
        target = f"{nh:02d}:{nm:02d}:{ns:02d}"
        self.send(f":Sr{target}#")
        self.send(f":Sd{dec}#")
        r = self.send(":CM#", wait_ms=200)
        self.info(f":CM# reply='{r}' (sync RA shift {ra_offset_deg:+.2f}°)")
        time.sleep(0.3)

    def wait_for_stall(self, move_cmd, stop_cmd, max_wait=60):
        """Send move_cmd, poll PAGp axis1 each second, stop on stall."""
        self.send(move_cmd, expect_reply=False)
        t0 = time.time()
        last = self.get_pagp()[0]
        last_resend = time.time()
        stall = 0
        while time.time() - t0 < max_wait:
            time.sleep(1.0)
            if time.time() - last_resend >= 5.0:
                self.send(move_cmd, expect_reply=False)
                last_resend = time.time()
            pos = self.get_pagp()[0]
            print(f"    {Fore.CYAN}absPos1: {pos:.2f}°", end='\r')
            if abs(pos - last) < 0.5:
                stall += 1
                if stall >= 3:
                    break
            else:
                stall = 0
            last = pos
        self.send(stop_cmd, expect_reply=False)
        time.sleep(0.5)
        return self.get_pagp()[0]

    def wait_for_horizon_trip(self, move_cmd, stop_cmd, max_wait=120):
        """Send move_cmd, resend every 5s, stop when PAGr.errorHorizon fires.
        Returns True if the flag tripped, False on timeout."""
        self.send(move_cmd, expect_reply=False)
        t0 = time.time()
        last_resend = time.time()
        tripped = False
        while time.time() - t0 < max_wait:
            time.sleep(0.4)
            if time.time() - last_resend >= 5.0:
                self.send(move_cmd, expect_reply=False)
                last_resend = time.time()
            h, e, w, hz = self.get_pagr()
            p1, p2 = self.get_pagp()
            print(f"    {Fore.CYAN}axis2: {p2:+.2f}° (hz={int(hz)})", end='\r')
            if hz:
                tripped = True
                break
        self.send(stop_cmd, expect_reply=False)
        time.sleep(0.5)
        print()
        return tripped

    # ── basic tests ─────────────────────────────────────────────────────────
    def _banner(self, title, movement=False):
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{Fore.CYAN}{title}")
        print(f"{Fore.CYAN}{'='*60}")
        if movement:
            print(f"  {Fore.YELLOW}⚠ This test moves the mount.")

    def test_get_all_getters_format(self):
        self._banner("Getters: PAGp / Gt / Ge / Gw / Gh / Gd / Gr")

        r = self.send(":PAGp#")
        ok = ',' in r and len(r.split(',')) == 2
        self.log("PAGp returns 'a,b'", ok, f"reply='{r}'")
        try:
            a, b = r.split(','); float(a); float(b)
            self.log("PAGp values are floats", True, f"a={a} b={b}")
        except Exception as e:
            self.log("PAGp values are floats", False, f"{e}")

        for sub in ('t', 'e', 'w', 'h'):
            r = self.send(f":PAG{sub}#")
            try:
                float(r); ok = True
            except Exception:
                ok = False
            self.log(f"PAG{sub} returns float", ok, f"reply='{r}'")

        r = self.send(":PAGd#")
        try:
            a, b = r.split(','); float(a); float(b); ok = True
        except Exception:
            ok = False
        self.log("PAGd returns 'a,b' floats", ok, f"reply='{r}'")

        r = self.send(":PAGr#")
        parts = r.split(',')
        ok = len(parts) == 4 and all(p in ('0', '1') for p in parts)
        self.log("PAGr returns 'h,e,w,r' (0/1)", ok, f"reply='{r}'")

    def test_set_drift_threshold(self):
        self._banner("Set drift threshold (range 1..90)")
        try:
            self.log("Set 1°  accepted",   self.send(":PASt,1#")   == '1')
            self.log("Set 45° accepted",   self.send(":PASt,45#")  == '1')
            self.log("Set 90° accepted",   self.send(":PASt,90#")  == '1')
            self.log("Set 0°  rejected",   self.send(":PASt,0#")   == '0')
            self.log("Set 91° rejected",   self.send(":PASt,91#")  == '0')
            self.log("Set -1° rejected",   self.send(":PASt,-1#")  == '0')

            self.send(":PASt,42#")
            r = self.send(":PAGt#")
            self.log("Persistence: 42° round-trip", r == '42.00', f"PAGt='{r}'")
        finally:
            self.restore_amp_defaults()

    def test_set_east_limit(self):
        self._banner("Set east limit (range 1..180)")
        try:
            self.log("Set 1°   accepted", self.send(":PASe,1#")   == '1')
            self.log("Set 90°  accepted", self.send(":PASe,90#")  == '1')
            self.log("Set 180° accepted", self.send(":PASe,180#") == '1')
            self.log("Set 0°   rejected", self.send(":PASe,0#")   == '0')
            self.log("Set 181° rejected", self.send(":PASe,181#") == '0')
        finally:
            self.restore_amp_defaults()

    def test_set_west_limit(self):
        self._banner("Set west limit (range 1..180)")
        try:
            self.log("Set 1°   accepted", self.send(":PASw,1#")   == '1')
            self.log("Set 180° accepted", self.send(":PASw,180#") == '1')
            self.log("Set 0°   rejected", self.send(":PASw,0#")   == '0')
            self.log("Set 181° rejected", self.send(":PASw,181#") == '0')
        finally:
            self.restore_amp_defaults()

    def test_set_horizon_limit(self):
        self._banner("Set horizon (range -30..30)")
        try:
            self.log("Set -30° accepted", self.send(":PASh,-30#") == '1')
            self.log("Set 0°   accepted", self.send(":PASh,0#")   == '1')
            self.log("Set 30°  accepted", self.send(":PASh,30#")  == '1')
            self.log("Set -31° rejected", self.send(":PASh,-31#") == '0')
            self.log("Set 31°  rejected", self.send(":PASh,31#")  == '0')
        finally:
            self.restore_amp_defaults()

    def test_non_numeric_value_rejected(self):
        self._banner("PASt,abc -> form error")
        r = self.send(":PASt,abc#")
        self.log("Non-numeric drift rejected", r == '0', f"reply='{r}'")
        r = self.send(":PASe,xyz#")
        self.log("Non-numeric east rejected",  r == '0', f"reply='{r}'")

    def test_unknown_subcommand_rejected(self):
        self._banner("PAGx / PASx unhandled")
        r = self.send(":PAGx#")
        self.log("PAGx not handled (empty/error)", r in ('', '0'), f"reply='{r}'")
        r = self.send(":PASq,5#")
        self.log("PASq,5 not handled",             r in ('', '0'), f"reply='{r}'")

    def test_set_get_round_trip(self):
        self._banner("Set then Get round-trip")
        try:
            for sub, val in (('t', '17'), ('e', '88'), ('w', '92'), ('h', '-5')):
                self.send(f":PAS{sub},{val}#")
                r = self.send(f":PAG{sub}#")
                self.log(f"PAG{sub} after set {val}", abs(float(r) - float(val)) < 0.01,
                         f"set={val} get='{r}'")
        finally:
            self.restore_amp_defaults()

    # ── movement tests ──────────────────────────────────────────────────────
    def test_home_resets_drift(self):
        self._banner("Home: PAGd ~ 0,0 and PAGp ~ 90,90", movement=True)
        self.setup_site()
        self.home()
        d1, d2 = self.get_pagd()
        p1, p2 = self.get_pagp()
        h, e, w, hz = self.get_pagr()
        self.log("PAGd axis1 ~ 0", abs(d1) < 1.0, f"d1={d1:.4f}°")
        self.log("PAGd axis2 ~ 0", abs(d2) < 1.0, f"d2={d2:.4f}°")
        self.log("PAGp axis1 ~ 90 (GEM home)", abs(p1 - 90.0) < 1.0, f"p1={p1:.2f}°")
        self.log("PAGp axis2 ~ 90 (GEM home)", abs(p2 - 90.0) < 1.0, f"p2={p2:.2f}°")
        self.log("PAGr homed=1 after :hF#", h, f"PAGr='h={h} e={e} w={w} r={hz}'")
        self.log("No limit errors at home", not (e or w or hz))

    def test_sync_immune(self):
        self._banner("Sync does not move PAGp", movement=True)
        self.setup_site()
        self.home()
        # slew off home so motor moves
        self.goto_ha(+1.0, label="position pre-sync")
        before = self.get_pagp()
        self.info(f"PAGp before sync: {before}")

        # induce a small sync (5° in RA)
        self.sync_to_offset_ra(5.0)
        after = self.get_pagp()
        self.info(f"PAGp after sync:  {after}")

        d1 = abs(after[0] - before[0])
        d2 = abs(after[1] - before[1])
        self.log("PAGp axis1 unchanged by sync", d1 < 0.5, f"Δ={d1:.4f}°")
        self.log("PAGp axis2 unchanged by sync", d2 < 0.5, f"Δ={d2:.4f}°")

    def test_sync_creates_drift(self):
        self._banner("Sync delta visible in PAGd", movement=True)
        self.setup_site()
        self.home()
        self.goto_ha(+1.0, label="position pre-sync")
        d_before = self.get_pagd()
        self.sync_to_offset_ra(7.5)  # 7.5° below 15° default threshold
        d_after = self.get_pagd()

        delta1 = abs(d_after[0] - d_before[0])
        self.log("PAGd reflects sync (axis1 changed > 1°)",
                 delta1 > 1.0,
                 f"d1 before={d_before[0]:.4f}° after={d_after[0]:.4f}° Δ={delta1:.4f}°")

    def test_drift_above_threshold_corrected(self):
        self._banner("GOTO with drift > threshold -> corrected", movement=True)
        self.setup_site()
        try:
            self.send(":PASt,10#")  # threshold = 10°
            self.home()
            self.goto_ha(+1.0, label="position pre-sync")
            self.sync_to_offset_ra(15.0)  # 15° > 10° threshold
            d_after_sync = self.get_pagd()
            self.log("Drift introduced > 10°", abs(d_after_sync[0]) > 10.0,
                     f"d1={d_after_sync[0]:.4f}°")

            # GOTO triggers applyDriftCorrection
            pier, rc = self.goto_ha(+0.5, label="trigger correction")
            if rc != '0':
                self.log("GOTO accepted", False, f"rc='{rc}'")
                return
            d_after_goto = self.get_pagd()
            self.log("Drift snapped to ~ 0 after GOTO",
                     abs(d_after_goto[0]) < 1.0,
                     f"before={d_after_sync[0]:.4f}° after={d_after_goto[0]:.4f}°")
        finally:
            self.restore_amp_defaults()

    def test_drift_below_threshold_preserved(self):
        self._banner("GOTO with drift < threshold -> kept", movement=True)
        self.setup_site()
        try:
            self.send(":PASt,20#")  # threshold = 20°
            self.home()
            self.goto_ha(+1.0, label="position pre-sync")
            self.sync_to_offset_ra(5.0)  # 5° < 20° threshold
            d_after_sync = self.get_pagd()
            self.log("Drift introduced < 20°", abs(d_after_sync[0]) < 20.0)

            pier, rc = self.goto_ha(+0.5, label="should not correct")
            if rc != '0':
                self.log("GOTO accepted", False, f"rc='{rc}'")
                return
            d_after_goto = self.get_pagd()
            # within-threshold sync survives - drift stays similar magnitude
            self.log("Drift preserved (still > 1°)",
                     abs(d_after_goto[0]) > 1.0,
                     f"before={d_after_sync[0]:.4f}° after={d_after_goto[0]:.4f}°")
        finally:
            self.restore_amp_defaults()

    # At R9 the mount overshoots the limit by ~5-7° before Limits::poll (100ms)
    # catches the trip and stops the motor. The right framing is: motor stopped
    # PAST the limit (limit was hit, not just approached) AND within a deceleration
    # budget (not runaway). 15° budget covers R9 worst case observed in practice.
    OVERSHOOT_BUDGET_DEG = 15.0

    def test_east_limit_stops_motor(self):
        self._banner("Slew east into limit, verify stop / re-slew blocked / recovery", movement=True)
        try:
            self.send(":PASe,30#")
            self.send(":R9#")
            self.home()
            p_home = self.get_pagp()[0]
            limit = 90.0 - 30.0  # east absPos1 limit (60°)
            self.info(f"Home absPos1: {p_home:.2f}°  east limit: 30° (stops at {limit:.2f}°)")

            self.send(":Te#", wait_ms=1000)
            self.log("Tracking enabled after home", self.is_tracking(), ":GW# 2nd char")

            # ── 1. slew east into limit → motor stalls ───────────────────────
            self.step("Slewing east (:Me#) until stall...")
            final = self.wait_for_stall(":Me#", ":Qe#")
            overshoot = limit - final
            self.info(f"Stalled at absPos1 = {final:.2f}° (overshoot {overshoot:+.2f}° past {limit:.2f}°)")

            h, e, w, hz = self.get_pagr()
            self.log("PAGr errorEast set", e, f"h={h} e={e} w={w} r={hz}")
            self.log("Tracking off after east limit trip", not self.is_tracking())
            self.log("Motor went past east limit",
                     final < limit, f"final={final:.2f}° limit={limit:.2f}°")
            self.log(f"Overshoot within deceleration budget ({self.OVERSHOOT_BUDGET_DEG}°)",
                     0 <= overshoot <= self.OVERSHOOT_BUDGET_DEG,
                     f"overshoot={overshoot:.2f}° (R9 deceleration)")

            # ── 2. re-slew into limit blocked: motor doesn't move further ───
            self.step("Re-slewing east while errorEast active (should be blocked)...")
            pos_before = self.get_pagp()[0]
            self.send(":Me#", expect_reply=False)
            time.sleep(3.0)
            self.send(":Qe#", expect_reply=False)
            time.sleep(0.5)
            pos_after = self.get_pagp()[0]
            drift = abs(pos_after - pos_before)
            self.log("Re-slew east blocked at limit",
                     drift < 2.0,
                     f"before={pos_before:.2f}° after={pos_after:.2f}° drift={drift:.2f}°")

            # ── 3. :Te# and :MS# while errorEast active ──────────────────────
            # Sidereal moves axis1 forward (east limit = axis1.min) -> AWAY
            # from limit -> tracking stays on. :MS# still rejected.
            self.assert_te_ms_blocked("errorEast", expect_te_blocked=False)

            # ── 4. recovery: slew west moves motor away from limit ──────────
            self.step("Slewing west (recovery, away from east limit)...")
            pos_before = self.get_pagp()[0]
            self.send(":Mw#", expect_reply=False)
            time.sleep(1.0)
            self.send(":Qw#", expect_reply=False)
            time.sleep(0.5)
            pos_after = self.get_pagp()[0]
            recovered = pos_after - pos_before
            self.log("Recovery: west slew moved motor (away from east)",
                     recovered > 5.0,
                     f"before={pos_before:.2f}° after={pos_after:.2f}° Δ={recovered:+.2f}°")

            # ── 5. error clears once back in range ──────────────────────────
            time.sleep(0.3)  # allow Limits::poll cycle (100ms) to re-evaluate
            h, e, w, hz = self.get_pagr()
            in_range = pos_after >= limit
            if in_range:
                self.log("errorEast clears once motor back in range",
                         not e, f"PAGr h={h} e={e} w={w} r={hz}")
                self.send(":Te#", wait_ms=200)
                self.log("Tracking re-enables after recovery", self.is_tracking())
            else:
                self.warn(f"Recovery didn't clear east limit zone (final={pos_after:.2f}° vs limit={limit:.2f}°), skipping clear-check")
        finally:
            self.send(":Qe#", expect_reply=False)
            self.send(":Qw#", expect_reply=False)
            self.send(":Q#",  expect_reply=False)
            self.send(":Td#", expect_reply=False)
            self.restore_amp_defaults()

    def test_west_limit_stops_motor(self):
        self._banner("Slew west into limit, verify stop / re-slew blocked / recovery", movement=True)
        try:
            self.send(":PASw,30#")
            self.send(":R9#")
            self.home()
            p_home = self.get_pagp()[0]
            limit = 90.0 + 30.0  # west absPos1 limit (120°)
            self.info(f"Home absPos1: {p_home:.2f}°  west limit: 30° (stops at {limit:.2f}°)")

            self.send(":Te#", wait_ms=200)
            self.log("Tracking enabled after home", self.is_tracking(), ":GW# 2nd char")

            # ── 1. slew west into limit → motor stalls ───────────────────────
            self.step("Slewing west (:Mw#) until stall...")
            final = self.wait_for_stall(":Mw#", ":Qw#")
            overshoot = final - limit
            self.info(f"Stalled at absPos1 = {final:.2f}° (overshoot {overshoot:+.2f}° past {limit:.2f}°)")

            h, e, w, hz = self.get_pagr()
            self.log("PAGr errorWest set", w, f"h={h} e={e} w={w} r={hz}")
            self.log("Tracking off after west limit trip", not self.is_tracking())
            self.log("Motor went past west limit",
                     final > limit, f"final={final:.2f}° limit={limit:.2f}°")
            self.log(f"Overshoot within deceleration budget ({self.OVERSHOOT_BUDGET_DEG}°)",
                     0 <= overshoot <= self.OVERSHOOT_BUDGET_DEG,
                     f"overshoot={overshoot:.2f}° (R9 deceleration)")

            # ── 2. re-slew into limit blocked ────────────────────────────────
            self.step("Re-slewing west while errorWest active (should be blocked)...")
            pos_before = self.get_pagp()[0]
            self.send(":Mw#", expect_reply=False)
            time.sleep(3.0)
            self.send(":Qw#", expect_reply=False)
            time.sleep(0.5)
            pos_after = self.get_pagp()[0]
            drift = abs(pos_after - pos_before)
            self.log("Re-slew west blocked at limit",
                     drift < 2.0,
                     f"before={pos_before:.2f}° after={pos_after:.2f}° drift={drift:.2f}°")

            # ── 3. :Te# and :MS# while errorWest active ──────────────────────
            # Sidereal moves axis1 forward (west limit = axis1.max) -> PAST
            # limit -> Limits::poll re-trips within 100ms -> tracking off.
            self.assert_te_ms_blocked("errorWest", expect_te_blocked=True)

            # ── 4. recovery: slew east moves motor away from limit ──────────
            self.step("Slewing east (recovery, away from west limit)...")
            pos_before = self.get_pagp()[0]
            self.send(":Me#", expect_reply=False)
            time.sleep(1.0)
            self.send(":Qe#", expect_reply=False)
            time.sleep(0.5)
            pos_after = self.get_pagp()[0]
            recovered = pos_before - pos_after
            self.log("Recovery: east slew moved motor (away from west)",
                     recovered > 5.0,
                     f"before={pos_before:.2f}° after={pos_after:.2f}° Δ={-recovered:+.2f}°")

            # ── 5. error clears once back in range ──────────────────────────
            time.sleep(0.3)
            h, e, w, hz = self.get_pagr()
            in_range = pos_after <= limit
            if in_range:
                self.log("errorWest clears once motor back in range",
                         not w, f"PAGr h={h} e={e} w={w} r={hz}")
                self.send(":Te#", wait_ms=200)
                self.log("Tracking re-enables after recovery", self.is_tracking())
            else:
                self.warn(f"Recovery didn't clear west limit zone (final={pos_after:.2f}° vs limit={limit:.2f}°), skipping clear-check")
        finally:
            self.send(":Qe#", expect_reply=False)
            self.send(":Qw#", expect_reply=False)
            self.send(":Q#",  expect_reply=False)
            self.send(":Td#", expect_reply=False)
            self.restore_amp_defaults()

    # Probe rate for the 4-direction blocked/allowed assertions. R7 = 48x sidereal
    # = ~0.2°/s. Pulse 3s so axis1 motion near the meridian (small sin(HA), low
    # alt sensitivity ~0.05°/°) has time to drop alt past worsening hyst (0.02°
    # = 0.4° axis1 motion at HA~3°): allowed shows ~0.6° (full pulse), blocked
    # shows ~0.45° (worsening detected ~2s in, decel adds 0.05°). Threshold 0.5°
    # separates. Axis2 probes detect within 100ms regardless (high sensitivity).
    PROBE_RATE_CMD = ":R7#"
    PROBE_PULSE_MS = 3000
    PROBE_THRESHOLD_DEG = 0.5

    # ── horizon trip suite ──────────────────────────────────────────────────
    # Pattern shared across all 4 horizon tests:
    #   home -> Te (sets PIER=E) -> R9 -> GOTO deterministic target above
    #   horizon -> snapshot pier via :Gm# -> arm tight horizon -> trip via R9
    #   slew into limit -> 4-direction probes at PROBE_RATE_CMD with re-trip
    #   between unblocked probes (so each probe runs while errorHorizon=1) ->
    #   R9 recovery via slew_until_horizon_clears -> verify pier unchanged at
    #   trip / after probes / at end.
    # R7 (PROBE_RATE_CMD) is used only for probes; everything else is R9.

    # Geometry: lat=30, GOTO HA=0 Dec=-30 -> alt=30 at south meridian.
    # Trip via Ms drops dec, alt drops past horizon=29. At HA=0 the 1st-order
    # alt-derivative wrt axis1 is zero, so Me/Mw probes don't appreciably
    # worsen alt - they're allowed (not blocked) by the horizon limit.
    def test_horizon_trip_axis2_south(self):
        self._banner("Horizon trip via axis2 south (HA=0 Dec=-30 alt~30)", movement=True)
        try:
            self.setup_site()
            self.home()
            self.send(":Te#", wait_ms=200)
            self.log("Tracking enabled after home", self.is_tracking())

            self.send(":R9#")
            self.goto_ha(0.0, dec="-30:00:00",
                         label="pre-trip start (HA=0 Dec=-30 alt~30)")

            pier_ref = self.send(":Gm#", quiet=True).strip()
            self.info(f"Pier side at target: {pier_ref}")

            self.send(":PASh,29#")
            self.info("Armed horizon=29 (margin ~1)")

            h, e, w, hz = self.get_pagr()
            self.log("Pre-slew: no horizon error",
                     not hz, f"PAGr h={h} e={e} w={w} r={hz}")

            self.step("Slewing south (:Ms#) at :R9# until horizon trip...")
            self.send(":R9#")
            tripped = self.wait_for_horizon_trip(":Ms#", ":Qs#", max_wait=30)
            self.log("Slew south tripped errorHorizon", tripped)

            h, e, w, hz = self.get_pagr()
            self.log("PAGr errorHorizon set", hz, f"PAGr h={h} e={e} w={w} r={hz}")
            self.log("Tracking off after horizon trip", not self.is_tracking())
            self.log(f"Pier unchanged after trip ({pier_ref})",
                     self.send(":Gm#", quiet=True).strip() == pier_ref)

            p1, p2 = self.get_pagp()
            self.info(f"trip pos: axis1={p1:+.2f} axis2={p2:+.2f}")

            # At HA=0 Me/Mw barely change alt (2nd-order), so they're not
            # blocked. Only Ms is blocked. Mn raises alt and clears the limit;
            # re-trip via Ms before each subsequent probe.
            probes = [
                ('s', 1, True,  "continues south, worsens alt"),
                ('n', 1, False, "dec north, raises alt"),
                ('e', 0, False, "HA shift east of meridian (2nd-order, allowed)"),
                ('w', 0, False, "HA shift west of meridian (2nd-order, allowed)"),
            ]
            for dir_char, axis_idx, expect_blocked, label in probes:
                _, _, _, hz = self.get_pagr()
                if not hz:
                    self.step(f"  Re-tripping via :Ms# before M{dir_char.upper()} probe...")
                    self.send(":R9#")
                    self.wait_for_horizon_trip(":Ms#", ":Qs#", max_wait=30)
                self.send(self.PROBE_RATE_CMD)
                _, _, _, hz = self.get_pagr()
                self.log(f"In horizon limit before M{dir_char.upper()} probe", hz)
                self.probe_direction(dir_char, axis_idx, expect_blocked,
                                     pulse_ms=self.PROBE_PULSE_MS,
                                     threshold=self.PROBE_THRESHOLD_DEG,
                                     label=label)

            self.log(f"Pier unchanged after probes ({pier_ref})",
                     self.send(":Gm#", quiet=True).strip() == pier_ref)

            # :Te# / :MS# while errorHorizon active. At HA=0 sidereal barely
            # moves alt (2nd-order at meridian) -> tracking stays on. Re-trip
            # first so the checks run in-limit.
            _, _, _, hz = self.get_pagr()
            if not hz:
                self.step("  Re-tripping via :Ms# before Te/MS check...")
                self.send(":R9#")
                self.wait_for_horizon_trip(":Ms#", ":Qs#", max_wait=30)
            self.assert_te_ms_blocked("errorHorizon", expect_te_blocked=False)

            self.step("Recovering via :Mn# at :R9# until horizon clears...")
            p2_before, p2_after, cleared = self.slew_until_horizon_clears('n', 1, ":R9#")
            self.log("Mn recovery cleared horizon",
                     cleared,
                     f"before={p2_before:+.2f} after={p2_after:+.2f} d={p2_after-p2_before:+.2f}")

            time.sleep(0.5)
            h, e, w, hz = self.get_pagr()
            self.log("errorHorizon clear at end of test",
                     not hz, f"PAGr h={h} e={e} w={w} r={hz}")
            self.log(f"Pier unchanged at end ({pier_ref})",
                     self.send(":Gm#", quiet=True).strip() == pier_ref)
            self.send(":Te#", wait_ms=200)
            self.log("Tracking re-enables", self.is_tracking())
        finally:
            for q in (":Qe#", ":Qw#", ":Qn#", ":Qs#", ":Q#", ":Td#"):
                self.send(q, expect_reply=False)
            self.restore_amp_defaults()

    def test_horizon_trip_axis1_east(self):
        self._banner("Horizon trip via axis1 east (HA=-3h Dec=+41:25:02)", movement=True)
        try:
            self.setup_site()
            self.home()
            self.send(":Te#", wait_ms=200)
            self.log("Tracking enabled after home", self.is_tracking())

            self.send(":R9#")
            self.goto_ha(-3, dec="+41:25:02", label="pre-trip start (HA=-3h Dec=+41.4, alt~19.3)")

            self.send(":PASh,18#")
            self.info("Armed horizon=18 (alt margin ~1.3 at start)")

            h, e, w, hz = self.get_pagr()
            self.log("Pre-slew: no horizon error",
                     not hz, f"PAGr h={h} e={e} w={w} r={hz}")

            self.step("Slewing east at :R9# until horizon trip...")
            self.send(":R9#")
            tripped = self.wait_for_horizon_trip(":Me#", ":Qe#", max_wait=30)
            self.log("Slew east tripped errorHorizon", tripped)

            h, e, w, hz = self.get_pagr()
            self.log("Trip is horizon (not east limit)",
                     hz and not e, f"PAGr h={h} e={e} w={w} r={hz}")
            self.log("Tracking off after horizon trip", not self.is_tracking())

            p1, p2 = self.get_pagp()
            self.info(f"trip pos: axis1={p1:+.2f} axis2={p2:+.2f}")

            # Sidereal at HA<0 moves axis1 toward meridian; with the trip
            # axis1 deeper east of meridian, tracking can drop alt below
            # horizon again -> re-trip -> tracking off.
            self.assert_te_ms_blocked("errorHorizon", expect_te_blocked=True)

            self.step("Recovering via :Mw# at :R9# until horizon clears...")
            p1_before, p1_after, cleared = self.slew_until_horizon_clears('w', 0, ":R9#")
            self.log("Mw recovery cleared horizon",
                     cleared,
                     f"before={p1_before:+.2f} after={p1_after:+.2f} d={p1_after-p1_before:+.2f}")

            time.sleep(0.5)
            h, e, w, hz = self.get_pagr()
            self.log("errorHorizon clear at end of test",
                     not hz, f"PAGr h={h} e={e} w={w} r={hz}")
            self.send(":Te#", wait_ms=200)
            self.log("Tracking re-enables", self.is_tracking())
        finally:
            for q in (":Qe#", ":Qw#", ":Q#", ":Td#"):
                self.send(q, expect_reply=False)
            self.restore_amp_defaults()

    def test_horizon_trip_axis1_west(self):
        self._banner("Horizon trip via axis1 west (HA=+3h Dec=+38:40:40, alt~20.7)", movement=True)
        try:
            self.setup_site()
            self.home()
            self.send(":Te#", wait_ms=200)
            self.log("Tracking enabled after home", self.is_tracking())

            self.send(":R9#")
            self.goto_ha(3, dec="+38:40:40", label="pre-trip start (HA=+3h Dec=+38.7, alt~20.7)")

            self.send(":PASh,19#")
            self.info("Armed horizon=19 (alt margin ~1.7 at start)")

            h, e, w, hz = self.get_pagr()
            self.log("Pre-slew: no horizon error",
                     not hz, f"PAGr h={h} e={e} w={w} r={hz}")

            self.step("Slewing west at :R9# until horizon trip...")
            self.send(":R9#")
            tripped = self.wait_for_horizon_trip(":Mw#", ":Qw#", max_wait=30)
            self.log("Slew west tripped errorHorizon", tripped)

            h, e, w, hz = self.get_pagr()
            self.log("Trip is horizon (not west limit)",
                     hz and not w, f"PAGr h={h} e={e} w={w} r={hz}")
            self.log("Tracking off after horizon trip", not self.is_tracking())

            p1, p2 = self.get_pagp()
            self.info(f"trip pos: axis1={p1:+.2f} axis2={p2:+.2f}")

            # Sidereal at HA>0 moves axis1 west, away from meridian; with the
            # trip axis1 well west of meridian, tracking drops alt -> re-trip
            # -> tracking off.
            self.assert_te_ms_blocked("errorHorizon", expect_te_blocked=True)

            self.step("Recovering via :Me# at :R9# until horizon clears...")
            p1_before, p1_after, cleared = self.slew_until_horizon_clears('e', 0, ":R9#")
            self.log("Me recovery cleared horizon",
                     cleared,
                     f"before={p1_before:+.2f} after={p1_after:+.2f} d={p1_after-p1_before:+.2f}")

            time.sleep(0.5)
            h, e, w, hz = self.get_pagr()
            self.log("errorHorizon clear at end of test",
                     not hz, f"PAGr h={h} e={e} w={w} r={hz}")
            self.send(":Te#", wait_ms=200)
            self.log("Tracking re-enables", self.is_tracking())
        finally:
            for q in (":Qe#", ":Qw#", ":Q#", ":Td#"):
                self.send(q, expect_reply=False)
            self.restore_amp_defaults()

    # Geometry: lat=30, GOTO HA=-6h Dec=+63 -> alt~26.5 (cos(HA)=0 so
    # alt=arcsin(sin(lat)*sin(dec))). Off-meridian by 6h and well clear of the
    # pole, so pier side stays put through trip and probes (no flip).
    # Trip via Ms (drops dec, alt drops past horizon=25). Probes test all 4
    # directions while errorHorizon=1: Ms/Me worsen alt (expect blocked),
    # Mn/Mw raise alt (expect motion). After each unblocked probe the limit
    # may clear; re-trip via Ms before the next probe so every probe runs
    # in-limit. Recovery: Mn at R9 until errorHorizon clears.
    def test_horizon_trip_axis2_north(self):
        self._banner("Horizon trip 4-dir probes (HA=-6h Dec=+63 alt~26.5)", movement=True)
        try:
            self.setup_site()
            self.home()
            self.send(":Te#", wait_ms=200)
            self.log("Tracking enabled after home", self.is_tracking())

            self.send(":R9#")
            self.goto_ha(-6.0, dec="+63:00:00",
                         label="pre-trip start (HA=-6 Dec=+63 alt~26.5)")

            pier_ref = self.send(":Gm#", quiet=True).strip()
            self.info(f"Pier side at target: {pier_ref}")

            self.send(":PASh,25#")
            self.info("Armed horizon=25 (margin ~1.5)")

            h, e, w, hz = self.get_pagr()
            self.log("Pre-slew: no horizon error",
                     not hz, f"PAGr h={h} e={e} w={w} r={hz}")

            self.step("Slewing south (:Ms#) at :R9# until horizon trip...")
            self.send(":R9#")
            tripped = self.wait_for_horizon_trip(":Ms#", ":Qs#", max_wait=30)
            self.log("Slew south tripped errorHorizon", tripped)

            h, e, w, hz = self.get_pagr()
            self.log("PAGr errorHorizon set", hz, f"PAGr h={h} e={e} w={w} r={hz}")
            self.log("Tracking off after horizon trip", not self.is_tracking())
            self.log(f"Pier unchanged after trip ({pier_ref})",
                     self.send(":Gm#", quiet=True).strip() == pier_ref)

            p1, p2 = self.get_pagp()
            self.info(f"trip pos: axis1={p1:+.2f} axis2={p2:+.2f}")

            # Each probe runs while errorHorizon=1. Unblocked probes (Mn/Mw)
            # may clear the limit; re-trip via Ms before the next one.
            probes = [
                ('s', 1, True,  "continues south, worsens alt"),
                ('e', 0, True,  "HA more east, worsens alt at HA=-6h"),
                ('n', 1, False, "dec north, raises alt"),
                ('w', 0, False, "HA toward meridian, raises alt"),
            ]
            for dir_char, axis_idx, expect_blocked, label in probes:
                _, _, _, hz = self.get_pagr()
                if not hz:
                    self.step(f"  Re-tripping via :Ms# before M{dir_char.upper()} probe...")
                    self.send(":R9#")
                    self.wait_for_horizon_trip(":Ms#", ":Qs#", max_wait=30)
                self.send(self.PROBE_RATE_CMD)
                _, _, _, hz = self.get_pagr()
                self.log(f"In horizon limit before M{dir_char.upper()} probe", hz)
                self.probe_direction(dir_char, axis_idx, expect_blocked,
                                     pulse_ms=self.PROBE_PULSE_MS,
                                     threshold=self.PROBE_THRESHOLD_DEG,
                                     label=label)

            self.log(f"Pier unchanged after probes ({pier_ref})",
                     self.send(":Gm#", quiet=True).strip() == pier_ref)

            # Sidereal at HA=-6h moves axis1 toward meridian -> alt rises ->
            # AWAY from horizon -> tracking stays on.
            _, _, _, hz = self.get_pagr()
            if not hz:
                self.step("  Re-tripping via :Ms# before Te/MS check...")
                self.send(":R9#")
                self.wait_for_horizon_trip(":Ms#", ":Qs#", max_wait=30)
            self.assert_te_ms_blocked("errorHorizon", expect_te_blocked=False)

            self.step("Recovering via :Mn# at :R9# until horizon clears...")
            p2_before, p2_after, cleared = self.slew_until_horizon_clears('n', 1, ":R9#")
            self.log("Mn recovery cleared horizon",
                     cleared,
                     f"before={p2_before:+.2f} after={p2_after:+.2f} d={p2_after-p2_before:+.2f}")

            time.sleep(0.5)
            h, e, w, hz = self.get_pagr()
            self.log("errorHorizon clear at end of test",
                     not hz, f"PAGr h={h} e={e} w={w} r={hz}")
            self.log(f"Pier unchanged at end ({pier_ref})",
                     self.send(":Gm#", quiet=True).strip() == pier_ref)
            self.send(":Te#", wait_ms=200)
            self.log("Tracking re-enables", self.is_tracking())
        finally:
            for q in (":Qe#", ":Qw#", ":Qn#", ":Qs#", ":Q#", ":Td#"):
                self.send(q, expect_reply=False)
            self.restore_amp_defaults()

    def test_manual_reanchor_zeros_drift(self):
        self._banner("PASz after big sync -> PAGd ~ 0", movement=True)
        self.setup_site()
        try:
            self.send(":PASt,10#")   # threshold = 10°
            self.home()
            self.goto_ha(+1.0, label="position pre-sync")
            self.sync_to_offset_ra(20.0)  # well above threshold
            d_after_sync = self.get_pagd()
            self.log("Drift > threshold introduced",
                     abs(d_after_sync[0]) > 10.0, f"d1={d_after_sync[0]:.4f}°")

            r = self.send(":PASz#")
            self.info(f":PASz# reply='{r}'")
            d_after_z = self.get_pagd()
            self.log("PAGd ~ 0 after manual re-anchor",
                     abs(d_after_z[0]) < 1.0 and abs(d_after_z[1]) < 1.0,
                     f"d1={d_after_z[0]:.4f}° d2={d_after_z[1]:.4f}°")
        finally:
            self.restore_amp_defaults()

    # ── runner ──────────────────────────────────────────────────────────────
    def run_suite(self, suite):
        if suite not in ('basic', 'movement', 'limits', 'all'):
            print(f"{Fore.RED}Unknown suite: {suite}")
            return

        keys = [k for k, (_, _, s) in self.available_tests.items()
                if suite == 'all' or s == suite]
        if not keys:
            print(f"{Fore.YELLOW}No tests in suite '{suite}'")
            return

        has_movement = any(self.available_tests[k][2] in ('movement', 'limits') for k in keys)
        if has_movement and not self.auto_confirm:
            print(f"\n{Fore.RED}{'!'*60}")
            print(f"{Fore.RED}WARNING: tests in suite '{suite}' move the mount.")
            print(f"{Fore.RED}{'!'*60}")
            if input(f"{Fore.YELLOW}Continue? (yes/no): ").strip().lower() != 'yes':
                return

        self._run_keys(keys)

    def run_one(self, key):
        if key not in self.available_tests:
            print(f"{Fore.RED}Unknown test: {key}. Use --list-tests")
            return
        suite = self.available_tests[key][2]
        if suite in ('movement', 'limits') and not self.auto_confirm:
            print(f"\n{Fore.RED}WARNING: this test moves the mount.")
            if input(f"{Fore.YELLOW}Continue? (yes/no): ").strip().lower() != 'yes':
                return
        self._run_keys([key])

    def run_last_failed(self):
        keys = self._load_last_failed()
        if not keys:
            print(f"{Fore.YELLOW}No previously failed tests found.")
            return
        print(f"\n{Fore.YELLOW}Re-running {len(keys)} previously failed test(s):")
        for k in keys:
            name = self.available_tests[k][0]
            print(f"  {Fore.CYAN}{k:36s} - {name}")

        has_movement = any(self.available_tests[k][2] in ('movement', 'limits') for k in keys)
        if has_movement and not self.auto_confirm:
            print(f"\n{Fore.RED}WARNING: some failed tests move the mount.")
            if input(f"{Fore.YELLOW}Continue? (yes/no): ").strip().lower() != 'yes':
                return
        self._run_keys(keys)

    def _run_keys(self, keys: List[str]):
        for k in keys:
            print(f"\n{Fore.YELLOW}>>> {k}")
            self._current_test_key = k
            try:
                self.available_tests[k][1]()
            except Exception as e:
                import traceback
                traceback.print_exc()
                self.log(f"{k} raised", False, str(e))
            self._current_test_key = None
            print(f"{Fore.YELLOW}<<< {k}")
        self.summary()
        self._save_last_failed()

    def _save_last_failed(self):
        failed_keys = sorted({
            r['test_key'] for r in self.results
            if not r['ok'] and r.get('test_key')
        })
        if failed_keys:
            with open(LAST_FAILED_FILE, 'w') as f:
                json.dump(failed_keys, f)
            print(f"{Fore.YELLOW}Failed test keys saved. Re-run with --last-failed")
        elif os.path.exists(LAST_FAILED_FILE):
            os.remove(LAST_FAILED_FILE)

    def _load_last_failed(self) -> List[str]:
        if not os.path.exists(LAST_FAILED_FILE):
            return []
        try:
            with open(LAST_FAILED_FILE) as f:
                keys = json.load(f)
        except (OSError, json.JSONDecodeError):
            return []
        return [k for k in keys if k in self.available_tests]

    def list_tests(self):
        print(f"\n{Fore.CYAN}Available AMP tests:")
        by_suite = {}
        for k, (name, _, suite) in self.available_tests.items():
            by_suite.setdefault(suite, []).append((k, name))
        for suite in ('basic', 'movement', 'limits'):
            print(f"\n  {Fore.YELLOW}[{suite}]")
            for k, name in by_suite.get(suite, []):
                print(f"    {Fore.YELLOW}{k:36s} {Fore.CYAN}- {name}")

    def summary(self):
        if not self.results:
            return
        passed = sum(1 for r in self.results if r['ok'])
        failed = len(self.results) - passed
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{Fore.CYAN}Summary: {passed}/{len(self.results)} passed, {failed} failed")
        print(f"{Fore.CYAN}{'='*60}")
        if failed:
            print(f"{Fore.RED}Failed:")
            for r in self.results:
                if not r['ok']:
                    print(f"  {Fore.RED}✗ {r['name']}")
                    if r['details']:
                        print(f"      {Fore.CYAN}{r['details']}")


def main():
    ap = argparse.ArgumentParser(description='AMP serial tests')
    ap.add_argument('--port', help='Serial port (e.g. COM5)')
    ap.add_argument('--baud', type=int, default=9600)
    ap.add_argument('--timeout', type=float, default=2.0)
    ap.add_argument('--suite', default='all', choices=('basic', 'movement', 'limits', 'all'))
    ap.add_argument('--test', help='Run a single test by key')
    ap.add_argument('--list-tests', action='store_true')
    ap.add_argument('--last-failed', action='store_true', help='Re-run tests that failed in the last run')
    ap.add_argument('--no-wait-confirm', action='store_true')
    ap.add_argument('--quiet-serial', action='store_true')
    args = ap.parse_args()

    t = AmpTester(args.port or 'COM3', args.baud, args.timeout,
                  auto_confirm=args.no_wait_confirm,
                  verbose_serial=not args.quiet_serial)

    if args.list_tests:
        t.list_tests()
        return 0

    if not args.port:
        print(f"{Fore.RED}--port required")
        return 1

    if not t.connect():
        return 1
    try:
        if args.last_failed:
            t.run_last_failed()
        elif args.test:
            t.run_one(args.test)
        else:
            t.run_suite(args.suite)
    finally:
        t.disconnect()
    return 0


if __name__ == '__main__':
    sys.exit(main())
