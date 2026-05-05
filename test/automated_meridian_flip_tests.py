#!/usr/bin/env python3
"""
Automated serial tests for meridian flip redesign.

Goal: verify pier-side selection on GEM for PSS_BEST GOTO follows HA sign
(meridian static at HA=0), and that pastMeridianE/W act only as motion
limits - not as reachability/selection narrowing.

Tests:
  case_1_at_meridian        : HA=0,  curr=WEST, BEST -> EAST
  case_2_past_meridian      : HA=+1, curr=WEST, BEST -> EAST
  case_4_before_meridian    : HA=-1, curr=WEST, BEST -> stay WEST
  case_5_already_east       : HA=+1, curr=EAST, BEST -> stay EAST
  static_meridian_vs_pastW  : pastMeridianW=+15, HA=+2 on WEST -> flip EAST
  boundary_ha_zero          : HA=0 exactly -> EAST
  boundary_ha_neg_small     : HA=-0.1 (~-24s RA) -> WEST
  sync_preserves_pier       : sync not-at-home does not flip
  home_sync_preferredW_ha_pos : home+GEM sync at HA=+1 with preferredPierSide=W → EAST (HA-rule overrides)
  home_sync_preferredE_ha_neg : home+GEM sync at HA=-1 with preferredPierSide=E → WEST (HA-rule overrides)

Prereq:
  - GEM mount, stepper, no encoders (user default)
  - PIER_SIDE_SYNC_CHANGE_SIDES = OFF (default)
  - Clearance around mount - tests slew across meridian
  - Mount homed on startup (EAST pier by convention)

OnStepX commands used:
  :Sr# :Sd# :MS# :CM# :GR# :GD# :GS# :Gm# :GU# :hF# :hC# :Te# :Td# :Q#
  :GXE9# / :SXE9,n#  pastMeridianE (degrees)
  :GXEA# / :SXEA,n#  pastMeridianW (degrees)

  NOTE: :SXE9 / :SXEA command forms may vary by OnStepX build. If
  set_past_meridian_* fails, adjust the helper.

Usage:
  python test/automated_meridian_flip_tests.py --port COM5 --baud 9600
  python test/automated_meridian_flip_tests.py --port COM5 --test case_2_past_meridian
  python test/automated_meridian_flip_tests.py --port COM5 --list-tests
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
    os.path.dirname(os.path.abspath(__file__)), '.meridian_last_failed.json'
)


TEST_DEC_NORTH = "+45:00:00"   # safe Dec for all HAs, stays well above horizon
DEFAULT_LAT = "+30*00"
DEFAULT_LON = "-034*00"
SLEW_TIMEOUT = 90
HOME_TIMEOUT = 120


class MeridianFlipTester:
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
        self.available_tests = {
            'case_1_at_meridian':       ('Case 1: HA=0 WEST -> EAST',         self.test_case_1_at_meridian),
            'case_2_past_meridian':     ('Case 2: HA=+1 WEST -> EAST',        self.test_case_2_past_meridian),
            'case_4_before_meridian':   ('Case 4: HA=-1 WEST -> stay WEST',   self.test_case_4_before_meridian),
            'case_5_already_east':      ('Case 5: HA=+1 EAST -> stay EAST',   self.test_case_5_already_east),
            'static_meridian_vs_pastW': ('B1: static meridian vs pastMeridianW=+15', self.test_static_meridian_vs_pastW),
            'boundary_ha_zero':         ('B2: HA=0 boundary -> EAST',         self.test_boundary_ha_zero),
            'boundary_ha_neg_small':    ('B3: HA=-0.1 -> WEST',               self.test_boundary_ha_neg_small),
            'sync_preserves_pier':      ('Sync: not-home preserves pier',     self.test_sync_preserves_pier),
            'home_sync_preferredW_ha_pos': ('Home sync: preferredW + HA=+1 → EAST (guardrail)', self.test_home_sync_preferredW_ha_pos),
            'home_sync_preferredE_ha_neg': ('Home sync: preferredE + HA=-1 → WEST (guardrail)', self.test_home_sync_preferredE_ha_neg),
        }

    # --- serial plumbing ---
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
                print(f"  {Fore.BLUE}→ {cmd}  {Fore.CYAN}(no reply expected)")
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
            print(f"  {Fore.BLUE}→ {cmd}  {Fore.CYAN}← '{reply}'")
        return reply

    # --- logging ---
    def log(self, name, ok, details=""):
        tag = f"{Fore.GREEN}PASS" if ok else f"{Fore.RED}FAIL"
        print(f"  {tag} - {name}")
        if details:
            print(f"       {Fore.CYAN}{details}")
        self.results.append({
            'name': name, 'ok': ok, 'details': details,
            'test_key': self._current_test_key,
        })

    def info(self, msg):
        print(f"  {Fore.CYAN}ℹ {msg}")

    def warn(self, msg):
        print(f"  {Fore.YELLOW}⚠ {msg}")

    def step(self, msg):
        print(f"  {Fore.YELLOW}⏳ {msg}")

    def log_state(self, label="state"):
        """Dump RA, Dec, HA, pier, LST, status - crucial for failure diagnosis."""
        print(f"  {Fore.MAGENTA}── mount state [{label}] ──")
        ra = self.send(":GR#", quiet=True)
        dec = self.send(":GD#", quiet=True)
        pier = self.send(":Gm#", quiet=True)
        lst_s = self.send(":GS#", quiet=True)
        status = self.send(":GU#", quiet=True)
        ha_str = "?"
        try:
            lst_h = self._parse_hms(lst_s)
            ra_h = self._parse_hms(ra)
            ha = (lst_h - ra_h + 12.0) % 24.0 - 12.0
            ha_str = f"{ha:+.3f}h"
        except Exception:
            pass
        print(f"    {Fore.MAGENTA}RA={ra}  Dec={dec}  HA={ha_str}  pier={pier}  LST={lst_s}  status={status}")

    @staticmethod
    def _parse_hms(s):
        p = s.split(':')
        return int(p[0]) + int(p[1]) / 60.0 + float(p[2]) / 3600.0

    # --- helpers ---
    def get_lst_hours(self):
        s = self.send(":GS#", quiet=True)
        self.info(f"LST query: :GS# → '{s}'")
        return self._parse_hms(s)

    def ha_to_ra_str(self, ha_hours):
        ra = (self.lst - ha_hours) % 24.0
        h = int(ra)
        m = int((ra - h) * 60)
        s = int(((ra - h) * 60 - m) * 60)
        ra_s = f"{h:02d}:{m:02d}:{s:02d}"
        self.info(f"ha_to_ra: HA={ha_hours:+.4f}h, LST={self.lst:.4f}h → RA={ra_s}")
        return ra_s

    def setup_site(self, lat=DEFAULT_LAT):
        self.step(f"Setting site lat={lat}, lon={DEFAULT_LON}")
        r1 = self.send(f":St{lat}#")
        r2 = self.send(f":Sg{DEFAULT_LON}#")
        self.info(f":St reply='{r1}'  :Sg reply='{r2}'")
        self.lst = self.get_lst_hours()
        self.info(f"Cached LST = {self.lst:.4f}h")

    def home(self):
        """Set home (:hF#). Pier reads 'N' until next GOTO commits it - expected."""
        self.step("Tracking off (:Td#) then set home (:hF#)")
        self.send(":Td#", wait_ms=100)
        self.send(":hF#")
        ok = self.wait_for_slew(HOME_TIMEOUT)
        self.info(f"Home complete: {ok}  (pier='N' at home is expected)")
        self.log_state("after-home")

    def wait_for_slew(self, max_wait=SLEW_TIMEOUT):
        self.step(f"Waiting for slew (up to {max_wait}s)...")
        time.sleep(0.5)
        t0 = time.time()
        last_status = ""
        while time.time() - t0 < max_wait:
            st = self.send(":GU#", quiet=True)
            if st != last_status:
                print(f"    {Fore.CYAN}status: '{st}'  (t+{time.time()-t0:.1f}s)")
                last_status = st
            if st and 'G' not in st and 'h' not in st and ('N' in st or 'n' in st):
                print(f"  {Fore.GREEN}✓ Slew done in {time.time()-t0:.1f}s [status='{st}']")
                return True
            time.sleep(0.4)
        print(f"  {Fore.RED}✗ Slew timeout after {max_wait}s [last status='{last_status}']")
        return False

    def pier(self, quiet=False):
        p = self.send(":Gm#", quiet=quiet)
        if not quiet:
            self.info(f"pier side: '{p}'  (E=east, W=west, N=none)")
        return p

    def goto_ha(self, ha_hours, dec=TEST_DEC_NORTH, label=""):
        """Issue GOTO to a target at given HA, wait, return final pier."""
        tag = f" [{label}]" if label else ""
        self.step(f"GOTO to HA={ha_hours:+.4f}h, Dec={dec}{tag}")
        ra = self.ha_to_ra_str(ha_hours)
        self.info(f"Target: RA={ra}  Dec={dec}")
        r_sr = self.send(f":Sr{ra}#")
        r_sd = self.send(f":Sd{dec}#")
        self.info(f":Sr reply='{r_sr}'  :Sd reply='{r_sd}'")
        self.log_state("pre-MS")
        r = self.send(":MS#")
        self.info(f":MS# reply='{r}'  ({'ACCEPTED' if r == '0' else 'REJECTED'})")
        if r != '0':
            self.warn(f"GOTO rejected with code '{r}' - returning early")
            return None, r
        self.wait_for_slew()
        p = self.pier(quiet=True)
        self.log_state(f"post-GOTO-HA={ha_hours:+.2f}h")
        self.info(f"Final pier after GOTO: '{p}'")
        return p, r

    def set_past_meridian_e(self, deg):
        self.step(f"Set pastMeridianE = {deg}°")
        r = self.send(f":SXE9,{deg}#")
        self.info(f":SXE9,{deg}# reply='{r}'")
        return r

    def set_past_meridian_w(self, deg):
        self.step(f"Set pastMeridianW = {deg}°")
        r = self.send(f":SXEA,{deg}#")
        self.info(f":SXEA,{deg}# reply='{r}'")
        return r

    def get_past_meridian_e(self):
        r = self.send(":GXE9#")
        self.info(f"pastMeridianE reply='{r}'")
        return r

    def get_past_meridian_w(self):
        r = self.send(":GXEA#")
        self.info(f"pastMeridianW reply='{r}'")
        return r

    def position_on_west_pier(self, ha_hours=-2.0):
        """Put the mount on WEST pier by slewing to HA<0 (BEST -> WEST rule)."""
        print(f"\n  {Fore.YELLOW}── Positioning on WEST pier (home then GOTO to HA={ha_hours:+.2f}h) ──")
        self.home()
        pier, r = self.goto_ha(ha_hours, label="position-WEST")
        if pier != 'W':
            self.warn(f"Expected WEST after slew to HA={ha_hours}, got pier='{pier}' (rc='{r}')")
        else:
            self.info(f"WEST pier confirmed (pier='{pier}')")
        return pier

    def position_on_east_pier(self):
        """Home the mount - EAST is home convention. pier='N' is accepted
        (OnStepX doesn't commit pier until first GOTO/tracking)."""
        print(f"\n  {Fore.YELLOW}── Positioning on EAST pier (home) ──")
        self.home()
        p = self.pier()
        if p in ('E', 'N'):
            self.info(f"Home reached (pier='{p}'; 'N' = not-yet-committed, EAST by convention)")
        else:
            self.warn(f"Unexpected pier at home: '{p}' (expected E or N)")
        return p

    def set_preferred_pier_side(self, side):
        """side ∈ {'E','W','B'}. Affects settings.preferredPierSide used by sync/goto."""
        self.step(f"Set preferredPierSide = {side}")
        r = self.send(f":SX96,{side}#")
        self.info(f":SX96,{side}# reply='{r}'")
        return r

    def reset_past_meridian(self):
        self.step("Restoring pastMeridianE/W defaults (5°/5°)")
        self.set_past_meridian_e(5)
        self.set_past_meridian_w(5)

    # --- tests ---
    def _banner(self, title):
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{Fore.CYAN}{title}")
        print(f"{Fore.CYAN}{'='*60}")
        print(f"  {Fore.YELLOW}⚠ This test moves the mount across meridian!")

    def test_case_1_at_meridian(self):
        self._banner("Case 1: HA=0, WEST, PSS_BEST -> EAST")
        self.setup_site()
        self.reset_past_meridian()
        self.position_on_west_pier(-1.5)
        start_pier = self.pier()
        self.log("Positioned WEST pier", start_pier == 'W', f"pier='{start_pier}'")
        if start_pier != 'W':
            self.warn("Cannot continue - did not reach WEST pier")
            return

        self.info("Expecting: GOTO to HA=0 flips to EAST (HA-rule: h<0 is false → PSS_EAST)")
        self.lst = self.get_lst_hours()
        pier, rc = self.goto_ha(0.0, label="target HA=0 (meridian)")
        self.log("GOTO accepted (MS=0)", rc == '0', f":MS# reply='{rc}'")
        self.log("Flipped to EAST", pier == 'E',
                 f"start pier='W', final pier='{pier}'")

    def test_case_2_past_meridian(self):
        self._banner("Case 2: HA=+1h, WEST, PSS_BEST -> EAST (distance fallback must NOT keep us on WEST)")
        self.setup_site()
        self.reset_past_meridian()
        self.position_on_west_pier(-1.5)
        start_pier = self.pier()
        self.log("Positioned WEST pier", start_pier == 'W', f"pier='{start_pier}'")
        if start_pier != 'W':
            self.warn("Cannot continue - did not reach WEST pier")
            return

        self.info("Expecting: GOTO to HA=+1 flips to EAST despite WEST being physically closer.")
        self.info("If this test fails (pier='W'), the distance fallback gate is not working.")
        self.lst = self.get_lst_hours()
        pier, rc = self.goto_ha(+1.0, label="target HA=+1h (past meridian)")
        self.log("GOTO accepted", rc == '0', f":MS# reply='{rc}'")
        self.log("Flipped to EAST despite WEST being closer", pier == 'E',
                 f"start pier='W', final pier='{pier}'")

    def test_case_4_before_meridian(self):
        self._banner("Case 4: HA=-1h, WEST, PSS_BEST -> stay WEST")
        self.setup_site()
        self.reset_past_meridian()
        self.position_on_west_pier(-2.0)
        start_pier = self.pier()
        self.log("Positioned WEST pier", start_pier == 'W', f"pier='{start_pier}'")
        if start_pier != 'W':
            self.warn("Cannot continue")
            return

        self.info("Expecting: HA=-1 keeps WEST (HA<0 → PSS_WEST)")
        self.lst = self.get_lst_hours()
        pier, rc = self.goto_ha(-1.0, label="target HA=-1h (east of meridian)")
        self.log("GOTO accepted", rc == '0', f":MS# reply='{rc}'")
        self.log("Stayed on WEST", pier == 'W',
                 f"start pier='W', final pier='{pier}'")

    def test_case_5_already_east(self):
        self._banner("Case 5: HA=+1h, EAST, PSS_BEST -> stay EAST")
        self.setup_site()
        self.reset_past_meridian()
        self.position_on_east_pier()
        start_pier = self.pier()
        self.log("Positioned EAST pier (home)", start_pier in ('E', 'N'),
                 f"pier='{start_pier}' (N=uncommitted home, both accepted)")
        if start_pier not in ('E', 'N'):
            self.warn("Cannot continue - not at home")
            return

        self.info("Expecting: HA=+1 on EAST pier stays EAST (no flip)")
        self.lst = self.get_lst_hours()
        pier, rc = self.goto_ha(+1.0, label="target HA=+1h")
        self.log("GOTO accepted", rc == '0', f":MS# reply='{rc}'")
        self.log("Stayed on EAST", pier == 'E',
                 f"start pier='E', final pier='{pier}'")

    def test_static_meridian_vs_pastW(self):
        self._banner("B1: pastMeridianW=+15, HA=+2 on WEST, PSS_BEST -> flip EAST (meridian static)")
        self.setup_site()
        self.info("Expanding pastMeridianW to +15° - meridian must stay static at HA=0 regardless")
        r1 = self.set_past_meridian_w(15)
        self.log("set pastMeridianW=15", r1 == '1' or r1 == '', f"reply='{r1}' (empty OK on some builds)")
        self.set_past_meridian_e(5)
        self.get_past_meridian_w()
        self.get_past_meridian_e()

        self.position_on_west_pier(-1.0)
        start_pier = self.pier()
        self.log("Positioned WEST pier", start_pier == 'W', f"pier='{start_pier}'")
        if start_pier != 'W':
            self.warn("Cannot continue")
            self.reset_past_meridian()
            return

        self.info("Expecting: HA=+2 on WEST (within pastW=+15) still flips EAST - meridian is static at HA=0")
        self.lst = self.get_lst_hours()
        pier, rc = self.goto_ha(+2.0, label="target HA=+2h (within pastW=15)")
        self.log("GOTO accepted", rc == '0', f":MS# reply='{rc}'")
        self.log("Flipped EAST (meridian static at HA=0, not shifted by pastW)", pier == 'E',
                 f"start pier='W', final pier='{pier}'")
        self.reset_past_meridian()

    def test_boundary_ha_zero(self):
        self._banner("B2: HA=0 exactly -> EAST")
        self.setup_site()
        self.reset_past_meridian()
        self.position_on_west_pier(-1.5)
        start_pier = self.pier()
        self.log("Positioned WEST pier", start_pier == 'W', f"pier='{start_pier}'")
        if start_pier != 'W':
            self.warn("Cannot continue")
            return

        self.info("Expecting: exact meridian boundary (HA=0) goes to EAST (h<0 is false)")
        self.lst = self.get_lst_hours()
        ra = self.ha_to_ra_str(0.0)
        self.info(f"Sending target RA={ra} (HA exactly 0)")
        r_sr = self.send(f":Sr{ra}#")
        r_sd = self.send(f":Sd{TEST_DEC_NORTH}#")
        self.info(f":Sr reply='{r_sr}'  :Sd reply='{r_sd}'")
        self.log_state("pre-MS")
        rc = self.send(":MS#")
        self.info(f":MS# reply='{rc}'")
        self.log("GOTO accepted", rc == '0', f"rc='{rc}'")
        if rc != '0':
            return
        self.wait_for_slew()
        p = self.pier()
        self.log_state("after-GOTO")
        self.log("Pier=EAST (boundary rule h<0 false)", p == 'E', f"pier='{p}'")

    def test_boundary_ha_neg_small(self):
        self._banner("B3: HA=-0.1h (~-6min RA) -> WEST")
        self.setup_site()
        self.reset_past_meridian()
        self.position_on_east_pier()
        start_pier = self.pier()
        self.log("Positioned EAST pier", start_pier in ('E', 'N'),
                 f"pier='{start_pier}' (N=uncommitted home, both accepted)")
        if start_pier not in ('E', 'N'):
            self.warn("Cannot continue - not at home")
            return

        self.info("Expecting: HA=-0.1 (just negative) flips WEST (HA<0 → PSS_WEST)")
        self.lst = self.get_lst_hours()
        pier, rc = self.goto_ha(-0.1, label="target HA=-0.1h")
        self.log("GOTO accepted", rc == '0', f":MS# reply='{rc}'")
        self.log("Flipped WEST (h<0)", pier == 'W',
                 f"start pier='E', final pier='{pier}'")

    def test_sync_preserves_pier(self):
        self._banner("Sync: not at home, PSS_BEST sync does not change pier")
        self.setup_site()
        self.reset_past_meridian()
        self.position_on_west_pier(-1.5)
        start_pier = self.pier()
        self.log("Positioned WEST pier", start_pier == 'W', f"pier='{start_pier}'")
        if start_pier != 'W':
            self.warn("Cannot continue")
            return

        self.info("Crafting a sync target at HA=+1h - HA-sign suggests EAST but this is a sync, not GOTO")
        self.info("PIER_SIDE_SYNC_CHANGE_SIDES=OFF forces PSS_SAME_ONLY for non-home sync, so pier stays WEST")
        self.lst = self.get_lst_hours()
        ra = self.ha_to_ra_str(+1.0)
        r_sr = self.send(f":Sr{ra}#")
        r_sd = self.send(f":Sd{TEST_DEC_NORTH}#")
        self.info(f":Sr reply='{r_sr}'  :Sd reply='{r_sd}'")
        self.log_state("pre-CM")
        r_cm = self.send(":CM#", wait_ms=200)
        self.info(f":CM# reply='{r_cm}'")
        time.sleep(0.5)
        self.log_state("post-CM")

        end_pier = self.pier()
        self.log("Pier unchanged by sync", end_pier == 'W',
                 f"before='W', after='{end_pier}' - sync must not flip pier when not at home")

    def _home_sync_guardrail(self, preferred, ha_hours, expect_pier):
        """At home+GEM, HA-rule must override preferredPierSide on sync."""
        self.setup_site()
        self.reset_past_meridian()
        self.position_on_east_pier()
        start_pier = self.pier()
        self.log("At home", start_pier in ('E', 'N'), f"pier='{start_pier}'")
        if start_pier not in ('E', 'N'):
            self.warn("Cannot continue - not at home")
            return

        self.set_preferred_pier_side(preferred)
        try:
            self.info(f"Sync target: HA={ha_hours:+.2f}h; preferredPierSide={preferred}; expect pier={expect_pier}")
            self.lst = self.get_lst_hours()
            ra = self.ha_to_ra_str(ha_hours)
            r_sr = self.send(f":Sr{ra}#")
            r_sd = self.send(f":Sd{TEST_DEC_NORTH}#")
            self.info(f":Sr reply='{r_sr}'  :Sd reply='{r_sd}'")
            self.log_state("pre-CM")
            r_cm = self.send(":CM#", wait_ms=200)
            self.info(f":CM# reply='{r_cm}'")
            time.sleep(0.5)
            self.log_state("post-CM")
            end_pier = self.pier()
            self.log(f"HA-rule overrode preferred={preferred}", end_pier == expect_pier,
                     f"expected='{expect_pier}' got='{end_pier}' (guardrail: home+GEM sync uses HA sign)")
        finally:
            self.set_preferred_pier_side('B')

    def test_home_sync_preferredW_ha_pos(self):
        self._banner("Home sync guardrail: preferredPierSide=W, HA=+1h → EAST")
        self._home_sync_guardrail('W', +1.0, 'E')

    def test_home_sync_preferredE_ha_neg(self):
        self._banner("Home sync guardrail: preferredPierSide=E, HA=-1h → WEST")
        self._home_sync_guardrail('E', -1.0, 'W')

    # --- runner ---
    def run_all(self):
        print(f"\n{Fore.RED}{'!'*60}")
        print(f"{Fore.RED}WARNING: these tests slew the mount across meridian.")
        print(f"{Fore.RED}Ensure clearance around OTA and cables.")
        print(f"{Fore.RED}{'!'*60}")
        if not self.auto_confirm:
            if input(f"{Fore.YELLOW}Continue? (yes/no): ").strip().lower() != 'yes':
                print("Cancelled")
                return
        self._run_keys(list(self.available_tests.keys()))

    def run_one(self, key):
        if key not in self.available_tests:
            print(f"{Fore.RED}Unknown test: {key}")
            print("Use --list-tests to see options")
            return
        if not self.auto_confirm:
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
            print(f"  {Fore.CYAN}{k:30s} - {name}")
        if not self.auto_confirm:
            print(f"\n{Fore.RED}WARNING: tests move the mount.")
            if input(f"{Fore.YELLOW}Continue? (yes/no): ").strip().lower() != 'yes':
                return
        self._run_keys(keys)

    def _run_keys(self, keys: List[str]):
        for key in keys:
            print(f"\n{Fore.YELLOW}>>> Starting test: {key}")
            self._current_test_key = key
            try:
                self.available_tests[key][1]()
            except Exception as e:
                import traceback
                traceback.print_exc()
                self.log(f"{key} raised", False, str(e))
            self._current_test_key = None
            print(f"{Fore.YELLOW}<<< Finished test: {key}\n")
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
        print(f"\n{Fore.CYAN}Available meridian-flip tests:")
        for k, (name, _) in self.available_tests.items():
            print(f"  {Fore.YELLOW}{k:30s} {Fore.CYAN}- {name}")

    def summary(self):
        if not self.results:
            return
        passed = sum(1 for r in self.results if r['ok'])
        failed = len(self.results) - passed
        print(f"\n{Fore.CYAN}{'='*60}")
        print(f"{Fore.CYAN}Summary: {passed}/{len(self.results)} passed, {failed} failed")
        print(f"{Fore.CYAN}{'='*60}")
        if failed:
            print(f"{Fore.RED}Failed assertions:")
            for r in self.results:
                if not r['ok']:
                    print(f"  {Fore.RED}✗ {r['name']}")
                    if r['details']:
                        print(f"      {Fore.CYAN}{r['details']}")


def main():
    ap = argparse.ArgumentParser(description='Meridian-flip serial tests')
    ap.add_argument('--port', required=False, help='Serial port')
    ap.add_argument('--baud', type=int, default=9600)
    ap.add_argument('--timeout', type=float, default=2.0)
    ap.add_argument('--test', help='Run a single test by key')
    ap.add_argument('--list-tests', action='store_true')
    ap.add_argument('--last-failed', action='store_true', help='Re-run tests that failed in the last run')
    ap.add_argument('--no-wait-confirm', action='store_true')
    ap.add_argument('--quiet-serial', action='store_true',
                    help='Suppress per-command serial echo (still logs high-level steps)')
    args = ap.parse_args()

    t = MeridianFlipTester(
        args.port or 'COM3',
        args.baud,
        args.timeout,
        auto_confirm=args.no_wait_confirm,
        verbose_serial=not args.quiet_serial,
    )

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
            t.run_all()
    finally:
        t.disconnect()
    return 0


if __name__ == '__main__':
    sys.exit(main())
