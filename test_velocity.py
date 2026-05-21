#!/usr/bin/env python3
"""
End-to-end tests for ADS-B velocity message encoding / decoding.

Coverage
--------
- Subtype selection (1 = subsonic <1022 kt, 2 = supersonic ≥1022 kt)
- CRC validity for a wide range of speeds and headings
- Speed round-trip fidelity within quantisation tolerance
  · subtype 1: 1 kt/LSB  → tolerance 2 kt  (diagonal worst-case ≈1.4 kt)
  · subtype 2: 4 kt/LSB  → tolerance 6 kt  (diagonal worst-case ≈5.7 kt)
- Track-angle round-trip fidelity (≤2° for speeds ≥1 kt)
- Saturation: inputs above 4 088 kt clamp to 4 088 kt
"""

import math
import unittest

import pyModeS as pms
from pyModeS.util import crc

from aircraft_emulator import build_velocity

_ICAO = "4840D6"

SUBTYPE1_TOL = 2   # kt — max error for 1 kt/LSB encoding at any heading
SUBTYPE2_TOL = 6   # kt — max error for 4 kt/LSB encoding at any heading
HEADING_TOL  = 2.0 # deg


def _decode(speed_kt, heading_deg, vrate=0, icao=_ICAO):
    """Build a velocity message and return the pyModeS decoded dict."""
    msg = build_velocity(icao, speed_kt, heading_deg, vrate)
    return pms.decode(msg.strip("*;"))


def _heading_diff(a, b):
    """Smallest angular difference between two headings (degrees)."""
    d = abs(a - b) % 360
    return d if d <= 180 else 360 - d


# ── Subtype selection ─────────────────────────────────────────────────────────

class TestSubtypeSelection(unittest.TestCase):
    """Correct subtype is chosen purely based on total speed."""

    def _sub(self, speed):
        return _decode(speed, 0.0)["subtype"]

    def test_0_kt_is_subtype1(self):
        self.assertEqual(self._sub(0), 1)

    def test_100_kt_is_subtype1(self):
        self.assertEqual(self._sub(100), 1)

    def test_480_kt_is_subtype1(self):
        self.assertEqual(self._sub(480), 1)

    def test_1021_kt_is_subtype1(self):
        self.assertEqual(self._sub(1021), 1)

    def test_1022_kt_switches_to_subtype2(self):
        self.assertEqual(self._sub(1022), 2)

    def test_1023_kt_is_subtype2(self):
        self.assertEqual(self._sub(1023), 2)

    def test_1500_kt_is_subtype2(self):
        self.assertEqual(self._sub(1500), 2)

    def test_4088_kt_is_subtype2(self):
        self.assertEqual(self._sub(4088), 2)

    def test_over_max_is_subtype2(self):
        self.assertEqual(self._sub(5000), 2)


# ── CRC validity ──────────────────────────────────────────────────────────────

class TestCRC(unittest.TestCase):
    """Every generated message must have a valid CRC."""

    def _assert_valid(self, speed, heading):
        msg = build_velocity(_ICAO, speed, heading, 0)
        raw = msg.strip("*;")
        self.assertEqual(crc(raw), 0,
                         f"CRC mismatch: speed={speed} hdg={heading}")
        self.assertTrue(_decode(speed, heading)["crc_valid"])

    def test_crc_grid(self):
        for speed in [0, 1, 100, 480, 1021, 1022, 1500, 4088, 5000]:
            for hdg in [0, 45, 90, 135, 180, 225, 270, 315]:
                with self.subTest(speed=speed, hdg=hdg):
                    self._assert_valid(speed, hdg)


# ── Speed fidelity ────────────────────────────────────────────────────────────

class TestSpeedFidelity(unittest.TestCase):
    """Decoded groundspeed matches input within quantisation tolerance."""

    def _assert_speed(self, speed_kt, heading_deg, tol):
        d = _decode(speed_kt, heading_deg)
        got = d["groundspeed"]
        self.assertIsNotNone(got)
        self.assertAlmostEqual(
            got, speed_kt, delta=tol,
            msg=f"speed={speed_kt} hdg={heading_deg}: decoded {got}"
        )

    # subsonic (1 kt/LSB) ──────────────────────────────────────────────────────

    def test_stationary_0_kt(self):
        self._assert_speed(0, 0.0, SUBTYPE1_TOL)

    def test_1_kt_north(self):
        self._assert_speed(1, 0.0, SUBTYPE1_TOL)

    def test_typical_prop_150_kt(self):
        self._assert_speed(150, 45.0, SUBTYPE1_TOL)

    def test_typical_jet_480_kt(self):
        self._assert_speed(480, 90.0, SUBTYPE1_TOL)

    def test_just_below_threshold_1021_kt(self):
        self._assert_speed(1021, 0.0, SUBTYPE1_TOL)

    # threshold ────────────────────────────────────────────────────────────────

    def test_threshold_exactly_1022_kt_cardinal(self):
        # Pure north: no diagonal loss, so decodes exactly
        self._assert_speed(1022, 0.0, SUBTYPE2_TOL)

    def test_threshold_exactly_1022_kt_diagonal(self):
        # Diagonal: each component loses up to 4 kt → total ≤ 5.7 kt off
        self._assert_speed(1022, 45.0, SUBTYPE2_TOL)

    def test_just_above_threshold_1023_kt(self):
        self._assert_speed(1023, 0.0, SUBTYPE2_TOL)

    # supersonic (4 kt/LSB) ───────────────────────────────────────────────────

    def test_supersonic_1500_kt_east(self):
        # Pure east: only one component → decodes exactly
        self._assert_speed(1500, 90.0, SUBTYPE2_TOL)

    def test_supersonic_1500_kt_diagonal(self):
        self._assert_speed(1500, 45.0, SUBTYPE2_TOL)

    def test_supersonic_2000_kt(self):
        self._assert_speed(2000, 135.0, SUBTYPE2_TOL)

    def test_max_encodable_4088_kt(self):
        self._assert_speed(4088, 0.0, SUBTYPE2_TOL)

    # saturation ───────────────────────────────────────────────────────────────

    def test_4089_kt_saturates_to_max(self):
        # 4089 kt overflows the 10-bit field; NS component clamps to mag=1023
        # heading=0 (pure north) → no diagonal loss → decodes exactly 4088 kt
        d = _decode(4089, 0.0)
        self.assertEqual(d["groundspeed"], 4088,
                         f"expected saturation to 4088, got {d['groundspeed']}")

    def test_5000_kt_saturates_to_max(self):
        # Same saturation: anything above 4088 kt encodes identically to 4088 kt
        d = _decode(5000, 0.0)
        self.assertEqual(d["groundspeed"], 4088,
                         f"expected saturation to 4088, got {d['groundspeed']}")

    # cardinal headings at a typical speed ────────────────────────────────────

    def test_cardinal_north_450_kt(self):
        self._assert_speed(450, 0.0, SUBTYPE1_TOL)

    def test_cardinal_east_450_kt(self):
        self._assert_speed(450, 90.0, SUBTYPE1_TOL)

    def test_cardinal_south_450_kt(self):
        self._assert_speed(450, 180.0, SUBTYPE1_TOL)

    def test_cardinal_west_450_kt(self):
        self._assert_speed(450, 270.0, SUBTYPE1_TOL)


# ── Track / heading fidelity ──────────────────────────────────────────────────

class TestHeadingFidelity(unittest.TestCase):
    """Decoded track angle matches input heading to within HEADING_TOL degrees."""

    def _assert_heading(self, speed_kt, heading_deg):
        if speed_kt == 0:
            return  # track undefined at 0 kt
        d = _decode(speed_kt, heading_deg)
        got = d["track"]
        self.assertIsNotNone(got)
        diff = _heading_diff(got, heading_deg)
        self.assertLessEqual(
            diff, HEADING_TOL,
            msg=f"speed={speed_kt} expected_hdg={heading_deg} decoded_hdg={got}"
        )

    def test_north_0_deg(self):
        self._assert_heading(450, 0.0)

    def test_northeast_45_deg(self):
        self._assert_heading(450, 45.0)

    def test_east_90_deg(self):
        self._assert_heading(450, 90.0)

    def test_southeast_135_deg(self):
        self._assert_heading(450, 135.0)

    def test_south_180_deg(self):
        self._assert_heading(450, 180.0)

    def test_southwest_225_deg(self):
        self._assert_heading(450, 225.0)

    def test_west_270_deg(self):
        self._assert_heading(450, 270.0)

    def test_northwest_315_deg(self):
        self._assert_heading(450, 315.0)

    def test_supersonic_heading_ne(self):
        self._assert_heading(1500, 45.0)

    def test_supersonic_heading_sw(self):
        self._assert_heading(1500, 225.0)


# ── Vertical rate round-trip ──────────────────────────────────────────────────

class TestVerticalRate(unittest.TestCase):
    """Vertical rate is encoded in 64 fpm steps; decoded value within 64 fpm."""

    _TOL = 64  # fpm — one LSB

    def _assert_vrate(self, vrate_fpm):
        d = _decode(450, 0.0, vrate=vrate_fpm)
        got = d["vertical_rate"]
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got, vrate_fpm, delta=self._TOL,
                               msg=f"vrate={vrate_fpm}: decoded {got}")

    def test_level_0_fpm(self):
        self._assert_vrate(0)

    def test_climb_1000_fpm(self):
        self._assert_vrate(1000)

    def test_descent_minus_1500_fpm(self):
        self._assert_vrate(-1500)

    def test_max_climb_32576_fpm(self):
        # 511 * 64 - 64 = 32 576 fpm (max encodable magnitude)
        self._assert_vrate(32576)


if __name__ == "__main__":
    unittest.main(verbosity=2)
