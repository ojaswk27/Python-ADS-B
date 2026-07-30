"""
Propagation channel between an aircraft and the radar site
=========================================================
The one place allowed to read aircraft ground truth on the receive path, and
only for geometry: slant range and line-of-sight.  Everything the display
eventually shows has to arrive as an encoded frame that survives this module.

Applied in order:

  1. maximum range
  2. radio horizon (both antenna heights)
  3. reply / frame probability
  4. round-trip delay      — IFF only, and the sole source of measured range
  5. garble                — IFF Modes A/C only, across one interrogation

Every deliver_* function returns None when the frame is lost.
"""

import math
import random
from dataclasses import dataclass

import iff_protocol as iff


# ── Site / channel configuration ──────────────────────────────────────────────

@dataclass
class RadarSite:
    """Where the interrogator is and how far it can hear."""
    lat:               float
    lon:               float
    ant_height_ft:     float = 100.0
    iff_max_range_nm:  float = 250.0
    adsb_max_range_nm: float = 250.0
    iff_reply_prob:    float = 0.97
    adsb_frame_prob:   float = 0.95
    # Monopulse azimuth estimate is good but not exact.
    brg_noise_deg:     float = 0.1


# Two Mode A/C replies whose slant ranges are closer than this overlap in the
# receiver's range gate and garble each other.  This is the characteristic SSR
# artefact, and it is what makes all-call acquisition probabilistic rather than
# instantaneous.
GARBLE_WINDOW_NM = 1.7


def radio_horizon_nm(h_ant_ft, h_ac_ft):
    """Line-of-sight range for two heights, in nautical miles.

    d ≈ 1.23 * (sqrt(h_ant) + sqrt(h_ac)) with heights in feet — the standard
    4/3-earth refraction approximation.
    """
    h_ant = max(h_ant_ft, 0.0)
    h_ac  = max(h_ac_ft, 0.0)
    return 1.23 * (math.sqrt(h_ant) + math.sqrt(h_ac))


def visible(ac, site, max_range_nm):
    """(rng_nm, True) if the aircraft is within both the range limit and the
    radio horizon, else (rng_nm, False).  rng_nm is None with no position."""
    if ac.lat is None or ac.lon is None:
        return None, False
    _brg, rng = iff.bearing_range_nm(site.lat, site.lon, ac.lat, ac.lon)
    if rng > max_range_nm:
        return rng, False
    if rng > radio_horizon_nm(site.ant_height_ft, ac.alt_ft or 0.0):
        return rng, False
    return rng, True


# ── IFF ───────────────────────────────────────────────────────────────────────

def deliver_iff(ac, site, mode, t_tx, prt_no, rng_nm=None,
                qnh_hpa=iff.STD_QNH_HPA):
    """Carry one IFF reply from `ac` back to the site.

    Returns (frame_bytes, t_rx) or None if the reply never arrives.  t_rx is
    t_tx plus the true round-trip delay — the receiver's only honest source of
    range, so nothing else in the pipeline may hand it a range.

    `rng_nm` may be supplied when the caller has already computed the slant
    range for its beam test, to avoid recomputing it per PRT.
    """
    if rng_nm is None:
        rng_nm, ok = visible(ac, site, site.iff_max_range_nm)
        if not ok:
            return None
    else:
        if rng_nm > site.iff_max_range_nm:
            return None
        if rng_nm > radio_horizon_nm(site.ant_height_ft, ac.alt_ft or 0.0):
            return None

    if random.random() > site.iff_reply_prob:
        return None

    frame = ac.iff_reply(mode, prt_no, qnh_hpa)
    if frame is None:
        return None

    rtt_us = iff.range_to_rtt_us(rng_nm, mode)
    return frame, t_tx + rtt_us * 1e-6


def garbled(ranges_nm, window_nm=GARBLE_WINDOW_NM):
    """Given the slant ranges of every reply to one Mode A/C interrogation,
    return the set of indices whose replies garble each other.

    Any two replies closer together than `window_nm` overlap in the range gate,
    so both are corrupted — not just the later one.
    """
    n = len(ranges_nm)
    if n < 2:
        return set()
    order = sorted(range(n), key=lambda i: ranges_nm[i])
    bad = set()
    for a, b in zip(order, order[1:]):
        if abs(ranges_nm[b] - ranges_nm[a]) < window_nm:
            bad.add(a)
            bad.add(b)
    return bad


def measure_bearing(beam_az_deg, site):
    """The azimuth the receiver reports for a reply: the beam pointing angle
    plus monopulse estimation noise."""
    if site.brg_noise_deg <= 0.0:
        return beam_az_deg % 360.0
    return (beam_az_deg + random.gauss(0.0, site.brg_noise_deg)) % 360.0


# ── ADS-B ─────────────────────────────────────────────────────────────────────

def deliver_adsb(ac, site, frame, t):
    """Carry one ADS-B squitter to the site.  Returns the frame or None.

    ADS-B is a broadcast: no round-trip, no delay to model, and no garble
    window — just range, horizon, and per-frame loss.
    """
    if frame is None:
        return None
    _rng, ok = visible(ac, site, site.adsb_max_range_nm)
    if not ok:
        return None
    if random.random() > site.adsb_frame_prob:
        return None
    return frame
