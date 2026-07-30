"""
Shared harness for the acceptance suites
========================================
Each suite in this directory checks one phase of the correctness & fidelity
work against the running application, not against a mock.  That means driving
a real Tk window, so a display is required.

The three things every suite needs:

  Clock      monkeypatches time.monotonic so frames advance deterministically.
             Real wall-clock timing would make anything involving the 4 s scan
             period or the 4 s field-decay limit unrunnable.

  make_app   builds a CombinedApp and steps it via _frame(), bypassing the Tk
             timer so the test owns the clock.

  chk        records a pass/fail with a detail string.  Detail is printed for
             passes too: a check that passes for the wrong reason is worse
             than one that fails, and the detail is what exposes it.

A note on scan phase.  At the default settings the beam yields about 1.1
interrogations per dwell, so a target is genuinely skipped on some scans (see
plan 3.2 — a real SSR constraint, deliberately not compensated for).  Suites
that are testing something else entirely call match_prf() to get reliable
once-per-scan replies, rather than asserting over a coin flip.
"""

import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

import simulator as S            # noqa: E402
import iff_protocol as iff       # noqa: E402,F401  (re-exported)

_real_monotonic = time.monotonic


# ── source inspection ─────────────────────────────────────────────────────────

def source(name):
    """Read one of the project's source files, for the handful of checks that
    assert on structure (e.g. that no lock survives, that the draw path does
    not reference ground truth)."""
    with open(os.path.join(REPO, name)) as fh:
        return fh.read()


# ── deterministic clock ───────────────────────────────────────────────────────

class Clock:
    """A monotonic clock the test advances by hand."""

    def __init__(self, dt=0.05):
        self.t = _real_monotonic()
        self.dt = dt

    def __call__(self):
        return self.t

    def install(self):
        time.monotonic = self
        S.time.monotonic = self
        return self

    def restore(self):
        time.monotonic = _real_monotonic
        S.time.monotonic = _real_monotonic

    def advance(self, secs):
        self.t += secs


# ── application under test ────────────────────────────────────────────────────

def make_app(clock, rng=200.0):
    app = S.CombinedApp(51.477, -0.461, rng, 0.0)
    app._cw = app._ch = S.ui.CANVAS_SZ
    app._tick = clock.t
    app._sweep_anchor_t = clock.t
    app.update()
    return app


def add_ac(app, wps, select=True, **kw):
    ac = S.SimAircraft(**kw)
    ac.waypoints = list(wps)
    app._aircraft.append(ac)
    if select:
        app._select(ac)
    return ac


def frames(app, clock, n, draw=True):
    """Advance n simulation frames.  draw=False skips the Tk redraw, which is
    a large speedup for suites that only inspect model state."""
    for _ in range(n):
        clock.advance(clock.dt)
        app._frame()
        if draw:
            app.update()


def match_prf(app, rpm=30, prt_us=5000):
    """Match PRF to rotation rate, as an operator would, giving >3 hits per
    dwell and therefore a reply every scan.  Use in suites where detection
    probability is not the thing under test."""
    app._v_rpm.set(rpm)
    app._v_prt.set(prt_us)


# ── result recording ──────────────────────────────────────────────────────────

class Suite:
    """Collects check results and reports a single exit status."""

    def __init__(self, title):
        self.title = title
        self.failures = []
        self.passes = 0
        print(f"── {title} " + "─" * max(0, 68 - len(title)))

    def chk(self, name, cond, detail=""):
        suffix = f"   {detail}" if detail else ""
        print(("  PASS  " if cond else "  FAIL  ") + name + suffix)
        if cond:
            self.passes += 1
        else:
            self.failures.append(name)
        return bool(cond)

    def report(self):
        print()
        if self.failures:
            print(f"{self.title}: {len(self.failures)} FAILURES "
                  f"({self.passes} passed)")
            for f in self.failures:
                print(f"  - {f}")
        else:
            print(f"{self.title}: ALL PASS ({self.passes} checks)")
        return 1 if self.failures else 0
