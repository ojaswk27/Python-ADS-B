# Acceptance suites

Checks for the correctness & fidelity work, one suite per phase. These drive
the **real application** — a live Tk window, the real frame loop, the real
channel and receiver — rather than a mock, because several of the defects they
cover only appear once Tk owns the widget tree.

```
python acceptance/run_all.py           # everything, once
python acceptance/run_all.py -n 5      # five times, to surface flaky checks
python acceptance/run_all.py -v        # print every check
python acceptance/phase4_pipeline.py   # one suite, for detail
```

**A display is required.** `run_all.py` also runs the two display-free codec
tests at the repo root; `--no-unit` skips them.

| Suite | Covers |
|---|---|
| `phase1_motion.py` | Loop-path teleport, speed-0 snap-back, degenerate segments, track-store drain, altitude formatting, dead locks removed |
| `phase2_tracks.py` | `modes_addr` as sole identity, one address-keyed store with one TRK counter, per-field ageing, target-menu provenance |
| `phase3_timing.py` | Per-PRT interrogation (not clamped to the frame rate), hits-per-dwell readout, Mode S lockout and roll-call scheduling |
| `phase4_pipeline.py` | Aircraft emit frames only, channel range/horizon/loss/delay/garble, measured range and bearing, CPR acquisition, plots from the store |
| `phase56_fidelity.py` | Mode C pressure altitude and QNH, Mode 1 code range, DO-260B message rates, vertical rate, turn rate, emergency codes and SPI, duplicate-address alert, dropdown stability |

## Two things worth knowing before editing these

**The clock is fake.** `harness.Clock` monkeypatches `time.monotonic` and the
suite advances it by hand. Real wall-clock timing would make anything touching
the 4 s scan period or the 4 s field-decay limit impractical to test. Always
`install()` in a `try` and `restore()` in the `finally`.

**Scan phase is not something to assert against.** At the application's default
settings the beam yields about 1.1 interrogations per beam dwell, so a target is
genuinely skipped on some scans — a real SSR constraint that the app surfaces
rather than hides. A check that asserts "this field is fresh" at an arbitrary
stop time is therefore a coin flip. Either call `harness.match_prf(app)` to get
reliable once-per-scan replies, or phrase the assertion so it does not depend on
phase (e.g. "this timestamp keeps advancing while that one does not").

Several checks in these suites originally passed for the wrong reason and were
rewritten: a tautology, a turn whose bearing happened to be due north so any
value satisfied it, and three that were quietly fighting the hits-per-dwell
constraint. Print detail on passes as well as failures — that is what exposes
this class of mistake.
