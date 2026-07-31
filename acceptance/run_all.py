#!/usr/bin/env python3
"""
Run every acceptance suite and summarise.

    python acceptance/run_all.py            # once
    python acceptance/run_all.py -n 5       # five times, to surface flakes
    python acceptance/run_all.py -v         # show every check, not just failures

Each suite runs in its own subprocess: they all build a Tk window and
monkeypatch time.monotonic, so sharing an interpreter would let one suite's
clock leak into the next.

Needs a display.  Suites drive the real application rather than a mock, which
is the point — several of the bugs these checks cover only appear once Tk owns
the widget tree.
"""

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))

SUITES = [
    "phase1_motion.py",
    "phase2_tracks.py",
    "phase3_timing.py",
    "phase4_pipeline.py",
    "phase56_fidelity.py",
    "phase8_custom1090.py",
]

# Codec round-trips live at the repo root and need no display.
UNIT = ["test_codecs.py", "test_pseudo1090.py"]


def run(path, cwd, verbose):
    t0 = time.monotonic()
    p = subprocess.run([sys.executable, path], cwd=cwd,
                       capture_output=True, text=True)
    dt = time.monotonic() - t0
    out = p.stdout + p.stderr
    if verbose or p.returncode != 0:
        print(out.rstrip())
    return p.returncode, dt, out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", "--runs", type=int, default=1,
                    help="repeat everything N times to surface flaky checks")
    ap.add_argument("-v", "--verbose", action="store_true",
                    help="print every check, not just failures")
    ap.add_argument("--no-unit", action="store_true",
                    help="skip the display-free codec tests at the repo root")
    args = ap.parse_args()

    repo = os.path.dirname(HERE)
    jobs = [(s, HERE) for s in SUITES]
    if not args.no_unit:
        jobs += [(u, repo) for u in UNIT]

    bad = {}
    for run_no in range(1, args.runs + 1):
        line = []
        for name, cwd in jobs:
            rc, dt, _ = run(name, cwd, args.verbose)
            line.append("." if rc == 0 else "X")
            if rc != 0:
                bad.setdefault(name, 0)
                bad[name] += 1
        label = f"run {run_no}/{args.runs}" if args.runs > 1 else "run"
        print(f"{label}: {' '.join(line)}   "
              f"({len(jobs)} suites, {sum(1 for c in line if c == '.')} ok)")

    print()
    if bad:
        print("FAILURES")
        for name, n in sorted(bad.items()):
            of = f" in {args.runs} runs" if args.runs > 1 else ""
            print(f"  {name}: failed {n} time(s){of}")
        print("\nRe-run one suite on its own for detail, e.g.:")
        print(f"  python acceptance/{sorted(bad)[0]}")
        return 1

    print(f"ALL PASS — {len(jobs)} suites"
          + (f" x {args.runs} runs" if args.runs > 1 else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
