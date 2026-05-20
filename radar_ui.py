"""
Shared UI constants, helpers, and drawing primitives for the ADS-B radar tools.

To resize the entire UI uniformly, change SCALE and restart:
  SCALE = 1  →  original compact size
  SCALE = 2  →  double size (current default)
"""

import math
import tkinter as tk

# ── Scale ─────────────────────────────────────────────────────────────────────

SCALE = 2   # change this one number to resize everything

# ── Palette ───────────────────────────────────────────────────────────────────

BG      = "#000000"
RADAR   = "#050505"
RING_D  = "#1c1c1c"
RING_B  = "#444444"
SWEEP   = "#ffffff"
SWEEP_T = "#1a1a1a"
DIM     = "#444444"
PANEL   = "#0c0c0c"
FG      = "#cccccc"
FG_DIM  = "#555555"
SEP     = "#1e1e1e"
ENTRY   = "#111111"
BTN     = "#1a1a1a"
BTN_ACT = "#2a2a2a"

# ── Scale-derived sizes ───────────────────────────────────────────────────────

CANVAS_SZ  = 680 * SCALE    # canvas width/height in pixels
PANEL_W    = 200 * SCALE    # side-panel width in pixels
SWEEP_SPD  = 36.0           # degrees/second — not a pixel value

HIT_WP    = 10 * SCALE      # waypoint click-hit radius
BLIP_SZ   = 8  * SCALE      # aircraft triangle half-size
WP_DOT    = 4  * SCALE      # waypoint marker radius
TRAIL_DOT = 2  * SCALE      # trail history dot radius
LBL_DX    = 12 * SCALE      # blip label x offset from centre
LBL_DY    = 12 * SCALE      # blip label y offset from centre

PAD  = round(8 * SCALE)     # standard panel margin
PAD2 = round(4 * SCALE)     # half margin

# ── Fonts ─────────────────────────────────────────────────────────────────────
# Edit these four lines to change all text sizes at once.

F_SM  = ("Courier", round(7 * SCALE))           # range labels, hints, status
F_MD  = ("Courier", round(8 * SCALE))           # panel labels, list items
F_BLD = ("Courier", round(8 * SCALE), "bold")   # callsigns, section headers
F_ENT = ("Courier", round(9 * SCALE))           # text-entry fields


# ── Geometry ──────────────────────────────────────────────────────────────────

def geom():
    """Return (cx, cy, r) for the radar PPI canvas."""
    c = CANVAS_SZ // 2
    return c, c, c - round(18 * SCALE)


def ll_to_xy(lat, lon, cx, cy, r, c_lat, c_lon, rng):
    """Convert lat/lon to canvas (x, y). Returns None if outside range."""
    s    = r / rng
    nm_e = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    nm_n = (lat - c_lat) * 60.0
    if math.hypot(nm_e, nm_n) > rng * 1.02:
        return None
    return cx + nm_e * s, cy - nm_n * s


def xy_to_ll(x, y, cx, cy, r, c_lat, c_lon, rng):
    """Convert canvas (x, y) to (lat, lon). Returns None if outside disc."""
    s    = r / rng
    nm_e = (x - cx) / s
    nm_n = (cy - y) / s
    if math.hypot(nm_e, nm_n) > rng:
        return None
    return (c_lat + nm_n / 60.0,
            c_lon + nm_e / (60.0 * math.cos(math.radians(c_lat))))


# ── Widget helpers ────────────────────────────────────────────────────────────

def sep(parent):
    """Pack a thin horizontal separator bar into parent."""
    tk.Frame(parent, bg=SEP, height=max(1, SCALE)).pack(
        fill=tk.X, padx=0, pady=PAD2)


def entry_row(parent, label, var):
    """Pack a label + entry field pair into parent. Returns the Entry widget."""
    f = tk.Frame(parent, bg=PANEL)
    f.pack(fill=tk.X, padx=PAD, pady=max(1, PAD // 4))
    tk.Label(f, text=label, bg=PANEL, fg=FG_DIM,
             font=F_MD, width=9, anchor="w").pack(side=tk.LEFT)
    e = tk.Entry(f, textvariable=var, width=8,
                 bg=ENTRY, fg=FG, insertbackground=FG,
                 font=F_ENT, relief=tk.FLAT, bd=round(4 * SCALE))
    e.pack(side=tk.LEFT, fill=tk.X, expand=True)
    return e


def make_panel(root):
    """Create and pack the right-side panel Frame. Returns the Frame."""
    p = tk.Frame(root, bg=PANEL, width=PANEL_W)
    p.pack(side=tk.LEFT, fill=tk.Y)
    p.pack_propagate(False)
    return p


# ── Radar canvas drawing ──────────────────────────────────────────────────────

def draw_radar_frame(cv, cx, cy, r, rng, sweep, c_lat, c_lon):
    """
    Draw the full radar background each frame: disc fill, range rings,
    axis cross, cardinal letters, rotating sweep line, and coord text.
    Call this first before drawing any targets or routes.
    """
    cv.create_oval(cx-r, cy-r, cx+r, cy+r, fill=RADAR, outline="")

    step = 50 if rng <= 350 else 100
    ring = step
    _rdx = round(2 * SCALE)
    _rdy = round(8 * SCALE)
    while ring < rng:
        rp = int(r * ring / rng)
        cv.create_oval(cx-rp, cy-rp, cx+rp, cy+rp, outline=RING_D, width=1)
        a = math.radians(42)
        cv.create_text(cx + int(rp * math.sin(a)) + _rdx,
                       cy - int(rp * math.cos(a)) - _rdy,
                       text=str(ring), fill=DIM, font=F_SM)
        ring += step

    cv.create_oval(cx-r, cy-r, cx+r, cy+r, outline=RING_B, width=1)
    cv.create_line(cx, cy-r, cx, cy+r, fill=RING_D)
    cv.create_line(cx-r, cy, cx+r, cy, fill=RING_D)

    c1 = round(12 * SCALE)
    c2 = round(13 * SCALE)
    for txt, dx, dy in (("N", 0, -(r+c1)), ("S", 0, r+c1),
                        ("W", -(r+c2), 0), ("E", r+c2, 0)):
        cv.create_text(cx+dx, cy+dy, text=txt, fill=RING_B, font=F_MD)

    for off in range(-20, 1):
        ang = math.radians((sweep + off) % 360)
        cv.create_line(cx, cy,
                       cx + int(r * math.sin(ang)),
                       cy - int(r * math.cos(ang)),
                       fill=(SWEEP if off == 0 else SWEEP_T))

    org = round(6 * SCALE)
    cv.create_text(org, org,
                   text=f"{c_lat:+.3f}  {c_lon:+.3f}  {rng:.0f}nm",
                   fill=DIM, font=F_SM, anchor="nw")


def draw_blip(cv, x, y, hdg_rad, col):
    """Draw a filled aircraft triangle at (x, y) pointing along hdg_rad."""
    sz, v = BLIP_SZ, []
    for a in (hdg_rad,
              hdg_rad + math.radians(148),
              hdg_rad - math.radians(148)):
        v += [x + sz * math.sin(a), y - sz * math.cos(a)]
    cv.create_polygon(v, fill=col, outline="")
