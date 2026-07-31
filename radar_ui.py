"""
Shared UI constants, helpers, and drawing primitives for the ADS-B radar tools.

To resize the entire UI uniformly, change SCALE and restart:
  SCALE = 1  →  original compact size
  SCALE = 2  →  double size (current default)
"""

import colorsys
import math
import random
import tkinter as tk

# Scale

SCALE = 1  # change this one number to resize everything

# Palette

BG = "#000000"
RADAR = "#050505"
RING_D = "#1c1c1c"
RING_B = "#444444"
DIM = "#444444"
PANEL = "#0c0c0c"
FG = "#cccccc"
FG_DIM = "#555555"
SEP = "#1e1e1e"
ENTRY = "#111111"
BTN = "#1a1a1a"
BTN_ACT = "#2a2a2a"
BTN_RED = "#8a2020"  # destructive-action button
BTN_RED_A = "#a83030"  # …and its active/hover state
GRID = "#161616"  # lat/lon graticule lines

# Scale-derived sizes

CANVAS_SZ = 680 * SCALE  # canvas width/height in pixels
PANEL_W = 200 * SCALE  # side-panel width in pixels

HIT_WP = 10 * SCALE  # waypoint click-hit radius
BLIP_SZ = 8 * SCALE  # aircraft triangle half-size
WP_DOT = 4 * SCALE  # waypoint marker radius
TRAIL_DOT = 2 * SCALE  # trail history dot radius
LBL_DX = 12 * SCALE  # blip label x offset from centre
LBL_DY = 12 * SCALE  # blip label y offset from centre

PAD = round(8 * SCALE)  # standard panel margin
PAD2 = round(4 * SCALE)  # half margin

# Fonts
# Edit these four lines to change all text sizes at once.

F_SM = ("Courier", round(7 * SCALE))  # range labels, hints, status
F_MD = ("Courier", round(8 * SCALE))  # panel labels, list items
F_BLD = ("Courier", round(8 * SCALE), "bold")  # callsigns, section headers
F_ENT = ("Courier", round(9 * SCALE))  # text-entry fields

# Base canvas-text point sizes (at scale factor 1.0).  Pass these through
# sfont() to scale with the window; the F_* tuples above stay fixed for the
# side-panel widgets, which do not scale.
PT_SM = round(7 * SCALE)
PT_MD = round(8 * SCALE)


# Dynamic scaling


def scale_for(w=None, h=None):
    """Return a drawing scale factor for the live canvas size (1.0 = baseline).

    Elements drawn on the radar canvas multiply their base size by this so the
    whole picture — blips, dots, fonts, label offsets — grows and shrinks with
    the window.  Clamped to a sane range so text stays legible.
    """
    if w is None:
        w = CANVAS_SZ
    if h is None:
        h = w
    return max(0.5, min(min(w, h) / CANVAS_SZ, 3.0))


_FONT_CACHE: dict = {}


def sfont(base_pt, sf=1.0, bold=False):
    """Scale a base point size by sf and return a Courier font tuple.

    Cached by (base_pt, rounded sf, bold) so the per-frame draw doesn't keep
    materialising the same tuple — Tk re-parses it into an internal font handle
    on every call to create_text, and the canvas hits create_text once per
    blip per text label per frame.
    """
    key = (base_pt, round(sf, 2), bold)
    f = _FONT_CACHE.get(key)
    if f is None:
        size = max(6, round(base_pt * sf))
        f = ("Courier", size, "bold") if bold else ("Courier", size)
        _FONT_CACHE[key] = f
    return f


# Geometry


def geom(w=None, h=None):
    """Return (cx, cy, r) for the radar PPI canvas.

    Pass the live canvas width/height to autoscale with the window; with no
    args it falls back to the fixed CANVAS_SZ (used by non-resizable displays).
    The disc is centred in the canvas and sized to the shorter dimension.
    """
    if w is None:
        w = CANVAS_SZ
    if h is None:
        h = w
    cx, cy = w // 2, h // 2
    r = min(w, h) // 2 - round(18 * SCALE)
    return cx, cy, max(r, 1)


def ll_to_xy(lat, lon, cx, cy, r, c_lat, c_lon, rng):
    """Convert lat/lon to canvas (x, y). Returns None if outside range."""
    s = r / rng
    nm_e = (lon - c_lon) * 60.0 * math.cos(math.radians(c_lat))
    nm_n = (lat - c_lat) * 60.0
    if math.hypot(nm_e, nm_n) > rng * 1.02:
        return None
    return cx + nm_e * s, cy - nm_n * s


def xy_to_ll(x, y, cx, cy, r, c_lat, c_lon, rng):
    """Convert canvas (x, y) to (lat, lon). Returns None if outside disc."""
    s = r / rng
    nm_e = (x - cx) / s
    nm_n = (cy - y) / s
    if math.hypot(nm_e, nm_n) > rng:
        return None
    return (c_lat + nm_n / 60.0, c_lon + nm_e / (60.0 * math.cos(math.radians(c_lat))))


# Colour

# Successive hues advance by the golden-ratio conjugate — a low-discrepancy
# sequence that spreads colours around the wheel so two targets never land on
# near-identical hues.  (Independent random hues clump: with only a few targets
# you'd routinely get two pinks by chance.)  The starting point is random, so
# the overall set is still unpredictable run-to-run.
_GOLDEN_CONJ = 0.6180339887498949
_hue = [random.random()]


def random_color():
    """Return a vivid hex colour, well separated from the previously issued one."""
    _hue[0] = (_hue[0] + _GOLDEN_CONJ) % 1.0
    r, g, b = colorsys.hsv_to_rgb(_hue[0], 0.6, 1.0)
    return f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"


def shade(hex_color, factor):
    """Scale a #rrggbb colour's brightness by factor (0..1)."""
    h = hex_color.lstrip("#")
    r, g, b = (int(h[i : i + 2], 16) for i in (0, 2, 4))
    f = max(0.0, min(1.0, factor))
    return f"#{int(r * f):02x}{int(g * f):02x}{int(b * f):02x}"


def blend(hex_a, hex_b, t):
    """Linear-interpolate two #rrggbb colours; t=0 → a, t=1 → b."""
    def _rgb(h):
        h = h.lstrip("#")
        return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))
    ra, ga, ba = _rgb(hex_a)
    rb, gb, bb = _rgb(hex_b)
    t = max(0.0, min(1.0, t))
    r = int(ra + (rb - ra) * t)
    g = int(ga + (gb - ga) * t)
    b = int(ba + (bb - ba) * t)
    return f"#{r:02x}{g:02x}{b:02x}"


# Widget helpers


def sep(parent):
    """Pack a thin horizontal separator bar into parent."""
    tk.Frame(parent, bg=SEP, height=max(1, SCALE)).pack(fill=tk.X, padx=0, pady=PAD2)


def entry_row(parent, label, var):
    """Pack a label + entry field pair into parent. Returns the Entry widget."""
    f = tk.Frame(parent, bg=PANEL)
    f.pack(fill=tk.X, padx=PAD, pady=max(1, PAD // 4))
    tk.Label(f, text=label, bg=PANEL, fg=FG_DIM, font=F_MD, width=9, anchor="w").pack(
        side=tk.LEFT
    )
    e = tk.Entry(
        f,
        textvariable=var,
        width=8,
        bg=ENTRY,
        fg=FG,
        insertbackground=FG,
        font=F_ENT,
        relief=tk.FLAT,
        bd=round(4 * SCALE),
    )
    e.pack(side=tk.LEFT, fill=tk.X, expand=True)
    return e


def slider_row(parent, label, var, from_, to, resolution=1, command=None):
    """Pack a label/value header + horizontal slider into parent. Returns the Scale widget."""
    f = tk.Frame(parent, bg=PANEL)
    f.pack(fill=tk.X, padx=PAD, pady=max(1, PAD // 4))
    hf = tk.Frame(f, bg=PANEL)
    hf.pack(fill=tk.X)
    tk.Label(hf, text=label, bg=PANEL, fg=FG_DIM, font=F_MD, anchor="w").pack(
        side=tk.LEFT
    )
    tk.Label(hf, textvariable=var, bg=PANEL, fg=FG, font=F_MD, anchor="e").pack(
        side=tk.RIGHT
    )
    kw = {"command": command} if command else {}
    s = tk.Scale(
        f,
        variable=var,
        from_=from_,
        to=to,
        resolution=resolution,
        orient=tk.HORIZONTAL,
        bg=PANEL,
        fg=FG_DIM,
        troughcolor=ENTRY,
        highlightthickness=0,
        bd=0,
        showvalue=False,
        relief=tk.FLAT,
        sliderlength=round(12 * SCALE),
        **kw,
    )
    s.pack(fill=tk.X)
    return s


def flat_button(parent, text, command, bg=BTN, fg=FG, active=None):
    """A Label styled as a flat button.

    tk.Button ignores its face colour under macOS Aqua, so colours like the
    destructive red never show; a Label honours bg everywhere and keeps the
    flat look.  Returns the widget unpacked — the caller packs it.
    """
    active = active or BTN_ACT
    b = tk.Label(
        parent,
        text=text,
        bg=bg,
        fg=fg,
        font=F_MD,
        cursor="hand2",
        pady=round(4 * SCALE),
    )
    b.bind("<Enter>", lambda _e: b.config(bg=active))
    b.bind("<Leave>", lambda _e: b.config(bg=bg))
    b.bind("<ButtonRelease-1>", lambda _e: command())
    return b


def make_panel(root, side=tk.LEFT, width=None):
    kw = PANEL_W if width is None else width
    p = tk.Frame(root, bg=PANEL, width=kw)
    p.pack(side=side, fill=tk.Y)
    p.pack_propagate(False)
    return p


def make_scroll_panel(root, side=tk.LEFT, width=None):
    """A fixed-width side panel whose contents scroll vertically.

    Returns the inner Frame to pack children into — callers use it exactly like
    the Frame from make_panel.  Tk has no scrollable Frame, so this is the
    standard Canvas-plus-window arrangement.

    The scrollbar appears only when the contents are taller than the panel, so
    a panel that fits looks and behaves identically to a plain one.
    """
    outer = tk.Frame(root, bg=PANEL, width=(PANEL_W if width is None else width))
    outer.pack(side=side, fill=tk.Y)
    outer.pack_propagate(False)

    cv = tk.Canvas(outer, bg=PANEL, highlightthickness=0, bd=0, takefocus=0)
    bar = tk.Scrollbar(
        outer,
        orient=tk.VERTICAL,
        command=cv.yview,
        bg=PANEL,
        troughcolor=PANEL,
        activebackground=BTN_ACT,
        relief=tk.FLAT,
        bd=0,
        width=round(11 * SCALE),
    )
    cv.configure(yscrollcommand=bar.set)
    cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    inner = tk.Frame(cv, bg=PANEL)
    win = cv.create_window(0, 0, window=inner, anchor="nw")

    def _overflowing():
        return inner.winfo_reqheight() > cv.winfo_height()

    def _sync(_ev=None):
        # Keep the embedded frame exactly as wide as the visible canvas so
        # fill=tk.X children lay out against the panel width, not their own.
        cv.itemconfigure(win, width=cv.winfo_width())
        cv.configure(scrollregion=cv.bbox("all"))
        if _overflowing():
            if not bar.winfo_ismapped():
                # before=cv matters: the canvas is packed with expand=True and
                # claims the whole cavity, so a bar packed after it gets no
                # width at all.
                bar.pack(side=tk.RIGHT, fill=tk.Y, before=cv)
        elif bar.winfo_ismapped():
            bar.pack_forget()
            cv.yview_moveto(0.0)

    inner.bind("<Configure>", _sync)
    cv.bind("<Configure>", _sync)

    def _wheel(ev):
        if not _overflowing():
            return
        if ev.num == 4:                     # X11 wheel up
            step = -1
        elif ev.num == 5:                   # X11 wheel down
            step = 1
        else:                               # Windows / macOS
            step = -1 if ev.delta > 0 else 1
        cv.yview_scroll(step, "units")

    # Claim the wheel only while the pointer is over this panel, so the radar
    # canvas keeps it the rest of the time.
    def _claim(_ev=None):
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            cv.bind_all(seq, _wheel)

    def _release(_ev=None):
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            cv.unbind_all(seq)

    outer.bind("<Enter>", _claim)
    outer.bind("<Leave>", _release)

    return inner


# Radar canvas drawing


def draw_radar_frame(cv, cx, cy, r, rng, c_lat, c_lon, sf=1.0, tag=""):
    """
    Draw the radar background: disc fill, range rings, axis cross, cardinal
    letters, and coord text.  This is static between view changes, so callers
    pass a `tag` and only redraw it when the view (centre/range/size) changes.

    sf  — drawing scale factor from scale_for(); scales fonts and offsets.
    tag — canvas tag applied to every item (for layered redraw).
    """
    cv.create_oval(cx - r, cy - r, cx + r, cy + r, fill=RADAR, outline="", tags=tag)

    f_sm = sfont(PT_SM, sf)
    step = 50 if rng <= 350 else 100
    ring = step
    _rdx = round(2 * SCALE * sf)
    _rdy = round(8 * SCALE * sf)
    while ring < rng:
        rp = int(r * ring / rng)
        cv.create_oval(
            cx - rp, cy - rp, cx + rp, cy + rp, outline=RING_D, width=1, tags=tag
        )
        a = math.radians(42)
        cv.create_text(
            cx + int(rp * math.sin(a)) + _rdx,
            cy - int(rp * math.cos(a)) - _rdy,
            text=str(ring),
            fill=DIM,
            font=f_sm,
            tags=tag,
        )
        ring += step

    cv.create_oval(cx - r, cy - r, cx + r, cy + r, outline=RING_B, width=1, tags=tag)
    cv.create_line(cx, cy - r, cx, cy + r, fill=RING_D, tags=tag)
    cv.create_line(cx - r, cy, cx + r, cy, fill=RING_D, tags=tag)

    c1 = round(12 * SCALE * sf)
    c2 = round(13 * SCALE * sf)
    f_md = sfont(PT_MD, sf)
    for txt, dx, dy in (
        ("N", 0, -(r + c1)),
        ("S", 0, r + c1),
        ("W", -(r + c2), 0),
        ("E", r + c2, 0),
    ):
        cv.create_text(cx + dx, cy + dy, text=txt, fill=RING_B, font=f_md, tags=tag)

    org = round(6 * SCALE * sf)
    cv.create_text(
        org,
        org,
        text=f"{c_lat:+.3f}  {c_lon:+.3f}  {rng:.0f}nm",
        fill=DIM,
        font=f_sm,
        anchor="nw",
        tags=tag,
    )


def _grid_step(span_deg):
    """Pick a 'nice' graticule step (degrees) giving ≲12 lines across a span."""
    for s in (0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0):
        if span_deg / s <= 12:
            return s
    return 10.0


def draw_latlon_grid(cv, cx, cy, r, rng, c_lat, c_lon, tag=""):
    """Draw a WGS-84 lat/lon graticule clipped to the radar disc.

    Parallels (constant latitude) and meridians (constant longitude) are
    sampled across the visible span and drawn as polylines; ll_to_xy returns
    None outside the disc, which breaks each line at the disc edge.  Static
    between view changes — callers pass a `tag` and cache it.
    """
    rad_lat = rng / 60.0
    coslat = max(math.cos(math.radians(c_lat)), 1e-6)
    rad_lon = rng / (60.0 * coslat)
    step = _grid_step(2 * rad_lat)

    def polyline(points):
        run = []
        for p in points:
            if p is None:
                if len(run) >= 4:
                    cv.create_line(run, fill=GRID, tags=tag)
                run = []
            else:
                run += [p[0], p[1]]
        if len(run) >= 4:
            cv.create_line(run, fill=GRID, tags=tag)

    n = 48
    lat = math.ceil((c_lat - rad_lat) / step) * step
    while lat <= c_lat + rad_lat + 1e-9:
        polyline(
            [
                ll_to_xy(
                    lat,
                    c_lon - rad_lon + 2 * rad_lon * i / n,
                    cx,
                    cy,
                    r,
                    c_lat,
                    c_lon,
                    rng,
                )
                for i in range(n + 1)
            ]
        )
        lat += step

    lon = math.ceil((c_lon - rad_lon) / step) * step
    while lon <= c_lon + rad_lon + 1e-9:
        polyline(
            [
                ll_to_xy(
                    c_lat - rad_lat + 2 * rad_lat * i / n,
                    lon,
                    cx,
                    cy,
                    r,
                    c_lat,
                    c_lon,
                    rng,
                )
                for i in range(n + 1)
            ]
        )
        lon += step


def draw_blip(cv, x, y, hdg_rad, col, sf=1.0, tag=""):
    """Draw a filled aircraft triangle at (x, y) pointing along hdg_rad."""
    sz, v = BLIP_SZ * sf, []
    for a in (hdg_rad, hdg_rad + math.radians(148), hdg_rad - math.radians(148)):
        v += [x + sz * math.sin(a), y - sz * math.cos(a)]
    cv.create_polygon(v, fill=col, outline="", tags=tag)
