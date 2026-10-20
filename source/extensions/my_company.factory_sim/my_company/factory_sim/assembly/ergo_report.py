"""Pictures for the ergonomics: a timeline chart of both workers' RULA scores over the assembly steps, a per-step
summary table, and the live side panel (HUD) that is composited next to each movie frame.

    timeline_png(scores, steps, path)
    summary_md(scores, steps, path)
    hud(scores, steps, frame, size) -> PIL image        composite_frames(in_dir, out_dir, scores, steps)
"""
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from . import ergo as E

NAMES = {"A": "Worker A", "B": "Worker B"}
BG = (24, 26, 30)
FG = (235, 236, 240)
DIM = (150, 154, 162)
GRID = (60, 64, 72)


def _font(size, bold=False):
    for name in (("segoeuib.ttf" if bold else "segoeui.ttf"), ("arialbd.ttf" if bold else "arial.ttf")):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _band_rgb(score):
    return E.band(score)[1]


# --------------------------------------------------------------------------------------------- timeline chart
def timeline_png(scores, steps, path, width=1800, row=150):
    t_end = max(r["t"] for rows in scores.values() for r in rows)
    pad_l, pad_r, top = 150, 30, 70
    H = top + row * len(scores) + 120
    im = Image.new("RGB", (width, H), BG)
    d = ImageDraw.Draw(im)
    f, fb, fs = _font(20), _font(26, True), _font(16)
    d.text((pad_l, 18), "RULA score over the assembly (1-2 acceptable, 3-4 investigate, 5-6 change soon, 7 change now)",
           fill=FG, font=fb)
    X = lambda t: pad_l + (width - pad_l - pad_r) * t / t_end
    for k, (who, rows) in enumerate(scores.items()):
        y0 = top + k * row + 10
        y1 = y0 + row - 30
        d.text((20, y0 + (y1 - y0) / 2 - 12), NAMES.get(who, who), fill=FG, font=f)
        for s in range(1, 8):
            y = y1 - (y1 - y0) * (s - 1) / 6
            d.line((pad_l, y, width - pad_r, y), fill=GRID)
            d.text((pad_l - 22, y - 9), str(s), fill=DIM, font=fs)
        sm = E.smooth_scores(rows)
        for r, s, r2 in zip(rows, sm, rows[1:] + rows[-1:]):
            xa, xb = X(r["t"]), max(X(r2["t"]), X(r["t"]) + 1)
            y = y1 - (y1 - y0) * (s - 1) / 6
            d.rectangle((xa, y, xb, y1), fill=_band_rgb(s))
    yb = top + row * len(scores) + 4
    for t0, t1, lab in steps:
        d.line((X(t0), top, X(t0), yb), fill=(110, 114, 122))
        d.text((X(t0) + 4, yb + 8 + 22 * (steps.index((t0, t1, lab)) % 3)), lab, fill=FG, font=fs)
    for k in range(0, int(t_end) + 1, 20):
        d.text((X(k) - 8, H - 30), f"{k}s", fill=DIM, font=fs)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    im.save(path)
    return path


# --------------------------------------------------------------------------------------------- summary table
def summary_md(scores, steps, path):
    rows = E.summary(scores, steps)
    lines = ["# Rack assembly: RULA ergonomics by step", "",
             "Scores 1-7 (McAtamney & Corlett 1993) for every frame of the animation; seconds per action level. "
             "Walking or waiting with nothing in the hands is not scored as a task (capped at 2). The scores describe this "
             "animated method, not measured people: use them to compare methods and find the bad steps.", "",
             "| Step | Worker | Peak | Mean | Acceptable | Investigate | Change soon | Change now | Main reasons at the peak |",
             "|---|---|---|---|---|---|---|---|---|"]
    for lab, who, peak, mean, secs, cause in rows:
        b = [secs[n] for _, n, _ in E.BANDS]
        lines.append(f"| {lab} | {who} | {peak} | {mean:.1f} | " + " | ".join(f"{v:.1f} s" for v in b) + f" | {cause} |")
    open(path, "w", encoding="utf-8").write("\n".join(lines) + "\n")
    return path


# --------------------------------------------------------------------------------------------- live panel (HUD)
BODY = {   # 2D body map (panel units): segments and the RULA parts' dots
    "segs": [((50, 18), (50, 30)), ((50, 30), (50, 70)), ((30, 34), (70, 34)), ((30, 34), (24, 58)),
             ((70, 34), (76, 58)), ((40, 72), (60, 72)), ((42, 72), (40, 98)), ((58, 72), (60, 98)),
             ((40, 98), (38, 124)), ((60, 98), (62, 124))],
    "head": (50, 11, 8),
    "dots": {"neck": (50, 26), "trunk": (50, 52), "upper_arm_L": (70, 38), "upper_arm_R": (30, 38),
             "legs_L": (60, 98), "legs_R": (40, 98)},
}
PART_RGB = {0: (46, 170, 80), 1: (240, 200, 30), 2: (215, 40, 40)}


def _part_rgb(key, score):
    k = "legs" if key.startswith("legs") else key
    return PART_RGB[E._part_level(k, score)]


def hud(scores, steps, frame, size=(460, 1080), cache={}):
    """Side panel for one movie frame: per worker the live RULA score, its action level, a body map with the
    back / neck / shoulders / knees coloured by their own scores, and a short score trace."""
    key = id(scores)
    if key not in cache:
        cache.clear()
        cache[key] = {w: {k: E.smooth_scores(rows, k) for k in ("score", "trunk", "neck", "legs", "upper_arm_L",
                                                                "upper_arm_R")} for w, rows in scores.items()}
        cache["frames"] = {w: np.array([r["frame"] for r in rows]) for w, rows in scores.items()}
    sm = cache[key]
    W, H = size
    im = Image.new("RGB", size, BG)
    d = ImageDraw.Draw(im)
    f_t, f_s = _font(30, True), _font(18)
    t = frame / E.FPS
    step = next((lab for t0, t1, lab in steps if t0 <= t < t1), "")
    d.text((22, 18), "Ergonomics (RULA)", fill=FG, font=f_t)
    d.text((22, 58), step, fill=DIM, font=f_s)
    block = (H - 100) // max(len(scores), 1)
    u = max(0.6, min(1.25, block / 470.0))                     # layout scale with the panel height
    f_b, f_s, f_n = _font(int(22 * u) + 2, True), _font(int(17 * u) + 1), _font(int(60 * u), True)
    for k, (who, rows) in enumerate(scores.items()):
        y0 = 100 + k * block
        i = int(np.clip(np.searchsorted(cache["frames"][who], frame), 0, len(rows) - 1))
        s = sm[who]["score"][i]
        name, rgb = E.band(s)
        d.rounded_rectangle((16, y0, W - 16, y0 + block - 16), radius=14, outline=GRID, width=2)
        d.text((32, y0 + 10 * u), NAMES.get(who, who), fill=FG, font=f_b)
        # body map, top right of the block
        bw = 100 * 1.15 * u
        ox, oy = W - 26 - bw, y0 + 14 * u
        sc = bw / 100
        P = lambda p: (ox + p[0] * sc, oy + p[1] * sc)
        for a_, b_ in BODY["segs"]:
            d.line(P(a_) + P(b_), fill=(120, 124, 132), width=max(3, int(5 * u)))
        hx, hy, hr = BODY["head"]
        d.ellipse((ox + (hx - hr) * sc, oy + (hy - hr) * sc, ox + (hx + hr) * sc, oy + (hy + hr) * sc),
                  outline=(120, 124, 132), width=max(3, int(4 * u)))
        for part, p in BODY["dots"].items():
            src = "legs" if part.startswith("legs") else part
            c = _part_rgb(part, sm[who][src][i])
            x, y = P(p)
            rr = 9 * u
            d.ellipse((x - rr, y - rr, x + rr, y + rr), fill=c, outline=(20, 20, 20), width=2)
        # score tile and what drives it, left
        ty = y0 + 46 * u
        d.rounded_rectangle((32, ty, 32 + 100 * u, ty + 84 * u), radius=12, fill=rgb)
        d.text((32 + 50 * u, ty + 42 * u), str(s), fill=(20, 20, 20), font=f_n, anchor="mm")
        d.text((32, ty + 94 * u), name.upper(), fill=rgb, font=f_b)
        r = rows[i]
        info = [f"back {max(r['trunk_deg'], 0):.0f}°  neck {r['neck_deg']:.0f}°",
                f"arms {r['ua_L_deg']:.0f}°/{r['ua_R_deg']:.0f}°  knee {r['knee_deg']:.0f}°",
                f"load {r['kg']:.1f} kg" if r["kg"] >= 0.5 else "no load"]
        for j, line in enumerate(info):
            d.text((32 + 112 * u, ty + 6 * u + 24 * u * j), line, fill=DIM, font=f_s)
        # spine load bar: L5/S1 compression against the NIOSH action limit (3400 N) and maximum (6400 N)
        n = r.get("l5s1_N", 0.0)
        bx0, bx1 = 32 + 112 * u, W - 40 - bw
        by = ty + 6 * u + 24 * u * 3 + 6
        bh = 14 * u
        nmax = E.NIOSH_MPL
        col = (215, 40, 40) if n >= E.NIOSH_AL else ((240, 200, 30) if n >= 2000 else (46, 170, 80))
        d.rectangle((bx0, by, bx1, by + bh), outline=GRID, fill=(40, 43, 50))
        d.rectangle((bx0, by, bx0 + (bx1 - bx0) * min(n, nmax) / nmax, by + bh), fill=col)
        xa = bx0 + (bx1 - bx0) * E.NIOSH_AL / nmax
        d.line((xa, by - 3, xa, by + bh + 3), fill=FG, width=2)
        d.text((bx0, by + bh + 2), f"spine {n:,.0f} N", fill=col, font=f_s)
        # trace of the last 20 s, bottom of the block
        tx0, tx1 = 32, W - 32
        ty0 = max(ty + 160 * u, oy + 130 * sc + 8)
        ty1 = y0 + block - 40 * u
        if ty1 - ty0 > 20:
            d.rectangle((tx0, ty0, tx1, ty1), outline=GRID)
            for s2 in (2, 4, 6):
                y = ty1 - (ty1 - ty0) * (s2 - 1) / 6
                d.line((tx0, y, tx1, y), fill=GRID)
            span = int(20 * E.FPS)
            stepf = max(1, int(rows[1]["frame"] - rows[0]["frame"])) if len(rows) > 1 else 1
            for j in range(max(0, i - span // stepf), i + 1):
                fr = rows[j]["frame"]
                x = tx1 - (frame - fr) / span * (tx1 - tx0)
                sj = sm[who]["score"][j]
                y = ty1 - (ty1 - ty0) * (sj - 1) / 6
                d.line((x, y, x, ty1), fill=_band_rgb(sj), width=2)
            d.text((tx0, ty1 + 3), "last 20 s", fill=DIM, font=f_s)
    return im


def composite_frames(in_dir, out_dir, scores, steps, panel_w=420, pattern=None):
    """Put the HUD to the right of every rendered frame (PNG sequence, frame number at the end of the name)."""
    import re
    os.makedirs(out_dir, exist_ok=True)
    files = sorted(f for f in os.listdir(in_dir) if f.lower().endswith(".png"))
    for fn in files:
        m = re.search(r"(\d+)\.png$", fn)
        if not m:
            continue
        frame = int(m.group(1))
        src = Image.open(os.path.join(in_dir, fn)).convert("RGB")
        panel = hud(scores, steps, frame, (panel_w, src.height))
        out = Image.new("RGB", (src.width + panel_w, src.height), BG)
        out.paste(src, (0, 0))
        out.paste(panel, (src.width, 0))
        out.save(os.path.join(out_dir, fn))
    return len(files)


# --------------------------------------------------------------------------------------------- floor hotspot plan
def floor_png(H, extent, path, px_per_m=90, outlines=(), labels=()):
    """Top view of the floor heat (time at RULA 5+): the bay and its storage drawn in, aisle (front, +Z) at the
    bottom. outlines: [((x0, z0), (x1, z1), label)] in cm."""
    from . import ergo_viz as V
    (x0, x1), (z0, z1) = extent
    W, Hh = int((x1 - x0) / 100 * px_per_m), int((z1 - z0) / 100 * px_per_m)
    rgba = V.heat_rgba(H)
    heat = Image.fromarray(rgba, "RGBA").resize((W, Hh), Image.BILINEAR)
    base = Image.new("RGBA", (W, Hh), (40, 43, 50, 255))
    d = ImageDraw.Draw(base)
    P = lambda x, z: ((x - x0) / (x1 - x0) * W, (z - z0) / (z1 - z0) * Hh)
    for m in range(int(x0 // 100) * 100, int(x1) + 1, 100):
        d.line(P(m, z0) + P(m, z1), fill=(55, 58, 66))
    for m in range(int(z0 // 100) * 100, int(z1) + 1, 100):
        d.line(P(x0, m) + P(x1, m), fill=(55, 58, 66))
    img = Image.alpha_composite(base, heat)
    d = ImageDraw.Draw(img)
    f, fb = _font(18), _font(24, True)
    for (a, b, lab) in outlines:
        pa, pb = P(*a), P(*b)
        d.rectangle((min(pa[0], pb[0]), min(pa[1], pb[1]), max(pa[0], pb[0]), max(pa[1], pb[1])), outline=FG, width=2)
        if lab:
            d.text((min(pa[0], pb[0]) + 4, min(pa[1], pb[1]) + 2), lab, fill=FG, font=f)
    for (x, z, lab) in labels:
        d.text(P(x, z), lab, fill=FG, font=f, anchor="mm")
    d.text((14, 10), "Where the risky work happens (time at RULA 5-7, both workers)", fill=FG, font=fb)
    d.text((14, Hh - 30), "front aisle", fill=DIM, font=f)
    d.text((W - 260, Hh - 30), "1 m grid", fill=DIM, font=f)
    img.convert("RGB").save(path)
    return path
