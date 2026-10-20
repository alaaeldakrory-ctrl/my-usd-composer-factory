"""Compose the before/after rack ergonomics movie: two RTX renders side by side (current | improved), live stats
under each (both workers' RULA, body parts, spine load, running time-at-risk counters), a title card and a results
card; encoded with ffmpeg.

    python rack_movie_compose.py [--test N]       (--test: write N sample PNGs instead of the movie)
"""
import csv
import json
import os
import subprocess
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = r"C:\OmniverseData\RackMovie"
PLAN = os.path.join(ROOT, "plan.json")
CSV = {"current": r"C:\OmniverseData\Rack_Ergo\rula_per_frame.csv",
       "improved": r"C:\OmniverseData\Rack_Ergo_Improved\rula_per_frame.csv"}
OUT = os.path.join(ROOT, "Rack_Ergonomics_Current_vs_Improved.mp4")
W, H = 1920, 1080
FPS = 30
INTRO, OUTRO, FADE = 3.5, 6.5, 0.6
BG = (16, 18, 22)
PANEL = (28, 31, 37)
FG = (238, 239, 242)
DIM = (150, 155, 165)
GRID = (60, 64, 72)
METHOD = {"current": ("CURRENT METHOD", (245, 130, 50), "beams on 40 cm trestles  ·  frame top on a 34 cm block  ·  anchors drilled crouching"),
          "improved": ("IMPROVED METHOD", (70, 200, 110), "beams on 80 cm trestles  ·  frame top on an 80 cm sawhorse  ·  stand-up drilling")}
BANDS = [(2, "ACCEPTABLE", (46, 170, 80)), (4, "INVESTIGATE", (240, 200, 30)), (6, "CHANGE SOON", (245, 120, 30)),
         (7, "CHANGE NOW", (215, 40, 40))]
AL, MPL = 3400.0, 6400.0


def font(size, bold=False):
    return ImageFont.truetype("segoeuib.ttf" if bold else "segoeui.ttf", size)


F = {k: font(*v) for k, v in {"t": (30, True), "h": (22, True), "s": (18, False), "sb": (18, True), "n": (54, True),
                              "xs": (15, False), "big": (64, True), "huge": (96, True), "m": (26, False)}.items()}


def band(s):
    for hi, name, c in BANDS:
        if s <= hi:
            return name, c
    return BANDS[-1][1], BANDS[-1][2]


def part_col(v, g, y):
    return (46, 170, 80) if v <= g else ((240, 200, 30) if v <= y else (215, 40, 40))


def spine_col(n):
    return (215, 40, 40) if n >= AL else ((240, 200, 30) if n >= 2000 else (46, 170, 80))


def load_csv(path):
    rows = {}
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            d = {k: (float(v) if k not in ("worker", "step", "band") else v) for k, v in r.items()}
            rows.setdefault(r["worker"], []).append(d)
    data = {}
    for who, rs in rows.items():
        fr = np.array([r["frame"] for r in rs])
        red = np.cumsum([(1 / FPS) if r["rula"] >= 5 else 0.0 for r in rs])
        over = np.cumsum([(1 / FPS) if r["l5s1_N"] >= AL else 0.0 for r in rs])
        data[who] = (fr, rs, red, over)
    return data


def at(data, who, frame):
    fr, rs, red, over = data[who]
    i = int(np.clip(np.searchsorted(fr, frame, side="right") - 1, 0, len(rs) - 1))
    return rs[i], red[i], over[i]


def worker_card(d, x, y, w, h, who, r):
    d.rounded_rectangle((x, y, x + w, y + h), radius=14, fill=PANEL)
    s = int(r["rula"])
    name, col = band(s)
    d.text((x + 18, y + 12), f"Worker {who}", fill=FG, font=F["h"])
    d.rounded_rectangle((x + 18, y + 46, x + 118, y + 132), radius=12, fill=col)
    d.text((x + 68, y + 89), str(s), fill=(18, 18, 18), font=F["n"], anchor="mm")
    d.text((x + 132, y + 48), name, fill=col, font=F["sb"])
    d.text((x + 132, y + 74), f"back {max(r['trunk_deg'], 0):.0f}°   neck {r['neck_deg']:.0f}°", fill=DIM, font=F["s"])
    d.text((x + 132, y + 98), f"arms {r['ua_L_deg']:.0f}° / {r['ua_R_deg']:.0f}°   knee {r['knee_deg']:.0f}°", fill=DIM, font=F["s"])
    d.text((x + 132, y + 122), f"load {r['load_kg']:.1f} kg" if r["load_kg"] >= 0.5 else "no load", fill=DIM, font=F["s"])
    parts = [("back", r["trunk"], 2, 3), ("neck", r["neck"], 2, 3), ("arm L", r["upper_arm_L"], 2, 3),
             ("arm R", r["upper_arm_R"], 2, 3), ("legs", r["legs"], 1, 1)]
    bw = (w - 36 - 4 * 6) / 5
    for k, (lab, v, g, yv) in enumerate(parts):
        bx = x + 18 + k * (bw + 6)
        d.rounded_rectangle((bx, y + 146, bx + bw, y + 172), radius=6, fill=part_col(v, g, yv))
        d.text((bx + bw / 2, y + 159), lab, fill=(18, 18, 18), font=F["xs"], anchor="mm")
    n = r["l5s1_N"]
    sx0, sx1, sy = x + 18, x + w - 18, y + 204
    d.text((sx0, sy - 22), "spine (L5/S1)", fill=DIM, font=F["xs"])
    d.text((sx1, sy - 22), f"{n:,.0f} N", fill=spine_col(n), font=F["sb"], anchor="ra")
    d.rounded_rectangle((sx0, sy, sx1, sy + 16), radius=5, fill=(44, 48, 56))
    d.rounded_rectangle((sx0, sy, sx0 + max(6, (sx1 - sx0) * min(n, MPL) / MPL), sy + 16), radius=5, fill=spine_col(n))
    xa = sx0 + (sx1 - sx0) * AL / MPL
    d.line((xa, sy - 4, xa, sy + 20), fill=FG, width=2)
    d.text((xa + 4, sy + 18), "NIOSH limit", fill=DIM, font=F["xs"])


def method_stats(d, x, y, w, m, data, frame, other_red, other_over):
    (title, col, desc) = METHOD[m]
    cw = (w - 12) / 2
    rows = {who: at(data, who, frame) for who in ("A", "B")}
    for k, who in enumerate(("A", "B")):
        worker_card(d, x + k * (cw + 12), y, cw, 246, who, rows[who][0])
    red = sum(rows[w_][1] for w_ in rows)
    over = sum(rows[w_][2] for w_ in rows)
    yy = y + 262
    d.rounded_rectangle((x, yy, x + w, yy + 110), radius=14, fill=PANEL)
    d.text((x + 18, yy + 10), "Running totals, both workers", fill=DIM, font=F["s"])
    for j, (lab, v, ov, mx) in enumerate((("time at high risk (RULA 5-7)", red, other_red, 130.0),
                                          ("time over the spine limit", over, other_over, 20.0))):
        by = yy + 40 + j * 34
        d.text((x + 18, by), lab, fill=FG, font=F["s"])
        bx0, bx1 = x + 300, x + w - 110
        d.rounded_rectangle((bx0, by + 4, bx1, by + 22), radius=5, fill=(44, 48, 56))
        d.rounded_rectangle((bx0, by + 4, bx0 + max(4, (bx1 - bx0) * min(v, mx) / mx), by + 22), radius=5, fill=col)
        d.text((x + w - 18, by), f"{v:5.1f} s", fill=FG, font=F["sb"], anchor="ra")


def frame_image(k, plan, data, cache):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    tc = plan["times"][k]
    pw, ph = 952, 536
    ref = at(data["current"], "A", tc)[0]
    step = ref["step"] or ""
    d.text((24, 18), "Shelving rack assembly  ·  live ergonomics", fill=FG, font=F["t"])
    d.text((W / 2 + 120, 22), step, fill=(255, 210, 120), font=F["h"])
    d.text((W - 24, 22), f"{tc / FPS:5.1f} s  (x{(plan['times'][min(k + 1, len(plan['times']) - 1)] - tc):.1f})", fill=DIM,
           font=F["s"], anchor="ra")
    reds = {m: sum(at(data[m], w_, tc)[1] for w_ in ("A", "B")) for m in data}
    overs = {m: sum(at(data[m], w_, tc)[2] for w_ in ("A", "B")) for m in data}
    for j, m in enumerate(("current", "improved")):
        x0 = 8 + j * (pw + 8)
        p = os.path.join(ROOT, m, f"f{k:05d}.png")
        if os.path.exists(p):
            im = Image.open(p).convert("RGB")
            img.paste(im.resize((pw, ph), Image.LANCZOS), (x0, 70))
        title, col, desc = METHOD[m]
        ov = Image.new("RGBA", (pw, 64), (0, 0, 0, 150))
        img.paste(ov, (x0, 70 + ph - 64), ov)
        d.rounded_rectangle((x0 + 14, 84, x0 + 14 + 250, 120), radius=8, fill=col)
        d.text((x0 + 139, 102), title, fill=(15, 15, 15), font=F["sb"], anchor="mm")
        d.text((x0 + 16, 70 + ph - 50), desc, fill=FG, font=F["s"])
        method_stats(d, x0, 70 + ph + 12, pw, m, data[m], tc, None, None)
    # step progress strip
    steps = cache.get("steps")
    if steps is None:
        steps, prev = [], None
        fr, rs, _, _ = data["current"]["A"]
        for r in rs:
            if r["step"] != prev:
                if steps:
                    steps[-1][2] = r["frame"]
                if r["step"]:
                    steps.append([r["step"], r["frame"], r["frame"]])
                prev = r["step"]
        if steps:
            steps[-1][2] = fr[-1]
        cache["steps"] = steps
    t_end = data["current"]["A"][0][-1]
    x0, x1, y0 = 8, W - 8, 1012
    X = lambda f: x0 + (x1 - x0) * f / t_end
    for lab, a0, a1 in steps:
        on = a0 <= tc < a1
        d.rounded_rectangle((X(a0) + 2, y0, X(a1) - 2, y0 + 40), radius=6, fill=(60, 52, 30) if on else PANEL)
        d.text(((X(a0) + X(a1)) / 2, y0 + 20), lab.split(" ", 1)[0] + "  " + lab.split(" ", 1)[1][:22],
               fill=(255, 210, 120) if on else DIM, font=F["xs"], anchor="mm")
    d.line((X(tc), y0 - 4, X(tc), y0 + 44), fill=FG, width=3)
    return img


def card(lines, t, dur):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    y = 300
    for text, f, col in lines:
        d.text((W / 2, y), text, fill=col, font=F[f], anchor="mm")
        y += {"huge": 120, "big": 90, "t": 60, "m": 50, "h": 46, "s": 36}[f]
    a = min(1.0, t / FADE, (dur - t) / FADE)
    return Image.blend(Image.new("RGB", (W, H), (0, 0, 0)), img, max(0.0, a))


def totals(data):
    out = {}
    for m, dd in data.items():
        out[m] = {"red": sum(dd[w_][2][-1] for w_ in dd), "over": sum(dd[w_][3][-1] for w_ in dd),
                  "drill_spine": {w_: max(r["l5s1_N"] for r in dd[w_][1] if r["step"].startswith("4")) for w_ in dd}}
    return out


def main():
    plan = json.load(open(PLAN))
    data = {m: load_csv(p) for m, p in CSV.items()}
    tot = totals(data)
    c, i = tot["current"], tot["improved"]
    intro = [("Shelving rack assembly", "huge", FG), ("Ergonomics, live: current method vs improved method", "t", (255, 210, 120)),
             ("Two workers, same rack, same steps. Each frame scored with RULA and a lower-back (L5/S1) load model.", "m", DIM),
             ("Improved: beams at waist height  ·  frame on a sawhorse  ·  stand-up drilling", "m", (70, 200, 110))]
    pct_red = 100 * (1 - i["red"] / c["red"])
    pct_over = 100 * (1 - i["over"] / max(c["over"], 1e-6))
    pct_drill = 100 * (1 - i["drill_spine"]["A"] / c["drill_spine"]["A"])
    outro = [("Results", "big", FG),
             (f"Time at high risk (RULA 5-7):  {c['red']:.0f} s  ->  {i['red']:.0f} s   (-{pct_red:.0f}%)", "t", (70, 200, 110)),
             (f"Time over the NIOSH spine limit:  {c['over']:.1f} s  ->  {i['over']:.1f} s   (-{pct_over:.0f}%)", "t", (70, 200, 110)),
             (f"Drilling the anchors, back load:  {c['drill_spine']['A']:,.0f} N  ->  {i['drill_spine']['A']:,.0f} N   (-{pct_drill:.0f}%)", "t", (70, 200, 110)),
             ("Still red: walking the 3 m frames up (12.5 kg each, overhead)  ->  next: a frame-lifting aid", "m", (245, 130, 50)),
             ("Simulated motion (NVIDIA Omniverse); RULA + static L5/S1 model. For comparing methods, not a clinical measurement.", "s", DIM)]
    test = int(sys.argv[sys.argv.index("--test") + 1]) if "--test" in sys.argv else 0
    if test:
        for k in np.linspace(0, len(plan["times"]) - 1, test).astype(int):
            if os.path.exists(os.path.join(ROOT, "current", f"f{k:05d}.png")):
                frame_image(k, plan, data, {}).save(os.path.join(ROOT, f"test_{k:05d}.png"))
        card(intro, 1.5, INTRO).save(os.path.join(ROOT, "test_intro.png"))
        card(outro, 3.0, OUTRO).save(os.path.join(ROOT, "test_outro.png"))
        return
    import imageio_ffmpeg
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    proc = subprocess.Popen([ff, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
                             "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", OUT],
                            stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
    n_in, n_out = int(INTRO * FPS), int(OUTRO * FPS)
    for f in range(n_in):
        proc.stdin.write(card(intro, f / FPS, INTRO).tobytes())
    last = None
    cut_file = os.path.join(ROOT, "cut.txt")          # end before the floor heat map is revealed
    n_main = int(open(cut_file).read()) if os.path.exists(cut_file) else len(plan["times"])
    for k in range(n_main):
        last = frame_image(k, plan, data, {})
        if k < FADE * FPS:
            last = Image.blend(Image.new("RGB", (W, H), (0, 0, 0)), last, k / (FADE * FPS))
        proc.stdin.write(last.tobytes())
        if k % 300 == 0:
            print("frame", k)
    for f in range(int(FADE * FPS)):
        proc.stdin.write(Image.blend(last, Image.new("RGB", (W, H), (0, 0, 0)), f / (FADE * FPS)).tobytes())
    for f in range(n_out):
        proc.stdin.write(card(outro, f / FPS, OUTRO).tobytes())
    proc.stdin.close()
    proc.wait()
    print("->", OUT, f"{(n_in + n_main + FADE * FPS + n_out) / FPS:.1f} s")


if __name__ == "__main__":
    main()
