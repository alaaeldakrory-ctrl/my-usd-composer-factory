"""The Ergonomics window (part of Factory Sim; Window > Ergonomics): live RULA / spine-load readout of the open assembly demo, synced to the timeline.

Reads the per-frame scores the demo build wrote (rula_per_frame.csv, found from the open stage: /World/Ergo
customData "ergo:csv", else the C:/OmniverseData/<name>_Ergo convention). View switches are session-layer edits
(nothing is saved into the scene).
"""
import csv
import os

import carb
import omni.kit.app
import omni.timeline
import omni.ui as ui
import omni.usd
from pxr import Sdf, Usd

BANDS = [(2, "ACCEPTABLE", 0xFF50AA2E), (4, "INVESTIGATE", 0xFF1EC8F0), (6, "CHANGE SOON", 0xFF1E78F5),
         (7, "CHANGE NOW", 0xFF2828D7)]           # ABGR
NIOSH_AL, NIOSH_MPL = 3400.0, 6400.0
GREY = 0xFF9AA0A6
WHITE = 0xFFEEEEEE
DARK = 0xFF302B28


def band(score):
    for hi, name, col in BANDS:
        if score <= hi:
            return name, col
    return BANDS[-1][1], BANDS[-1][2]


def spine_col(n):
    return 0xFF2828D7 if n >= NIOSH_AL else (0xFF1EC8F0 if n >= 2000 else 0xFF50AA2E)


def csv_for_stage(stage):
    """The demo's per-frame ergonomics CSV for the open stage, or None."""
    if not stage:
        return None
    prim = stage.GetPrimAtPath("/World/Ergo")
    if prim:
        p = prim.GetCustomDataByKey("ergo:csv")
        if p and os.path.exists(p):
            return p
    path = stage.GetRootLayer().realPath or ""
    for layer in stage.GetLayerStack():
        if layer.realPath and layer.realPath.lower().endswith(".usd"):
            path = layer.realPath
            break
    name = os.path.splitext(os.path.basename(path))[0]               # Rack_Assembly[_Improved][_XRay]
    name = name.replace("_XRay", "")
    base = name.split("_")[0]                                         # Rack
    suffix = name[len(base + "_Assembly"):]                           # "" or "_Improved"
    cand = os.path.join(os.path.dirname(path), f"{base}_Ergo{suffix}", "rula_per_frame.csv")
    return cand if os.path.exists(cand) else None


class ErgoData:
    """Per worker: rows by frame (dicts of floats / strings)."""

    def __init__(self, path):
        self.path = path
        self.rows = {}
        with open(path, newline="") as fh:
            for r in csv.DictReader(fh):
                d = {k: (float(v) if k not in ("worker", "step", "band") and v not in ("", None) else v)
                     for k, v in r.items()}
                self.rows.setdefault(r["worker"], []).append(d)
        self.frames = {w: [int(r["frame"]) for r in rows] for w, rows in self.rows.items()}
        self.steps = []
        for r in next(iter(self.rows.values()), []):
            if r["step"] and (not self.steps or self.steps[-1] != r["step"]):
                self.steps.append(r["step"])

    def at(self, who, frame):
        fr = self.frames[who]
        lo, hi = 0, len(fr) - 1
        while lo < hi:
            m = (lo + hi + 1) // 2
            if fr[m] <= frame:
                lo = m
            else:
                hi = m - 1
        return self.rows[who][lo]

    def step_stats(self, who, step):
        rows = [r for r in self.rows[who] if r["step"] == step]
        if not rows:
            return None
        dt = (rows[1]["t"] - rows[0]["t"]) if len(rows) > 1 else 1 / 30
        worst = max(rows, key=lambda r: (r["rula"], r.get("l5s1_N", 0)))
        return {"peak": int(worst["rula"]), "red": sum(dt for r in rows if r["rula"] >= 5),
                "spine": max(r.get("l5s1_N", 0.0) for r in rows), "worst_frame": int(worst["frame"])}


class ErgoWindow:
    def __init__(self):
        old = ui.Workspace.get_window("Ergonomics")
        if old is not None:
            old.visible = False
        self._data = None
        self._compare = None
        self._last_frame = None
        self._w = {}
        self.window = ui.Window("Ergonomics", width=440, height=820)
        self._build()
        self._sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._tick, name="ergo panel")
        self._stage_sub = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            lambda e: self.reload(), name="ergo stage")
        self.reload()

    def destroy(self):
        self._sub = None
        self._stage_sub = None
        if self.window:
            self.window.destroy()
            self.window = None

    # ------------------------------------------------------------------ layout
    def _build(self):
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6, height=0):
                    self._status = ui.Label("", word_wrap=True, style={"color": GREY})
                    self._step = ui.Label("", style={"font_size": 18, "color": WHITE})
                    for who in ("A", "B"):
                        self._worker_block(who)
                    with ui.CollapsableFrame("View", collapsed=False):
                        with ui.VStack(spacing=4, height=0):
                            with ui.HStack(height=26, spacing=4):
                                ui.Button("Dots + rings", clicked_fn=lambda: self._view("markers"))
                                ui.Button("X-ray bones", clicked_fn=lambda: self._view("xray"))
                                ui.Button("Hide all", clicked_fn=lambda: self._view("off"))
                            with ui.HStack(height=26, spacing=4):
                                ui.Button("Floor heat: show", clicked_fn=lambda: self._heat(True))
                                ui.Button("Floor heat: end only", clicked_fn=lambda: self._heat(None))
                            ui.Button("Open see-through version (_XRay.usda)", height=26, clicked_fn=self._open_xray)
                    with ui.CollapsableFrame("Steps (click to jump to the worst moment)", collapsed=False):
                        self._steps_frame = ui.VStack(spacing=2, height=0)
                    with ui.CollapsableFrame("Current vs improved method", collapsed=True):
                        self._cmp_frame = ui.VStack(spacing=2, height=0)

    def _worker_block(self, who):
        w = {}
        with ui.ZStack(height=0):
            ui.Rectangle(style={"background_color": 0xFF1E1A18, "border_radius": 8})
            with ui.VStack(spacing=4, height=0):
                ui.Spacer(height=4)
                with ui.HStack(height=0, spacing=8):
                    ui.Spacer(width=6)
                    with ui.ZStack(width=70, height=60):
                        w["tile"] = ui.Rectangle(style={"background_color": 0xFF50AA2E, "border_radius": 8})
                        w["score"] = ui.Label("-", alignment=ui.Alignment.CENTER,
                                              style={"font_size": 40, "color": 0xFF141414})
                    with ui.VStack(spacing=1):
                        ui.Label(f"Worker {who}", style={"font_size": 18, "color": WHITE})
                        w["band"] = ui.Label("", style={"font_size": 16})
                        w["angles"] = ui.Label("", style={"color": GREY})
                        w["load"] = ui.Label("", style={"color": GREY})
                with ui.HStack(height=18, spacing=6):
                    ui.Spacer(width=6)
                    ui.Label("Parts", width=40, style={"color": GREY})
                    for part in ("back", "neck", "arm L", "arm R", "legs"):
                        with ui.ZStack(width=62):
                            w["part_" + part] = ui.Rectangle(style={"background_color": DARK, "border_radius": 4})
                            ui.Label(part, alignment=ui.Alignment.CENTER, style={"color": 0xFF141414})
                with ui.HStack(height=20, spacing=6):
                    ui.Spacer(width=6)
                    ui.Label("Spine", width=40, style={"color": GREY})
                    with ui.ZStack():
                        ui.Rectangle(style={"background_color": DARK, "border_radius": 3})
                        with ui.HStack():
                            w["spine_bar"] = ui.Rectangle(width=ui.Fraction(0.1), style={"background_color": 0xFF50AA2E})
                            w["spine_rest"] = ui.Spacer(width=ui.Fraction(0.9))
                        with ui.HStack():                       # NIOSH action limit mark
                            ui.Spacer(width=ui.Fraction(NIOSH_AL / NIOSH_MPL))
                            ui.Rectangle(width=2, style={"background_color": WHITE})
                            ui.Spacer(width=ui.Fraction(1 - NIOSH_AL / NIOSH_MPL))
                    w["spine"] = ui.Label("", width=90, style={"color": GREY})
                    ui.Spacer(width=6)
                ui.Spacer(height=4)
        self._w[who] = w

    # ------------------------------------------------------------------ data
    def reload(self):
        stage = omni.usd.get_context().get_stage()
        path = csv_for_stage(stage)
        self._data = None
        self._last_frame = None
        if not path:
            self._status.text = ("No ergonomics data for this stage. Open C:/OmniverseData/Rack_Assembly.usd "
                                 "(or _Improved / _XRay) built with the ergonomics pass.")
            return
        try:
            self._data = ErgoData(path)
        except Exception as e:   # noqa: BLE001
            self._status.text = f"Could not read {path}: {e}"
            return
        method = "improved method" if "Improved" in path else "current method"
        self._status.text = f"{method}  |  {os.path.basename(os.path.dirname(path))}"
        other = path.replace("Rack_Ergo_Improved", "Rack_Ergo") if "Improved" in path else \
            path.replace("Rack_Ergo", "Rack_Ergo_Improved")
        self._compare = ErgoData(other) if os.path.exists(other) and other != path else None
        self._fill_steps()
        self._fill_compare(path)

    def _fill_steps(self):
        self._steps_frame.clear()
        d = self._data
        with self._steps_frame:
            with ui.HStack(height=20):
                ui.Label("Step", style={"color": GREY})
                for h in ("peak", "at 5-7", "spine"):
                    ui.Label(h, width=58, style={"color": GREY})
            for st in d.steps:
                for who in d.rows:
                    s = d.step_stats(who, st)
                    if not s:
                        continue
                    with ui.HStack(height=22):
                        ui.Button(f"{st}  [{who}]", alignment=ui.Alignment.LEFT_CENTER,
                                  clicked_fn=lambda f=s["worst_frame"]: self._jump(f))
                        ui.Label(f" {s['peak']}", width=58, style={"color": band(s["peak"])[1]})
                        ui.Label(f"{s['red']:.1f} s", width=58, style={"color": WHITE})
                        ui.Label(f"{s['spine']:,.0f}", width=58, style={"color": spine_col(s["spine"])})

    def _fill_compare(self, path):
        self._cmp_frame.clear()
        with self._cmp_frame:
            if not self._compare:
                ui.Label("Build both methods (build_rack.py and build_rack.py --improved) to compare.",
                         word_wrap=True, style={"color": GREY})
                return
            cur, imp = (self._compare, self._data) if "Improved" in path else (self._data, self._compare)
            with ui.HStack(height=20):
                ui.Label("Step / worker", style={"color": GREY})
                ui.Label("time at 5-7", width=110, style={"color": GREY})
                ui.Label("spine peak", width=110, style={"color": GREY})
            tot = [0.0, 0.0]
            for st in cur.steps:
                for who in cur.rows:
                    a, b = cur.step_stats(who, st), imp.step_stats(who, st)
                    if not a or not b:
                        continue
                    tot[0] += a["red"]
                    tot[1] += b["red"]
                    better = 0xFF50AA2E if b["red"] < a["red"] - 0.3 else WHITE
                    with ui.HStack(height=20):
                        ui.Label(f"{st} [{who}]", style={"color": WHITE})
                        ui.Label(f"{a['red']:.1f} -> {b['red']:.1f} s", width=110, style={"color": better})
                        ui.Label(f"{a['spine']:,.0f} -> {b['spine']:,.0f}", width=110,
                                 style={"color": 0xFF50AA2E if b["spine"] < a["spine"] - 150 else WHITE})
            ui.Label(f"Whole job: {tot[0]:.0f} s -> {tot[1]:.0f} s at RULA 5-7 "
                     f"({100 * (1 - tot[1] / max(tot[0], 1e-6)):.0f}% less)", style={"color": WHITE, "font_size": 16})

    # ------------------------------------------------------------------ live update
    def _tick(self, _e):
        if not self._data or not self.window or not self.window.visible:
            return
        tl = omni.timeline.get_timeline_interface()
        fps = tl.get_time_codes_per_seconds() or 30.0
        frame = int(round(tl.get_current_time() * fps))
        if frame == self._last_frame:
            return
        self._last_frame = frame
        step = ""
        for who, w in self._w.items():
            if who not in self._data.rows:
                continue
            r = self._data.at(who, frame)
            s = int(r["rula"])
            name, col = band(s)
            w["score"].text = str(s)
            w["tile"].set_style({"background_color": col, "border_radius": 8})
            w["band"].text = name
            w["band"].set_style({"font_size": 16, "color": col})
            w["angles"].text = (f"back {max(r['trunk_deg'], 0):.0f}°  neck {r['neck_deg']:.0f}°  "
                                f"arms {r['ua_L_deg']:.0f}°/{r['ua_R_deg']:.0f}°  knee {r['knee_deg']:.0f}°")
            w["load"].text = f"load {r['load_kg']:.1f} kg" if r["load_kg"] >= 0.5 else "no load"
            parts = {"back": (r["trunk"], 2, 3), "neck": (r["neck"], 2, 3), "arm L": (r["upper_arm_L"], 2, 3),
                     "arm R": (r["upper_arm_R"], 2, 3), "legs": (r["legs"], 1, 1)}
            for p, (v, g, y) in parts.items():
                c = 0xFF50AA2E if v <= g else (0xFF1EC8F0 if v <= y else 0xFF2828D7)
                w["part_" + p].set_style({"background_color": c, "border_radius": 4})
            n = r.get("l5s1_N", 0.0) or 0.0
            u = max(0.01, min(1.0, n / NIOSH_MPL))
            w["spine_bar"].width = ui.Fraction(u)
            w["spine_rest"].width = ui.Fraction(1.0 - u)
            w["spine_bar"].set_style({"background_color": spine_col(n)})
            w["spine"].text = f"{n:,.0f} N"
            step = r["step"] or step
        self._step.text = step

    # ------------------------------------------------------------------ actions
    def _jump(self, frame):
        tl = omni.timeline.get_timeline_interface()
        tl.pause()
        tl.set_current_time(frame / (tl.get_time_codes_per_seconds() or 30.0))

    def _session_edit(self):
        stage = omni.usd.get_context().get_stage()
        if stage:
            stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
        return stage

    def _view(self, mode):
        stage = omni.usd.get_context().get_stage()
        if not stage or not stage.GetPrimAtPath("/World/Ergo"):
            return
        prev = stage.GetEditTarget()
        self._session_edit()
        try:
            ergo = stage.GetPrimAtPath("/World/Ergo")
            for who in ("A", "B"):
                p = stage.GetPrimAtPath(f"/World/Ergo/{who}")
                if p:
                    p.CreateAttribute("visibility", Sdf.ValueTypeNames.Token).Set("invisible" if mode == "off" else "inherited")
            if mode in ("markers", "xray"):
                vs = ergo.GetVariantSets().GetVariantSet("ergoView")
                if vs:
                    vs.SetVariantSelection(mode)
        finally:
            stage.SetEditTarget(prev)

    def _heat(self, show):
        stage = omni.usd.get_context().get_stage()
        if not stage or not stage.GetPrimAtPath("/World/Ergo/FloorHeat"):
            return
        prev = stage.GetEditTarget()
        self._session_edit()
        try:
            attr = stage.GetPrimAtPath("/World/Ergo/FloorHeat").GetAttribute("visibility")
            if show:
                attr.Set("inherited")                  # a session default beats the scene's time samples
            else:
                attr.Clear()
        finally:
            stage.SetEditTarget(prev)

    def _open_xray(self):
        stage = omni.usd.get_context().get_stage()
        path = stage.GetRootLayer().realPath if stage else ""
        if not path:
            return
        x = os.path.splitext(path)[0].replace("_XRay", "") + "_XRay.usda"
        if os.path.exists(x):
            omni.usd.get_context().open_stage(x)
        else:
            carb.log_warn(f"[ergo] no see-through version next to {path}")
