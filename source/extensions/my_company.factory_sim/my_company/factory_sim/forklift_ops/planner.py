"""Plan forklift jobs: assign pallets to rack slots and forklifts, route around obstacles, and build a
time-stamped motion timeline per forklift that never collides with the others.

All in plan coordinates (x, z) in cm, headings in degrees (see scene.py). No USD writes here.
"""
import heapq
import math
from dataclasses import dataclass, field

from .scene import Box2, fwd_of, heading_of, left_of, wrap180

# ------------------------------------------------------------------------------------------------ settings
@dataclass
class Settings:
    drive_speed: float = 140.0      # cm/s
    creep_speed: float = 45.0
    accel: float = 90.0             # cm/s^2
    lift_speed: float = 50.0        # cm/s
    pivot_speed: float = 45.0       # deg/s
    fill: str = "bottom"            # "bottom" or "top" shelves first
    cell: float = 15.0              # planning grid (cm)
    margin: float = 25.0            # extra clearance around obstacles (cm)
    standoff_extra: float = 40.0    # fork tips stop this far from a pallet / rack face before going in
    lead_in: float = 200.0          # straight run lined up before arriving
    fillet_max: float = 240.0
    fillet_min: float = 110.0
    start_delay: float = 1.0
    return_home: bool = True
    max_wait: float = 90.0
    mode: str = "store"             # "store": floor pallets -> racks, "retrieve": rack pallets -> dock
    max_jobs: int = 0               # 0 = every pallet


# ------------------------------------------------------------------------------------------------ geometry
def _rect(pos, heading, back, front, half):
    f, l = fwd_of(heading), left_of(heading)
    c = (pos[0] + f[0] * (front - back) / 2, pos[1] + f[1] * (front - back) / 2)
    return Box2(c=c, u=f, hu=(front + back) / 2, hv=half)


def _sat(a, b):
    pa, pb = a.corners(), b.corners()
    for poly in (pa, pb):
        for i in range(4):
            p, q = poly[i], poly[(i + 1) % 4]
            n = (q[1] - p[1], p[0] - q[0])
            ra = [n[0] * x + n[1] * y for x, y in pa]
            rb = [n[0] * x + n[1] * y for x, y in pb]
            if max(ra) < min(rb) or max(rb) < min(ra):
                return False
    return True


def _point_box_dist(p, b):
    dx, dz = p[0] - b.c[0], p[1] - b.c[1]
    u = dx * b.u[0] + dz * b.u[1]
    v = dx * b.v[0] + dz * b.v[1]
    eu, ev = max(abs(u) - b.hu, 0.0), max(abs(v) - b.hv, 0.0)
    return math.hypot(eu, ev)


# ------------------------------------------------------------------------------------------------ grid + A*
class Grid:
    def __init__(self, lo, hi, cell):
        self.lo, self.cell = lo, cell
        self.nx = int((hi[0] - lo[0]) / cell) + 1
        self.nz = int((hi[1] - lo[1]) / cell) + 1
        self.blocked = bytearray(self.nx * self.nz)

    def idx(self, i, k):
        return k * self.nx + i

    def cell_of(self, p):
        return (min(max(int((p[0] - self.lo[0]) / self.cell + 0.5), 0), self.nx - 1),
                min(max(int((p[1] - self.lo[1]) / self.cell + 0.5), 0), self.nz - 1))

    def center(self, i, k):
        return (self.lo[0] + i * self.cell, self.lo[1] + k * self.cell)

    def add_box(self, b, inflate):
        r = max(b.hu, b.hv) * 1.42 + inflate
        i0, k0 = self.cell_of((b.c[0] - r, b.c[1] - r))
        i1, k1 = self.cell_of((b.c[0] + r, b.c[1] + r))
        for k in range(k0, k1 + 1):
            for i in range(i0, i1 + 1):
                if _point_box_dist(self.center(i, k), b) <= inflate:
                    self.blocked[self.idx(i, k)] = 1

    def free(self, p):
        i, k = self.cell_of(p)
        return not self.blocked[self.idx(i, k)]

    def clear_disc(self, p, r):
        i0, k0 = self.cell_of((p[0] - r, p[1] - r))
        i1, k1 = self.cell_of((p[0] + r, p[1] + r))
        for k in range(k0, k1 + 1):
            for i in range(i0, i1 + 1):
                c = self.center(i, k)
                if math.dist(c, p) <= r:
                    self.blocked[self.idx(i, k)] = 0

    def line_free(self, a, b):
        n = max(2, int(math.dist(a, b) / (self.cell * 0.5)))
        for s in range(n + 1):
            t = s / n
            if not self.free((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)):
                return False
        return True

    def astar(self, a, b):
        s, g = self.cell_of(a), self.cell_of(b)
        if self.blocked[self.idx(*s)] or self.blocked[self.idx(*g)]:
            return None
        nx = self.nx
        D2 = math.sqrt(2)
        nbrs = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, D2), (1, -1, D2), (-1, 1, D2), (-1, -1, D2)]

        def h(i, k):
            dx, dz = abs(i - g[0]), abs(k - g[1])
            return (dx + dz) + (D2 - 2) * min(dx, dz)
        start = self.idx(*s)
        best = {start: 0.0}
        came = {}
        heap = [(h(*s), 0.0, s[0], s[1])]
        goal = self.idx(*g)
        while heap:
            _, cost, i, k = heapq.heappop(heap)
            cur = k * nx + i
            if cur == goal:
                break
            if cost > best.get(cur, 1e18):
                continue
            for di, dk, w in nbrs:
                ni, nk = i + di, k + dk
                if not (0 <= ni < nx and 0 <= nk < self.nz):
                    continue
                nid = nk * nx + ni
                if self.blocked[nid]:
                    continue
                if di and dk and (self.blocked[k * nx + ni] or self.blocked[nk * nx + i]):
                    continue
                nc = cost + w
                if nc < best.get(nid, 1e18):
                    best[nid] = nc
                    came[nid] = cur
                    heapq.heappush(heap, (nc + h(ni, nk), nc, ni, nk))
        if goal not in best:
            return None
        path, cur = [], goal
        while True:
            path.append(self.center(cur % nx, cur // nx))
            if cur == start:
                break
            cur = came[cur]
        path.reverse()
        path[0], path[-1] = a, b
        # string pulling
        out, i = [path[0]], 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.line_free(path[i], path[j]):
                j -= 1
            out.append(path[j])
            i = j
        return out


# ------------------------------------------------------------------------------------------------ motion
@dataclass
class Seg:
    """One motion piece. Path segments carry a dense polyline; pivots turn on the spot."""
    kind: str                 # "path", "pivot", "lift", "wait"
    t0: float
    dur: float
    pos0: tuple
    h0: float
    L0: float
    L1: float = None
    pts: list = None          # path: [(x, z)], heading follows the tangent
    lens: list = None
    direction: int = 1       # +1 forward, -1 reverse
    v: float = 0.0
    accel: float = 1.0
    h1: float = None          # pivot target heading
    note: str = ""

    def _s_at(self, t):
        """Distance travelled along a trapezoid speed profile."""
        S, v, a = self.lens[-1], self.v, self.accel
        ta = v / a
        if 2 * ta > self.dur:                     # triangle
            ta = self.dur / 2
            vp = a * ta
            if t < ta:
                return 0.5 * a * t * t
            return S - 0.5 * a * (self.dur - t) ** 2 if t < self.dur else S
        if t < ta:
            return 0.5 * a * t * t
        if t > self.dur - ta:
            return S - 0.5 * a * (self.dur - t) ** 2
        return 0.5 * a * ta * ta + v * (t - ta)

    def state(self, t):
        """(pos, heading, L, odo_delta) at absolute time t inside this segment."""
        u = min(max((t - self.t0) / self.dur, 0.0), 1.0) if self.dur > 0 else 1.0
        e = u * u * (3 - 2 * u)
        L = self.L0 if self.L1 is None else self.L0 + (self.L1 - self.L0) * e
        if self.kind == "path":
            s = min(max(self._s_at(t - self.t0), 0.0), self.lens[-1])
            pos, tan = _sample(self.pts, self.lens, s)
            h = tan if self.direction > 0 else wrap180(tan + 180.0)
            return pos, h, L, s * self.direction
        if self.kind == "pivot":
            return self.pos0, self.h0 + wrap180(self.h1 - self.h0) * e, L, 0.0
        return self.pos0, self.h0, L, 0.0

    @property
    def t1(self):
        return self.t0 + self.dur


def _sample(pts, lens, s):
    lo, hi = 0, len(lens) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if lens[mid] < s:
            lo = mid
        else:
            hi = mid
    seg = lens[hi] - lens[lo]
    f = 0.0 if seg <= 0 else (s - lens[lo]) / seg
    p = (pts[lo][0] + (pts[hi][0] - pts[lo][0]) * f, pts[lo][1] + (pts[hi][1] - pts[lo][1]) * f)
    return p, heading_of(pts[hi][0] - pts[lo][0], pts[hi][1] - pts[lo][1])


def _lens(pts):
    out = [0.0]
    for i in range(1, len(pts)):
        out.append(out[-1] + math.dist(pts[i], pts[i - 1]))
    return out


def _densify(a, b, step=4.0):
    n = max(1, int(math.dist(a, b) / step))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]


def _angle(d):
    return heading_of(d[0], d[1])


def route_pieces(start, h_start, waypoints, settings):
    """Turn a polyline into drivable pieces: [("pivot", pos, h_from, h_to) | ("path", pts)].
    Corners get a circular fillet when there is room, otherwise the forklift stops and turns on the spot."""
    pts = [start] + [w for w in waypoints if math.dist(w, start) > 1.0]
    clean = [pts[0]]
    for p in pts[1:]:
        if math.dist(p, clean[-1]) > 1.0:
            clean.append(p)
    pts = clean
    if len(pts) < 2:
        return []
    pieces = []
    h = h_start
    first = _angle((pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]))
    if abs(wrap180(first - h)) > 0.3:
        pieces.append(("pivot", pts[0], h, h + wrap180(first - h)))
    cur = [pts[0]]
    for i in range(1, len(pts) - 1):
        a, v, b = pts[i - 1], pts[i], pts[i + 1]
        din = _angle((v[0] - a[0], v[1] - a[1]))
        dout = _angle((b[0] - v[0], b[1] - v[1]))
        turn = wrap180(dout - din)
        th = abs(math.radians(turn))
        if th < math.radians(3):
            cur += _densify(cur[-1], v)
            continue
        lin, lout = math.dist(a, v), math.dist(v, b)
        R = min(settings.fillet_max, 0.5 * min(lin, lout) / math.tan(th / 2)) if th < math.radians(179) else 0
        if R >= settings.fillet_min and abs(turn) <= 120:
            tl = R * math.tan(th / 2)
            fi, fo = fwd_of(din), fwd_of(dout)
            p_in = (v[0] - fi[0] * tl, v[1] - fi[1] * tl)
            cur += _densify(cur[-1], p_in)
            side = 1.0 if turn > 0 else -1.0                     # heading increasing = turning to the left
            lft = left_of(din)
            nrm = (lft[0] * side, lft[1] * side)
            c = (p_in[0] + nrm[0] * R, p_in[1] + nrm[1] * R)
            n_arc = max(4, int(R * th / 4.0))
            a0 = math.atan2(p_in[1] - c[1], p_in[0] - c[0])
            p_out = (v[0] + fo[0] * tl, v[1] + fo[1] * tl)
            a1 = math.atan2(p_out[1] - c[1], p_out[0] - c[0])
            da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
            for k in range(1, n_arc + 1):
                ang = a0 + da * k / n_arc
                cur.append((c[0] + R * math.cos(ang), c[1] + R * math.sin(ang)))
        else:
            cur += _densify(cur[-1], v)
            pieces.append(("path", cur))
            pieces.append(("pivot", v, din, din + turn))
            cur = [v]
    cur += _densify(cur[-1], pts[-1])
    pieces.append(("path", cur))
    return [p for p in pieces if p[0] == "pivot" or len(p[1]) >= 2]


# ------------------------------------------------------------------------------------------------ agents
@dataclass
class PalletTrack:
    name: str
    rest_center: tuple
    rest_y: float
    half_fwd: float                 # half size along the forklift's forward axis when carried
    half_lat: float
    opening_top: float = 10.0
    events: list = field(default_factory=list)    # (t, "attach", agent, h_pick, t_lifted) / (t, "detach", center, y, dh, t_setdown)


@dataclass
class Agent:
    fl: object                      # scene.Forklift
    pos: tuple
    heading: float
    L: float = 0.0
    t: float = 0.0
    segs: list = field(default_factory=list)
    carrying: object = None         # PalletTrack
    log: list = field(default_factory=list)

    @property
    def end_t(self):
        return self.segs[-1].t1 if self.segs else 0.0

    def state(self, t):
        """(pos, heading, L, odo, seg) at time t; parked at the last pose outside the timeline."""
        if not self.segs or t <= self.segs[0].t0:
            return self.segs[0].state(self.segs[0].t0)[:3] + (0.0, None) if self.segs else (self.pos, self.heading, self.L, 0.0, None)
        lo, hi = 0, len(self.segs) - 1
        if t >= self.segs[-1].t1:
            s = self.segs[-1]
            return s.state(s.t1)[:3] + (0.0, s)
        while lo < hi:
            mid = (lo + hi) // 2
            if self.segs[mid].t1 < t:
                lo = mid + 1
            else:
                hi = mid
        s = self.segs[lo]
        return s.state(t)[:3] + (0.0, s)


class Builder:
    """Appends motion segments for one agent starting from its current state."""

    def __init__(self, agent, settings):
        self.a, self.S = agent, settings
        self.segs = []
        self.t, self.pos, self.h, self.L = agent.t, agent.pos, agent.heading, agent.L

    def _add(self, seg):
        self.segs.append(seg)
        self.t = seg.t1

    def wait(self, dt, note="wait"):
        if dt > 1e-3:
            self._add(Seg("wait", self.t, dt, self.pos, self.h, self.L, note=note))

    def lift(self, L, note="lift"):
        dur = abs(L - self.L) / self.S.lift_speed
        if dur > 1e-3:
            self._add(Seg("lift", self.t, max(dur, 0.6), self.pos, self.h, self.L, L, note=note))
            self.L = L

    def pivot(self, h_to, note="turn"):
        d = wrap180(h_to - self.h)
        if abs(d) > 0.5:
            self._add(Seg("pivot", self.t, 0.3 + abs(d) / self.S.pivot_speed, self.pos, self.h, self.L,
                          h1=self.h + d, note=note))
            self.h = self.h + d

    def path(self, pts, direction=1, speed=None, L_to=None, note="drive"):
        lens = _lens(pts)
        S = lens[-1]
        if S < 0.5:
            return
        v = speed or self.S.drive_speed
        a = self.S.accel
        dur = (S / v + v / a) if S > v * v / a else 2 * math.sqrt(S / a)
        self._add(Seg("path", self.t, dur, self.pos, self.h, self.L, L_to, pts=pts, lens=lens, direction=direction,
                      v=v, accel=a, note=note))
        end = pts[-1]
        tan = heading_of(pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1])
        self.pos = end
        self.h = tan if direction > 0 else wrap180(tan + 180)
        if L_to is not None:
            self.L = L_to

    def straight(self, dist, direction=1, speed=None, note="creep"):
        f = fwd_of(self.h)
        sgn = 1 if direction > 0 else -1
        end = (self.pos[0] + f[0] * dist * sgn, self.pos[1] + f[1] * dist * sgn)
        self.path([self.pos] + _densify(self.pos, end), direction, speed or self.S.creep_speed, note=note)

    def route(self, waypoints, note="drive"):
        for piece in route_pieces(self.pos, self.h, waypoints, self.S):
            if piece[0] == "pivot":
                self.pivot(piece[3], note=note)
            else:
                self.path(piece[1], 1, note=note)


# ------------------------------------------------------------------------------------------------ world
class World:
    def __init__(self, scene, settings):
        self.sc, self.S = scene, settings
        pts = []
        for fl in scene.forklifts:
            pts.append(fl.origin)
        for r in scene.racks:
            pts += r.box.corners()
        for p in scene.pallets:
            pts += p.box.corners()
        for b in scene.obstacles:
            pts += b.corners()
        pad = 900.0
        self.lo = (min(p[0] for p in pts) - pad, min(p[1] for p in pts) - pad)
        self.hi = (max(p[0] for p in pts) + pad, max(p[1] for p in pts) + pad)
        self.floor_pallets = {p.name: p for p in scene.pallets if p.base_y < 5.0}
        self.placed = {}            # pallet name -> Box2 of pallets put down on a dock (new floor obstacles)

    def static_boxes(self, exclude_pallet=None):
        out = [r.box for r in self.sc.racks] + list(self.sc.obstacles)
        out += [p.box for n, p in self.floor_pallets.items() if n != exclude_pallet]
        out += [b for n, b in self.placed.items() if n != exclude_pallet]
        return out

    def grid(self, inflate, exclude_pallet=None, parked=()):
        """Occupancy grid for a robot of radius `inflate`. The static part is cached (it only changes when a
        floor pallet is picked up); parked forklifts are stamped onto a copy."""
        key = (round(inflate, 1), exclude_pallet, frozenset(self.floor_pallets), frozenset(self.placed))
        cache = self.__dict__.setdefault("_grid_cache", {})
        base = cache.get(key)
        if base is None:
            base = Grid(self.lo, self.hi, self.S.cell)
            for b in self.static_boxes(exclude_pallet):
                base.add_box(b, inflate)
            cache[key] = base
        g = Grid(self.lo, self.hi, self.S.cell)
        g.blocked = bytearray(base.blocked)
        for b in parked:
            g.add_box(b, inflate)
        return g


def _footprint_circles(fl, pos, h, carrying):
    """Discs covering the forklift (and its load) for agent-vs-agent checks."""
    f = fwd_of(h)
    r = fl.half_width + 12.0
    out = []
    a = -fl.rear + r * 0.8
    while a < fl.front:
        out.append((pos[0] + f[0] * a, pos[1] + f[1] * a, r))
        a += r * 1.2
    reach = fl.fork_tip
    half = fl.fork_spread / 2 + 10.0
    if carrying is not None:
        reach = max(reach, fl.fork_heel + 2 + 2 * carrying.half_fwd)
        half = max(half, carrying.half_lat + 10.0)
    a = fl.front + half * 0.7
    while a < reach + half * 0.3:
        out.append((pos[0] + f[0] * a, pos[1] + f[1] * a, half))
        a += half * 1.2
    return out


class _Load:
    def __init__(self, pal):
        self.half_fwd = max(pal.box.hu, pal.box.hv)
        self.half_lat = max(pal.box.hu, pal.box.hv)


def _body_boxes(fl, pos, h, carrying):
    body = _rect(pos, h, fl.rear, fl.front, fl.half_width)
    reach = fl.fork_tip
    half = fl.fork_spread / 2
    if carrying is not None:
        reach = max(reach, fl.fork_heel + 2 + 2 * carrying.half_fwd)
        half = max(half, carrying.half_lat)
    f = fwd_of(h)
    front = _rect((pos[0] + f[0] * fl.front, pos[1] + f[1] * fl.front), h, 0.0, reach - fl.front, half)
    return body, front


# ------------------------------------------------------------------------------------------------ planner
@dataclass
class Plan:
    agents: list
    pallets: dict                   # name -> PalletTrack
    jobs: list
    end_t: float
    warnings: list


class Planner:
    def __init__(self, scene, settings=None, log=print):
        self.sc, self.S, self.log = scene, settings or Settings(), log
        self.W = World(scene, self.S)
        self.agents = [Agent(fl=fl, pos=fl.origin, heading=fl.heading, t=self.S.start_delay) for fl in scene.forklifts]
        self.tracks = {}
        self.warnings = []
        self.jobs = []
        self._homing = False
        self._pending_carry = None

    # ---------------------------------------------------------------- conflict checks
    def _conflict(self, me, segs, t0, t1, ignore=None):
        """(time, other agent) of the first overlap in [t0, t1] between `segs` (for `me`) and another forklift."""
        others = [a for a in self.agents if a is not me and a is not ignore]
        if not others:
            return None
        tmp = Agent(fl=me.fl, pos=me.pos, heading=me.heading, segs=segs)
        t = t0
        while t <= t1:
            pos, h, L, _, seg = tmp.state(t)
            mine = _footprint_circles(me.fl, pos, h, self._carry_at(me, t, segs))
            for o in others:
                opos, oh, oL, _, oseg = o.state(t)
                theirs = _footprint_circles(o.fl, opos, oh, self._carry_at(o, t))
                for x, z, r in mine:
                    for x2, z2, r2 in theirs:
                        if (x - x2) ** 2 + (z - z2) ** 2 < (r + r2) ** 2:
                            return t, o
            t += 0.25
        return None

    def _carry_at(self, agent, t, segs=None):
        for tr in self.tracks.values():
            att = [e for e in tr.events if e[1] == "attach" and e[2] is agent and e[0] <= t]
            if att:
                det = [e for e in tr.events if e[1] == "detach" and e[0] >= att[-1][0]]
                if not det or det[0][0] > t:
                    return tr
        if segs is not None and getattr(self, "_pending_carry", None) is not None:
            pc = self._pending_carry
            if pc[1] <= t <= pc[2]:
                return pc[0]
        return None

    def _parked_boxes(self, me, t):
        out = []
        for a in self.agents:
            if a is me:
                continue
            if a.end_t <= t:                      # idle from now on (as far as we know)
                pos, h, L, _, _ = a.state(t)
                body, front = _body_boxes(a.fl, pos, h, None)
                out += [body, front]
        return out

    # ---------------------------------------------------------------- routing
    def _route_to(self, agent, b, goal, goal_h, exclude_pallet=None, carrying=None, extra0=0.0):
        """Drive from the builder's pose to `goal`, arriving facing `goal_h` after a straight lead-in."""
        fl = agent.fl
        f = fwd_of(goal_h)
        lead = (goal[0] - f[0] * self.S.lead_in, goal[1] - f[1] * self.S.lead_in)
        reach_half = max(fl.half_width, (carrying.half_lat if carrying else 0))
        for extra in (0.0, 25.0, 50.0):
            inflate = reach_half + self.S.margin + extra0 + extra
            g = self.W.grid(inflate, exclude_pallet, self._parked_boxes(agent, b.t))
            g.clear_disc(b.pos, inflate + 10)
            g.clear_disc(goal, inflate + 10)
            g.clear_disc(lead, inflate + 10)
            wps = g.astar(b.pos, lead)
            if wps is None:
                continue
            return wps + [goal]
        return None

    # ---------------------------------------------------------------- jobs
    # A job picks a pallet up from the floor or a rack slot and puts it down in a rack slot or on a dock spot.
    # Picks give approach poses (E = forklift origin with the forks fully under the pallet); places give
    # A = forklift origin when the pallet sits at its destination.

    def _standoff(self, fl):
        return (fl.fork_tip - fl.fork_heel) + self.S.standoff_extra

    def _pick_options(self, fl, pal):
        """[(E, heading, half_fwd, half_lat)] for picking `pal` where it is now."""
        b = pal.box
        if pal.slot is None:                                  # on the floor: any of the four sides
            dirs = [b.u, (-b.u[0], -b.u[1]), b.v, (-b.v[0], -b.v[1])]
        else:                                                 # in a rack: the face it is flush with
            rb = pal.rack.box
            dirs = []
            for sgn in (1.0, -1.0):
                n = (rb.u[0] * sgn, rb.u[1] * sgn)
                depth = (pal.box.c[0] - rb.c[0]) * n[0] + (pal.box.c[1] - rb.c[1]) * n[1]
                gap = rb.hu - (depth + pal.half_along(n))      # pallet face to rack face on this side
                if gap <= max(8.0, rb.hu - (fl.fork_tip - fl.fork_heel) + 5.0) and gap < rb.hu:
                    dirs.append((gap, n))
            dirs = [n for gap, n in sorted(dirs)[:1]]          # the face it is (nearly) flush with
        out = []
        for d in dirs:
            half = pal.half_along(d)
            other = pal.half_along((-d[1], d[0]))
            engage = fl.fork_heel + 2.0 + half
            out.append(((b.c[0] + d[0] * engage, b.c[1] + d[1] * engage), heading_of(-d[0], -d[1]), half, other))
        return out

    def _place_options(self, fl, dest, half_fwd):
        """[(A, heading, C)] for putting a pallet (half size `half_fwd` along the forks) down at `dest`."""
        out = []
        engage = fl.fork_heel + 2.0 + half_fwd
        if dest[0] == "rack":
            _, rack, slot = dest
            bx = rack.box
            for sgn in (1.0, -1.0):
                n = (bx.u[0] * sgn, bx.u[1] * sgn)
                inset = bx.hu - half_fwd                      # pallet face flush with the rack face
                C = (slot.center[0] + n[0] * inset, slot.center[1] + n[1] * inset)
                out.append(((C[0] + n[0] * engage, C[1] + n[1] * engage), heading_of(-n[0], -n[1]), C))
        else:
            _, dock, C = dest
            u = dock.box.u
            for n in (u, (-u[0], -u[1]), (-u[1], u[0]), (u[1], -u[0])):
                out.append(((C[0] + n[0] * engage, C[1] + n[1] * engage), heading_of(-n[0], -n[1]), C))
        return out

    def _try_job(self, agent, pal, dest):
        """Build one pick-and-place; replan with more clearance if the swept footprint clips something."""
        last = None
        for extra in (0.0, 35.0, 70.0, 110.0):
            built, err = self._build_job(agent, pal, dest, extra)
            if built is None:
                return None, err
            hit = self._sweep_hit(agent, built["b"].segs, pal, dest, built["t_attach"])
            if hit is None:
                return built, None
            last = (built, hit)
        msg = f"{agent.fl.name}: {pal.name} route may clip {last[1]}"
        if msg not in self.warnings:
            self.warnings.append(msg)
        return last[0], None

    def _sweep_hit(self, agent, segs, pal, dest, t_attach):
        """Name of a static object the forklift body or its forks/load would pass through, else None.
        Inside the source / destination rack ("rack:" segments) the forks and load are allowed in."""
        fl = agent.fl
        statics = [(r.name, r.box) for r in self.sc.racks] + [(f"obstacle {i}", b) for i, b in enumerate(self.sc.obstacles)]
        statics += [(n, p.box) for n, p in self.W.floor_pallets.items() if n != pal.name]
        statics += [(n, b) for n, b in self.W.placed.items() if n != pal.name]
        work_racks = {pal.rack.name if pal.rack else None, dest[1].name if dest[0] == "rack" else None}
        for seg in segs:
            in_rack = seg.note.startswith("rack:")
            n = max(1, int(seg.dur / 0.2))
            for k in range(n + 1):
                t = seg.t0 + seg.dur * k / n
                pos, h, L, _ = seg.state(t)
                body, front = _body_boxes(fl, pos, h, _Load(pal) if t >= t_attach else None)
                for name, box in statics:
                    if _sat(body, box):
                        return f"{name} (body, while '{seg.note}' at {t:.1f}s)"
                    if in_rack and name in work_racks:
                        continue
                    if _sat(front, box):
                        return f"{name} (forks/load, while '{seg.note}' at {t:.1f}s)"
        return None

    def _route_options(self, agent, b, options, exclude_pallet, carrying, extra):
        """Shortest drivable approach among `options` [(target origin, heading, ...)] -> (option, waypoints, P)."""
        fl = agent.fl
        best = None
        for opt in options:
            T, h = opt[0], opt[1]
            f = fwd_of(h)
            P = (T[0] - f[0] * self._standoff(fl), T[1] - f[1] * self._standoff(fl))
            wps = self._route_to(agent, b, P, h, exclude_pallet=exclude_pallet, carrying=carrying, extra0=extra)
            if wps is None:
                continue
            length = sum(math.dist(wps[i], wps[i + 1]) for i in range(len(wps) - 1))
            if best is None or length < best[0]:
                best = (length, opt, wps, P)
        return None if best is None else best[1:]

    def _build_job(self, agent, pal, dest, extra):
        """Segments for one pick-and-place (dict), or (None, reason)."""
        S, fl = self.S, agent.fl
        found = self._route_options(agent, Builder(agent, S), self._pick_options(fl, pal), None, None, extra)
        if found is None:
            return None, "no route to the pallet"
        (E, h, half_f, half_l), wps, P = found
        track = PalletTrack(pal.name, pal.box.c, pal.base_y, half_f, half_l, pal.opening_top)
        floor_carry = pal.opening_top - fl.blade_top + 12.0          # pallet 12 cm off the floor
        in_rack = pal.slot is not None
        tag = "rack:" if in_rack else ""

        b = Builder(agent, S)
        t_start = b.t
        b.route(wps, note=f"to {pal.name}")
        b.pivot(h, note="line up")
        b.lift(pal.base_y + pal.opening_top - 3.0 - fl.blade_top, note=tag + "forks to pallet")
        b.straight(math.dist(P, E), 1, note=tag + "forks in")
        t_attach = b.t
        b.lift(pal.base_y + pal.opening_top - fl.blade_top + (8.0 if in_rack else 12.0), note=tag + "lift pallet")
        t_lifted = b.t
        b.straight(math.dist(P, E), -1, note=tag + "back out")
        if in_rack:
            b.lift(floor_carry, note="lower to carry")

        # to the destination
        tmp = Agent(fl=fl, pos=b.pos, heading=b.h, L=b.L, t=b.t)
        found = self._route_options(tmp, Builder(tmp, S), self._place_options(fl, dest, half_f), pal.name, track, extra)
        if found is None:
            return None, f"no route from {pal.name} to {dest[1].name}"
        (A, hA, C), w2, PP = found
        b.route(w2, note=f"to {dest[1].name}")
        b.pivot(hA, note="line up")
        if dest[0] == "rack":
            y = dest[2].y
            b.lift(y + 5.0 + pal.opening_top - fl.blade_top, note="rack:raise")
            b.straight(math.dist(PP, A), 1, note="rack:in")
            t_setdown = b.t
            b.lift(y + pal.opening_top - 4.0 - fl.blade_top, note="rack:set down")
            t_detach = b.t
            b.straight(math.dist(PP, A), -1, note="rack:back out")
        else:
            y = 0.0
            b.straight(math.dist(PP, A), 1, note="in")
            t_setdown = b.t
            b.lift(pal.opening_top - 4.0 - fl.blade_top, note="set down")
            t_detach = b.t
            b.straight(math.dist(PP, A), -1, note="back out")
        b.lift(0.0, note="lower forks")
        return {"b": b, "track": track, "t_start": t_start, "t_attach": t_attach, "t_lifted": t_lifted,
                "t_setdown": t_setdown, "t_detach": t_detach, "C": C, "y": y, "h_place": hA}, None

    def _commit(self, agent, built):
        b, track = built["b"], built["track"]
        t_attach = built["t_attach"]
        h_pick = None
        for s in b.segs:                                  # forklift heading when the forks go under the pallet
            if s.t0 <= t_attach <= s.t1:
                h_pick = s.state(t_attach)[1]
                break
        dh = wrap180(built["h_place"] - (h_pick or 0.0))
        track.events.append((t_attach, "attach", agent, h_pick, built["t_lifted"]))
        track.events.append((built["t_detach"], "detach", built["C"], built["y"], dh, built["t_setdown"]))
        self.tracks[track.name] = track
        agent.segs += b.segs
        agent.t, agent.pos, agent.heading, agent.L = b.t, b.pos, b.h, b.L

    def _with_waits(self, agent, make, ignore=None):
        """Run `make()` (which builds from the agent's current time) and push the start later until it does
        not collide with the other forklifts (except `ignore`)."""
        waited = 0.0
        t_start = agent.t
        while True:
            built, err = make()
            if built is None:
                return None, err, waited
            segs = built["b"].segs
            self._pending_carry = (built["track"], built["t_attach"], built["t_detach"]) if "track" in built else None
            hit = self._conflict(agent, agent.segs + segs, t_start, segs[-1].t1, ignore)
            self._pending_carry = None
            if hit is not None:
                t_hit, other = hit
                # an idle forklift parked in the way: send it home first, then try again
                if other.end_t <= t_hit and not self._homing and self._send_home(other, ignore=agent):
                    continue
            if hit is None or waited >= self.S.max_wait:
                if hit is not None:
                    self.warnings.append(f"{agent.fl.name}: could not avoid {hit[1].fl.name} at t={hit[0]:.1f}s")
                return built, None, waited
            agent.t += 1.0
            waited += 1.0
            if agent.segs and agent.segs[-1].kind == "wait" and agent.segs[-1].note == "give way":
                agent.segs[-1].dur += 1.0
            else:
                agent.segs.append(Seg("wait", agent.t - 1.0, 1.0, agent.pos, agent.heading, agent.L, note="give way"))

    def _send_home(self, agent, ignore=None):
        """Drive an idle forklift back to where it started. Returns True if it moved. `ignore`: the forklift
        that asked it to move; it stands still until this one has passed (its own plan is checked after),
        and the route already steers around it as a parked obstacle."""
        S = self.S
        if math.dist(agent.pos, agent.fl.origin) < 5 and abs(wrap180(agent.heading - agent.fl.heading)) < 2:
            return False

        def make():
            b = Builder(agent, S)
            wps = self._route_to(agent, b, agent.fl.origin, agent.fl.heading)
            if wps is None:
                return None, "no route home"
            b.route(wps, note="home")
            b.pivot(agent.fl.heading, note="park")
            return {"b": b}, None
        self._homing = True
        try:
            built, err, _ = self._with_waits(agent, make, ignore)
        finally:
            self._homing = False
        if built is None:
            self.warnings.append(f"{agent.fl.name}: {err}")
            return False
        b = built["b"]
        agent.segs += b.segs
        agent.t, agent.pos, agent.heading, agent.L = b.t, b.pos, b.h, b.L
        return True

    # ---------------------------------------------------------------- what to move where
    def _slot_order(self, slots, pal):
        key_level = (lambda s: s.y) if self.S.fill == "bottom" else (lambda s: -s.y)
        return sorted(slots, key=lambda s: (round(key_level(s)), math.dist(s.center, pal.box.c)))

    def _dock_spots(self, size):
        """Free floor spots on every dock, `size` apart (pallet + aisle), skipping pallets already there."""
        spots = []
        pitch = size + 40.0
        for dock in self.sc.docks:
            bx = dock.box
            nu = max(1, int((2 * bx.hu - 20.0) // pitch))
            nv = max(1, int((2 * bx.hv - 20.0) // pitch))
            for i in range(nu):
                for j in range(nv):
                    ou = -bx.hu + 10.0 + pitch * (i + 0.5) + ((2 * bx.hu - 20.0) - nu * pitch) / 2
                    ov = -bx.hv + 10.0 + pitch * (j + 0.5) + ((2 * bx.hv - 20.0) - nv * pitch) / 2
                    c = (bx.c[0] + bx.u[0] * ou + bx.v[0] * ov, bx.c[1] + bx.u[1] * ou + bx.v[1] * ov)
                    taken = any(_point_box_dist(c, p.box) < size / 2 for p in self.W.floor_pallets.values())
                    if not taken:
                        spots.append((dock, c))
        return spots

    def _destinations(self, pal, free_slots, free_spots, agent):
        """Candidate destinations for `pal`, best first: ("rack", rack, slot) or ("dock", dock, center)."""
        if self.S.mode == "retrieve":
            src = pal.box.c
            # far spots first so the ones near the aisle stay reachable
            spots = sorted(free_spots, key=lambda ds: -math.dist(ds[1], self._dock_entry(ds[0], src)))
            return [("dock", d, c) for d, c in spots[:4]]
        fits = [(r, s) for r, s in free_slots if s.y + pal.opening_top + 10 <= agent.fl.max_lift + agent.fl.blade_top
                and s.clearance >= pal.height + 5.0]
        rack_of = {id(s): r for r, s in fits}
        return [("rack", rack_of[id(s)], s) for s in self._slot_order([s for _, s in fits], pal)[:4]]

    def _dock_entry(self, dock, src):
        """The point of the dock's edge nearest to where pallets come from."""
        bx = dock.box
        best = None
        for p in bx.corners() + [bx.c]:
            if best is None or math.dist(p, src) < math.dist(best, src):
                best = p
        return best

    def plan(self):
        S = self.S
        if not self.agents:
            raise ValueError("no forklifts found")
        free_slots = [(r, s) for r in self.sc.racks for s in r.slots if s.occupant is None]
        if S.mode == "retrieve":
            if not self.sc.docks:
                raise ValueError("Retrieve needs a dock: add a plane named Dock (or tag a floor area as Dock)")
            todo = sorted([p for p in self.sc.pallets if p.slot is not None], key=lambda p: p.name)
            size = max([2 * max(p.box.hu, p.box.hv) for p in todo] or [100.0])
            free_spots = self._dock_spots(size)
        else:
            todo = sorted(self.W.floor_pallets.values(), key=lambda p: p.name)
            free_spots = []
        if S.max_jobs:
            todo = todo[:S.max_jobs]
        while todo:
            agent = min(self.agents, key=lambda a: a.t)
            pal = min(todo, key=lambda p: math.dist(p.box.c, agent.pos))
            dests = self._destinations(pal, free_slots, free_spots, agent)
            if not dests:
                self.warnings.append(f"{pal.name}: nowhere free to put it")
                todo.remove(pal)
                continue
            done = False
            for dest in dests:
                t_ready = agent.t
                built, err, waited = self._with_waits(agent, lambda: self._try_job(agent, pal, dest))
                if built is None:
                    self.log(f"  {agent.fl.name}: {pal.name} -> {dest[1].name}: {err}")
                    continue
                self._commit(agent, built)
                if dest[0] == "rack":
                    free_slots = [(r, s) for r, s in free_slots if s is not dest[2]]
                    dest[2].occupant = pal.name
                    where = f"{dest[1].name} shelf {dest[2].level + 1}"
                else:
                    free_spots = [x for x in free_spots if x[1] is not dest[2]]
                    pb = pal.box
                    a = math.radians(built["track"].events[-1][4])
                    u = (pb.u[0] * math.cos(a) + pb.u[1] * math.sin(a), pb.u[1] * math.cos(a) - pb.u[0] * math.sin(a))
                    self.W.placed[pal.name] = Box2(c=dest[2], u=u, hu=pb.hu, hv=pb.hv)
                    where = dest[1].name
                if pal.slot is not None:                      # its old slot is free again
                    pal.slot.occupant = None
                self.W.floor_pallets.pop(pal.name, None)
                self.jobs.append({"forklift": agent.fl.name, "pallet": pal.name,
                                  "from": pal.rack.name if pal.rack else "floor", "to": where,
                                  "ready": t_ready, "start": built["t_start"], "picked": built["t_attach"],
                                  "placed": built["t_detach"], "end": built["b"].t, "waited": waited})
                self.log(f"  {agent.fl.name}: {pal.name} -> {where} (picked {built['t_attach']:.0f}s, "
                         f"placed {built['t_detach']:.0f}s{', waited %.0fs' % waited if waited else ''})")
                done = True
                break
            if not done:
                self.warnings.append(f"{pal.name}: no route found for {agent.fl.name}")
                agent.t += 5.0            # let others try, then give up on it
            todo.remove(pal)

        if S.return_home:
            for agent in sorted(self.agents, key=lambda a: a.t):
                self._send_home(agent)
        end = max([a.end_t for a in self.agents] + [S.start_delay]) + 2.0
        return Plan(self.agents, self.tracks, self.jobs, end, self.warnings)


# ------------------------------------------------------------------------------------------------ sampling
def pallet_state(track, t, agents_by_name=None):
    """(center (x, z), base y, heading delta) of a pallet at time t, or None while it sits untouched."""
    att = [e for e in track.events if e[1] == "attach" and e[0] <= t]
    if not att:
        return None
    a = att[-1]
    det = [e for e in track.events if e[1] == "detach" and e[0] >= a[0]]
    if det and det[0][0] <= t:
        d = det[0]
        return d[2], d[3], d[4]
    agent = a[2]
    pos, h, L, _, _ = agent.state(t)
    fl = agent.fl
    f = fwd_of(h)
    engage = fl.fork_heel + 2.0 + track.half_fwd
    c = (pos[0] + f[0] * engage, pos[1] + f[1] * engage)
    y = fl.blade_top + L - track.opening_top                         # riding on the forks
    if t < a[4]:                                                      # forks not up yet: still on its support
        y = max(y, track.rest_y)
    if det and t >= det[0][5]:                                        # being set down: the shelf / floor holds it
        y = max(y, det[0][3])
    return c, y, wrap180(h - a[3])
