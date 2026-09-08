#!/usr/bin/env python3
"""LQFP fanout + 2-layer HV A* router (F.Cu horizontal, B.Cu vertical)."""
from __future__ import annotations

import heapq
import math
import sys

import pcbnew

PCB = r"E:\kicad projects\stm32 breakout board\stm32breakout\stm32breakout.kicad_pcb"

CLR = 0.15
TW = 0.15
PW = 0.30
VIA_D = 0.50
VIA_DRILL = 0.30
EDGE = 0.35
G = 0.15  # A* grid (mm)

F = pcbnew.F_Cu
B = pcbnew.B_Cu


def mm(v):
    return pcbnew.FromMM(v)


def tomm(v):
    return pcbnew.ToMM(v)


def hypot(ax, ay, bx, by):
    return math.hypot(bx - ax, by - ay)


def point_seg(px, py, x1, y1, x2, y2):
    vx, vy = x2 - x1, y2 - y1
    l2 = vx * vx + vy * vy
    if l2 < 1e-18:
        return math.hypot(px - x1, py - y1)
    t = max(0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / l2))
    return math.hypot(px - (x1 + t * vx), py - (y1 + t * vy))


def _cross(x1, y1, x2, y2):
    return x1 * y2 - y1 * x2


def segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
    rx, ry = bx - ax, by - ay
    sx, sy = dx - cx, dy - cy
    den = _cross(rx, ry, sx, sy)
    if abs(den) < 1e-18:
        return False
    qx, qy = cx - ax, cy - ay
    t = _cross(qx, qy, sx, sy) / den
    u = _cross(qx, qy, rx, ry) / den
    return 0.002 < t < 0.998 and 0.002 < u < 0.998


def seg_seg(ax, ay, bx, by, cx, cy, dx, dy):
    if segments_intersect(ax, ay, bx, by, cx, cy, dx, dy):
        return 0.0
    return min(
        point_seg(ax, ay, cx, cy, dx, dy),
        point_seg(bx, by, cx, cy, dx, dy),
        point_seg(cx, cy, ax, ay, bx, by),
        point_seg(dx, dy, ax, ay, bx, by),
    )


def rot_local(dx, dy, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return dx * c + dy * s, -dx * s + dy * c


class R:
    def __init__(self, board):
        self.board = board
        self.tracks = []  # layer, x1,y1,x2,y2,w,net
        self.vias = []  # x,y,net
        self.pads = []
        self.net_items = {}
        bb = board.GetBoardEdgesBoundingBox()
        xs = (tomm(bb.GetLeft()), tomm(bb.GetRight()))
        ys = (tomm(bb.GetBottom()), tomm(bb.GetTop()))
        self.bbox = (min(xs), min(ys), max(xs), max(ys))
        self.w = self.bbox[2]
        self.h = self.bbox[3]
        for fp in board.GetFootprints():
            ref = fp.GetReference()
            for pad in fp.Pads():
                p = pad.GetPosition()
                x, y = tomm(p.x), tomm(p.y)
                sz = pad.GetSize()
                pw, ph = tomm(sz.x), tomm(sz.y)
                n = pad.GetNetname() or ""
                ni = pad.GetNet()
                if ni:
                    self.net_items[n] = ni
                self.pads.append(
                    {
                        "x": x,
                        "y": y,
                        "w": pw,
                        "h": ph,
                        "rot": pad.GetOrientation().AsDegrees(),
                        "r": min(pw, ph) * 0.5 + 0.02,
                        "net": n,
                        "pth": pad.HasHole(),
                        "ref": ref,
                        "num": pad.GetNumber(),
                    }
                )

        u1 = [p for p in self.pads if p["ref"] == "U1"]
        self.cx = sum(p["x"] for p in u1) / len(u1)
        self.cy = sum(p["y"] for p in u1) / len(u1)
        self.apron_i = 0

        self.nx = int(self.w / G) + 1
        self.ny = int(self.h / G) + 1
        ncells = self.nx * self.ny
        # 0 free, -1 board-blocked, >0 net id
        self.gf = [0] * ncells
        self.gb = [0] * ncells
        self.net_id = {}
        self.id_net = {}
        self._next_id = 1
        self._paint_edge()
        self._paint_all_pads_block()

    def nid(self, net):
        if net not in self.net_id:
            i = self._next_id
            self._next_id += 1
            self.net_id[net] = i
            self.id_net[i] = net
        return self.net_id[net]

    def idx(self, ix, iy):
        return iy * self.nx + ix

    def gxy(self, x, y):
        return int(round(x / G)), int(round(y / G))

    def xyg(self, ix, iy):
        return ix * G, iy * G

    def in_grid(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def _paint_edge(self):
        m = int(EDGE / G) + 1
        for iy in range(self.ny):
            for ix in range(self.nx):
                if ix < m or iy < m or ix >= self.nx - m or iy >= self.ny - m:
                    k = self.idx(ix, iy)
                    self.gf[k] = -1
                    self.gb[k] = -1
        # USB notch: x in [36.5, 43.5], y > 88.6
        for iy in range(self.ny):
            y = iy * G
            if y < self.h - 1.6:
                continue
            for ix in range(self.nx):
                x = ix * G
                if abs(x - self.w / 2) <= 3.7:
                    k = self.idx(ix, iy)
                    self.gf[k] = -1
                    self.gb[k] = -1

    def pad_hit(self, x, y, pad, expand):
        dx, dy = rot_local(x - pad["x"], y - pad["y"], pad["rot"])
        return abs(dx) <= pad["w"] * 0.5 + expand and abs(dy) <= pad["h"] * 0.5 + expand

    def _paint_all_pads_block(self):
        """Paint pad *copper* (not keepout) as that net's id. Clearance is checked while walking."""
        for pad in self.pads:
            nid = self.nid(pad["net"]) if pad["net"] and not pad["net"].startswith("unconnected") else -1
            if pad["net"] == "GND":
                nid = -1
            r = math.hypot(pad["w"], pad["h"]) * 0.5 + 0.04
            ix0, iy0 = self.gxy(pad["x"], pad["y"])
            span = int(r / G) + 2
            for iy in range(iy0 - span, iy0 + span + 1):
                for ix in range(ix0 - span, ix0 + span + 1):
                    if not self.in_grid(ix, iy):
                        continue
                    x, y = self.xyg(ix, iy)
                    if not self.pad_hit(x, y, pad, 0.02):
                        continue
                    k = self.idx(ix, iy)
                    if pad["pth"]:
                        if self.gf[k] == 0:
                            self.gf[k] = nid
                        if self.gb[k] == 0:
                            self.gb[k] = nid
                    else:
                        if self.gf[k] == 0:
                            self.gf[k] = nid

    def reopen_pad(self, pad, grid_layer):
        """Allow this pad's cells for its own net on the given layer list."""
        expand = TW / 2 + 0.02
        r = math.hypot(pad["w"], pad["h"]) * 0.5 + expand
        ix0, iy0 = self.gxy(pad["x"], pad["y"])
        span = int(r / G) + 2
        cells = []
        nid = self.nid(pad["net"]) if pad["net"] else 0
        for iy in range(iy0 - span, iy0 + span + 1):
            for ix in range(ix0 - span, ix0 + span + 1):
                if not self.in_grid(ix, iy):
                    continue
                x, y = self.xyg(ix, iy)
                if self.pad_hit(x, y, pad, expand):
                    k = self.idx(ix, iy)
                    if grid_layer[k] == -1:
                        grid_layer[k] = nid if nid else 0
                    cells.append((ix, iy))
        return cells

    def layer_grid(self, layer):
        return self.gf if layer == F else self.gb

    def track_ok(self, x1, y1, x2, y2, w, layer, net):
        need = w / 2 + CLR
        # Do not run through the LQFP body / pad ring (shorts opposite pins).
        if layer == F:
            # Reject only traces that span across the QFP interior.
            if abs(y1 - y2) < 0.25 and 40.2 <= y1 <= 49.8:
                if min(x1, x2) < 37.2 and max(x1, x2) > 42.8:
                    return False
            if abs(x1 - x2) < 0.25 and 35.2 <= x1 <= 44.8:
                if min(y1, y2) < 41.8 and max(y1, y2) > 48.2:
                    return False
        for x, y in ((x1, y1), (x2, y2)):
            if x < EDGE or y < EDGE or x > self.bbox[2] - EDGE or y > self.bbox[3] - EDGE:
                return False
        for t in self.tracks:
            if t[0] != layer or t[6] == net:
                continue
            d = seg_seg(x1, y1, x2, y2, t[1], t[2], t[3], t[4])
            if d < need + t[5] / 2:
                return False
        for vx, vy, vn in self.vias:
            if vn == net:
                continue
            if point_seg(vx, vy, x1, y1, x2, y2) < VIA_D / 2 + need:
                return False
        for p in self.pads:
            if p["net"] == net:
                continue
            if layer == B and not p["pth"]:
                continue
            if self.seg_hits_pad(x1, y1, x2, y2, p, need):
                return False
        return True

    def seg_hits_pad(self, x1, y1, x2, y2, p, need):
        steps = max(2, int(hypot(x1, y1, x2, y2) / 0.15))
        hw, hh = p["w"] * 0.5 + need, p["h"] * 0.5 + need
        for s in range(steps + 1):
            t = s / steps
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            dx, dy = rot_local(x - p["x"], y - p["y"], p["rot"])
            if abs(dx) <= hw and abs(dy) <= hh:
                return True
        return False

    def via_ok(self, x, y, net):
        if x < EDGE + 0.4 or y < EDGE + 0.4 or x > self.bbox[2] - EDGE - 0.4 or y > self.bbox[3] - EDGE - 0.4:
            return False
        for vx, vy, vn in self.vias:
            if vn == net:
                continue
            if math.hypot(x - vx, y - vy) < VIA_D + CLR:
                return False
        for p in self.pads:
            if p["net"] == net:
                continue
            dpad = math.hypot(x - p["x"], y - p["y"])
            if p["pth"]:
                if dpad < 1.5:
                    return False
            elif dpad < 0.90:
                return False
        for t in self.tracks:
            if t[6] == net:
                continue
            if point_seg(x, y, t[1], t[2], t[3], t[4]) < VIA_D / 2 + t[5] / 2 + CLR:
                return False
        return True

    def add_track(self, x1, y1, x2, y2, w, layer, net):
        if hypot(x1, y1, x2, y2) < 0.04:
            return True
        if not self.track_ok(x1, y1, x2, y2, w, layer, net):
            return False
        tr = pcbnew.PCB_TRACK(self.board)
        tr.SetStart(pcbnew.VECTOR2I(mm(x1), mm(y1)))
        tr.SetEnd(pcbnew.VECTOR2I(mm(x2), mm(y2)))
        tr.SetWidth(mm(w))
        tr.SetLayer(layer)
        tr.SetNet(self.net_items[net])
        self.board.Add(tr)
        self.tracks.append((layer, x1, y1, x2, y2, w, net))
        self._paint_seg(x1, y1, x2, y2, w, layer, net)
        return True

    def add_via(self, x, y, net):
        if not self.via_ok(x, y, net):
            return False
        via = pcbnew.PCB_VIA(self.board)
        via.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetWidth(mm(VIA_D))
        via.SetDrill(mm(VIA_DRILL))
        via.SetNet(self.net_items[net])
        self.board.Add(via)
        self.vias.append((x, y, net))
        self._paint_via(x, y, net)
        return True

    def _paint_seg(self, x1, y1, x2, y2, w, layer, net):
        nid = self.nid(net)
        r = w / 2 + 0.02
        grid = self.layer_grid(layer)
        steps = max(1, int(hypot(x1, y1, x2, y2) / (G * 0.5)))
        span = int((r + CLR + TW / 2) / G) + 1
        for s in range(steps + 1):
            t = s / steps
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            ix0, iy0 = self.gxy(x, y)
            for iy in range(iy0 - span, iy0 + span + 1):
                for ix in range(ix0 - span, ix0 + span + 1):
                    if not self.in_grid(ix, iy):
                        continue
                    cx, cy = self.xyg(ix, iy)
                    if math.hypot(cx - x, cy - y) <= r:
                        grid[self.idx(ix, iy)] = nid

    def _paint_via(self, x, y, net):
        nid = self.nid(net)
        r = VIA_D / 2 + 0.02
        span = int((r + CLR + TW / 2) / G) + 1
        ix0, iy0 = self.gxy(x, y)
        for iy in range(iy0 - span, iy0 + span + 1):
            for ix in range(ix0 - span, ix0 + span + 1):
                if not self.in_grid(ix, iy):
                    continue
                cx, cy = self.xyg(ix, iy)
                if math.hypot(cx - x, cy - y) <= r:
                    k = self.idx(ix, iy)
                    self.gf[k] = nid
                    self.gb[k] = nid

    def side(self, x, y):
        dx, dy = x - self.cx, y - self.cy
        if abs(dx) >= abs(dy):
            return "R" if dx > 0 else "L"
        return "T" if dy > 0 else "B"

    def fanout(self, pad, net, w=TW):
        if pad["pth"]:
            return pad["x"], pad["y"]
        x, y = pad["x"], pad["y"]
        s = self.side(x, y)
        try:
            n = int("".join(ch for ch in pad["num"] if ch.isdigit()) or "0")
        except ValueError:
            n = 0
        far = 3.20 if n % 2 == 0 else 2.00
        dists = [far, far + 0.9, far + 1.6, far + 2.4]
        for dist in dists:
            if s == "L":
                vx, vy = x - dist, y
            elif s == "R":
                vx, vy = x + dist, y
            elif s == "T":
                vx, vy = x, y + dist
            else:
                vx, vy = x, y - dist
            cands = [(vx, vy)]
            for ax, ay in cands:
                if self.track_ok(x, y, ax, ay, w, F, net) and self.via_ok(ax, ay, net):
                    self.add_track(x, y, ax, ay, w, F, net)
                    self.add_via(ax, ay, net)
                    return ax, ay
        return None

    def escape_usb(self, pad, net, w=TW):
        """Drop straight down off the USB-C 0.5 mm row before routing."""
        x, y = pad["x"], pad["y"]
        for dist in (1.4, 1.8, 2.2, 2.8):
            tx, ty = x, y - dist
            if self.track_ok(x, y, tx, ty, w, F, net):
                self.add_track(x, y, tx, ty, w, F, net)
                return tx, ty
        return x, y

    def cell_walkable(self, ix, iy, layer, nid, allow_goal=False):
        if not self.in_grid(ix, iy):
            return False
        grid = self.gf if layer == 0 else self.gb
        v0 = grid[self.idx(ix, iy)]
        if v0 not in (0, nid):
            return False
        need = TW / 2 + CLR
        span = int(need / G) + 1
        for dy in range(-span, span + 1):
            for dx in range(-span, span + 1):
                d = math.hypot(dx * G, dy * G)
                if d > need + 0.02:
                    continue
                jx, jy = ix + dx, iy + dy
                if not self.in_grid(jx, jy):
                    if d < need:
                        return False
                    continue
                v = grid[self.idx(jx, jy)]
                if v not in (0, nid) and d < need:
                    return False
        return True

    def astar(self, starts, goals, net, w, max_nodes=80000):
        """HV A*: layer 0 = F.Cu (horizontal), layer 1 = B.Cu (vertical).
        starts/goals: list of (x, y, layer) with layer 0/1.
        """
        nid = self.nid(net)
        goal_set = set()
        for x, y, lyr in goals:
            ix, iy = self.gxy(x, y)
            for dx in range(-1, 2):
                for dy in range(-1, 2):
                    if self.in_grid(ix + dx, iy + dy):
                        goal_set.add((ix + dx, iy + dy, lyr))
        if not goal_set:
            return None

        # reopen own pads on both grids so we can stand on them
        # (already painted -1 from foreign keepout)
        # starts are explicit.

        counter = 0
        heap = []
        came = {}
        gscore = {}
        closed = set()

        def heur(ix, iy, lyr, gx, gy, gl):
            # HV: F moves X, B moves Y, via to switch
            dx = abs(ix - gx)
            dy = abs(iy - gy)
            extra = 0
            if lyr == 0 and dy:
                extra += 8
            if lyr == 1 and dx:
                extra += 8
            if lyr != gl:
                extra += 8
            return dx + dy + extra

        # pick a representative goal for heuristic
        gx, gy, gl = self.gxy(goals[0][0], goals[0][1])[0], self.gxy(goals[0][0], goals[0][1])[1], goals[0][2]

        for x, y, lyr in starts:
            ix, iy = self.gxy(x, y)
            if not self.in_grid(ix, iy):
                continue
            st = (ix, iy, lyr)
            gscore[st] = 0
            heapq.heappush(heap, (heur(ix, iy, lyr, gx, gy, gl), counter, st))
            counter += 1

        nodes = 0
        found = None
        while heap and nodes < max_nodes:
            _f, _c, cur = heapq.heappop(heap)
            if cur in closed:
                continue
            closed.add(cur)
            nodes += 1
            ix, iy, lyr = cur
            if cur in goal_set or (ix, iy, lyr) in goal_set:
                found = cur
                break
            # Strict HV: F.Cu horizontal, B.Cu vertical. Via to turn.
            if lyr == 0:
                moves = (
                    (ix + 1, iy, 0, 1),
                    (ix - 1, iy, 0, 1),
                    (ix, iy, 1, 12),
                )
            else:
                moves = (
                    (ix, iy + 1, 1, 1),
                    (ix, iy - 1, 1, 1),
                    (ix, iy, 0, 12),
                )
            for nix, niy, nlyr, cost in moves:
                if not self.in_grid(nix, niy):
                    continue
                nxt = (nix, niy, nlyr)
                if nxt in closed:
                    continue
                if nlyr != lyr:
                    if not self.via_cells_ok(ix, iy, nid):
                        continue
                elif not self.cell_walkable(nix, niy, nlyr, nid):
                    continue
                ng = gscore[cur] + cost
                if ng >= gscore.get(nxt, 1e18):
                    continue
                came[nxt] = cur
                gscore[nxt] = ng
                h = heur(nix, niy, nlyr, gx, gy, gl)
                heapq.heappush(heap, (ng + h, counter, nxt))
                counter += 1

        if found is None:
            print(f"    A* miss nodes={nodes} closed={len(closed)} nid={nid}", flush=True)
            return None

        # reconstruct
        path = [found]
        while path[-1] in came:
            path.append(came[path[-1]])
        path.reverse()
        return path, nodes

    def via_cells_ok(self, ix, iy, nid):
        span = int((VIA_D / 2 + CLR) / G) + 1
        for dy in range(-span, span + 1):
            for dx in range(-span, span + 1):
                if dx * dx + dy * dy > span * span:
                    continue
                jx, jy = ix + dx, iy + dy
                if not self.in_grid(jx, jy):
                    return False
                k = self.idx(jx, jy)
                vf, vb = self.gf[k], self.gb[k]
                if vf not in (0, nid) or vb not in (0, nid):
                    return False
        return True

    def _has_via(self, x, y, net):
        return any(math.hypot(x - vx, y - vy) < 0.2 and vn == net for vx, vy, vn in self.vias)

    def commit_path(self, path, net, w):
        """path: list of (ix,iy,lyr). Collapse only collinear runs; via on layer change."""
        if not path:
            return False
        pts = [(ix * G, iy * G, lyr) for ix, iy, lyr in path]
        i = 0
        n = len(pts)
        while i < n - 1:
            x1, y1, l1 = pts[i]
            x2, y2, l2 = pts[i + 1]
            if l2 != l1:
                if not self._has_via(x1, y1, net):
                    if not self.add_via(x1, y1, net):
                        return False
                i += 1
                continue
            dx = x2 - x1
            dy = y2 - y1
            j = i + 1
            while j < n - 1 and pts[j + 1][2] == l1:
                ndx = pts[j + 1][0] - pts[j][0]
                ndy = pts[j + 1][1] - pts[j][1]
                if abs(ndx) < 1e-9 and abs(ndy) < 1e-9:
                    j += 1
                    continue
                # must stay axis-aligned and in the same direction
                same_h = abs(dy) < 1e-9 and abs(ndy) < 1e-9 and dx * ndx > 0
                same_v = abs(dx) < 1e-9 and abs(ndx) < 1e-9 and dy * ndy > 0
                if not (same_h or same_v):
                    break
                dx, dy = ndx, ndy
                j += 1
            x2, y2 = pts[j][0], pts[j][1]
            layer = F if l1 == 0 else B
            if hypot(x1, y1, x2, y2) >= 0.04:
                if not self.add_track(x1, y1, x2, y2, w, layer, net):
                    return False
            i = j
        return True

    def _place_via(self, x, y, net):
        if self._has_via(x, y, net):
            return True
        return self.add_via(x, y, net)

    def _preflight(self, segs, vias, w, net):
        for x1, y1, x2, y2, lyr in segs:
            if hypot(x1, y1, x2, y2) < 0.04:
                continue
            if not self.track_ok(x1, y1, x2, y2, w, lyr, net):
                return False
        for vx, vy in vias:
            if not self._has_via(vx, vy, net) and not self.via_ok(vx, vy, net):
                return False
        return True

    def _commit_segs(self, segs, vias, w, net):
        for vx, vy in vias:
            if not self._place_via(vx, vy, net):
                return False
        for x1, y1, x2, y2, lyr in segs:
            if hypot(x1, y1, x2, y2) < 0.04:
                continue
            if not self.add_track(x1, y1, x2, y2, w, lyr, net):
                return False
            if lyr == B:
                for p in self.pads:
                    if p["net"] != net or p["pth"]:
                        continue
                    if hypot(x1, y1, p["x"], p["y"]) < 0.35 or hypot(x2, y2, p["x"], p["y"]) < 0.35:
                        self._place_via(p["x"], p["y"], net)
        return True

    def try_channel(self, sx, sy, gx, hx, hy, net, w, left):
        """F H to gx, B V to header Y, F H to PTH. Bypass if header Y is busy on F."""
        segs = []
        vias = []
        if abs(sx - gx) >= 0.04:
            segs.append((sx, sy, gx, sy, F))
        vias.append((gx, sy))
        segs.append((gx, sy, gx, hy, B))
        vias.append((gx, hy))
        segs.append((gx, hy, hx, hy, F))
        if self._preflight(segs, vias, w, net):
            return self._commit_segs(segs, vias, w, net)

        # Header Y collides on F.Cu. Use a free Y-bus, F.Cu H to the apron,
        # then B.Cu vertical+stub only for x<12 / x>68.
        ax = (11.0 + (self.apron_i % 5) * 0.7) if left else (69.0 - (self.apron_i % 5) * 0.7)
        for yb in (50.8, 39.2, 54.2, 35.8, 58.0, 32.0, 62.0, 28.0, 16.5, 74.0, 21.0, 23.5, 26.1):
            segs = []
            vias = []
            if abs(sx - gx) >= 0.04:
                segs.append((sx, sy, gx, sy, F))
            vias.append((gx, sy))
            segs.append((gx, sy, gx, yb, B))
            vias.append((gx, yb))
            segs.append((gx, yb, ax, yb, F))
            vias.append((ax, yb))
            segs.append((ax, yb, ax, hy, B))
            segs.append((ax, hy, hx, hy, B))
            if self._preflight(segs, vias, w, net):
                self.apron_i += 1
                return self._commit_segs(segs, vias, w, net)
        return False

    def route_to_header(self, src, hdr, net, w):
        sx, sy = src
        hx, hy = hdr["x"], hdr["y"]
        w = min(w, TW)
        if hdr["ref"] == "J4":
            for gx in (sx, 30.2, 29.0, 49.8, 51.0, 27.6, 52.4):
                segs = []
                vias = []
                if abs(sx - gx) >= 0.04:
                    segs.append((sx, sy, gx, sy, F))
                    vias.append((gx, sy))
                segs.append((gx, sy, gx, hy, B))
                vias.append((gx, hy))
                segs.append((gx, hy, hx, hy, F))
                if self._preflight(segs, vias, w, net):
                    return self._commit_segs(segs, vias, w, net)
            return self.escape_ring(sx, sy, hx, hy, net, w)
        left = hx < 40
        gxs = [12.6 + i * 0.58 for i in range(36)] if left else [67.4 - i * 0.58 for i in range(36)]
        # Top/bottom fanout vias sit under the pad X. Do NOT run B.Cu through
        # the opposite via row; jog to a unique Y-bus then into a side channel.
        if 35.5 <= sx <= 44.5:
            above = sy >= 45.0
            high = [55.8 + k * 0.70 for k in range(14)]
            low = [26.0, 23.5, 21.0, 18.2, 16.5, 12.6, 28.4, 30.2, 32.0, 35.2]
            ybs = (high + low) if above else (low + high)
            for yb in ybs:
                for gx in gxs:
                    segs = [
                        (sx, sy, sx, yb, B),
                        (sx, yb, gx, yb, F),
                        (gx, yb, gx, hy, B),
                        (gx, hy, hx, hy, F),
                    ]
                    vias = [(sx, yb), (gx, yb), (gx, hy)]
                    if self._preflight(segs, vias, w, net):
                        return self._commit_segs(segs, vias, w, net)
                    for ax in ((11.2, 6.4, 5.7) if left else (68.8, 73.6, 74.3, 75.0)):
                        segs = [
                            (sx, sy, sx, yb, B),
                            (sx, yb, ax, yb, F),
                            (ax, yb, ax, hy, B),
                            (ax, hy, hx, hy, B),
                        ]
                        vias = [(sx, yb), (ax, yb)]
                        if self._preflight(segs, vias, w, net):
                            return self._commit_segs(segs, vias, w, net)
        for gx in gxs:
            if self.try_channel(sx, sy, gx, hx, hy, net, w, left):
                return True
        return False

    def try_path(self, segs, vias, w, net):
        if self._preflight(segs, vias, w, net):
            return self._commit_segs(segs, vias, w, net)
        return False

    def hv_jog(self, ax, ay, bx, by, net, w):
        """Leave a via column, then HV to the target. Used for local nets."""
        w = min(w, TW)
        mxs = [34.7, 45.3, 31.0, 49.0, 29.4, 50.6, 25.2, 54.8]
        for mx in mxs:
            for yb in (35.2, 56.5, 30.0, 60.0, 26.0, 64.0, 20.0, 74.0, 16.5, 12.6):
                segs = [
                    (ax, ay, mx, ay, F),
                    (mx, ay, mx, yb, B),
                    (mx, yb, bx, yb, F),
                    (bx, yb, bx, by, B),
                ]
                vias = [(mx, ay), (mx, yb), (bx, yb)]
                if abs(yb - by) < 0.04:
                    segs = [
                        (ax, ay, mx, ay, F),
                        (mx, ay, mx, by, B),
                        (mx, by, bx, by, F),
                    ]
                    vias = [(mx, ay), (mx, by)]
                if self.try_path(segs, vias, w, net):
                    return True
        return False

    def u_route(self, a, b, w, layer, net):
        ax, ay = a
        bx, by = b
        horiz = abs(ax - bx) >= abs(ay - by)
        for off in (0.6, -0.6, 1.0, -1.0, 1.4, -1.4, 2.0, -2.0, 2.8, -2.8):
            if horiz:
                pts = [(ax, ay), (ax, ay + off), (bx, ay + off), (bx, by)]
            else:
                mx = ax + off
                if 34.0 <= mx <= 46.0:
                    continue
                pts = [(ax, ay), (mx, ay), (mx, by), (bx, by)]
            segs = list(zip(pts, pts[1:]))
            if all(
                hypot(p[0], p[1], q[0], q[1]) < 0.04 or self.track_ok(p[0], p[1], q[0], q[1], w, layer, net)
                for p, q in segs
            ):
                for p, q in segs:
                    if hypot(p[0], p[1], q[0], q[1]) >= 0.04:
                        if not self.add_track(p[0], p[1], q[0], q[1], w, layer, net):
                            return False
                return True
        return False

    def connect_hv(self, ax, ay, a_layer, bx, by, b_layer, net, w):
        """Try cheap L / HV channel before A*."""
        # same-layer L
        for layer in (F, B):
            if a_layer is not None and layer != a_layer and a_layer == b_layer and a_layer != layer:
                continue
        # Skip cheap HV when start/end share a Y band — F.Cu H would sit on
        # top of another net's MCU escape.
        close_y = abs(ay - by) < 0.45
        if (not close_y) and abs(ax - bx) > 0.05 and abs(ay - by) > 0.05:
            # Prefer B vertical first (fanout already ended on a via).
            if self.track_ok(ax, ay, ax, by, w, B, net) and (
                self.via_ok(ax, by, net) or self._has_via(ax, by, net)
            ) and self.track_ok(ax, by, bx, by, w, F, net):
                self.add_track(ax, ay, ax, by, w, B, net)
                if not self._has_via(ax, by, net):
                    self.add_via(ax, by, net)
                self.add_track(ax, by, bx, by, w, F, net)
                return True
            if self.track_ok(ax, ay, bx, ay, w, F, net) and self.via_ok(bx, ay, net) and self.track_ok(
                bx, ay, bx, by, w, B, net
            ):
                pth_end = any(
                    p["pth"] and p["net"] == net and hypot(p["x"], p["y"], bx, by) < 0.4 for p in self.pads
                )
                if pth_end or self.via_ok(bx, by, net) or self._has_via(bx, by, net):
                    self.add_track(ax, ay, bx, ay, w, F, net)
                    self.add_via(bx, ay, net)
                    self.add_track(bx, ay, bx, by, w, B, net)
                    if not pth_end and not self._has_via(bx, by, net) and self.via_ok(bx, by, net):
                        self.add_via(bx, by, net)
                    return True
        elif abs(ay - by) <= 0.05:
            if self.track_ok(ax, ay, bx, by, w, F, net):
                return self.add_track(ax, ay, bx, by, w, F, net)
        elif abs(ax - bx) <= 0.05:
            if self.track_ok(ax, ay, bx, by, w, B, net):
                return self.add_track(ax, ay, bx, by, w, B, net)

        if self.u_route((ax, ay), (bx, by), w, F, net):
            return True
        # Do not U-route on B.Cu (would mix H and V on the same layer).
        if self.hv_jog(ax, ay, bx, by, net, w):
            return True
        if self.hv_jog(bx, by, ax, ay, net, w):
            return True
        if self.around_mcu(ax, ay, bx, by, net, w):
            return True
        if self.escape_ring(ax, ay, bx, by, net, w):
            return True
        return False

    def escape_ring(self, ax, ay, bx, by, net, w):
        """Force the ring X outside the QFP so we never ride a via column."""
        w = min(w, TW)
        starts = [x for x in (30.2, 29.0, 27.6, 49.8, 51.0, 52.4) if abs(x - ax) > 0.4]
        ends = [x for x in (30.2, 29.0, 27.6, 49.8, 51.0, 52.4) if abs(x - bx) > 0.4]
        yrs = (12.6, 16.5, 21.0, 26.0, 31.0, 34.5, 55.8, 58.2, 62.0, 70.0, 74.0)
        for sx in starts:
            for px in ends:
                for yr in yrs:
                    variants = [
                        (
                            [
                                (ax, ay, sx, ay, F),
                                (sx, ay, sx, yr, B),
                                (sx, yr, px, yr, F),
                                (px, yr, px, by, B),
                                (px, by, bx, by, F),
                            ],
                            [(sx, ay), (sx, yr), (px, yr), (px, by)],
                        ),
                        (
                            [
                                (ax, ay, ax, yr, B),
                                (ax, yr, px, yr, F),
                                (px, yr, px, by, B),
                                (px, by, bx, by, F),
                            ],
                            [(ax, yr), (px, yr), (px, by)],
                        ),
                    ]
                    for segs, vias in variants:
                        if self._preflight(segs, vias, w, net):
                            return self._commit_segs(segs, vias, w, net)
        return False

    def around_mcu(self, ax, ay, bx, by, net, w):
        """Ring around the QFP: Y-bus above/below, approach dest off the via column."""
        w = min(w, TW)
        for dx in (-2.0, 2.0, -2.8, 2.8, -3.6, 3.6, -1.4, 1.4):
            px = bx + dx
            if px < 4.0 or px > 76.0:
                continue
            for yr in (
                16.5, 14.2, 18.2, 21.0, 23.5, 26.1,
                29.4, 32.2, 34.5, 55.8, 57.0, 58.4,
                60.6, 64.0, 70.0, 74.0, 12.6,
            ):
                for ox in (0.0, 1.4, -1.4, 2.4, -2.4):
                    sx = ax + ox
                    if sx < 4.0 or sx > 76.0:
                        continue
                    segs = [
                        (ax, ay, sx, ay, F),
                        (sx, ay, sx, yr, B),
                        (sx, yr, px, yr, F),
                        (px, yr, px, by, B),
                        (px, by, bx, by, F),
                    ]
                    vias = [(sx, ay), (sx, yr), (px, yr), (px, by)]
                    if self._preflight(segs, vias, w, net):
                        return self._commit_segs(segs, vias, w, net)
        return False

    def stitch_leftovers(self, terminals):
        """Explicit HV recipes for nets the generic router could not finish."""
        w = TW

        def src(ref, num, net):
            return terminals.get((ref, str(num), net), terminals.get((ref, num, net)))

        def pad(ref, num, net):
            for p in self.pads:
                if p["ref"] == ref and str(p["num"]) == str(num) and p["net"] == net:
                    return p["x"], p["y"]
            return None

        def go(sxy, dxy, net, cols=None, ybs=None):
            if not sxy or not dxy:
                return False
            sx, sy = sxy
            dx, dy = dxy
            cols = cols or (34.7, 45.3, 31.0, 49.0, 29.2, 50.8, 25.0, 55.0)
            ybs = ybs or (35.2, 56.6, 30.0, 12.6, 16.5, 21.0, 26.0, 60.4, 64.0, 70.0, 74.0)
            for col in cols:
                for yb in ybs:
                    variants = [
                        (
                            [
                                (sx, sy, sx, yb, B),
                                (sx, yb, dx, yb, F),
                                (dx, yb, dx, dy, B),
                            ],
                            [(sx, yb), (dx, yb), (dx, dy)],
                        ),
                        (
                            [
                                (sx, sy, col, sy, F),
                                (col, sy, col, yb, B),
                                (col, yb, dx, yb, F),
                                (dx, yb, dx, dy, B),
                            ],
                            [(col, sy), (col, yb), (dx, yb), (dx, dy)],
                        ),
                        (
                            [
                                (sx, sy, col, sy, F),
                                (col, sy, col, dy, B),
                                (col, dy, dx, dy, F),
                            ],
                            [(col, sy), (col, dy)],
                        ),
                    ]
                    for segs, vias in variants:
                        if self.try_path(segs, vias, w, net):
                            return True
            return False

        # BOOT0: bottom even via -> SW2 / R6 / J3
        b0 = src("U1", "44", "BOOT0")
        go(b0, pad("SW2", "1", "BOOT0"), "BOOT0", cols=(50.6, 49.0, 29.2), ybs=(35.2, 16.5, 12.6, 26.0))
        go(b0, pad("R6", "1", "BOOT0"), "BOOT0", cols=(50.6, 29.2, 31.0), ybs=(35.2, 16.5, 26.0))
        go(b0, pad("J3", "18", "BOOT0"), "BOOT0", cols=(50.6, 55.0, 67.4), ybs=(35.2, 26.0, 16.5, 12.6))

        # NRST
        n0 = src("U1", "7", "NRST")
        go(n0, pad("J2", "4", "NRST"), "NRST", cols=(34.7, 31.0, 14.0), ybs=(60.38, 56.6, 35.2))
        go(n0, pad("J4", "4", "NRST"), "NRST", cols=(34.7, 49.0, 29.2), ybs=(12.6, 16.5, 35.2))
        go(n0, pad("R5", "2", "NRST"), "NRST", cols=(34.7, 45.3, 49.0), ybs=(35.2, 36.2, 56.6))
        go(n0, pad("C15", "1", "NRST"), "NRST", cols=(34.7, 45.3, 49.0), ybs=(35.2, 36.2))
        go(n0, pad("SW1", "1", "NRST"), "NRST", cols=(29.2, 25.0, 34.7), ybs=(12.6, 16.5, 35.2))

        # +3V3A ferrite / caps above MCU
        a0 = src("U1", "9", "+3V3A")
        go(a0, pad("FB1", "2", "+3V3A"), "+3V3A", cols=(34.7, 45.3), ybs=(56.6, 58.0, 60.0))
        go(a0, pad("C8", "1", "+3V3A"), "+3V3A", cols=(34.7, 45.3), ybs=(56.6, 58.0))
        go(a0, pad("C9", "1", "+3V3A"), "+3V3A", cols=(34.7, 31.0), ybs=(56.6, 58.0))

        # +3V3 to FB1 and nearby caps
        v3 = src("U1", "24", "+3V3") or src("U1", "1", "+3V3")
        go(v3, pad("FB1", "1", "+3V3"), "+3V3", cols=(45.3, 34.7), ybs=(56.6, 58.0, 52.0))
        go(v3, pad("C7", "1", "+3V3"), "+3V3", cols=(34.7, 31.0), ybs=(56.6, 50.0))
        go(v3, pad("C10", "1", "+3V3"), "+3V3", cols=(34.7, 31.0), ybs=(56.6, 50.0))
        go(src("U1", "1", "+3V3") or v3, pad("C5", "1", "+3V3"), "+3V3", cols=(34.7, 31.0, 29.2), ybs=(35.2, 36.2, 30.0))
        go(v3, pad("C3", "1", "+3V3"), "+3V3", cols=(34.7, 31.0, 25.0), ybs=(70.0, 64.0, 56.6))
        go(v3, pad("C4", "1", "+3V3"), "+3V3", cols=(34.7, 31.0, 25.0), ybs=(70.0, 64.0, 56.6))
        go(v3, pad("U2", "5", "+3V3"), "+3V3", cols=(31.0, 25.0, 34.7), ybs=(70.0, 74.0, 64.0))
        go(pad("FB1", "1", "+3V3"), pad("C6", "1", "+3V3"), "+3V3", cols=(45.3, 47.6, 42.8), ybs=(56.8, 57.6, 51.2))
        go(pad("FB1", "1", "+3V3"), pad("C10", "1", "+3V3"), "+3V3", cols=(34.7, 37.0, 45.3), ybs=(56.8, 51.2, 58.0))
        go(pad("C3", "1", "+3V3"), pad("J2", "2", "+3V3"), "+3V3", cols=(25.2, 19.6, 31.0), ybs=(65.46, 71.0, 68.0, 74.0))
        go(pad("C3", "1", "+3V3"), pad("C4", "1", "+3V3"), "+3V3", cols=(25.2, 30.0), ybs=(71.0, 73.0, 68.0))
        go(pad("C3", "1", "+3V3"), pad("U2", "5", "+3V3"), "+3V3", cols=(21.0, 25.2), ybs=(72.0, 74.0, 68.0))
        go(src("U1", "48", "+3V3"), src("U1", "1", "+3V3"), "+3V3", cols=(34.7, 31.0), ybs=(37.0, 35.2, 39.0))
        go(src("U1", "24", "+3V3"), src("U1", "36", "+3V3"), "+3V3", cols=(45.3, 47.0), ybs=(51.0, 56.6, 47.0))

        # PC14 crystal / header
        p14 = src("U1", "3", "PC14")
        go(p14, pad("Y2", "1", "PC14"), "PC14", cols=(34.7, 45.3, 49.0), ybs=(26.0, 21.0, 30.0, 35.2))
        go(p14, pad("C13", "1", "PC14"), "PC14", cols=(45.3, 49.0, 34.7), ybs=(21.0, 26.0, 16.5))
        go(p14, pad("J2", "19", "PC14"), "PC14", cols=(34.7, 31.0, 14.0), ybs=(22.28, 21.0, 26.0, 16.5))

        def f_jog(a, b, net, ybs, xks=None):
            if not a or not b:
                return False
            ax, ay = a
            bx, by = b
            cols = xks or (bx, ax)
            for yb in ybs:
                for xk in cols:
                    variants = [
                        [
                            (ax, ay, ax, yb, F),
                            (ax, yb, xk, yb, F),
                            (xk, yb, xk, by, F),
                            (xk, by, bx, by, F),
                        ],
                        [
                            (ax, ay, xk, ay, F),
                            (xk, ay, xk, yb, F),
                            (xk, yb, bx, yb, F),
                            (bx, yb, bx, by, F),
                        ],
                    ]
                    for segs in variants:
                        if self.try_path(segs, [], w, net):
                            print(f"  last-mile {net} F yb={yb} xk={xk}", flush=True)
                            return True
            return False

        def hv_last(a, b, net, cols, ybs):
            if not a or not b:
                return False
            ax, ay = a
            bx, by = b
            for col in cols:
                for yb in ybs:
                    segs = [
                        (ax, ay, col, ay, F),
                        (col, ay, col, yb, B),
                        (col, yb, bx, yb, F),
                        (bx, yb, bx, by, F),
                    ]
                    vias = [(col, ay), (col, yb)]
                    if self.try_path(segs, vias, w, net):
                        print(f"  last-mile {net} HV col={col} yb={yb}", flush=True)
                        return True
            return False

        # Analog filter sits left of the QFP (FB1/C8/C9). Join FB1.1 to C7/C10.
        fb = pad("FB1", "1", "+3V3")
        c6 = pad("C6", "1", "+3V3")
        c7 = pad("C7", "1", "+3V3")
        c10 = pad("C10", "1", "+3V3")
        fb_y = (51.0, 51.6, 50.4, 52.4, 49.6, 53.2, 48.4, 47.2, 54.2, 55.4, 57.2, 58.6)
        fb_x = (28.3, 31.625, 32.20, 28.825, 30.0, 26.4, 24.8, 33.2, 35.0)
        if fb:
            for tgt in (c7, c10, c6):
                if f_jog(fb, tgt, "+3V3", fb_y, fb_x):
                    break
        a0 = src("U1", "9", "+3V3A")
        c9a = pad("C9", "1", "+3V3A")
        f_jog(a0, c9a, "+3V3A", (46.25, 47.2, 48.0, 45.4, 44.2, 49.2, 41.0, 50.4), (30.0, 29.2, 31.0, 27.5, 24.0, 21.2, 33.8))
        f_jog(pad("C9", "1", "+3V3A"), pad("FB1", "2", "+3V3A"), "+3V3A", (48.0, 49.2, 50.0, 51.0, 46.2), (24.0, 26.7, 27.5))
        f_jog(pad("FB1", "2", "+3V3A"), pad("C8", "1", "+3V3A"), "+3V3A", (51.0, 50.2, 51.8, 49.4), (26.7, 24.8, 24.0))
        # U1.9 sits on the left via column; jog west off the column then into C9.
        if a0 and c9a:
            hv_last(a0, c9a, "+3V3A", (30.0, 29.2, 31.0, 28.2, 27.0, 25.2, 21.2), (48.0, 47.4, 49.0, 50.2, 44.8, 41.2, 51.0))
            f_jog(a0, c9a, "+3V3A", (44.8, 43.6, 41.2, 40.2, 48.8, 50.2, 51.0), (30.0, 28.4, 26.0, 24.0, 21.2))

        c3 = pad("C3", "1", "+3V3")
        j2v = pad("J2", "2", "+3V3")
        if c3 and j2v:
            self.try_path(
                [
                    (c3[0], c3[1], c3[0], j2v[1], F),
                    (c3[0], j2v[1], j2v[0], j2v[1], F),
                ],
                [],
                w,
                "+3V3",
            )
        u148 = src("U1", "48", "+3V3")
        u11 = src("U1", "1", "+3V3")
        if u148 and u11:
            self.try_path(
                [
                    (u148[0], u148[1], 34.7, u148[1], F),
                    (34.7, u148[1], 34.7, u11[1], B),
                    (34.7, u11[1], u11[0], u11[1], F),
                ],
                [(34.7, u148[1]), (34.7, u11[1])],
                w,
                "+3V3",
            )

        # NRST to J4 pin 4. Do not run along y=8 (crosses +3V3/PA13/PA14).
        # 1) Below the SWD row on F.Cu, then up into the pad.
        # 2) B.Cu column from R5, F jog in from above the header.
        n0 = src("U1", "7", "NRST")
        j4 = pad("J4", "4", "NRST")
        c15 = pad("C15", "1", "NRST")
        r5n = pad("R5", "2", "NRST")
        sw1 = pad("SW1", "1", "NRST")
        if j4:
            below = (6.50, 6.35, 6.65, 6.20, 6.80, 5.90, 5.60, 7.00)
            if not f_jog(sw1, j4, "NRST", below, (j4[0], 21.15, 19.15)):
                f_jog(r5n, j4, "NRST", below, (j4[0], 48.425, 47.0))
            if not any(
                hypot(j4[0], j4[1], t[1], t[2]) < 0.9 or hypot(j4[0], j4[1], t[3], t[4]) < 0.9
                for t in self.tracks
                if t[6] == "NRST"
            ):
                hv_last(
                    r5n or c15 or n0,
                    j4,
                    "NRST",
                    (48.425, 47.62, 47.00, 46.20, 49.20, 51.40, 54.00, 55.40, 44.80),
                    (11.0, 10.4, 11.6, 9.6, 13.2, 15.2, 16.0, 17.5, 19.0),
                )

    def stitch_gnd(self):
        """Tie F.Cu GND pad-pockets to the B.Cu pour so zone-to-zone DRC clears."""
        net = "GND"
        if net not in self.net_items:
            return
        n = 0

        def gnd_ok(x, y):
            if x < EDGE + 0.5 or y < EDGE + 0.5 or x > self.bbox[2] - EDGE - 0.5 or y > self.bbox[3] - EDGE - 0.5:
                return False
            if any(q["pth"] and hypot(x, y, q["x"], q["y"]) < 1.7 for q in self.pads):
                return False
            for q in self.pads:
                if q["net"] == net:
                    continue
                d = hypot(x, y, q["x"], q["y"])
                keep = 0.92 if not q["pth"] else 1.7
                if d < keep:
                    return False
            for t in self.tracks:
                if t[6] == net:
                    continue
                if point_seg(x, y, t[1], t[2], t[3], t[4]) < VIA_D / 2 + t[5] / 2 + CLR + 0.05:
                    return False
            for vx, vy, vn in self.vias:
                if hypot(x, y, vx, vy) < VIA_D + CLR:
                    return False
            return True

        for p in self.pads:
            if p["net"] != net or p["pth"]:
                continue
            for dx, dy in (
                (0.60, 0.0),
                (-0.60, 0.0),
                (0.0, 0.60),
                (0.0, -0.60),
                (0.70, 0.50),
                (-0.70, 0.50),
                (0.70, -0.50),
                (-0.70, -0.50),
                (0.0, 0.0),
            ):
                x, y = p["x"] + dx, p["y"] + dy
                if not gnd_ok(x, y):
                    continue
                if self._place_via(x, y, net):
                    n += 1
                    break
        # SOT-23 GND pads need via-in-pad; 0.6 mm offsets miss the F.Cu pocket.
        for p in self.pads:
            if p["net"] != net or p["pth"] or p["ref"] not in ("U2",):
                continue
            if self._place_via(p["x"], p["y"], net):
                n += 1
        print(f"  GND stitch vias {n}", flush=True)


def pads_of(rr, net):
    seen = set()
    out = []
    for p in rr.pads:
        if p["net"] != net:
            continue
        k = (round(p["x"], 2), round(p["y"], 2), p["ref"], p["num"])
        if k in seen:
            continue
        seen.add(k)
        out.append(p)
    return out


def main():
    board = pcbnew.LoadBoard(PCB)
    ds = board.GetDesignSettings()
    ds.m_MinClearance = mm(0.15)
    nc = ds.m_NetSettings.GetDefaultNetclass()
    nc.SetClearance(mm(0.15))
    nc.SetTrackWidth(mm(0.15))
    nc.SetViaDiameter(mm(0.5))
    nc.SetViaDrill(mm(0.3))
    rr = R(board)

    skip = {"", "GND"}
    nets = sorted(
        {p["net"] for p in rr.pads if p["net"] not in skip and not p["net"].startswith("unconnected")}
    )

    terminals = {}  # (ref,num,net) -> (x,y)

    print("fanout U1", flush=True)
    for p in rr.pads:
        if p["net"] in skip or p["net"].startswith("unconnected"):
            continue
        if p["pth"] or p["ref"] != "U1":
            terminals[(p["ref"], p["num"], p["net"])] = (p["x"], p["y"])
            continue
        pos = rr.fanout(p, p["net"], TW)
        if pos:
            terminals[(p["ref"], p["num"], p["net"])] = pos
        else:
            terminals[(p["ref"], p["num"], p["net"])] = (p["x"], p["y"])
            print(f"  fanout fail {p['ref']}.{p['num']} {p['net']}", flush=True)

    print("stitch duplicate pads", flush=True)
    by_key = {}
    for p in rr.pads:
        if p["pth"] or p["net"] in skip:
            continue
        k = (p["ref"], p["num"], p["net"])
        by_key.setdefault(k, []).append(p)
    for _k, group in by_key.items():
        if len(group) < 2:
            continue
        a, b = group[0], group[1]
        if hypot(a["x"], a["y"], b["x"], b["y"]) < 8:
            rr.u_route((a["x"], a["y"]), (b["x"], b["y"]), TW, F, a["net"]) or rr.add_track(
                a["x"], a["y"], b["x"], b["y"], TW, F, a["net"]
            )

    print("escape J1", flush=True)
    for p in rr.pads:
        if p["ref"] != "J1":
            continue
        if p["net"] in skip or p["net"].startswith("unconnected"):
            continue
        if p["pth"]:
            continue
        pos = rr.escape_usb(p, p["net"], TW)
        terminals[(p["ref"], p["num"], p["net"])] = pos

    def term(p):
        return terminals.get((p["ref"], p["num"], p["net"]), (p["x"], p["y"]))

    def net_span(n):
        ps = pads_of(rr, n)
        if len(ps) < 2:
            return 0
        xs = [p["x"] for p in ps]
        ys = [p["y"] for p in ps]
        return max(xs) - min(xs) + max(ys) - min(ys)

    def mst_pairs(padlist):
        points = [term(p) for p in padlist]
        n = len(points)
        if n < 2:
            return []

        def is_conn(i):
            return padlist[i]["ref"].startswith("J")

        used = set()
        start = next((i for i in range(n) if not is_conn(i)), 0)
        used.add(start)
        pairs = []
        while len(used) < n:
            best = None
            for i in used:
                for j in range(n):
                    if j in used:
                        continue
                    if is_conn(i) and is_conn(j):
                        continue
                    d = hypot(points[i][0], points[i][1], points[j][0], points[j][1])
                    if is_conn(j) and not is_conn(i):
                        d -= 0.5
                    if best is None or d < best[0]:
                        best = (d, i, j)
            if best is None:
                for i in used:
                    for j in range(n):
                        if j in used:
                            continue
                        d = hypot(points[i][0], points[i][1], points[j][0], points[j][1])
                        if best is None or d < best[0]:
                            best = (d, i, j)
            pairs.append((best[1], best[2]))
            used.add(best[2])
        return pairs

    order = [n for n in nets if not n.startswith("+")] + [n for n in nets if n.startswith("+")]
    order.sort(key=lambda n: (0 if n in ("CC1", "CC2", "LED_PWR", "LED_USER") else 1 if not n.startswith("+") else 2, net_span(n)))

    failed = []
    for net in order:
        ps = pads_of(rr, net)
        if len(ps) < 2:
            continue
        w = PW if net.startswith("+") else TW
        print(f"route {net} {len(ps)}", flush=True)
        ok = True
        u1s = [p for p in ps if p["ref"] == "U1"]
        hdrs = [p for p in ps if p["ref"] in ("J2", "J3", "J4")]
        others = [p for p in ps if p["ref"] not in ("U1", "J2", "J3", "J4")]

        def link(pa, pb):
            a, b = term(pa), term(pb)
            if hypot(a[0], a[1], b[0], b[1]) < 0.25:
                return True
            if (pa["ref"] in ("J2", "J3", "J4") and pb["ref"] != pa["ref"]) or (
                pb["ref"] in ("J2", "J3", "J4") and pa["ref"] != pb["ref"]
            ):
                src_p, hdr_p = (pb, pa) if pa["ref"] in ("J2", "J3", "J4") else (pa, pb)
                if rr.route_to_header(term(src_p), hdr_p, net, w):
                    return True
            return rr.connect_hv(a[0], a[1], None, b[0], b[1], None, net, w)

        # Local SMD to nearest U1 pin (avoid MST across the QFP).
        if u1s and others:
            for o in others:
                src = min(u1s, key=lambda p: hypot(term(p)[0], term(p)[1], term(o)[0], term(o)[1]))
                if not link(src, o):
                    print(f"  FAIL local {term(src)} -> {term(o)}", flush=True)
                    ok = False
            if len(others) >= 2:
                for i, j in mst_pairs(others):
                    link(others[i], others[j])
        elif len(others) >= 2:
            for i, j in mst_pairs(others):
                if not link(others[i], others[j]):
                    print(f"  FAIL local {term(others[i])} -> {term(others[j])}", flush=True)
                    ok = False
        elif len(u1s) >= 2 and not others:
            for i, j in mst_pairs(u1s):
                link(u1s[i], u1s[j])
        srcs = u1s or others
        if srcs and hdrs:
            for h in hdrs:
                src = min(srcs, key=lambda p: hypot(term(p)[0], term(p)[1], h["x"], h["y"]))
                if not link(src, h):
                    print(f"  FAIL hdr {term(src)} -> ({h['x']},{h['y']})", flush=True)
                    ok = False
        elif not srcs:
            for i, j in mst_pairs(ps):
                if not link(ps[i], ps[j]):
                    print(f"  FAIL {term(ps[i])} -> {term(ps[j])}", flush=True)
                    ok = False
        if not ok:
            failed.append(net)

    print("stitch leftovers", flush=True)
    rr.stitch_leftovers(terminals)
    print("stitch GND", flush=True)
    rr.stitch_gnd()

    # Via on SMD pads that already have B.Cu copper (layer change).
    for p in rr.pads:
        if p["pth"] or p["net"] in skip or p["net"].startswith("unconnected"):
            continue
        for t in rr.tracks:
            if t[0] != B or t[6] != p["net"]:
                continue
            if point_seg(p["x"], p["y"], t[1], t[2], t[3], t[4]) < 0.25:
                rr._place_via(p["x"], p["y"], p["net"])
                break

    # Via wherever F and B of the same net share an endpoint.
    for t in list(rr.tracks):
        if t[0] != F:
            continue
        for u in rr.tracks:
            if u[0] != B or u[6] != t[6]:
                continue
            for ax, ay in ((t[1], t[2]), (t[3], t[4])):
                for bx, by in ((u[1], u[2]), (u[3], u[4])):
                    if hypot(ax, ay, bx, by) < 0.25:
                        rr._place_via((ax + bx) / 2, (ay + by) / 2, t[6])

    pcbnew.SaveBoard(PCB, board)
    print("saved")
    print("failed:", failed or "none")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
