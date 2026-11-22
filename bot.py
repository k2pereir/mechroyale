import json
import math
import os
import sys
import threading
import time
from collections import deque

import mechfight as mf

PI = math.pi
TAU = 2 * PI


def wrap(a):
    return (a + PI) % TAU - PI


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(v):
    d = math.hypot(v[0], v[1])
    return [v[0] / d, v[1] / d] if d > 1e-8 else [0.0, 0.0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def pip(ring, x, y):
    ins = False
    n = len(ring) // 2
    j = n - 1
    for i in range(n):
        xi, yi, xj, yj = ring[2 * i], ring[2 * i + 1], ring[2 * j], ring[2 * j + 1]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            ins = not ins
        j = i
    return ins


DIRS8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]


class Grid:
    CS = 3.0

    def __init__(self, info):
        self.ok = False
        self.spots = []
        try:
            self._load(info or {})
        except Exception:
            sys.stderr.write("grid load failed\n")

    def _load(self, info):
        t0 = time.time()
        w = json.load(open(os.path.join(info.get("dir", "/arena"), "v2", "world.json")))
        ter = w["terrain"]
        self.minx, self.miny = float(ter["minX"]), float(ter["minY"])
        cs = self.CS
        self.gx = int(ter["nx"] / cs) + 2
        self.gy = int(ter["ny"] / cs) + 2
        gx, gy = self.gx, self.gy
        g = bytearray(gx * gy)
        for p in w.get("prisms", []):
            if time.time() - t0 > 4.0:
                break
            if p.get("base", 0) >= 9.2 or p.get("top", 0) < 1.5:
                continue
            ring = p.get("ring") or []
            if len(ring) < 6:
                continue
            holes = p.get("holes") or []
            if holes and not isinstance(holes[0], (list, tuple)):
                holes = [holes]
            xs, ys = ring[0::2], ring[1::2]
            i0 = max(0, int((min(xs) - self.minx) / cs)); i1 = min(gx - 1, int((max(xs) - self.minx) / cs))
            j0 = max(0, int((min(ys) - self.miny) / cs)); j1 = min(gy - 1, int((max(ys) - self.miny) / cs))
            for k in range(len(xs)):
                i = int((xs[k] - self.minx) / cs); j = int((ys[k] - self.miny) / cs)
                if 0 <= i < gx and 0 <= j < gy:
                    g[j * gx + i] = 1
            for j in range(j0, j1 + 1):
                cy = self.miny + (j + 0.5) * cs
                for i in range(i0, i1 + 1):
                    cx = self.minx + (i + 0.5) * cs
                    if pip(ring, cx, cy) and not any(pip(h, cx, cy) for h in holes):
                        g[j * gx + i] = 1
        self.raw = g
        dil = bytearray(g)
        for idx in range(gx * gy):
            if g[idx]:
                j, i = divmod(idx, gx)
                for dj in (-1, 0, 1):
                    for di in (-1, 0, 1):
                        jj, ii = j + dj, i + di
                        if 0 <= jj < gy and 0 <= ii < gx:
                            dil[jj * gx + ii] = 1
        self.dil = dil
        self.ok = True
        # enclosure score on a lattice
        spots = []
        step = 9.0
        x = self.minx + 20
        maxx = self.minx + ter["nx"] - 20
        maxy = self.miny + ter["ny"] - 20
        while x < maxx and time.time() - t0 < 7.0:
            y = self.miny + 20
            while y < maxy:
                if not self.blocked(x, y, True):
                    n = 0
                    for k in range(12):
                        a = k * TAU / 12
                        c, s = math.cos(a), math.sin(a)
                        for d in (3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0):
                            if self.blocked(x + c * d, y + s * d, False):
                                n += 1
                                break
                    if n >= 5:
                        spots.append((x, y, n))
                y += step
            x += step
        self.spots = spots

    def cell(self, x, y):
        return int((x - self.minx) / self.CS), int((y - self.miny) / self.CS)

    def blocked(self, x, y, dilated):
        i, j = self.cell(x, y)
        if i < 0 or j < 0 or i >= self.gx or j >= self.gy:
            return not dilated and False or dilated
        return (self.dil if dilated else self.raw)[j * self.gx + i] == 1

    def clear(self, x, y, ang, length):
        c, s = math.cos(ang), math.sin(ang)
        d = 3.0
        while d <= length:
            if self.blocked(x + c * d, y + s * d, True):
                return False
            d += 3.0
        return True

    def field(self, gx_, gy_):
        """BFS distance field over dilated-free cells from a goal point."""
        gx, gy = self.gx, self.gy
        dist_ = [-1] * (gx * gy)
        i, j = self.cell(gx_, gy_)
        i = clamp(i, 0, gx - 1); j = clamp(j, 0, gy - 1)
        # find nearest free cell to the goal
        start = None
        for r in range(0, 6):
            for dj in range(-r, r + 1):
                for di in range(-r, r + 1):
                    ii, jj = i + di, j + dj
                    if 0 <= ii < gx and 0 <= jj < gy and not self.dil[jj * gx + ii]:
                        start = (ii, jj)
                        break
                if start:
                    break
            if start:
                break
        if not start:
            return dist_
        dist_[start[1] * gx + start[0]] = 0
        q = deque([start])
        dil = self.dil
        while q:
            ci, cj = q.popleft()
            cd = dist_[cj * gx + ci]
            for di, dj in DIRS8:
                ni, nj = ci + di, cj + dj
                if ni < 0 or nj < 0 or ni >= gx or nj >= gy:
                    continue
                k = nj * gx + ni
                if dist_[k] != -1 or dil[k]:
                    continue
                if di and dj and (dil[cj * gx + ni] or dil[nj * gx + ci]):
                    continue
                dist_[k] = cd + 1
                q.append((ni, nj))
        return dist_


def intercept(pos, vel, origin, speed):
    r = [pos[i] - origin[i] for i in range(3)]
    a = sum(v * v for v in vel) - speed * speed
    b = 2 * sum(r[i] * vel[i] for i in range(3))
    c = sum(v * v for v in r)
    if abs(a) < 1e-8:
        t = -c / b if abs(b) > 1e-8 else 0.0
    else:
        disc = b * b - 4 * a * c
        roots = []
        if disc >= 0:
            s = math.sqrt(disc)
            roots = [(-b - s) / (2 * a), (-b + s) / (2 * a)]
        valid = [v for v in roots if v > 0]
        t = min(valid) if valid else 0.0
    t = clamp(t, 0.0, 1.0)
    return [pos[i] + vel[i] * t for i in range(3)]


def ballistic_pitch(speed, horizontal, dz, g):
    if horizontal < 1.0:
        return 0.0
    v2 = speed * speed
    disc = v2 * v2 - g * (g * horizontal * horizontal + 2 * dz * v2)
    if disc < 0:
        return math.atan2(dz, horizontal)
    return math.atan((v2 - math.sqrt(disc)) / (g * horizontal))


UPG = ["armor", "plating", "nanites", "gun_damage", "gun_rate", "reactor", "laser_damage", "lightning_efficiency", "gyros",
       "capacitor", "servos", "gun_mag", "mortar_reload", "laser_charge", "lightning_chain", "jets", "gun_velocity",
       "sensors", "mortar_splash", "lightning_range", "mortar_velocity", "laser_pierce"]
WPRI = {"laser": 3, "lightning": 2, "mortar": 1}


class Bot:
    name = "ghost-v1"

    def __init__(self, hello):
        self.cfg = hello["config"]
        self.slot = hello.get("slot", 0)
        self.grid = Grid(hello.get("map"))
        self.side = 1.0 if self.slot % 2 == 0 else -1.0
        self.next_flip = 0
        self.spot = None
        self.phase_key = None
        self.field = None
        self.field_key = None
        self.field_busy = False
        self.bad_spots = set()
        self.last_hit_t = -99.0
        self.last_hit_by = None
        self.last_hit_bearing = None
        self.last_hit_weapon = None
        self.face_until = -1
        self.face_yaw = 0.0
        self.stuck = 0
        self.unstick_until = -1
        self.prev_move = [0.0, 0.0]
        self.last_pos = None
        self.nlog = 0
        self.dmg_log = 0
        self.mode = "init"

    # ---------------- zone
    @staticmethod
    def zone_state(z, t):
        q = clamp((t - z["shrink_start"]) / max(1e-6, z["shrink_end"] - z["shrink_start"]), 0.0, 1.0)
        c = [z["center"][i] + (z["next_center"][i] - z["center"][i]) * q for i in range(2)]
        return c, z["radius"] + (z["next_radius"] - z["radius"]) * q

    # ---------------- hiding spot + flow field
    def pick_spot(self, me_pos, nc, nr):
        if not self.grid.ok or nr < 40 or not self.grid.spots:
            return (nc[0], nc[1])
        best, bs = None, -1e9
        lim = nr * 0.62
        for (x, y, n) in self.grid.spots:
            if (round(x), round(y)) in self.bad_spots:
                continue
            dc = math.hypot(x - nc[0], y - nc[1])
            if dc > lim:
                continue
            s = n * 2.0 - math.hypot(x - me_pos[0], y - me_pos[1]) / 22.0 - dc / 60.0
            if s > bs:
                bs, best = s, (x, y)
        return best or (nc[0], nc[1])

    def start_field(self, goal):
        key = (round(goal[0]), round(goal[1]))
        if self.field_key == key or self.field_busy or not self.grid.ok:
            return
        self.field_busy = True

        def work():
            try:
                f = self.grid.field(goal[0], goal[1])
                self.field = f
                self.field_key = key
            except Exception:
                pass
            self.field_busy = False

        threading.Thread(target=work, daemon=True).start()

    def flow_dir(self, x, y, goal):
        """direction along the BFS field toward the goal, or None if unavailable"""
        key = (round(goal[0]), round(goal[1]))
        if self.field is None or self.field_key != key:
            return None
        g = self.grid
        i, j = g.cell(x, y)
        if not (0 <= i < g.gx and 0 <= j < g.gy):
            return None
        d0 = self.field[j * g.gx + i]
        ci, cj = i, j
        if d0 < 0:
            # we may be inside a dilated cell: look at the best free neighbour
            best = None
            for di, dj in DIRS8:
                ni, nj = i + di, j + dj
                if 0 <= ni < g.gx and 0 <= nj < g.gy:
                    v = self.field[nj * g.gx + ni]
                    if v >= 0 and (best is None or v < best[0]):
                        best = (v, ni, nj)
            if best is None:
                return None
            ci, cj = best[1], best[2]
        for _ in range(3):
            cur = self.field[cj * g.gx + ci]
            nb = None
            for di, dj in DIRS8:
                ni, nj = ci + di, cj + dj
                if 0 <= ni < g.gx and 0 <= nj < g.gy:
                    v = self.field[nj * g.gx + ni]
                    if v >= 0 and v < cur and (nb is None or v < nb[0]):
                        nb = (v, ni, nj)
            if nb is None:
                break
            ci, cj = nb[1], nb[2]
        tx = g.minx + (ci + 0.5) * g.CS
        ty = g.miny + (cj + 0.5) * g.CS
        if math.hypot(tx - x, ty - y) < 1.0:
            return unit([goal[0] - x, goal[1] - y])
        return unit([tx - x, ty - y])

    def steer(self, x, y, d):
        if not self.grid.ok or (d[0] == 0 and d[1] == 0):
            return d
        base = math.atan2(d[1], d[0])
        for off in (0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.7, -1.7, 2.4, -2.4, PI):
            a = base + off * self.side
            if self.grid.clear(x, y, a, 12.0):
                return [math.cos(a), math.sin(a)]
        return d

    def has_shot_los(self, pos, target):
        if not self.grid.ok:
            return True
        x, y = pos[0], pos[1]
        tx, ty = target[0], target[1]
        dx, dy = tx - x, ty - y
        distance = math.hypot(dx, dy)
        if distance < 0.1:
            return True
        angle = math.atan2(dy, dx)
        return self.grid.clear(x, y, angle, distance)

    # ---------------- dodging
    def dodge(self, pos, projs):
        dv = [0.0, 0.0]
        x, y, z = pos
        for p in projs:
            pp, vv = p["pos"], p["vel"]
            if p.get("kind") == "mortar":
                g = self.cfg["weapons"]["mortar"]["gravity"]
                disc = vv[2] ** 2 + 2 * g * (pp[2] - z - 0.5)
                if disc < 0:
                    continue
                tl = (vv[2] + math.sqrt(disc)) / g
                if tl < 0 or tl > 2.8:
                    continue
                lx, ly = pp[0] + vv[0] * tl, pp[1] + vv[1] * tl
                if math.hypot(x - lx, y - ly) < 11:
                    a = unit([x - lx, y - ly])
                    dv[0] += a[0] * 2; dv[1] += a[1] * 2
                continue
            rx, ry = pp[0] - x, pp[1] - y
            vx, vy = vv[0], vv[1]
            v2 = vx * vx + vy * vy
            if v2 < 1:
                continue
            tc = -(rx * vx + ry * vy) / v2
            if tc < 0 or tc > 0.8:
                continue
            if abs(pp[2] + vv[2] * tc - (z + 4.5)) > 7:
                continue
            cx, cy = rx + vx * tc, ry + vy * tc
            d = math.hypot(cx, cy)
            if d < 5.5:
                vl = math.sqrt(v2)
                n = [-vy / vl, vx / vl]
                if n[0] * (-cx) + n[1] * (-cy) < 0:
                    n = [-n[0], -n[1]]
                if d < 0.5:
                    n = [n[0] * self.side, n[1] * self.side]
                dv[0] += n[0] * 1.6; dv[1] += n[1] * 1.6
        return dv

    # ---------------- main
    def act(self, obs):
        try:
            return self._act(obs)
        except Exception:
            import traceback
            if self.nlog < 5:
                self.nlog += 1
                sys.stderr.write(traceback.format_exc() + "\n")
            try:
                me = obs["self"]
                zc, _ = self.zone_state(obs["zone"], obs["t"])
                return {"tick": obs["tick"], "move": unit([zc[0] - me["pos"][0], zc[1] - me["pos"][1]]), "fire": [False, False]}
            except Exception:
                return {"tick": obs.get("tick", 0)}

    def _act(self, obs):
        me = obs["self"]; pos = me["pos"]; x, y, z = pos
        t = obs["t"]; tick = obs["tick"]; myid = me["id"]
        stats = me["stats"]; walk = stats.get("walk_speed", 11.7)
        hp, maxhp, energy = me["hp"], me["max_hp"], me["energy"]
        vel = me["vel"]; cyaw, cpit = me["aim"]
        zone = obs["zone"]
        zc, zr = self.zone_state(zone, t)
        fc, fr = self.zone_state(zone, t + 12.0)
        nc, nr = zone["next_center"], zone["next_radius"]
        act = {"tick": tick, "move": [0.0, 0.0], "fire": [False, False], "boost": False, "jump": False}

        if obs["offer"]:
            ch = [c["id"] for c in obs["offer"]["choices"]]
            act["pick"] = min(range(len(ch)), key=lambda i: UPG.index(ch[i]) if ch[i] in UPG else 99)

        weapons = me["weapons"]
        wk = [w["kind"] if w else None for w in weapons]
        by_kind = {w["kind"]: (i, w) for i, w in enumerate(weapons) if w}
        charging = bool(by_kind.get("laser") and by_kind["laser"][1].get("charge", 0) > 0)

        # events
        for e in obs["events"]:
            if e["type"] == "damaged" and e["weapon"] != "zone":
                self.last_hit_t = t
                self.last_hit_by = e.get("by")
                self.last_hit_bearing = e.get("bearing")
                self.last_hit_weapon = e["weapon"]
                if self.dmg_log < 40:
                    self.dmg_log += 1
                    sys.stderr.write("t=%.1f HIT %s %.0f rear=%s by=%s hp=%.0f pos=(%.0f,%.0f)\n" % (t, e["weapon"], e["amount"], e.get("rear"), e.get("by"), hp, x, y))
        recently_hit = (t - self.last_hit_t) < 3.0

        enemies = sorted((m for m in obs["mechs"] if m["kind"] == "mech"), key=lambda m: dist(pos, m["pos"]))
        npcs = [m for m in obs["mechs"] if m["kind"] == "npc"]
        tgt = enemies[0] if enemies else None
        td = dist(pos, tgt["pos"]) if tgt else 999.0

        # sound cue: face loud fights
        if not enemies:
            for s in obs["sounds"]:
                if s["age"] < 0.7 and s["kind"] in ("gun", "laser", "mortar", "lightning", "explosion", "laser_charge") and s["dist"] != "far":
                    self.face_until = tick + 20
                    self.face_yaw = s["bearing"]
                    break

        # engaged?
        engaged = False
        if tgt:
            to_me = math.atan2(y - tgt["pos"][1], x - tgt["pos"][0])
            aimed = abs(wrap(tgt["aim"][0] - to_me)) < 0.4
            engaged = td < 48 or (recently_hit and td < 150) or (aimed and td < 130 and (tgt.get("charging") or recently_hit or td < 90))
        npc_fight = None
        if not engaged and recently_hit and self.last_hit_weapon == "npc" and npcs:
            n = min(npcs, key=lambda m: dist(pos, m["pos"]))
            if dist(pos, n["pos"]) < 55:
                npc_fight = n

        # hiding spot management
        pk = (round(nc[0] / 5), round(nc[1] / 5), round(nr))
        if pk != self.phase_key:
            self.phase_key = pk
            keep = self.spot is not None and nr >= 40 and dist(self.spot, nc) <= nr * 0.62
            if not keep:
                self.spot = self.pick_spot(pos, nc, nr)
        if self.spot is None:
            self.spot = self.pick_spot(pos, nc, nr)
        if nr < 40:
            self.spot = (nc[0], nc[1])
        spot = self.spot
        self.start_field(spot)
        if self.field_key == (round(spot[0]), round(spot[1])) and self.field is not None and self.grid.ok:
            i, j = self.grid.cell(x, y)
            if 0 <= i < self.grid.gx and 0 <= j < self.grid.gy and self.field[j * self.grid.gx + i] < 0 and dist(pos, spot) > 30 and not self.grid.blocked(x, y, True):
                self.bad_spots.add((round(spot[0]), round(spot[1])))
                self.spot = self.pick_spot(pos, nc, nr)
                self.field_key = None

        in_zone_now = me["in_zone"] and dist(pos, zc) < zr - 5
        move = None
        boost = False
        hold = False

        hp_frac = hp / maxhp
        if not me["in_zone"] or dist(pos, fc) > fr - 4:
            self.mode = "zone"
            d = flow = self.flow_dir(x, y, spot)
            move = unit([zc[0] - x, zc[1] - y]) if not me["in_zone"] else (flow or unit([fc[0] - x, fc[1] - y]))
            boost = energy > 12
        elif engaged and tgt:
            self.mode = "fight"
            tp = tgt["pos"]
            away = unit([x - tp[0], y - tp[1]])
            tang = [-away[1] * self.side, away[0] * self.side]
            if tick >= self.next_flip:
                self.side = -self.side
                self.next_flip = tick + 6 + (tick * 7) % 9
            ideal = 34.0
            if "lightning" in by_kind:
                ideal = 18.0 if "laser" in (tgt.get("weapons") or []) or "gun" in (tgt.get("weapons") or []) else 30.0
            elif "laser" in by_kind or "mortar" in by_kind:
                ideal = 48.0
            if hp_frac < 0.3 and td < 90:
                f = self.flow_dir(x, y, spot)
                move = f if (f and dist(pos, spot) > 8) else away
                boost = energy > 20
                self.mode = "flee"
            else:
                radial = clamp((ideal - td) / 18.0, -1, 1)
                move = unit([tang[0] + away[0] * radial, tang[1] + away[1] * radial])
            if tgt.get("charging") and abs(wrap(tgt["aim"][0] - math.atan2(y - tp[1], x - tp[0]))) < 0.3:
                move = tang
                act["jump"] = tick % 5 == 0
        elif npc_fight:
            self.mode = "npc"
            hold = True
        else:
            # calm: small detours for loot near our path, otherwise walk to / hold the spot
            self.mode = "move"
            goal = None
            if not enemies and hp_frac > 0.6:
                near_picks = [p for p in obs["pickups"] if p["kind"] in WPRI and p["kind"] not in wk and dist(pos, p["pos"]) < 28 and abs(p["pos"][2] - z) < 3.5
                              and dist(p["pos"], fc) < fr * 0.9 and (wk[1] is None or (wk[1] == "mortar" and p["kind"] != "mortar"))]
                if near_picks:
                    goal = min(near_picks, key=lambda p: dist(pos, p["pos"]))["pos"]
                else:
                    cubes = [c for c in obs["cubes"] if dist(pos, c["pos"]) < 22 and abs(c["pos"][2] - z) < 4 and dist(c["pos"], fc) < fr * 0.9]
                    if cubes:
                        goal = min(cubes, key=lambda c: dist(pos, c["pos"]))["pos"]
            if goal is not None:
                move = unit([goal[0] - x, goal[1] - y])
            elif dist(pos, spot) > 3.0:
                move = self.flow_dir(x, y, spot) or unit([spot[0] - x, spot[1] - y])
                far = dist(pos, spot)
                boost = far > 120 and energy > 70 and t > 40
            else:
                hold = True
                self.mode = "hide"
            if (recently_hit or tick < self.face_until) and not enemies and hold and hp_frac < 0.5:
                # being shot by someone unseen: slip toward zone centre instead of sitting
                b = self.last_hit_bearing if recently_hit and self.last_hit_bearing is not None else self.face_yaw
                move = unit([-math.cos(b) + (nc[0] - x) / 200.0, -math.sin(b) + (nc[1] - y) / 200.0])
                hold = False

        # weapon pickup
        near = [p for p in obs["pickups"] if p["kind"] in WPRI and p["kind"] not in wk and math.hypot(p["pos"][0] - x, p["pos"][1] - y) < 3.6 and abs(p["pos"][2] - z) < 3.5]
        if near and not engaged:
            p = max(near, key=lambda p: WPRI[p["kind"]])
            if wk[1] is None:
                act["take"] = p["id"]
            elif WPRI[p["kind"]] > WPRI.get(wk[1], 0) and by_kind.get(wk[1], (0, {}))[1].get("charge", 0) == 0:
                act["take"] = p["id"]; act["drop"] = 1

        if move is None:
            move = [0.0, 0.0]
        dv = self.dodge(pos, obs["projectiles"])
        if abs(dv[0]) + abs(dv[1]) > 1e-6:
            move = [move[0] + dv[0], move[1] + dv[1]]
            ln = math.hypot(*move)
            if ln > 1:
                move = [move[0] / ln, move[1] / ln]
        ln = math.hypot(*move)
        if ln > 0.05 and self.mode in ("fight", "flee", "zone", "npc") or (ln > 0.05 and self.flow_dir(x, y, spot) is None):
            s = self.steer(x, y, [move[0] / ln, move[1] / ln])
            move = [s[0] * min(1, ln), s[1] * min(1, ln)]

        # unstick
        spd = math.hypot(vel[0], vel[1])
        pm = math.hypot(*self.prev_move)
        if pm > 0.5 and spd < 0.2 * walk and not charging:
            self.stuck += 1
        else:
            self.stuck = 0
        if self.stuck >= 4:
            self.unstick_until = tick + 12
            self.stuck = 0
            self.side = -self.side
        if tick < self.unstick_until:
            b = unit(self.prev_move if pm > 0.1 else move)
            move = unit([-b[1] * self.side + b[0] * 0.4, b[0] * self.side + b[1] * 0.4])
            act["jump"] = bool(me["grounded"])
            boost = energy > 25
        act["move"] = move
        self.prev_move = move
        act["boost"] = bool(boost and energy > 15 and not charging)

        # ---------------- aim and fire
        aim = [cyaw, 0.0]
        fire = [False, False]
        reload = False
        target = tgt if (engaged or (tgt and td < 130)) else None
        if npc_fight:
            target = npc_fight
        if target is not None:
            tp = target["pos"]; tv = target.get("vel") or [0, 0, 0]
            is_npc = target["kind"] == "npc"
            cz = 1.75 if is_npc else 4.5
            d = dist(pos, tp)
            origin = [x, y, z + stats.get("muzzle_height", 7)]
            cpt = [tp[0], tp[1], tp[2] + cz]

            def ang(pt):
                return math.atan2(pt[1] - y, pt[0] - x), math.atan2(pt[2] - origin[2], max(0.1, math.hypot(pt[0] - x, pt[1] - y)))

            can_fire = engaged or npc_fight is not None
            gun = by_kind.get("gun"); laser = by_kind.get("laser"); light = by_kind.get("lightning"); mort = by_kind.get("mortar")
            gaim = ang(intercept(cpt, tv, origin, gun[1]["stats"]["speed"])) if gun else None
            laim = ang([cpt[0] + tv[0] * 0.1, cpt[1] + tv[1] * 0.1, cpt[2] + tv[2] * 0.1]) if laser else None
            maim = None
            if mort and not is_npc:
                ms = mort[1]["stats"]; mz = z + stats.get("height", 9) + 0.5; tt = d / ms["speed"]
                for _ in range(2):
                    px, py = cpt[0] + tv[0] * tt, cpt[1] + tv[1] * tt
                    hz = math.hypot(px - x, py - y)
                    pit = ballistic_pitch(ms["speed"], hz, cpt[2] - mz, ms["gravity"])
                    tt = hz / max(1.0, ms["speed"] * math.cos(pit))
                maim = (math.atan2(py - y, px - x), pit)
            mode = "gun" if gun else None
            if laser and not is_npc and (charging or (laser[1]["cooldown"] <= 0.3 and 24 <= d <= laser[1]["stats"]["range"] and self.mode != "flee")):
                mode = "laser"
            elif maim and 50 < d < 220 and mort[1]["cooldown"] <= 0.3 and (not gun or d > 110):
                mode = "mortar"
            chosen = {"gun": gaim, "laser": laim, "mortar": maim}.get(mode) or ang(cpt)
            aim = [chosen[0], chosen[1]]
            tol = clamp(math.atan2(2.8, max(d, 1)), 0.02, 0.12)

            def err(a):
                return 9.0 if a is None else max(abs(wrap(cyaw - a[0])), abs(cpit - a[1]) * 0.8)

            if can_fire:
                if gun:
                    gi, gw = gun
                    if mode != "gun":
                        pass
                    elif gw.get("reload", 0) > 0:
                        pass
                    elif gw.get("ammo", 0) <= 0:
                        reload = True
                    elif (
                        d < gw["stats"]["range"] * 0.95
                        and err(gaim) < tol
                        and self.has_shot_los(origin, tp)
                    ):
                        fire[gi] = True
                if laser:
                    li, lw = laser; ch = lw.get("charge", 0); full = lw["stats"]["charge"] * 0.92
                    if mode == "laser" and d <= lw["stats"]["range"] and self.has_shot_los(origin, tp):
                        le = err(laim)
                        if ch < full:
                            fire[li] = True
                        elif le > tol * 1.5 and ch < full + 1.0:
                            fire[li] = True
                        else:
                            fire[li] = False
                if light:
                    li2, lw2 = light
                    lightning_yaw = math.atan2(tp[1] - y, tp[0] - x)
                    if d <= lw2["stats"]["range"] - 3 and lw2["cooldown"] <= 0 and energy > 8 and abs(wrap(cyaw - lightning_yaw)) < 0.45 and self.has_shot_los(origin, tp):
                        fire[li2] = True
                        if mode != "laser" and d < 30:
                            aim = [math.atan2(tp[1] - y, tp[0] - x), ang(cpt)[1]]
                if mort and mode == "mortar" and mort[1]["cooldown"] <= 0 and err(maim) < 0.05:
                    fire[mort[0]] = True
            if charging and mode != "laser":
                fire[by_kind["laser"][0]] = False
        else:
            if tick < self.face_until:
                aim = [self.face_yaw, 0.0]
            elif recently_hit and self.last_hit_bearing is not None:
                aim = [self.last_hit_bearing, 0.0]
            elif math.hypot(*move) > 0.2:
                aim = [math.atan2(move[1], move[0]) + 0.8 * math.sin(tick * 0.2), 0.0]
            else:
                base = math.atan2(nc[1] - y, nc[0] - x)
                aim = [base + 1.6 * math.sin(tick * 0.07), 0.0]
            g = by_kind.get("gun")
            if g and g[1].get("reload", 0) <= 0 and g[1].get("ammo", 99) < g[1]["stats"]["mag"] - 2 and not enemies:
                reload = True
        aim[0] = wrap(aim[0])
        act["aim"] = aim
        act["fire"] = fire
        if reload:
            act["reload"] = True

        if tick % 100 == 0 and self.nlog < 120:
            self.nlog += 1
            sys.stderr.write("t=%.0f hp=%.0f/%.0f cubes=%d alive=%d mode=%s foes=%d pos=(%.0f,%.0f) spot=(%.0f,%.0f) w=%s zone_r=%.0f\n" % (t, hp, maxhp, me["cubes"], obs["alive"], self.mode, len(enemies), x, y, spot[0], spot[1], wk, zr))
        mf.draw.label("%s hp%d" % (self.mode, hp))
        return act


if __name__ == "__main__":
    mf.run(Bot)