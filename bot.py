import json
import math
import os
import random
import sys
import time
import traceback

import mechfight as mf

PI = math.pi
TAU = 2.0 * math.pi
P = {'ideal': 95.0, 'engage_r': 110.0, 'flee_hp': 0.45, 'cover': True, 'cover_any': True}


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(v):
    d = math.hypot(v[0], v[1])
    return [v[0] / d, v[1] / d] if d > 1e-8 else [0.0, 0.0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_diff(a, b):
    return (a - b + PI) % TAU - PI


def pip(ring, x, y):
    inside = False
    n = len(ring) // 2
    j = n - 1
    for i in range(n):
        xi, yi = ring[2 * i], ring[2 * i + 1]
        xj, yj = ring[2 * j], ring[2 * j + 1]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def intercept(pos, vel, origin, speed):
    r = [pos[i] - origin[i] for i in range(3)]
    a = sum(v * v for v in vel) - speed * speed
    b = 2.0 * sum(r[i] * vel[i] for i in range(3))
    c = sum(v * v for v in r)
    if abs(a) < 1e-8:
        t = -c / b if abs(b) > 1e-8 else 0.0
    else:
        disc = b * b - 4.0 * a * c
        roots = []
        if disc >= 0:
            s = math.sqrt(disc)
            roots = [(-b - s) / (2.0 * a), (-b + s) / (2.0 * a)]
        valid = [v for v in roots if v > 0]
        t = min(valid) if valid else 0.0
    t = clamp(t, 0.0, 1.2)
    return [pos[i] + vel[i] * t for i in range(3)]


def ballistic_pitch(speed, horizontal, dz, gravity):
    if horizontal < 1.0:
        return 0.0
    v2 = speed * speed
    disc = v2 * v2 - gravity * (gravity * horizontal * horizontal + 2.0 * dz * v2)
    if disc < 0.0:
        return math.atan2(dz, horizontal)
    return math.atan((v2 - math.sqrt(disc)) / (gravity * horizontal))


class Grid:
    """Coarse 2D obstacle grid of ground-level buildings, for steering only."""

    def __init__(self, map_info):
        self.ok = False
        try:
            self._load(map_info or {})
        except Exception:
            self.ok = False
            sys.stderr.write("grid load failed:\n" + traceback.format_exc() + "\n")

    def _load(self, info):
        t0 = time.time()
        base = info.get("dir", "/arena")
        with open(os.path.join(base, "v2", "world.json")) as f:
            w = json.load(f)
        ter = w["terrain"]
        self.minx = float(ter["minX"])
        self.miny = float(ter["minY"])
        step = float(ter.get("step", 1.0))
        cs = self.cs = 3.0
        self.gx = int(ter["nx"] * step / cs) + 2
        self.gy = int(ter["ny"] * step / cs) + 2
        gx, gy = self.gx, self.gy
        g = bytearray(gx * gy)
        for p in w.get("prisms", []):
            if time.time() - t0 > 6.0:
                break
            if p.get("kind") == "bridge" or p.get("base", 0.0) > 4.0:
                continue
            ring = p.get("ring") or []
            if len(ring) < 6:
                continue
            holes = p.get("holes") or []
            if holes and not isinstance(holes[0], (list, tuple)):
                holes = []
            xs = ring[0::2]
            ys = ring[1::2]
            i0 = max(0, int((min(xs) - self.minx) / cs))
            i1 = min(gx - 1, int((max(xs) - self.minx) / cs))
            j0 = max(0, int((min(ys) - self.miny) / cs))
            j1 = min(gy - 1, int((max(ys) - self.miny) / cs))
            for k in range(len(xs)):
                i = int((xs[k] - self.minx) / cs)
                j = int((ys[k] - self.miny) / cs)
                if 0 <= i < gx and 0 <= j < gy:
                    g[j * gx + i] = 1
            for j in range(j0, j1 + 1):
                cy = self.miny + (j + 0.5) * cs
                for i in range(i0, i1 + 1):
                    cx = self.minx + (i + 0.5) * cs
                    if pip(ring, cx, cy):
                        if holes and any(pip(h, cx, cy) for h in holes):
                            continue
                        g[j * gx + i] = 1
        self.raw = g
        # dilate by one cell (mech radius 2.5 m)
        cells = [idx for idx in range(gx * gy) if g[idx]]
        out = bytearray(g)
        for idx in cells:
            j, i = divmod(idx, gx)
            for dj in (-1, 0, 1):
                for di in (-1, 0, 1):
                    jj, ii = j + dj, i + di
                    if 0 <= jj < gy and 0 <= ii < gx:
                        out[jj * gx + ii] = 1
        self.g = out
        self.ok = True

    def blocked(self, x, y):
        i = int((x - self.minx) / self.cs)
        j = int((y - self.miny) / self.cs)
        if i < 0 or j < 0 or i >= self.gx or j >= self.gy:
            return True
        return self.g[j * self.gx + i] == 1

    def raw_blocked(self, x, y):
        i = int((x - self.minx) / self.cs)
        j = int((y - self.miny) / self.cs)
        if i < 0 or j < 0 or i >= self.gx or j >= self.gy:
            return False
        return self.raw[j * self.gx + i] == 1

    def hidden_from(self, px, py, tx, ty):
        """True if a building lies between point p and threat t (ground-level footprint test)."""
        d = math.hypot(tx - px, ty - py)
        if d < 8.0:
            return False
        ux, uy = (tx - px) / d, (ty - py) / d
        s = 4.0
        top = min(d - 3.0, 260.0)
        while s < top:
            if self.raw_blocked(px + ux * s, py + uy * s):
                return True
            s += 2.0
        return False

    def cover_point(self, x, y, threats, fc, fr):
        if not self.ok or not threats:
            return None
        best, bs = None, 1e9
        for r in (7.0, 12.0, 18.0, 26.0, 36.0):
            for k in range(16):
                a = k * TAU / 16.0
                px, py = x + math.cos(a) * r, y + math.sin(a) * r
                if self.blocked(px, py):
                    continue
                if not all(self.hidden_from(px, py, t[0], t[1]) for t in threats):
                    continue
                if not self.clear(x, y, a, r):
                    continue
                sc = r + (500.0 if math.hypot(px - fc[0], py - fc[1]) > fr * 0.9 else 0.0)
                if sc < bs:
                    bs, best = sc, [px, py]
            if best is not None:
                break
        return best

    def clear(self, x, y, ang, length):
        c, s = math.cos(ang), math.sin(ang)
        d = 3.0
        while d <= length:
            if self.blocked(x + c * d, y + s * d):
                return False
            d += 3.0
        return True


class Bot:
    name = "avoider"

    def __init__(self, hello: mf.Hello):
        self.config = hello["config"]
        self.slot = hello.get("slot", 0)
        self.rng = random.Random(hello.get("seed", 0) or 0)
        self.grid = Grid(hello.get("map"))
        self.side = 1.0 if self.slot % 2 == 0 else -1.0
        self.next_flip = 0
        self.last_dmg_t = -99.0
        self.last_dmg_bearing = None
        self.stuck_count = 0
        self.unstick_until = -1
        self.prev_move = [0.0, 0.0]
        self.err_logged = False
        self.cover = None
        self.cover_until = -1.0
        self.hold_cover = False

    # ------------------------------------------------------------------ helpers
    def zone_state(self, zone, t):
        start = zone["shrink_start"]
        end = zone["shrink_end"]
        q = clamp((t - start) / max(1e-6, end - start), 0.0, 1.0)
        center = [
            zone["center"][i] + (zone["next_center"][i] - zone["center"][i]) * q
            for i in range(2)
        ]
        radius = zone["radius"] + (zone["next_radius"] - zone["radius"]) * q
        return center, radius

    def choose_upgrade(self, offer, me):
        choices = offer.get("choices", [])
        if not choices:
            return 0
        scores = []
        for choice in choices:
            text = (
                choice.get("id", "") + " " + choice.get("name", "") + " " + choice.get("desc", "")
            ).lower()
            score = 0
            if "armor" in text or "damage taken" in text or "resist" in text:
                score += 22
            if "regen" in text:
                score += 24
            if "max_hp" in text or "health" in text or "max hp" in text:
                score += 18
            if "walk_speed" in text or "movement" in text or "speed" in text:
                score += 14
            if "boost" in text or "energy" in text:
                score += 12
            if "gun_rate" in text or "fire rate" in text:
                score += 28
            if "damage" in text and "damage taken" not in text:
                score += 30
            if "cooldown" in text:
                score += 15
            if "sensors" in text or "sensor" in text:
                score += 7
            scores.append(score)
        return max(range(len(scores)), key=lambda i: scores[i])

    def steer(self, pos, d):
        """Pick a direction near d that is not blocked by a building."""
        if not self.grid.ok or (d[0] == 0.0 and d[1] == 0.0):
            return d
        base = math.atan2(d[1], d[0])
        x, y = pos[0], pos[1]
        for off in (0.0, 0.35, -0.35, 0.7, -0.7, 1.05, -1.05, 1.4, -1.4, 1.9, -1.9, 2.6, -2.6, PI):
            a = base + off * self.side
            if self.grid.clear(x, y, a, 15.0):
                return [math.cos(a), math.sin(a)]
        return d

    def dodge(self, pos, projectiles):
        dv = [0.0, 0.0]
        x, y, z = pos
        for p in projectiles:
            pp = p.get("pos")
            vv = p.get("vel")
            if not pp or not vv:
                continue
            kind = p.get("kind")
            if kind == "mortar":
                g = self.config.get("weapons", {}).get("mortar", {}).get("gravity", 22.0)
                disc = vv[2] * vv[2] + 2.0 * g * (pp[2] - z - 0.5)
                if disc < 0:
                    continue
                tl = (vv[2] + math.sqrt(disc)) / g
                if tl < 0 or tl > 2.8:
                    continue
                lx = pp[0] + vv[0] * tl
                ly = pp[1] + vv[1] * tl
                d = math.hypot(x - lx, y - ly)
                if d < 11.0:
                    away = unit([x - lx, y - ly])
                    if away == [0.0, 0.0]:
                        away = [1.0, 0.0]
                    dv[0] += away[0] * 2.0
                    dv[1] += away[1] * 2.0
                continue
            rx, ry = pp[0] - x, pp[1] - y
            vx, vy = vv[0], vv[1]
            v2 = vx * vx + vy * vy
            if v2 < 1.0:
                continue
            tc = -(rx * vx + ry * vy) / v2
            if tc < 0.0 or tc > 0.8:
                continue
            pz = pp[2] + vv[2] * tc
            if abs(pz - (z + 4.5)) > 7.0:
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
                dv[0] += n[0] * 1.6
                dv[1] += n[1] * 1.6
        return dv

    # ------------------------------------------------------------------ main
    def act(self, obs: mf.Observation) -> mf.Action:
        try:
            return self._act(obs)
        except Exception:
            if not self.err_logged:
                sys.stderr.write(traceback.format_exc() + "\n")
                self.err_logged = True
            try:
                me = obs["self"]
                zc, _ = self.zone_state(obs["zone"], obs["t"])
                mv = unit([zc[0] - me["pos"][0], zc[1] - me["pos"][1]])
                return {"tick": obs["tick"], "move": mv, "fire": [True, False]}
            except Exception:
                return {"tick": obs.get("tick", 0), "move": [0.0, 0.0]}

    def _act(self, obs):
        me = obs["self"]
        pos = me["pos"]
        x, y, z = pos
        t = obs["t"]
        tick = obs["tick"]
        myid = me["id"]
        stats = me.get("stats", {})
        walk = stats.get("walk_speed", 11.7)
        hp = me["hp"]
        maxhp = me["max_hp"]
        energy = me.get("energy", 0.0)
        vel = me.get("vel", [0.0, 0.0, 0.0])
        cur_aim = me.get("aim", [0.0, 0.0])
        cur_yaw, cur_pitch = cur_aim[0], cur_aim[1]

        # ---------- zone
        zc, zr = self.zone_state(obs["zone"], t)
        fc, fr = self.zone_state(obs["zone"], t + 10.0)
        zone_out = dist(pos, zc) > zr - 6.0
        inward_f = unit([fc[0] - x, fc[1] - y])
        out_of_future = dist(pos, fc) > max(10.0, fr * 0.75)

        # ---------- weapons
        weapons = me.get("weapons", [None, None])
        while len(weapons) < 2:
            weapons.append(None)
        by_kind = {}
        for i, w in enumerate(weapons):
            if w:
                by_kind[w.get("kind")] = (i, w)
        secondary = weapons[1]
        charging_laser = False
        if "laser" in by_kind and by_kind["laser"][1].get("charge", 0.0) > 0.0:
            charging_laser = True

        # ---------- events
        for e in obs.get("events", []):
            if e.get("type") == "damaged" and e.get("bearing") is not None:
                self.last_dmg_t = t
                self.last_dmg_bearing = e["bearing"]
        recently_hit = (t - self.last_dmg_t) < 2.0
        for e in obs.get("events", []):
            if e.get("type") == "damaged" and e.get("weapon") != "zone" and getattr(self, "_nl", 0) < 60:
                self._nl = getattr(self, "_nl", 0) + 1
                sys.stderr.write("t=%.1f HIT %s %.0f rear=%s by=%s hp=%.0f\n" % (t, e.get("weapon"), e.get("amount", 0), e.get("rear"), e.get("by"), hp))
        if tick % 100 == 0:
            sys.stderr.write("t=%.0f hp=%.0f/%.0f alive=%s cubes=%s w=%s pos=(%.0f,%.0f) in_zone=%s zr=%.0f\n" % (t, hp, maxhp, obs.get("alive"), me.get("cubes"), [w["kind"] if w else None for w in weapons], x, y, me.get("in_zone"), zr))

        # ---------- perception
        mechs = obs.get("mechs", [])
        enemies = [m for m in mechs if m.get("kind") == "mech" and m.get("id") != myid]
        enemies.sort(key=lambda m: dist(pos, m["pos"]))
        npcs = [m for m in mechs if m.get("kind") == "npc"]
        near = [m for m in enemies if dist(pos, m["pos"]) < 100.0]
        close = [m for m in enemies if dist(pos, m["pos"]) < 55.0]

        target = None
        tkind = None
        if enemies:
            # nearest enemy, with a mild preference for wounded ones (finishing kills = cubes + loot)
            def _tscore(m):
                frac = m.get("hp", 1400) / max(1.0, m.get("max_hp", 1400))
                return dist(pos, m["pos"]) + 25.0 * frac
            target = min(enemies[:5], key=_tscore)
            tkind = "mech"
        else:
            cand = [n for n in npcs if dist(pos, n["pos"]) < 110.0 and abs(n["pos"][2] - z) < 8.0]
            if cand and hp > 0.35 * maxhp:
                target = min(cand, key=lambda n: dist(pos, n["pos"]))
                tkind = "npc"
            else:
                crates = [
                    c for c in obs.get("crates", [])
                    if dist(pos, c["pos"]) < 65.0 and abs(c["pos"][2] - z) < 6.0
                ]
                if crates and hp > 0.5 * maxhp:
                    target = min(crates, key=lambda c: dist(pos, c["pos"]))
                    tkind = "crate"
        tdist = dist(pos, target["pos"]) if target else 999.0

        sounds = [s for s in obs.get("sounds", []) if s.get("age", 99) < 1.0]
        sound_threat = None
        if not enemies:
            for s in sounds:
                if s.get("dist") == "near" and s.get("kind") in (
                    "gun", "laser", "laser_charge", "lightning", "step", "boost", "jet", "mortar"
                ):
                    sound_threat = s["bearing"]
                    break

        # strafe flip
        if tick >= self.next_flip:
            self.side = -self.side
            self.next_flip = tick + self.rng.randint(5, 14)

        action = {
            "tick": tick,
            "move": [0.0, 0.0],
            "fire": [False, False],
            "boost": False,
            "jump": False,
        }

        if obs.get("offer"):
            action["pick"] = self.choose_upgrade(obs["offer"], me)

        # ---------- weapon pickups (take)
        pickups = [
            p for p in obs.get("pickups", [])
            if p.get("kind") in ("mortar", "laser", "lightning") and abs(z - p["pos"][2]) < 3.8
        ]
        held_kinds = set(by_kind.keys())
        wanted = []
        for p in pickups:
            k = p["kind"]
            if k in held_kinds:
                continue
            if secondary is None:
                wanted.append(p)
            elif (secondary.get("kind") == "mortar" and k in ("laser", "lightning")) or (secondary.get("kind") == "lightning" and k == "laser"):
                wanted.append(p)
        for p in wanted:
            if dist(pos, p["pos"]) <= 3.7:
                action["take"] = p["id"]
                if secondary is not None:
                    action["drop"] = 1
                break

        # ---------- movement decision
        move_dir = None
        flee = False
        boost_wanted = False
        hold_still = False

        flee_cond = (
            len(close) >= 3
            or (hp < maxhp * P["flee_hp"] and (near or recently_hit))
            or (tkind == "mech" and tdist < 14.0)
        )

        sniped = False
        self.hold_cover = False
        if tkind == "mech" and tdist >= 45.0 and not flee_cond:
            ang_to_me = math.atan2(y - target["pos"][1], x - target["pos"][0])
            aimed = abs(angle_diff(target["aim"][0], ang_to_me)) < 0.3
            if not P["cover"]:
                pass
            elif target.get("charging") and aimed:
                sniped = True
            elif P["cover_any"] and tdist < 230.0:
                sniped = True
            elif (t - self.last_dmg_t) < 1.5:
                sniped = True
            elif self.cover_until > t:
                sniped = True

        if zone_out:
            move_dir = inward_f
            boost_wanted = True
        elif sniped:
            if self.cover is None or t > self.cover_until:
                threats = [e["pos"] for e in enemies[:3] if dist(pos, e["pos"]) < 260.0]
                self.cover = self.grid.cover_point(x, y, threats, fc, fr)
                self.cover_until = t + 4.0
            if self.cover is not None and dist(pos, self.cover) > 2.5:
                move_dir = unit([self.cover[0] - x, self.cover[1] - y])
                boost_wanted = True
            elif self.cover is not None:
                self.hold_cover = True
            else:
                tp = target["pos"]
                away = unit([x - tp[0], y - tp[1]])
                tang = [-away[1] * self.side, away[0] * self.side]
                move_dir = unit([tang[0] + 0.3 * away[0], tang[1] + 0.3 * away[1]])
        elif flee_cond and (near or recently_hit):
            flee = True
            if near:
                sx = sy = 0.0
                for e in near:
                    dx = x - e["pos"][0]
                    dy = y - e["pos"][1]
                    d = max(8.0, math.hypot(dx, dy))
                    w = 1.0 / (d * d)
                    sx += dx * w
                    sy += dy * w
                esc = unit([sx, sy])
            elif self.last_dmg_bearing is not None:
                esc = [-math.cos(self.last_dmg_bearing), -math.sin(self.last_dmg_bearing)]
            else:
                esc = [-math.cos(cur_yaw), -math.sin(cur_yaw)]
            if esc == [0.0, 0.0]:
                esc = [math.cos(myid), math.sin(myid)]
            tang = [-esc[1] * self.side, esc[0] * self.side]
            vx = esc[0] + tang[0] * 0.35
            vy = esc[1] + tang[1] * 0.35
            if out_of_future:
                vx += inward_f[0] * 0.9
                vy += inward_f[1] * 0.9
            move_dir = unit([vx, vy])
            boost_wanted = True
        elif tkind == "mech" and tdist < P["engage_r"]:
            tp = target["pos"]
            away = unit([x - tp[0], y - tp[1]])
            if away == [0.0, 0.0]:
                away = [1.0, 0.0]
            tang = [-away[1] * self.side, away[0] * self.side]
            ideal = P["ideal"]
            tw = target.get("weapons") or []
            if "lightning" in by_kind:
                # lightning only reaches 45 m: close in so it actually fires (unless they hold a mortar)
                ideal = 55.0 if "mortar" in tw else 32.0
            elif "lightning" in tw:
                ideal = 70.0
            thp = target.get("hp", 1400) / max(1.0, target.get("max_hp", 1400))
            if thp < 0.3 and hp > maxhp * 0.5:
                ideal = min(ideal, 28.0)  # finish off a wounded enemy
            radial = clamp((ideal - tdist) / 20.0, -1.0, 1.0)
            vx = tang[0] + away[0] * radial
            vy = tang[1] + away[1] * radial
            if out_of_future:
                vx += inward_f[0]
                vy += inward_f[1]
            move_dir = unit([vx, vy])
        else:
            goal = None
            # supply drops
            for dr in obs.get("drops", []):
                dd = dist(pos, dr["pos"])
                holder = dr.get("holder")
                if holder is not None and holder != myid:
                    continue
                if dist(dr["pos"], fc) > fr + 10.0 or dd > 260.0:
                    continue
                if dr.get("lands_in", 0.0) <= dd / max(walk, 1.0) + 8.0 and not close:
                    if dd < dr.get("radius", 8.0) - 2.0 and dr.get("lands_in", 0.0) <= 0.5:
                        hold_still = True
                        goal = None
                    else:
                        goal = dr["pos"]
                    break
            if goal is None and not hold_still and not near:
                if wanted:
                    pk = min(wanted, key=lambda p: dist(pos, p["pos"]) - (70.0 if p["kind"] == "laser" else 25.0 if p["kind"] == "lightning" else 0.0))
                    if dist(pos, pk["pos"]) < 210.0 and dist(pk["pos"], fc) < fr * 0.95:
                        goal = pk["pos"]
                if goal is None:
                    cubes = [
                        c for c in obs.get("cubes", [])
                        if dist(pos, c["pos"]) < 55.0 and abs(c["pos"][2] - z) < 5.0
                        and dist(c["pos"], fc) < fr * 0.85
                    ]
                    if cubes:
                        goal = min(cubes, key=lambda c: dist(pos, c["pos"]))["pos"]
            if goal is None and not hold_still and target is not None and tkind in ("npc", "crate"):
                if tdist > 28.0:
                    goal = target["pos"]
                else:
                    tp = target["pos"]
                    away = unit([x - tp[0], y - tp[1]])
                    tang = [-away[1] * self.side, away[0] * self.side]
                    move_dir = unit([tang[0] * 0.6 + away[0] * clamp((24.0 - tdist) / 10.0, -1, 1),
                                     tang[1] * 0.6 + away[1] * clamp((24.0 - tdist) / 10.0, -1, 1)])
                    if tkind == "crate":
                        move_dir = [0.0, 0.0]
            if goal is None and move_dir is None and not hold_still:
                if sound_threat is not None or recently_hit:
                    b = sound_threat if sound_threat is not None else self.last_dmg_bearing
                    esc = [-math.cos(b), -math.sin(b)]
                    move_dir = unit([esc[0] + inward_f[0] * 0.6, esc[1] + inward_f[1] * 0.6])
                    boost_wanted = recently_hit
                elif dist(pos, fc) > max(10.0, fr * 0.55):
                    goal = fc
                else:
                    hold_still = True
            if goal is not None and move_dir is None:
                if dist(pos, goal) > 1.5:
                    move_dir = unit([goal[0] - x, goal[1] - y])
                    if dist(pos, goal) > 50.0:
                        boost_wanted = energy > 60.0

        if not sniped:
            self.cover = None
        if move_dir is None:
            move_dir = [0.0, 0.0]

        # ---------- dodge incoming shots
        dv = self.dodge(pos, obs.get("projectiles", []))
        dodging = abs(dv[0]) + abs(dv[1]) > 1e-6
        if dodging:
            move_dir = [move_dir[0] + dv[0], move_dir[1] + dv[1]]
            ln = math.hypot(move_dir[0], move_dir[1])
            if ln > 1.0:
                move_dir = [move_dir[0] / ln, move_dir[1] / ln]

        # ---------- obstacle steering
        if z < 8.0 and (move_dir[0] != 0.0 or move_dir[1] != 0.0):
            ln = math.hypot(move_dir[0], move_dir[1])
            st = self.steer(pos, [move_dir[0] / ln, move_dir[1] / ln])
            move_dir = [st[0] * min(1.0, ln), st[1] * min(1.0, ln)]

        # ---------- stuck detection
        speed = math.hypot(vel[0], vel[1])
        pm = math.hypot(self.prev_move[0], self.prev_move[1])
        if pm > 0.5 and speed < 0.2 * walk and not charging_laser:
            self.stuck_count += 1
        else:
            self.stuck_count = 0
        jump = False
        if self.stuck_count >= 3:
            self.unstick_until = tick + 14
            self.stuck_count = 0
            self.side = -self.side
        if tick < self.unstick_until:
            base = self.prev_move if pm > 0.1 else move_dir
            b = unit(base)
            move_dir = [-b[1] * self.side + b[0] * 0.3, b[0] * self.side + b[1] * 0.3]
            move_dir = unit(move_dir)
            jump = bool(me.get("grounded"))
            boost_wanted = energy > 20.0

        action["move"] = move_dir
        self.prev_move = move_dir

        if charging_laser:
            boost_wanted = False
        action["boost"] = bool(boost_wanted and energy > 25.0)
        action["jump"] = jump

        # ---------- aiming & firing
        aim_yaw = cur_yaw
        aim_pitch = 0.0
        fire = [False, False]
        reload = False

        if target is not None:
            tp = target["pos"]
            tv = target.get("vel") or [0.0, 0.0, 0.0]
            if len(tv) < 3:
                tv = [tv[0], tv[1], 0.0]
            cz = {"mech": 4.5, "npc": 1.75, "crate": 2.0}.get(tkind, 3.0)
            mh = stats.get("muzzle_height", 7.0)
            origin = [x, y, z + mh]
            cpt = [tp[0], tp[1], tp[2] + cz]
            d = tdist

            gun = by_kind.get("gun")
            laser = by_kind.get("laser")
            lightning = by_kind.get("lightning")
            mortar = by_kind.get("mortar")

            # ideal aims per weapon
            def ang_to(pt, o=origin):
                yaw = math.atan2(pt[1] - y, pt[0] - x)
                pit = math.atan2(pt[2] - o[2], max(0.1, math.hypot(pt[0] - x, pt[1] - y)))
                return yaw, pit

            gun_aim = None
            if gun:
                gp = intercept(cpt, tv, origin, gun[1].get("stats", {}).get("speed", 252.0))
                gun_aim = ang_to(gp)
            laser_aim = None
            if laser:
                lp = [cpt[0] + tv[0] * 0.12, cpt[1] + tv[1] * 0.12, cpt[2] + tv[2] * 0.12]
                laser_aim = ang_to(lp)
            mortar_aim = None
            if mortar and tkind == "mech":
                ms = mortar[1].get("stats", {})
                spd = ms.get("speed", 72.0)
                grav = ms.get("gravity", 22.0)
                morigin_z = z + stats.get("height", 9.0) + 0.5
                tt = d / spd
                yaw_m, pit_m = 0.0, 0.0
                for _ in range(2):
                    px = cpt[0] + tv[0] * tt
                    py = cpt[1] + tv[1] * tt
                    pz = cpt[2]
                    hz = math.hypot(px - x, py - y)
                    pit_m = ballistic_pitch(spd, hz, pz - morigin_z, grav)
                    yaw_m = math.atan2(py - y, px - x)
                    tt = hz / max(1.0, spd * math.cos(pit_m))
                mortar_aim = (yaw_m, pit_m)

            # choose which weapon steers the torso
            mode = "gun" if gun else None
            if laser and tkind in ("mech", "npc"):
                lrange = laser[1].get("stats", {}).get("range", 280.0)
                lready = laser[1].get("cooldown", 0.0) <= 0.3
                if d <= lrange and (charging_laser or (lready and d >= 22.0 and not (flee and d < 35.0) and not self.hold_cover)):
                    mode = "laser"
            if mode != "laser" and mortar_aim is not None:
                mrange = mortar[1].get("stats", {}).get("range", 235.0)
                if 50.0 < d < mrange and mortar[1].get("cooldown", 0.0) <= 0.3 and (
                    not gun or d > 120.0
                ):
                    mode = "mortar"
            if mode is None:
                mode = "gun"

            chosen = {"gun": gun_aim, "laser": laser_aim, "mortar": mortar_aim}.get(mode)
            if chosen is None:
                chosen = ang_to(cpt)
            aim_yaw, aim_pitch = chosen

            tol = clamp(math.atan2(2.8, max(d, 1.0)), 0.02, 0.12)

            def err(aim):
                if aim is None:
                    return 9.0
                return max(abs(angle_diff(cur_yaw, aim[0])), abs(cur_pitch - aim[1]) * 0.8)

            if gun:
                gi, gw = gun
                gs = gw.get("stats", {})
                if gw.get("reload", 0.0) > 0.0:
                    pass
                elif gw.get("ammo", 0) <= 0:
                    reload = True
                elif d < gs.get("range", 230.0) * 0.95 and err(gun_aim) < tol and not (self.hold_cover and d > 45.0):
                    fire[gi] = True

            if laser:
                li, lw = laser
                ls = lw.get("stats", {})
                charge = lw.get("charge", 0.0)
                if mode == "laser":
                    full = ls.get("charge", 0.75) * 0.9
                    le = err(laser_aim)
                    if charge < full:
                        fire[li] = True
                    elif le > tol * 1.5 and charge < full + 1.2:
                        fire[li] = True  # keep holding until the aim settles
                    else:
                        fire[li] = False  # release
                elif charge > 0.0:
                    fire[li] = False  # release stale charge

            if lightning:
                ni, nw = lightning
                ns = nw.get("stats", {})
                if (
                    d <= ns.get("range", 45.0) - 3.0
                    and nw.get("cooldown", 0.0) <= 0.0
                    and energy > 8.0
                    and abs(angle_diff(cur_yaw, math.atan2(tp[1] - y, tp[0] - x))) < 0.45
                ):
                    fire[ni] = True

            if mortar and mode == "mortar":
                mi, mw = mortar
                if mw.get("cooldown", 0.0) <= 0.0 and err(mortar_aim) < 0.05:
                    fire[mi] = True

            if charging_laser and mode != "laser":
                fire[by_kind["laser"][0]] = False
        else:
            if tick < 32:
                aim_yaw = cur_yaw + 1.0
            elif recently_hit and self.last_dmg_bearing is not None:
                aim_yaw = self.last_dmg_bearing
            elif sound_threat is not None:
                aim_yaw = sound_threat
            elif abs(move_dir[0]) + abs(move_dir[1]) > 0.2:
                aim_yaw = math.atan2(move_dir[1], move_dir[0]) + 0.7 * math.sin(tick * 0.25)
            else:
                aim_yaw = cur_yaw + 0.3
            gun = by_kind.get("gun")
            if gun and gun[1].get("reload", 0.0) <= 0.0 and gun[1].get("ammo", 99) < gun[1].get("stats", {}).get("mag", 12) - 2 and not near:
                reload = True

        aim_yaw = (aim_yaw + PI) % TAU - PI
        action["aim"] = [aim_yaw, aim_pitch]
        action["fire"] = fire
        if reload:
            action["reload"] = True
        return action


if __name__ == "__main__":
    mf.run(Bot)