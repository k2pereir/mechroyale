import json
import math
import os
import mechfight as mf

CELL = 20.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pip(x, y, ring):
    ins = False
    n = len(ring) // 2
    j = n - 1
    for i in range(n):
        xi, yi, xj, yj = ring[2 * i], ring[2 * i + 1], ring[2 * j], ring[2 * j + 1]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            ins = not ins
        j = i
    return ins


UPG = ["armor", "plating", "nanites", "gun_damage", "gun_rate", "laser_damage", "lightning_efficiency", "mortar_reload",
       "reactor", "capacitor", "gyros", "servos", "gun_mag", "laser_charge", "lightning_chain", "jets", "gun_velocity",
       "mortar_splash", "lightning_range", "mortar_velocity", "sensors", "laser_pierce"]
WPRI = {"laser": 3, "lightning": 2, "mortar": 1}


class Bot:
    name = "survivor-v1"

    def __init__(self, hello):
        cfg = hello["config"]
        self.reach = cfg["loot"]["pickup_radius"] - 0.5
        self.grid = {}
        self.prisms = []
        try:
            d = hello["map"]["dir"]
            w = json.load(open(os.path.join(d, "v2/world.json")))
            for p in w["prisms"]:
                holes = p.get("holes") or []
                if holes and not isinstance(holes[0], list):
                    holes = [holes]
                ring = p["ring"]
                xs, ys = ring[0::2], ring[1::2]
                bb = (min(xs), min(ys), max(xs), max(ys))
                pr = (p["base"], p["top"], ring, holes, bb)
                for cx in range(int(bb[0] // CELL), int(bb[2] // CELL) + 1):
                    for cy in range(int(bb[1] // CELL), int(bb[3] // CELL) + 1):
                        self.grid.setdefault((cx, cy), []).append(pr)
        except Exception:
            pass
        self.flip = 1
        self.flip_t = 0
        self.stuck = 0
        self.last_pos = None
        self.threat = None
        self.unstick_until = 0
        self.unstick_dir = 0.0

    def blocked(self, x, y, z):
        for base, top, ring, holes, bb in self.grid.get((int(x // CELL), int(y // CELL)), ()):
            if base < z + 9 and top > z + 1.2 and bb[0] <= x <= bb[2] and bb[1] <= y <= bb[3] and pip(x, y, ring) and not any(pip(x, y, h) for h in holes):
                return True
        return False

    def steer(self, x, y, z, yaw):
        for off in (0, 0.5, -0.5, 1.0, -1.0, 1.6, -1.6, 2.3, -2.3, 3.1):
            a = yaw + off
            ok = True
            for d in (4, 8, 14):
                if self.blocked(x + math.cos(a) * d, y + math.sin(a) * d, z):
                    ok = False
                    break
            if ok:
                return a
        return yaw

    def lead(self, me, t, speed):
        x, y, z = me["pos"]
        tx, ty, tz = t["pos"]
        vx, vy, vz = t["vel"]
        tz += 4.5
        mz = z + me["stats"].get("muzzle_height", 7)
        T = 0.0
        for _ in range(4):
            px, py, pz = tx + vx * T, ty + vy * T, tz + vz * T
            d = math.sqrt((px - x) ** 2 + (py - y) ** 2 + (pz - mz) ** 2)
            T = d / speed
        px, py, pz = tx + vx * T, ty + vy * T, tz + vz * T
        h = math.hypot(px - x, py - y)
        return math.atan2(py - y, px - x), math.atan2(pz - mz, h), math.sqrt(h * h + (pz - mz) ** 2)

    def mortar_aim(self, me, t, st):
        x, y, z = me["pos"]
        v, g = st["speed"], st["gravity"]
        px, py, pz = t["pos"]
        T = 0.0
        for _ in range(5):
            qx, qy = px + t["vel"][0] * T, py + t["vel"][1] * T
            R = math.hypot(qx - x, qy - y)
            s = g * R / (v * v)
            if s > 1:
                return None
            th = 0.5 * math.asin(s)
            T = R / (v * math.cos(th))
        return math.atan2(qy - y, qx - x), th, R

    def act(self, obs):
        me = obs["self"]
        tick = obs["tick"]
        x, y, z = me["pos"]
        act = {"tick": tick}
        zone = obs["zone"]
        stats = me["stats"]

        if obs["offer"]:
            ch = [c["id"] for c in obs["offer"]["choices"]]
            best = min(range(len(ch)), key=lambda i: UPG.index(ch[i]) if ch[i] in UPG else 99)
            act["pick"] = best

        wk = [w["kind"] if w else None for w in me["weapons"]]
        enemies = sorted((m for m in obs["mechs"] if m["kind"] == "mech"), key=lambda m: math.dist(m["pos"], me["pos"]))
        npcs = [m for m in obs["mechs"] if m["kind"] == "npc"]

        near = [p for p in obs["pickups"] if math.hypot(p["pos"][0] - x, p["pos"][1] - y) < self.reach and abs(p["pos"][2] - z) < 3.5 and p["kind"] not in wk]
        if near:
            near.sort(key=lambda p: -WPRI.get(p["kind"], 0))
            p = near[0]
            if wk[1] is None:
                act["take"] = p["id"]
            elif wk[1] is not None and WPRI.get(p["kind"], 0) > WPRI.get(wk[1], 0) and not (me["weapons"][1].get("charge", 0) > 0):
                act["take"] = p["id"]
                act["drop"] = 1

        for e in obs["events"]:
            if e["type"] == "damaged" and e.get("bearing") is not None and e["weapon"] != "zone":
                self.threat = (e["bearing"], tick)

        hp_frac = me["hp"] / me["max_hp"]
        tgt = None
        if enemies:
            tgt = enemies[0]
            for m in enemies[:3]:
                if math.dist(m["pos"], me["pos"]) < math.dist(tgt["pos"], me["pos"]) * 1.4 and m.get("hp", 1e9) < tgt.get("hp", 1e9) * 0.6:
                    tgt = m
        dist = math.dist(tgt["pos"], me["pos"]) if tgt else 1e9

        cx, cy = zone["next_center"] if tick * 0.05 >= zone["shrink_start"] - 15 else zone["center"]
        nr = zone["next_radius"] if tick * 0.05 >= zone["shrink_start"] - 15 else zone["radius"]
        gx, gy = cx, cy
        dc = math.hypot(cx - x, cy - y)
        safe = nr * 0.55
        hold = in_good = dc < safe and me["in_zone"]
        for d in obs["drops"]:
            dd = math.hypot(d["pos"][0] - x, d["pos"][1] - y)
            if d["lands_in"] < dd / 11 + 3 and dd < 160 and (not enemies or dist > 60) and hp_frac > 0.5:
                gx, gy = d["pos"][0], d["pos"][1]
                hold = False
                break
        else:
            if not enemies and hp_frac > 0.5 and me["in_zone"]:
                cand = []
                if wk[1] is None or True:
                    cand += [(math.hypot(p["pos"][0] - x, p["pos"][1] - y), p["pos"]) for p in obs["pickups"] if p["kind"] not in wk or wk[1] is None and p["kind"] not in wk]
                cand += [(math.hypot(c["pos"][0] - x, c["pos"][1] - y) * 0.7, c["pos"]) for c in obs["cubes"]]
                cand = [c for c in cand if c[0] < 60 and math.hypot(c[1][0] - cx, c[1][1] - cy) < nr * 0.95]
                if cand:
                    cand.sort(key=lambda c: c[0])
                    gx, gy = cand[0][1][0], cand[0][1][1]
                    hold = False

        speed_vec = None
        boost = False
        if tgt and dist < 90:
            to = math.atan2(tgt["pos"][1] - y, tgt["pos"][0] - x)
            if tick - self.flip_t > 12 + (tick * 7) % 9:
                self.flip = -self.flip
                self.flip_t = tick
            weak = hp_frac < 0.35 or (tgt.get("hp", 1) > me["hp"] * 1.8 and dist < 60)
            if weak:
                away = math.atan2(cy - y, cx - x) if dc > 20 else to + math.pi
                mv = away
                boost = me["energy"] > 25
                speed_vec = (mv, 1.0)
            else:
                want = 32.0
                if wk[1] == "lightning":
                    want = 18.0
                elif wk[1] == "laser" or wk[1] == "mortar":
                    want = 45.0
                radial = 0.5 if dist > want + 8 else (-0.5 if dist < want - 8 else 0.0)
                mv = to + (math.pi / 2) * self.flip * (1.0 if radial == 0 else 0.8) - (0 if radial == 0 else 0)
                vx, vy = math.cos(mv) + math.cos(to) * radial, math.sin(mv) + math.sin(to) * radial
                speed_vec = (math.atan2(vy, vx), 1.0)
            if tgt.get("charging"):
                aimed = abs(wrap(tgt["aim"][0] - (to + math.pi))) < 0.25
                if aimed:
                    speed_vec = (to + (math.pi / 2) * self.flip, 1.0)
                    act["jump"] = tick % 6 == 0
        elif self.threat and tick - self.threat[1] < 25 and not enemies:
            b = self.threat[0]
            gyaw = math.atan2(cy - y, cx - x)
            speed_vec = (gyaw if dc > 25 else b + math.pi / 2, 1.0)
        elif not hold:
            speed_vec = (math.atan2(gy - y, gx - x), 1.0)
        for p in obs["projectiles"]:
            rx, ry = x - p["pos"][0], y - p["pos"][1]
            sp = math.hypot(p["vel"][0], p["vel"][1]) + 1e-6
            along = (rx * p["vel"][0] + ry * p["vel"][1]) / sp
            if 0 < along < 60:
                cross = (rx * p["vel"][1] - ry * p["vel"][0]) / sp
                if abs(cross) < 6:
                    base = math.atan2(p["vel"][1], p["vel"][0])
                    speed_vec = (base + (math.pi / 2) * (1 if cross > 0 else -1) * -1, 1.0)
                    break

        if not me["in_zone"]:
            speed_vec = (math.atan2(zone["center"][1] - y, zone["center"][0] - x), 1.0)
            boost = me["energy"] > 10

        if speed_vec:
            yaw = self.steer(x, y, z, speed_vec[0])
            if self.last_pos and math.hypot(x - self.last_pos[0], y - self.last_pos[1]) < 0.4 and me["grounded"]:
                self.stuck += 1
            else:
                self.stuck = 0
            if self.stuck > 6:
                self.unstick_until = tick + 14
                self.unstick_dir = (1 if (tick // 14) % 2 else -1) * (1.2 + (tick % 5) * 0.3)
                self.stuck = 0
            if tick < self.unstick_until:
                yaw = speed_vec[0] + self.unstick_dir
                act["jump"] = True
                boost = boost or me["energy"] > 30
            act["move"] = [math.cos(yaw) * speed_vec[1], math.sin(yaw) * speed_vec[1]]
        else:
            act["move"] = [0.0, 0.0]
        self.last_pos = (x, y)
        act["boost"] = bool(boost)
        if "jump" not in act:
            act["jump"] = False

        fire = [False, False]
        aim = None
        if tgt and dist < 235:
            for i, w in enumerate(me["weapons"]):
                if not w:
                    continue
                k = w["kind"]
                st = w["stats"]
                if k == "gun":
                    ya, pi_, dd = self.lead(me, tgt, st["speed"])
                    if dd < st["range"]:
                        if aim is None:
                            aim = [ya, pi_]
                        err = abs(wrap(ya - me["aim"][0]))
                        fire[i] = err < 0.06 and w.get("ammo", 1) > 0 and w.get("reload", 0) <= 0
                        if w.get("ammo", 1) == 0:
                            act["reload"] = True
                elif k == "laser":
                    ya, pi_, dd = self.lead(me, tgt, 1e6)
                    if dd < st["range"] and dd > 12:
                        aim = [ya, pi_]
                        err = abs(wrap(ya - me["aim"][0]))
                        ch = w.get("charge", 0)
                        if w["cooldown"] <= 0.05 or ch > 0:
                            if ch >= st["charge"] - 0.06:
                                fire[i] = False if err < 0.05 else True
                                if ch >= st["charge"] + 0.5:
                                    fire[i] = False
                            else:
                                fire[i] = True
                        if ch > 0 and err > 0.4 and ch < st["charge"] * 0.5:
                            fire[i] = False
                elif k == "mortar":
                    r = self.mortar_aim(me, tgt, st)
                    if r and 25 < r[2] < 220:
                        if aim is None or not any(me["weapons"][j] and me["weapons"][j]["kind"] == "gun" for j in range(2)) or dist > 70:
                            aim = [r[0], r[1]]
                        fire[i] = abs(wrap(r[0] - me["aim"][0])) < 0.08 and abs(r[1] - me["aim"][1]) < 0.08
                elif k == "lightning":
                    if dist < st["range"] and me["energy"] > 10:
                        ya = math.atan2(tgt["pos"][1] - y, tgt["pos"][0] - x)
                        if aim is None or dist < 30:
                            aim = [ya, math.atan2(tgt["pos"][2] + 4.5 - z - 7, dist)]
                        fire[i] = abs(wrap(ya - me["aim"][0])) < 0.35
            if aim is None:
                aim = [math.atan2(tgt["pos"][1] - y, tgt["pos"][0] - x), 0.0]
        elif npcs and hp_frac > 0.7 and me["in_zone"]:
            n = min(npcs, key=lambda m: math.dist(m["pos"], me["pos"]))
            d = math.dist(n["pos"], me["pos"])
            if d < 60 and me["weapons"][0]:
                ya, pi_, dd = self.lead(me, n, me["weapons"][0]["stats"]["speed"])
                aim = [ya, pi_]
                fire[0] = abs(wrap(ya - me["aim"][0])) < 0.08 and me["weapons"][0].get("ammo", 1) > 0
        if aim is None:
            if self.threat and tick - self.threat[1] < 20:
                aim = [self.threat[0], 0.0]
            elif act["move"] != [0.0, 0.0]:
                aim = [math.atan2(act["move"][1], act["move"][0]) + math.sin(tick * 0.15) * 0.9, 0.0]
            else:
                aim = [me["aim"][0] + 0.5, 0.0]
            g = me["weapons"][0]
            if g and g["kind"] == "gun" and g.get("ammo", 12) < g["stats"]["mag"] * 0.5:
                act["reload"] = True
        act["aim"] = aim
        act["fire"] = fire
        mf.draw.label("hp %d  foes %d" % (me["hp"], len(enemies)))
        return act


if __name__ == "__main__":
    mf.run(Bot)