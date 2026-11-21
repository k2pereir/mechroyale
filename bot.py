
import json
import math
import os
import mechfight as mf


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def unit(v):
    d = math.hypot(v[0], v[1])
    return [v[0] / d, v[1] / d] if d > 1e-8 else [0.0, 0.0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_diff(a, b):
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


def intercept(pos, vel, origin, speed):
    r = [pos[i] - origin[i] for i in range(3)]
    a = sum(v * v for v in vel) - speed * speed
    b = 2.0 * sum(r[i] * vel[i] for i in range(3))
    c = sum(v * v for v in r)

    if abs(a) < 1e-8:
        t = -c / b if abs(b) > 1e-8 else 0.0
    else:
        disc = b * b - 4.0 * a * c
        roots = [
            (-b - math.sqrt(disc)) / (2.0 * a),
            (-b + math.sqrt(disc)) / (2.0 * a)
        ] if disc >= 0 else []
        valid = [v for v in roots if v > 0]
        t = min(valid) if valid else 0.0

    t = clamp(t, 0.0, 1.0)
    return [pos[i] + vel[i] * t for i in range(3)]


def ballistic_pitch(speed, horizontal, dz, gravity):
    if horizontal < 1.0:
        return 0.0

    v2 = speed * speed
    disc = v2 * v2 - gravity * (
        gravity * horizontal * horizontal + 2.0 * dz * v2
    )

    if disc < 0.0:
        return math.atan2(dz, horizontal)

    return math.atan((v2 - math.sqrt(disc)) / (gravity * horizontal))


class Bot:
    name = "mobile-survivor"

    def __init__(self, hello: mf.Hello):
        self.config = hello["config"]
        self.slot = hello.get("slot", 0)
        self.last_scan = 0.0

    def zone_state(self, zone, t):
        start = zone["shrink_start"]
        end = zone["shrink_end"]
        q = clamp((t - start) / max(1e-6, end - start), 0.0, 1.0)

        center = [
            zone["center"][i] +
            (zone["next_center"][i] - zone["center"][i]) * q
            for i in range(2)
        ]

        radius = zone["radius"] + (
            zone["next_radius"] - zone["radius"]
        ) * q

        return center, radius

    def choose_upgrade(self, offer, me):
        choices = offer.get("choices", [])
        if not choices:
            return 0

        scores = []

        for choice in choices:
            ident = choice.get("id", "").lower()
            text = (
                ident + " " +
                choice.get("name", "") + " " +
                choice.get("desc", "")
            ).lower()

            score = 0

            if "armor" in text or "damage taken" in text or "resist" in text:
                score += 18
            if "regen" in text:
                score += 17
            if "max_hp" in text or "health" in text:
                score += 15
            if "walk_speed" in text or "movement" in text or "speed" in text:
                score += 14
            if "boost" in text or "energy" in text:
                score += 12
            if "gun_rate" in text or "fire rate" in text:
                score += 11
            if "damage" in text:
                score += 10
            if "cooldown" in text:
                score += 9
            if "sensors" in text or "sensor" in text:
                score += 7

            scores.append(score)

        return max(range(len(scores)), key=lambda i: scores[i])

    def act(self, obs: mf.Observation) -> mf.Action:
        me = obs["self"]
        pos = me["pos"]
        x, y, z = pos
        t = obs["t"]
        tick = obs["tick"]

        zone_center, zone_radius = self.zone_state(obs["zone"], t)
        zone_dist = dist(pos, zone_center)
        safe_radius = max(5.0, zone_radius * 0.76)
        zone_margin = safe_radius - zone_dist

        enemies = [
            m for m in obs.get("mechs", [])
            if m.get("kind") == "mech" and m.get("id") != me["id"]
        ]
        enemies.sort(key=lambda m: dist(pos, m["pos"]))

        nearby = [
            m for m in enemies
            if dist(pos, m["pos"]) < 100.0
        ]

        close_enemies = [
            m for m in enemies
            if dist(pos, m["pos"]) < 55.0
        ]

        sounds = [
            s for s in obs.get("sounds", [])
            if s.get("age", 99) < 1.0
        ]
        sounds.sort(key=lambda s: s.get("age", 99))

        events = obs.get("events", [])
        damage_events = [
            e for e in events
            if e.get("type") == "damaged"
            and e.get("bearing") is not None
        ]

        weapons = me.get("weapons", [None, None])
        primary = weapons[0] if weapons else None
        secondary = weapons[1] if len(weapons) > 1 else None

        target = enemies[0] if enemies else None
        target_distance = dist(pos, target["pos"]) if target else 999.0

        threat_bearing = None
        if damage_events:
            threat_bearing = damage_events[-1]["bearing"]
        elif nearby:
            threat_bearing = math.atan2(
                nearby[0]["pos"][1] - y,
                nearby[0]["pos"][0] - x
            )
        elif sounds:
            threat_bearing = sounds[0]["bearing"]

        action = {
            "tick": tick,
            "move": [0.0, 0.0],
            "fire": [False, False],
            "boost": False,
            "jump": False
        }

        if obs.get("offer"):
            action["pick"] = self.choose_upgrade(obs["offer"], me)

        if secondary is None:
            pickups = [
                p for p in obs.get("pickups", [])
                if p.get("kind") in ("mortar", "laser", "lightning")
                and abs(z - p["pos"][2]) < 4.0
            ]

            if pickups:
                pickup = min(pickups, key=lambda p: dist(pos, p["pos"]))
                if dist(pos, pickup["pos"]) <= 4.0:
                    action["take"] = pickup["id"]

        move_goal = None
        retreat = False

        if zone_margin < 18.0:
            move_goal = zone_center
        elif len(close_enemies) >= 2:
            retreat = True
        elif me["hp"] < me["max_hp"] * 0.38 and nearby:
            retreat = True
        elif target and target_distance < 19.0:
            retreat = True

        if retreat and nearby:
            sx = 0.0
            sy = 0.0

            for enemy in nearby:
                dx = x - enemy["pos"][0]
                dy = y - enemy["pos"][1]
                d = max(8.0, math.hypot(dx, dy))
                weight = 1.0 / (d * d)
                sx += dx * weight
                sy += dy * weight

            escape = unit([sx, sy])
            if escape == [0.0, 0.0]:
                escape = [math.cos(me["id"]), math.sin(me["id"])]

            move_goal = [
                x + escape[0] * 65.0,
                y + escape[1] * 65.0
            ]

            if zone_dist > safe_radius * 0.65:
                inward = unit([
                    zone_center[0] - x,
                    zone_center[1] - y
                ])
                move_goal = [
                    x + escape[0] * 35.0 + inward[0] * 45.0,
                    y + escape[1] * 35.0 + inward[1] * 45.0
                ]

        if move_goal is None and target and target_distance < 95.0:
            dx = x - target["pos"][0]
            dy = y - target["pos"][1]
            away = unit([dx, dy])

            if away == [0.0, 0.0]:
                away = [1.0, 0.0]

            side = 1.0 if me["id"] % 2 == 0 else -1.0
            tangent = [-away[1] * side, away[0] * side]

            if len(nearby) == 1 and target_distance > 32.0:
                move_goal = [
                    x + tangent[0] * 45.0 - away[0] * 8.0,
                    y + tangent[1] * 45.0 - away[1] * 8.0
                ]
            else:
                move_goal = [
                    x + away[0] * 38.0 + tangent[0] * 22.0,
                    y + away[1] * 38.0 + tangent[1] * 22.0
                ]

        if move_goal is None:
            cubes = [
                c for c in obs.get("cubes", [])
                if dist(pos, c["pos"]) < 45.0
                and dist(c["pos"], zone_center) < safe_radius - 8.0
            ]

            if cubes and not close_enemies:
                cube = min(cubes, key=lambda c: dist(pos, c["pos"]))
                move_goal = cube["pos"][:2]

        if move_goal is None:
            angle = math.atan2(y - zone_center[1], x - zone_center[0])
            if zone_dist < 1.0:
                angle = (me["id"] % 16) * math.pi / 8.0

            direction = 1.0 if me["id"] % 2 == 0 else -1.0
            orbit_radius = min(
                safe_radius * 0.55,
                max(10.0, zone_radius * 0.38)
            )

            move_goal = [
                zone_center[0] + math.cos(angle + direction * 0.45) * orbit_radius,
                zone_center[1] + math.sin(angle + direction * 0.45) * orbit_radius
            ]

        movement = unit([
            move_goal[0] - x,
            move_goal[1] - y
        ])
        action["move"] = movement

        energy = me.get("energy", 0.0)
        action["boost"] = (
            energy > 35.0
            and (
                zone_margin < 35.0
                or retreat
                or dist(pos, move_goal) > 45.0
            )
        )

        action["jump"] = (
            bool(me.get("grounded"))
            and (len(close_enemies) >= 2 or target_distance < 22.0)
            and tick % 12 == 0
        )

        aim_yaw = me.get("aim", [0.0, 0.0])[0]
        aim_pitch = 0.0

        if target:
            tp = target["pos"]
            tv = target.get("vel", [0.0, 0.0, 0.0])
            distance = target_distance
            muzzle_height = me.get("stats", {}).get("muzzle_height", 7.0)
            gun_origin = [x, y, z + muzzle_height]
            target_point = [tp[0], tp[1], tp[2] + 4.5]

            aim_point = target_point[:]
            if primary and primary.get("kind") == "gun":
                aim_point = intercept(
                    target_point,
                    tv,
                    gun_origin,
                    primary.get("stats", {}).get("speed", 252.0)
                )

            aim_yaw = math.atan2(
                aim_point[1] - y,
                aim_point[0] - x
            )

            aim_pitch = math.atan2(
                aim_point[2] - gun_origin[2],
                max(0.1, dist(pos, aim_point))
            )

            for slot, weapon in enumerate(weapons):
                if not weapon:
                    continue

                kind = weapon.get("kind")
                stats = weapon.get("stats", {})
                cooldown = weapon.get("cooldown", 0.0)
                reload_time = weapon.get("reload", 0.0)

                if kind == "gun":
                    if weapon.get("ammo", 0) <= 1 and reload_time <= 0.0:
                        if slot == 0:
                            action["reload"] = True
                    elif reload_time <= 0.0 and distance < stats.get("range", 230.0):
                        action["fire"][slot] = True

                elif kind == "laser":
                    laser_range = stats.get("range", 280.0)
                    charge = weapon.get("charge", 0.0)

                    if distance <= laser_range:
                        action["fire"][slot] = charge < 0.68

                elif kind == "lightning":
                    if (
                        distance <= stats.get("range", 45.0)
                        and cooldown <= 0.0
                        and energy > 5.0
                    ):
                        action["fire"][slot] = True

                elif kind == "mortar":
                    mortar_range = stats.get("range", 235.0)

                    if 18.0 < distance < mortar_range:
                        mortar_origin = z + me.get("stats", {}).get("eye_height", 8.0) + 1.5
                        target_height = tp[2] + 4.5
                        aim_pitch = ballistic_pitch(
                            stats.get("speed", 72.0),
                            distance,
                            target_height - mortar_origin,
                            stats.get("gravity", 22.0)
                        )
                        action["fire"][slot] = cooldown <= 0.0

        elif threat_bearing is not None:
            aim_yaw = threat_bearing
        else:
            aim_yaw = (tick * 0.035 + me["id"] * 0.4) % (2.0 * math.pi) - math.pi

        action["aim"] = [aim_yaw, aim_pitch]

        return action


mf.run(Bot)