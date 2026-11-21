
import math
import mechfight as mf


def dist2(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def norm(v):
    d = math.hypot(v[0], v[1])
    return [v[0] / d, v[1] / d] if d > 1e-6 else [0.0, 0.0]


def angle_diff(a, b):
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def intercept(pos, vel, origin, speed):
    rx = pos[0] - origin[0]
    ry = pos[1] - origin[1]
    rz = pos[2] - origin[2]
    vx, vy, vz = vel

    a = vx * vx + vy * vy + vz * vz - speed * speed
    b = 2 * (rx * vx + ry * vy + rz * vz)
    c = rx * rx + ry * ry + rz * rz

    if abs(a) < 1e-6:
        t = -c / b if abs(b) > 1e-6 else 0
    else:
        disc = b * b - 4 * a * c
        if disc < 0:
            t = 0
        else:
            roots = [(-b - math.sqrt(disc)) / (2 * a),
                     (-b + math.sqrt(disc)) / (2 * a)]
            valid = [r for r in roots if r > 0]
            t = min(valid) if valid else 0

    t = max(0, min(t, 1.0))
    return [pos[0] + vx * t,
            pos[1] + vy * t,
            pos[2] + vz * t]


def ballistic_pitch(speed, horizontal, dz, gravity=22):
    if horizontal < 1:
        return 0.0

    v2 = speed * speed
    disc = v2 * v2 - gravity * (gravity * horizontal * horizontal + 2 * dz * v2)

    if disc < 0:
        return math.atan2(dz + horizontal, horizontal)

    tan_pitch = (v2 - math.sqrt(disc)) / (gravity * horizontal)
    return math.atan(tan_pitch)


class Bot:
    name = "survivor-ai"

    def __init__(self, hello: mf.Hello):
        self.config = hello["config"]
        self.reach = self.config["loot"]["pickup_radius"]
        self.weapon_value = {
            "laser": 4.0,
            "mortar": 3.2,
            "lightning": 2.8,
            "gun": 1.0,
        }

    def choose_upgrade(self, offer, me):
        choices = offer.get("choices", [])
        held = [w["kind"] for w in me["weapons"] if w]

        def score(c):
            text = (c.get("id", "") + " " +
                    c.get("name", "") + " " +
                    c.get("desc", "")).lower()

            value = 0

            if "armor" in text or "damage taken" in text:
                value += 15
            if "health" in text or "regen" in text:
                value += 12
            if "damage" in text:
                value += 10
            if "cooldown" in text or "fire rate" in text or "rate" in text:
                value += 9
            if "energy" in text:
                value += 7
            if "speed" in text:
                value += 6
            if "sensor" in text:
                value += 3

            for weapon in held:
                if weapon in text:
                    value += 12

            if "gun" in text and "gun" in held:
                value += 4

            return value

        return max(range(len(choices)), key=lambda i: score(choices[i])) if choices else 0

    def act(self, obs: mf.Observation) -> mf.Action:
        me = obs["self"]
        x, y, z = me["pos"]
        pos = me["pos"]
        tick = obs["tick"]
        t = obs["t"]

        action: mf.Action = {
            "tick": tick,
            "move": [0.0, 0.0],
            "fire": [False, False],
            "boost": False,
            "jump": False,
        }

        if obs.get("offer"):
            action["pick"] = self.choose_upgrade(obs["offer"], me)

        zone = obs["zone"]
        start = zone["shrink_start"]
        end = zone["shrink_end"]

        if t <= start:
            progress = 0.0
        elif t >= end:
            progress = 1.0
        else:
            progress = (t - start) / max(1e-6, end - start)

        center = zone["center"]
        next_center = zone["next_center"]

        zx = center[0] + (next_center[0] - center[0]) * progress
        zy = center[1] + (next_center[1] - center[1]) * progress

        radius = zone["radius"] + (
            zone["next_radius"] - zone["radius"]
        ) * progress

        zone_distance = math.hypot(x - zx, y - zy)

        safe_radius = max(5, radius * 0.72)

        zone_goal = [zx, zy]

        if zone_distance > safe_radius:
            move_goal = zone_goal
        else:
            move_goal = None

        enemies = [
            m for m in obs["mechs"]
            if m["kind"] == "mech" and m["id"] != me["id"]
        ]

        def enemy_score(enemy):
            d = dist2(enemy["pos"], pos)
            hp = enemy.get("hp", 1000)
            threat = 0

            if enemy.get("charging"):
                threat += 15

            if "mortar" in enemy.get("weapons", []):
                threat += 8

            if "laser" in enemy.get("weapons", []):
                threat += 7

            return d * 0.5 + hp * 0.012 - threat

        enemies.sort(key=enemy_score)
        target = enemies[0] if enemies else None

        weapons = me["weapons"]
        primary = weapons[0]
        secondary = weapons[1]

        if secondary is None:
            candidates = []

            for pickup in obs["pickups"]:
                if pickup["kind"] == "gun":
                    continue

                d = dist2(pickup["pos"], pos)
                dz = abs(pickup["pos"][2] - z)

                if d <= self.reach and dz <= 4:
                    value = self.weapon_value.get(pickup["kind"], 0)
                    candidates.append((value, d, pickup))

            if candidates:
                candidates.sort(key=lambda a: (-a[0], a[1]))
                action["take"] = candidates[0][2]["id"]

        objectives = []

        for cube in obs["cubes"]:
            d = dist2(cube["pos"], pos)
            if d < 45:
                objectives.append((d, cube["pos"][:2]))

        for drop in obs["drops"]:
            if drop["lands_in"] > 0:
                continue

            d = dist2(drop["pos"], pos)
            if d > 100:
                continue

            contested = any(
                m["id"] != me["id"]
                and dist2(m["pos"], drop["pos"]) < 10
                for m in enemies
            )

            if not contested:
                objectives.append((d * 0.7, drop["pos"][:2]))

        if move_goal is None:
            if target:
                tx, ty = target["pos"][:2]
                dx, dy = tx - x, ty - y
                d = math.hypot(dx, dy)

                if d < 75:
                    if d < 28:
                        move_goal = [x - dx, y - dy]
                    else:
                        side = 1 if me["id"] % 2 == 0 else -1
                        move_goal = [x - dy * side, y + dx * side]
                elif objectives:
                    move_goal = min(objectives, key=lambda a: a[0])[1]
                else:
                    side = 1 if me["id"] % 2 == 0 else -1
                    move_goal = [x - dy * side, y + dx * side]
            elif objectives:
                move_goal = min(objectives, key=lambda a: a[0])[1]
            else:
                move_goal = [zx + safe_radius * 0.25, zy]

        movement = norm([move_goal[0] - x, move_goal[1] - y])

        action["move"] = movement

        energy = me["energy"]
        action["boost"] = (
            energy > 35
            and (zone_distance > safe_radius or
                 (target is None and math.dist(pos[:2], move_goal) > 25))
        )

        action["jump"] = me["grounded"] and tick % 70 == 0

        if target:
            tp = target["pos"]
            tv = target.get("vel", [0, 0, 0])
            distance = dist2(tp, pos)

            aim_point = [tp[0], tp[1], tp[2] + 4.5]

            if primary and primary["kind"] == "gun":
                speed = primary["stats"]["speed"]
                origin = [x, y, z + me["stats"]["muzzle_height"]]
                aim_point = intercept(
                    [tp[0], tp[1], tp[2] + 4.5],
                    tv,
                    origin,
                    speed
                )

            yaw = math.atan2(aim_point[1] - y, aim_point[0] - x)
            horizontal = dist2(aim_point, pos)
            pitch = math.atan2(
                aim_point[2] - (z + me["stats"]["muzzle_height"]),
                max(0.1, horizontal)
            )

            action["aim"] = [yaw, pitch]

            if primary and primary["kind"] == "gun":
                if primary["ammo"] <= 1 and primary["reload"] <= 0:
                    action["reload"] = True
                else:
                    action["fire"][0] = primary["reload"] <= 0

            if secondary:
                kind = secondary["kind"]
                stats = secondary["stats"]

                if kind == "laser":
                    charge = secondary.get("charge", 0)
                    if distance < stats["range"]:
                        action["fire"][1] = charge < 0.70

                elif kind == "lightning":
                    if distance <= stats["range"]:
                        action["fire"][1] = True

                elif kind == "mortar":
                    if 18 < distance < stats["range"]:
                        mortar_origin = z + me["stats"]["muzzle_height"]
                        target_height = tp[2] + 4.5
                        mortar_pitch = ballistic_pitch(
                            stats["speed"],
                            distance,
                            target_height - mortar_origin
                        )

                        action["aim"] = [yaw, mortar_pitch]
                        action["fire"][1] = True
                        action["fire"][0] = False

            mf.draw.line(
                [x, y, z + me["stats"]["muzzle_height"]],
                [tp[0], tp[1], tp[2] + 4.5],
                "#ff5040"
            )

        else:
            action["aim"] = [math.atan2(movement[1], movement[0]), 0.0]

        mf.draw.point([zx, zy, z], "#40c0ff", 6)
        mf.draw.label(
            f"HP {me['hp']:.0f} | cubes {me['cubes']} | "
            f"enemies {len(enemies)}"
        )

        return action


mf.run(Bot)