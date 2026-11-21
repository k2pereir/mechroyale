
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


def point_in_poly(x, y, ring):
    inside = False
    n = len(ring) // 2
    j = n - 1
    for i in range(n):
        xi, yi = ring[2 * i], ring[2 * i + 1]
        xj, yj = ring[2 * j], ring[2 * j + 1]
        if (yi > y) != (yj > y):
            cross = (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            if x < cross:
                inside = not inside
        j = i
    return inside


def segment_poly_hit(a, b, ring):
    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]
    n = len(ring) // 2
    best = None
    dx, dy = bx - ax, by - ay
    for i in range(n):
        cx, cy = ring[2 * i], ring[2 * i + 1]
        ex, ey = ring[2 * ((i + 1) % n)], ring[2 * ((i + 1) % n) + 1]
        sx, sy = ex - cx, ey - cy
        den = dx * sy - dy * sx
        if abs(den) < 1e-9:
            continue
        t = ((cx - ax) * sy - (cy - ay) * sx) / den
        u = ((cx - ax) * dy - (cy - ay) * dx) / den
        if 0.0 < t < 1.0 and 0.0 <= u <= 1.0:
            if best is None or t < best:
                best = t
    return best


def segment_blocks_prism(a, b, prism):
    ring = prism["ring"]
    if len(ring) < 6:
        return False

    ax, ay = a[0], a[1]
    bx, by = b[0], b[1]

    if point_in_poly(ax, ay, ring):
        for hole in prism["holes"]:
            if len(hole) >= 6 and point_in_poly(ax, ay, hole):
                break
        else:
            hit_z = a[2] + (b[2] - a[2]) * 0.0
            return prism["base"] <= hit_z <= prism["top"]

    t_hit = segment_poly_hit(a, b, ring)
    if t_hit is None:
        return False

    if any(
        len(hole) >= 6
        and segment_poly_hit(a, b, hole) is not None
        for hole in prism["holes"]
    ):
        return False

    hit_z = a[2] + (b[2] - a[2]) * t_hit
    return prism["base"] <= hit_z <= prism["top"]


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

    t = clamp(t, 0.0, 0.8)
    return [pos[i] + vel[i] * t for i in range(3)]


def ballistic_pitch(speed, horizontal, dz, gravity):
    if horizontal < 1.0:
        return 0.0

    v2 = speed * speed
    disc = v2 * v2 - gravity * (
        gravity * horizontal * horizontal + 2.0 * dz * v2
    )
    if disc < 0.0:
        return math.atan2(dz + horizontal, horizontal)

    return math.atan((v2 - math.sqrt(disc)) / (gravity * horizontal))


class Bot:
    name = "quiet-survivor"

    def __init__(self, hello: mf.Hello):
        self.config = hello["config"]
        self.prisms = []
        self.trees = []
        self.cover_cache_tick = -100
        self.cover_cache = None
        self.cover_cache_pos = None
        self.cover_cache_threat = None
        self.load_map(hello.get("map", {}).get("dir", "/arena"))

    def load_map(self, root):
        path = os.path.join(root, "v2", "world.json")
        try:
            with open(path, "r") as f:
                world = json.load(f)

            for p in world.get("prisms", []):
                ring = p.get("ring", [])
                if len(ring) >= 6:
                    count = len(ring) // 2
                    cx = sum(ring[0::2]) / count
                    cy = sum(ring[1::2]) / count
                    radius = max(
                        math.hypot(ring[i] - cx, ring[i + 1] - cy)
                        for i in range(0, len(ring), 2)
                    )
                    self.prisms.append({
                        "ring": ring,
                        "base": p.get("base", -9999),
                        "top": p.get("top", 9999),
                        "holes": p.get("holes", []),
                        "cx": cx,
                        "cy": cy,
                        "radius": radius
                    })

            tree_data = world.get("trees", {})
            fields = tree_data.get("fields", [])
            data = tree_data.get("data", [])
            indexes = {name: i for i, name in enumerate(fields)}
            stride = len(fields)

            if stride:
                for offset in range(0, len(data) - stride + 1, stride):
                    row = data[offset:offset + stride]

                    def get(key, default=0.0):
                        return row[indexes[key]] if key in indexes else default

                    self.trees.append({
                        "x": get("x"),
                        "y": get("y"),
                        "z": get("z"),
                        "trunk": get("trunkR", 1.0),
                        "crown": get("crownR", 0.0)
                    })

        except (OSError, ValueError, TypeError):
            self.prisms = []
            self.trees = []

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

    def has_los(self, origin, target):
        for prism in self.prisms:
            cx = prism["cx"]
            cy = prism["cy"]
            if dist(origin, [cx, cy]) > 120.0 + prism["radius"] and dist(target, [cx, cy]) > 120.0 + prism["radius"]:
                continue
            if segment_blocks_prism(origin, target, prism):
                return False
        return True

    def choose_upgrade(self, offer):
        choices = offer.get("choices", [])
        if not choices:
            return 0

        weights = {
            "armor": 15,
            "regen": 13,
            "health": 13,
            "max_hp": 12,
            "energy": 10,
            "speed": 10,
            "boost": 8,
            "sensor": 5,
            "gun_rate": 3,
            "damage": 2
        }

        scores = []

        for choice in choices:
            ident = choice.get("id", "").lower()
            text = (
                ident + " " +
                choice.get("name", "") + " " +
                choice.get("desc", "")
            ).lower()

            score = sum(
                value for key, value in weights.items()
                if key in text
            )

            if "damage taken" in text or "resist" in text:
                score += 8

            if "walk_speed" in text or "movement" in text:
                score += 5

            scores.append(score)

        return max(range(len(scores)), key=lambda i: scores[i])

    def cover_goal(self, pos, threat, zone_center, safe_radius):
        if (
            self.cover_cache is not None
            and self.cover_cache_pos is not None
            and self.cover_cache_threat is not None
            and dist(pos, self.cover_cache_pos) < 3.0
            and dist(threat, self.cover_cache_threat) < 3.0
        ):
            return self.cover_cache

        result = self._cover_goal(pos, threat, zone_center, safe_radius)
        self.cover_cache = result
        self.cover_cache_pos = pos[:]
        self.cover_cache_threat = threat[:]
        return result

    def _cover_goal(self, pos, threat, zone_center, safe_radius):
        tx, ty = threat[0], threat[1]
        away = unit([pos[0] - tx, pos[1] - ty])

        if away == [0.0, 0.0]:
            away = [1.0, 0.0]

        best = None
        best_score = float("inf")

        for prism in self.prisms:
            cx = prism["cx"]
            cy = prism["cy"]
            radius = prism["radius"]

            if dist(pos, [cx, cy]) > 120.0 + radius:
                continue

            ring = prism["ring"]
            direction = unit([cx - tx, cy - ty])
            if direction == [0.0, 0.0]:
                direction = away

            for offset in (-0.65, -0.3, 0.0, 0.3, 0.65):
                angle = math.atan2(direction[1], direction[0]) + offset

                candidate = [
                    cx + math.cos(angle) * (radius + 7.0),
                    cy + math.sin(angle) * (radius + 7.0)
                ]

                if dist(candidate, zone_center) > safe_radius:
                    continue

                if point_in_poly(candidate[0], candidate[1], ring):
                    continue

                if any(
                    len(h) >= 6 and point_in_poly(candidate[0], candidate[1], h)
                    for h in prism["holes"]
                ):
                    continue

                if segment_poly_hit(threat, candidate, ring) is None:
                    continue

                eye_height = pos[2] + 8.0
                if prism["top"] < eye_height - 1.0:
                    continue
                if prism["base"] > eye_height + 1.0:
                    continue

                pos_to_threat_dist = dist(pos, threat)
                candidate_to_threat_dist = dist(candidate, threat)
                blocked = any(
                    segment_poly_hit(threat, candidate, other["ring"]) is not None
                    for other in self.prisms
                )
                if not blocked:
                    continue

                score = dist(pos, candidate) + abs(offset) * 12.0
                if candidate_to_threat_dist < pos_to_threat_dist:
                    score += (
                        pos_to_threat_dist - candidate_to_threat_dist
                    ) * 4.0

                px, py = pos[0], pos[1]
                qx, qy = candidate[0], candidate[1]
                dx, dy = qx - px, qy - py
                length2 = dx * dx + dy * dy
                if length2 > 1e-9:
                    u = clamp(
                        ((tx - px) * dx + (ty - py) * dy) / length2,
                        0.0,
                        1.0
                    )
                    closest_dist = math.hypot(
                        px + u * dx - tx,
                        py + u * dy - ty
                    )
                    if closest_dist < 8.0:
                        score += (8.0 - closest_dist) * 12.0

                if score < best_score:
                    best_score = score
                    best = candidate

        for tree in self.trees:
            if dist(pos, [tree["x"], tree["y"]]) > 120.0:
                continue

            dx = tree["x"] - tx
            dy = tree["y"] - ty
            d = math.hypot(dx, dy)

            if d < 1e-6:
                continue

            ux, uy = dx / d, dy / d

            for side in (-0.45, 0.0, 0.45):
                angle = math.atan2(uy, ux) + side

                candidate = [
                    tree["x"] + math.cos(angle) * (tree["trunk"] + 5.0),
                    tree["y"] + math.sin(angle) * (tree["trunk"] + 5.0)
                ]

                if dist(candidate, zone_center) > safe_radius:
                    continue

                vx = candidate[0] - tx
                vy = candidate[1] - ty
                wx = tree["x"] - tx
                wy = tree["y"] - ty
                denom = vx * vx + vy * vy

                u = clamp(
                    (wx * vx + wy * vy) / max(denom, 1e-9),
                    0.0,
                    1.0
                )

                closest = [tx + u * vx, ty + u * vy]

                if dist(closest, [tree["x"], tree["y"]]) > tree["trunk"] + 1.0:
                    continue

                score = dist(pos, candidate) + abs(side) * 12.0

                if score < best_score:
                    best_score = score
                    best = candidate

        return best

    def act(self, obs: mf.Observation) -> mf.Action:
        me = obs["self"]
        pos = me["pos"]
        x, y, z = pos
        t = obs["t"]

        zone_center, zone_radius = self.zone_state(obs["zone"], t)
        safe_radius = max(6.0, zone_radius * 0.78)
        zone_dist = dist(pos, zone_center)

        enemies = [
            m for m in obs.get("mechs", [])
            if m.get("kind") == "mech" and m.get("id") != me["id"]
        ]
        enemies.sort(key=lambda m: dist(pos, m["pos"]))

        nearby = [
            m for m in enemies
            if dist(pos, m["pos"]) < 115.0
        ]

        sounds = [
            s for s in obs.get("sounds", [])
            if s.get("age", 99) < 1.0
            and s.get("dist") in ("near", "mid")
        ]

        events = obs.get("events", [])
        damage_events = [
            e for e in events
            if e.get("type") == "damaged"
            and e.get("bearing") is not None
        ]

        threat = None
        threat_distance = 999.0

        if nearby:
            threat = nearby[0]["pos"]
            threat_distance = dist(pos, threat)
        elif damage_events:
            bearing = damage_events[-1]["bearing"]
            threat = [
                x + math.cos(bearing) * 45.0,
                y + math.sin(bearing) * 45.0,
                z
            ]
            threat_distance = 45.0
        elif sounds:
            sound = min(sounds, key=lambda s: s.get("age", 0.0))
            bearing = sound["bearing"]
            threat_distance = 35.0 if sound["dist"] == "near" else 95.0
            threat = [
                x + math.cos(bearing) * threat_distance,
                y + math.sin(bearing) * threat_distance,
                z
            ]

        action = {
            "tick": obs["tick"],
            "move": [0.0, 0.0],
            "fire": [False, False],
            "boost": False,
            "jump": False
        }

        if obs.get("offer"):
            action["pick"] = self.choose_upgrade(obs["offer"])

        weapons = me.get("weapons", [None, None])
        primary = weapons[0] if weapons else None
        secondary = weapons[1] if len(weapons) > 1 else None

        if secondary is None:
            for pickup in obs.get("pickups", []):
                if pickup.get("kind") == "gun":
                    continue

                if dist(pos, pickup["pos"]) > self.config["loot"]["pickup_radius"]:
                    continue

                if abs(z - pickup["pos"][2]) > 4.0:
                    continue

                action["take"] = pickup["id"]
                break

        move_goal = None

        if zone_dist > safe_radius - 12.0:
            move_goal = zone_center

        elif threat is not None and (
            threat_distance < 115.0 or damage_events or sounds
        ):
            cover = self.cover_goal(
                pos,
                threat,
                zone_center,
                max(8.0, safe_radius - 5.0)
            )

            if cover is not None:
                move_goal = cover
            else:
                dx = x - threat[0]
                dy = y - threat[1]
                side = 1.0 if me["id"] % 2 == 0 else -1.0

                move_goal = [
                    x + dx * 1.5 - dy * 0.8 * side,
                    y + dy * 1.5 + dx * 0.8 * side
                ]

        elif len(nearby) >= 2:
            sx = sum(x - e["pos"][0] for e in nearby)
            sy = sum(y - e["pos"][1] for e in nearby)
            move_goal = [x + sx, y + sy]

        else:
            cubes = [
                c for c in obs.get("cubes", [])
                if dist(pos, c["pos"]) < 22.0
            ]

            if cubes and not nearby:
                cube = min(cubes, key=lambda c: dist(pos, c["pos"]))
                move_goal = cube["pos"][:2]
            else:
                angle = math.atan2(
                    y - zone_center[1],
                    x - zone_center[0]
                )

                direction = 1.0 if me["id"] % 2 == 0 else -1.0
                orbit_radius = min(
                    safe_radius * 0.58,
                    max(12.0, zone_radius * 0.42)
                )

                move_goal = [
                    zone_center[0] + math.cos(angle + direction * 0.55) * orbit_radius,
                    zone_center[1] + math.sin(angle + direction * 0.55) * orbit_radius
                ]

        movement = unit([
            move_goal[0] - x,
            move_goal[1] - y
        ])

        action["move"] = movement

        action["boost"] = (
            me["energy"] > 45.0
            and (
                zone_dist > safe_radius - 20.0
                or threat_distance < 75.0
                or dist(pos, move_goal) > 35.0
            )
        )

        action["jump"] = (
            bool(me.get("grounded"))
            and threat_distance < 35.0
            and obs["tick"] % 9 == 0
        )

        aim_yaw = me.get("aim", [0.0, 0.0])[0]
        aim_pitch = 0.0
        target = enemies[0] if enemies else None

        if target:
            tp = target["pos"]
            tv = target.get("vel", [0.0, 0.0, 0.0])
            distance = dist(pos, tp)
            aim_point = [tp[0], tp[1], tp[2] + 4.5]

            if primary and primary.get("kind") == "gun":
                aim_point = intercept(
                    aim_point,
                    tv,
                    [x, y, z + me.get("stats", {}).get("muzzle_height", 7.0)],
                    primary.get("stats", {}).get("speed", 252.0)
                )

            aim_yaw = math.atan2(
                aim_point[1] - y,
                aim_point[0] - x
            )

            aim_pitch = math.atan2(
                aim_point[2] - (z + me.get("stats", {}).get("muzzle_height", 7.0)),
                max(0.1, dist(pos, aim_point))
            )

            isolated = len(nearby) <= 1
            muzzle_height = me.get("stats", {}).get("muzzle_height", 7.0)
            gun_origin = [x, y, z + muzzle_height]
            gun_target = [tp[0], tp[1], tp[2] + 4.5]
            gun_has_los = self.has_los(gun_origin, gun_target)

            if primary and primary.get("kind") == "gun" and isolated and distance < 135.0:
                if primary.get("ammo", 0) <= 1 and primary.get("reload", 0.0) <= 0.0:
                    action["reload"] = True
                elif primary.get("reload", 0.0) <= 0.0 and gun_has_los:
                    action["fire"][0] = True

            if secondary and isolated:
                kind = secondary.get("kind")
                stats = secondary.get("stats", {})

                if kind == "lightning" and distance <= stats.get("range", 45.0):
                    action["fire"][1] = gun_has_los

                elif kind == "laser" and distance <= stats.get("range", 280.0):
                    action["fire"][1] = (
                        gun_has_los
                        and secondary.get("charge", 0.0) < 0.7
                    )

                elif kind == "mortar" and 18.0 < distance < stats.get("range", 235.0):
                    mortar_origin = z + me.get("stats", {}).get("muzzle_height", 7.0)
                    target_height = tp[2] + 4.5
                    mortar_pitch = ballistic_pitch(
                        stats.get("speed", 72.0),
                        distance,
                        target_height - mortar_origin,
                        stats.get("gravity", 22.0)
                    )
                    aim_pitch = mortar_pitch
                    action["fire"][1] = True
                    action["fire"][0] = False

        elif damage_events:
            aim_yaw = damage_events[-1]["bearing"]

        elif sounds:
            aim_yaw = sounds[-1]["bearing"]

        action["aim"] = [aim_yaw, aim_pitch]

        return action


mf.run(Bot)