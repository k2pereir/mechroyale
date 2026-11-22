import json, math, os, sys, time, traceback
from array import array




def _spans(ring, y):
   """x-intervals where horizontal line y is inside polygon ring (flat list)."""
   xs = []
   n = len(ring) // 2
   j = n - 1
   for i in range(n):
       xi, yi = ring[2 * i], ring[2 * i + 1]
       xj, yj = ring[2 * j], ring[2 * j + 1]
       if (yi > y) != (yj > y):
           xs.append(xi + (y - yi) * (xj - xi) / (yj - yi))
       j = i
   xs.sort()
   return [(xs[k], xs[k + 1]) for k in range(0, len(xs) - 1, 2)]




class World:
   """1 m grid: terrain height T, solid-column top H (ground + attached buildings), and
   overhang intervals OV (bridges etc.) keyed by cell."""


   def __init__(self, mapdir):
       self.ok = False
       try:
           self._load(mapdir)
           self.ok = True
       except Exception:
           sys.stderr.write("world load failed\n" + traceback.format_exc())


   def _load(self, mapdir):
       with open(os.path.join(mapdir, "v2", "world.json")) as f:
           w = json.load(f)
       ter = w["terrain"]
       self.minx = float(ter["minX"])
       self.miny = float(ter["minY"])
       self.nx = nx = int(ter["nx"])
       self.ny = ny = int(ter["ny"])
       self.step = float(ter.get("step", 1.0))
       scale = float(ter.get("scale", 0.01))
       raw = array("h")
       with open(os.path.join(mapdir, "v2", ter.get("file", "terrain.i16")), "rb") as f:
           raw.frombytes(f.read())
       if sys.byteorder != "little":
           raw.byteswap()
       T = array("f", [v * scale for v in raw])
       self.T = T
       b = w.get("bounds", {})
       self.bminx = b.get("minX", self.minx)
       self.bmaxx = b.get("maxX", self.minx + nx)
       self.bminy = b.get("minY", self.miny)
       self.bmaxy = b.get("maxY", self.miny + ny)
       # collect intervals per cell
       cells = {}
       for p in w.get("prisms", []):
           ring = p.get("ring") or []
           if len(ring) < 6:
               continue
           holes = [h for h in (p.get("holes") or []) if isinstance(h, list) and len(h) >= 6]
           base = float(p.get("base", 0.0))
           top = float(p.get("top", 0.0))
           ys = ring[1::2]
           j0 = max(0, int(math.floor((min(ys) - self.miny) / self.step)))
           j1 = min(ny - 1, int(math.ceil((max(ys) - self.miny) / self.step)))
           for j in range(j0, j1 + 1):
               cy = self.miny + j * self.step
               sp = _spans(ring, cy)
               if not sp:
                   continue
               hs = []
               for h in holes:
                   hs.extend(_spans(h, cy))
               for (a, bb) in sp:
                   i0 = max(0, int(math.ceil((a - self.minx) / self.step)))
                   i1 = min(nx - 1, int(math.floor((bb - self.minx) / self.step)))
                   for i in range(i0, i1 + 1):
                       if hs:
                           cx = self.minx + i * self.step
                           if any(ha <= cx <= hb for (ha, hb) in hs):
                               continue
                       k = j * nx + i
                       lst = cells.get(k)
                       if lst is None:
                           cells[k] = [(base, top)]
                       else:
                           lst.append((base, top))
       H = array("f", T)
       OV = {}
       for k, lst in cells.items():
           lst.sort()
           top = T[k]
           ov = []
           for (bs, tp) in lst:
               if bs <= top + 1.3:
                   if tp > top:
                       top = tp
               else:
                   ov.append((bs, tp))
           H[k] = top
           ov = [(bs, tp) for (bs, tp) in ov if tp > top]
           if ov:
               OV[k] = ov
       self.H = H
       self.OV = OV


   # ---------------------------------------------------------------- queries
   def idx(self, x, y):
       i = int(round((x - self.minx) / self.step))
       j = int(round((y - self.miny) / self.step))
       if i < 0:
           i = 0
       elif i >= self.nx:
           i = self.nx - 1
       if j < 0:
           j = 0
       elif j >= self.ny:
           j = self.ny - 1
       return j * self.nx + i


   def surface(self, x, y):
       return self.H[self.idx(x, y)]


   def terrain(self, x, y):
       return self.T[self.idx(x, y)]


   def inside(self, x, y, margin=0.0):
       return (self.bminx + margin < x < self.bmaxx - margin and
               self.bminy + margin < y < self.bmaxy - margin)


   def los(self, x0, y0, z0, x1, y1, z1, stepm=1.5):
       """True if the segment clears solid columns and overhangs (trees ignored)."""
       dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
       d = math.sqrt(dx * dx + dy * dy)
       n = int(d / stepm)
       if n <= 1:
           return True
       H = self.H
       OV = self.OV
       nx, ny = self.nx, self.ny
       minx, miny, st = self.minx, self.miny, self.step
       ix = dx / n
       iy = dy / n
       iz = dz / n
       x, y, z = x0, y0, z0
       for _ in range(1, n):
           x += ix
           y += iy
           z += iz
           i = int((x - minx) / st + 0.5)
           j = int((y - miny) / st + 0.5)
           if i < 0 or j < 0 or i >= nx or j >= ny:
               return False
           k = j * nx + i
           if z < H[k]:
               return False
           if OV:
               o = OV.get(k)
               if o:
                   for (b, t) in o:
                       if b <= z <= t:
                           return False
       return True




import heapq




class Nav:
   """Coarse walkability grid (cell = 3 m) + time-capped weighted A*."""


   def __init__(self, world, cs=3.0):
       self.w = world
       self.cs = cs
       self.ok = False
       if not world.ok:
           return
       self.minx = world.bminx
       self.miny = world.bminy
       self.gx = int((world.bmaxx - world.bminx) / cs) + 1
       self.gy = int((world.bmaxy - world.bminy) / cs) + 1
       gx, gy = self.gx, self.gy
       # cell height = max surface over the cell's footprint (5 samples) -> conservative
       Hc = array("f", [0.0]) * (gx * gy)
       half = cs * 0.5
       for j in range(gy):
           cy = self.miny + (j + 0.5) * cs
           for i in range(gx):
               cx = self.minx + (i + 0.5) * cs
               h = max(world.surface(cx, cy), world.surface(cx - half, cy - half), world.surface(cx + half, cy - half),
                       world.surface(cx - half, cy + half), world.surface(cx + half, cy + half))
               Hc[j * gx + i] = h
       self.Hc = Hc
       # ground-level cells: surface close to terrain (not a roof)
       G = bytearray(gx * gy)
       for j in range(gy):
           cy = self.miny + (j + 0.5) * cs
           for i in range(gx):
               cx = self.minx + (i + 0.5) * cs
               k = j * gx + i
               if Hc[k] - world.terrain(cx, cy) < 2.0 and 6 < cx - self.minx < (world.bmaxx - world.bminx) - 6 and 6 < cy - self.miny < (world.bmaxy - world.bminy) - 6:
                   G[k] = 1
       # remove cells adjacent to tall steps (mech radius 2.5)
       W = bytearray(G)
       for j in range(1, gy - 1):
           for i in range(1, gx - 1):
               k = j * gx + i
               if not G[k]:
                   continue
               h = Hc[k]
               for dk in (1, -1, gx, -gx):
                   if not G[k + dk] and Hc[k + dk] > h + 1.2:
                       W[k] = 0
                       break
       self.W = W
       self.ok = True


   def cell(self, x, y):
       i = int((x - self.minx) / self.cs)
       j = int((y - self.miny) / self.cs)
       i = min(max(i, 0), self.gx - 1)
       j = min(max(j, 0), self.gy - 1)
       return j * self.gx + i


   def center(self, k):
       j, i = divmod(k, self.gx)
       return (self.minx + (i + 0.5) * self.cs, self.miny + (j + 0.5) * self.cs)


   def walkable(self, x, y):
       return self.ok and self.W[self.cell(x, y)] == 1


   def nearest_walkable(self, k, maxr=6):
       if self.W[k]:
           return k
       gx = self.gx
       j0, i0 = divmod(k, gx)
       for r in range(1, maxr + 1):
           for dj in range(-r, r + 1):
               for di in (-r, r) if abs(dj) != r else range(-r, r + 1):
                   j, i = j0 + dj, i0 + di
                   if 0 <= j < self.gy and 0 <= i < gx and self.W[j * gx + i]:
                       return j * gx + i
       return None


   def path(self, sx, sy, tx, ty, budget_s=0.012, cost_fn=None):
       """List of (x, y) waypoints from start to target, or None."""
       if not self.ok:
           return None
       t0 = time.perf_counter()
       s = self.nearest_walkable(self.cell(sx, sy), 3)
       g = self.nearest_walkable(self.cell(tx, ty), 8)
       if s is None or g is None:
           return None
       if s == g:
           return [(tx, ty)]
       gx = self.gx
       W, Hc = self.W, self.Hc
       gj, gi = divmod(g, gx)
       D2 = 1.41421356
       nb = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, D2), (1, -1, D2), (-1, 1, D2), (-1, -1, D2))
       best = {s: 0.0}
       prev = {s: -1}
       sj, si = divmod(s, gx)
       h0 = math.hypot(si - gi, sj - gj)
       openh = [(h0 * 1.3, 0.0, s)]
       closest, cd = s, h0
       n = 0
       found = False
       while openh:
           f, gc, k = heapq.heappop(openh)
           if k == g:
               found = True
               break
           if gc > best.get(k, 1e18):
               continue
           n += 1
           if (n & 255) == 0 and time.perf_counter() - t0 > budget_s:
               break
           j, i = divmod(k, gx)
           hk = Hc[k]
           for (di, dj, c) in nb:
               ii, jj = i + di, j + dj
               if ii < 0 or jj < 0 or ii >= gx or jj >= self.gy:
                   continue
               kk = jj * gx + ii
               if not W[kk]:
                   continue
               if Hc[kk] - hk > 1.2:
                   continue
               if di and dj and (not W[j * gx + ii] or not W[jj * gx + i]):
                   continue
               ng = gc + c
               if cost_fn is not None:
                   ng += cost_fn(kk)
               if ng < best.get(kk, 1e18):
                   best[kk] = ng
                   prev[kk] = k
                   hh = math.hypot(ii - gi, jj - gj)
                   if hh < cd:
                       cd, closest = hh, kk
                   heapq.heappush(openh, (ng + hh * 1.3, ng, kk))
       end = g if found else closest
       if end == s:
           return None
       pts = []
       k = end
       while k != -1:
           pts.append(self.center(k))
           k = prev[k]
       pts.reverse()
       if found:
           pts[-1] = (tx, ty)
       return pts




# =====================================================================================
#  Brain
# =====================================================================================
import random


import mechfight as mf


PI = math.pi
TAU = 2.0 * math.pi
DEBUG = bool(os.environ.get("MRDEBUG"))




def hyp(a, b):
   return math.hypot(a[0] - b[0], a[1] - b[1])




def wrap(a):
   return (a + PI) % TAU - PI




def clamp(v, lo, hi):
   return lo if v < lo else (hi if v > hi else v)




def unit(dx, dy):
   d = math.hypot(dx, dy)
   if d < 1e-9:
       return (0.0, 0.0)
   return (dx / d, dy / d)




# Tunable parameters. Two profiles: practice (house squad) and ffa (real field).
BASE = dict(
   laser_pref=105.0,       # preferred range when holding a laser
   gun_pref=75.0,          # preferred range with gun only
   lightning_pref=28.0,    # preferred range when lightning is our best close weapon and they lack lightning
   engage_r=260.0,         # consider targets out to this range
   loot_r=170.0,           # walk to weapon pickups within this range when calm
   laser_seek_r=260.0,     # walk to a laser within this range when we have none
   cube_fight_r=90.0,      # grab cubes during fights within this range
   cube_calm_r=180.0,
   farm=True,              # shoot drones / crates when calm
   farm_hp=0.5,
   exposure_pen=3.0,       # penalty per extra enemy with LOS to candidate spot
   lose_pulse=1.5,         # retreat seconds after "losing"
   heal_to=0.85,           # when calm and hurt, hide/regen until this fraction
   heal_below=0.6,
   drops=True,
   drop_r=200.0,
   drop_hp=0.55,
   strafe=1.0,
   quiet=False,            # walk slowly (no step sounds) when travelling calm
)
BASE.update(duel=0.0, disengage_s=0.0, npc_avoid=0.0)
PRACTICE = dict(BASE)
if os.environ.get("MRP"):
   PRACTICE.update(json.loads(os.environ["MRP"]))
FFA = dict(BASE, heal_below=0.7, heal_to=0.9, exposure_pen=5.0, quiet=True, laser_pref=95.0,
          drop_hp=0.7, farm_hp=0.6)




class Track:
   __slots__ = ("id", "pos", "vel", "aim", "hp", "max_hp", "cubes", "weapons", "charging", "seen_t", "first_t",
                "kind", "dmg_to_me", "vis")


   def __init__(self, mid):
       self.id = mid
       self.first_t = -1.0
       self.dmg_to_me = 0.0
       self.vis = False




class Bot:
   name = "kp-hunter"


   def __init__(self, hello):
       t0 = time.time()
       self.cfg = hello.get("config", {})
       self.slot = hello.get("slot", 0)
       self.rng = random.Random((hello.get("seed", 0) or 0) ^ 0x5EED)
       mapdir = (hello.get("map") or {}).get("dir", "/arena")
       self.world = World(mapdir)
       self.nav = Nav(self.world) if self.world.ok else None
       if self.nav is not None and not self.nav.ok:
           self.nav = None
       sys.stderr.write("init %.2fs world=%s nav=%s\n" % (time.time() - t0, self.world.ok, self.nav is not None))
       wc = self.cfg.get("weapons", {})
       self.wc = wc
       mc = self.cfg.get("mech", {})
       self.mc = mc
       self.rear_mult = mc.get("rear_mult", 1.5)
       self.cubes_cfg = self.cfg.get("cubes", {})
       self.tracks = {}
       self.my_kills = 0
       self.mode = "unknown"
       self.unexplained_ticks = 0
       self.alive0 = None
       self.side = 1.0
       self.side_until = 0.0
       self.path = None
       self.path_goal = None
       self.path_t = -99.0
       self.path_i = 0
       self.prev_move = (0.0, 0.0)
       self.stuck_n = 0
       self.unstick_until = -1.0
       self.unstick_dir = (1.0, 0.0)
       self.last_hit_t = -99.0     # we took damage
       self.last_dealt_t = -99.0   # we dealt damage
       self.last_dmg_bearing = None
       self.last_dmg_by = None
       self.lose_until = -99.0
       self.target_id = None
       self.target_t = -99.0
       self.pickmem = {}           # id -> (kind, pos, t)
       self.crates = {}            # id -> (pos, hp, t)
       self.cubemem = {}           # id -> (pos, t)
       self.bad_pick = {}          # id -> t until ignored
       self.bad_goal = {}
       self.laser_hold_t = -99.0
       self.laser_lastpt = None
       self.heal_mode = False
       self.sweep_phase = self.rng.random() * TAU
       self.err_n = 0
       self.t = 0.0
       self.npc_aggro = {}
       self.hunter_dirs = []
       self.farm = None


   # ------------------------------------------------------------------ utils
   def P(self):
       return FFA if self.mode == "ffa" else PRACTICE


   def zone_at(self, zone, t):
       s, e = zone["shrink_start"], zone["shrink_end"]
       q = clamp((t - s) / max(1e-6, e - s), 0.0, 1.0)
       c = (zone["center"][0] + (zone["next_center"][0] - zone["center"][0]) * q,
            zone["center"][1] + (zone["next_center"][1] - zone["center"][1]) * q)
       r = zone["radius"] + (zone["next_radius"] - zone["radius"]) * q
       return c, r


   def los(self, a, az, b, bz):
       if not self.world.ok:
           return True
       return self.world.los(a[0], a[1], az, b[0], b[1], bz)


   def ground_z(self, x, y):
       return self.world.surface(x, y) if self.world.ok else 0.0


   # ------------------------------------------------------------------ main
   def act(self, obs):
       t_start = time.perf_counter()
       self.deadline = t_start + 0.030
       try:
           return self._act(obs)
       except Exception:
           self.err_n += 1
           if self.err_n <= 5:
               sys.stderr.write(traceback.format_exc())
           try:
               me = obs["self"]
               zc, zr = self.zone_at(obs["zone"], obs["t"] + 5)
               mv = unit(zc[0] - me["pos"][0], zc[1] - me["pos"][1])
               return {"tick": obs["tick"], "move": [mv[0], mv[1]], "fire": [False, False]}
           except Exception:
               return {"tick": obs.get("tick", 0)}


   def time_left(self):
       return self.deadline - time.perf_counter()


   def _act(self, obs):
       t = obs["t"]
       self.t = t
       tick = obs["tick"]
       me = obs["self"]
       pos = me["pos"]
       x, y, z = pos
       myid = me["id"]
       hp = me["hp"]
       maxhp = max(1.0, me["max_hp"])
       hpf = hp / maxhp
       energy = me.get("energy", 0.0)
       stats = me.get("stats", {})
       walk = stats.get("walk_speed", 11.7)
       cur_yaw, cur_pitch = me.get("aim", [0.0, 0.0])
       mycubes = me.get("cubes", 0)
       dmg_mult = 1.0 + self.cubes_cfg.get("damage_per_cube", 0.04) * min(mycubes, self.cubes_cfg.get("stat_cap", 10))
       muzzle_z = z + stats.get("muzzle_height", 7.0)
       eye_z = z + stats.get("eye_height", 8.0)
       zone = obs["zone"]
       alive = obs.get("alive", 16)
       if self.alive0 is None:
           self.alive0 = alive


       # ---------------- events
       for e in obs.get("events", []):
           ty = e.get("type")
           if ty == "damaged":
               if e.get("weapon") != "zone":
                   self.last_hit_t = t
                   self.last_dmg_bearing = e.get("bearing")
                   self.last_dmg_by = e.get("by")
                   by = e.get("by")
                   if by is not None and by in self.tracks:
                       self.tracks[by].dmg_to_me += e.get("amount", 0)
                   if by is not None and by >= 100:
                       self.npc_aggro[by] = t
           elif ty == "hit":
               self.last_dealt_t = t
           elif ty == "kill":
               self.my_kills += 1
               self.tracks.pop(e.get("target"), None)
               if self.target_id == e.get("target"):
                   self.target_id = None
           elif ty == "take_failed":
               self.bad_pick[e.get("id")] = t + 20.0
               self.pickmem.pop(e.get("id"), None)
           elif ty == "took":
               self.pickmem.pop(e.get("id"), None)
           elif ty == "dropped":
               self.bad_pick[e.get("id")] = t + 1e9
       # ---------------- mode detection (practice squad vs real FFA)
       unexplained = (self.alive0 - alive) - self.my_kills
       if self.mode != "ffa":
           if unexplained >= 1:
               self.unexplained_ticks += 1
               if self.unexplained_ticks >= 5 or unexplained >= 2:
                   self.mode = "ffa"
           else:
               self.unexplained_ticks = 0
               if t > 25 and self.mode == "unknown":
                   self.mode = "practice"
       P = self.P()


       # ---------------- perception
       seen_ids = set()
       npcs = []
       for m in obs.get("mechs", []):
           if m.get("kind") == "npc":
               npcs.append(m)
               continue
           mid = m["id"]
           if mid == myid:
               continue
           tr = self.tracks.get(mid)
           if tr is None:
               tr = Track(mid)
               tr.first_t = t
               self.tracks[mid] = tr
           tr.pos = m["pos"]
           tr.vel = m.get("vel") or [0.0, 0.0, 0.0]
           tr.aim = m.get("aim") or [0.0, 0.0]
           tr.hp = m.get("hp", 1400.0)
           tr.max_hp = m.get("max_hp", 1400.0)
           tr.cubes = m.get("cubes", 0)
           tr.weapons = m.get("weapons") or []
           tr.charging = bool(m.get("charging"))
           tr.seen_t = t
           seen_ids.add(mid)
       for mid in list(self.tracks.keys()):
           tr = self.tracks[mid]
           tr.vis = mid in seen_ids
           if t - tr.seen_t > 25.0:
               del self.tracks[mid]
       enemies = [tr for tr in self.tracks.values() if tr.vis]
       # dead reckoning for recently lost tracks
       ghosts = [tr for tr in self.tracks.values() if not tr.vis and t - tr.seen_t < 6.0]


       for p in obs.get("pickups", []):
           self.pickmem[p["id"]] = (p["kind"], p["pos"], t)
       for c in obs.get("crates", []):
           self.crates[c["id"]] = (c["pos"], c.get("hp", 300), t)
       for c in obs.get("cubes", []):
           self.cubemem[c["id"]] = (c["pos"], t)
       # forget items we should see but don't
       vis_p = set(p["id"] for p in obs.get("pickups", []))
       vis_c = set(c["id"] for c in obs.get("crates", []))
       vis_k = set(c["id"] for c in obs.get("cubes", []))
       for store, vis in ((self.pickmem, vis_p), (self.crates, vis_c), (self.cubemem, vis_k)):
           for k in list(store.keys()):
               if k in vis:
                   continue
               ent = store[k]
               ppos = ent[1] if store is self.pickmem else ent[0]
               if hyp(ppos, pos) < 30.0 and abs(ppos[2] - z) < 5:
                   del store[k]
               elif t - ent[-1] > 120.0:
                   del store[k]


       # ---------------- zone
       zc, zr = self.zone_at(zone, t)
       fc, fr = self.zone_at(zone, t + 12.0)
       d_zc = hyp(pos, zc)
       shrink_left = zone["shrink_end"] - t
       nc, nr = zone["next_center"], zone["next_radius"]
       # how urgently we must move toward the next circle
       need_next = 0.0
       d_nc = hyp(pos, nc)
       if t > zone["shrink_start"] - 25:
           need_next = max(0.0, d_nc - nr * 0.8)
       zone_urgent = d_zc > zr - 8.0 or (need_next > 0 and need_next / max(walk, 1) + 6 > shrink_left)
       dps = zone.get("dps", 20)


       # ---------------- weapons
       weapons = list(me.get("weapons") or [None, None])
       while len(weapons) < 2:
           weapons.append(None)
       W = {}
       for i, w in enumerate(weapons):
           if w:
               W[w["kind"]] = (i, w)
       gun = W.get("gun")
       laser = W.get("laser")
       lightning = W.get("lightning")
       mortar = W.get("mortar")
       laser_charge = laser[1].get("charge", 0.0) if laser else 0.0
       charging = laser_charge > 0.0


       action = {"tick": tick}
       if obs.get("offer"):
           action["pick"] = self.choose_upgrade(obs["offer"], W)


       # ---------------- strafe side flip
       if t >= self.side_until:
           self.side = -self.side
           self.side_until = t + self.rng.uniform(0.7, 2.2)


       # ---------------- target selection
       target = self.pick_target(enemies, ghosts, pos, z, muzzle_z, hp, dmg_mult, W, t)
       tdist = hyp(pos, target.pos) if target else 1e9
       t_vis = bool(target and target.vis)
       t_center = None
       t_los = False
       if target:
           th = 9.0 * (1 + 0.05 * (target.cubes or 0))
           t_center = (target.pos[0], target.pos[1], target.pos[2] + th * 0.5)
           if t_vis:
               t_los = self.los(pos, muzzle_z, t_center, t_center[2]) or self.los(pos, muzzle_z, t_center, target.pos[2] + th * 0.85)


       threats = [tr for tr in enemies if hyp(tr.pos, pos) < 230.0]
       n_close = sum(1 for tr in threats if hyp(tr.pos, pos) < 120.0)
       recently_hit = t - self.last_hit_t < 1.5


       # ---------------- losing / retreat logic
       losing = False
       if threats:
           weakest_ratio = None
           if target is not None and target.vis:
               weakest_ratio = hp / max(1.0, target.hp)
           if n_close >= 3 and hpf < 0.8:
               losing = True
           elif n_close >= 2 and hpf < 0.55:
               losing = True
           elif hpf < 0.33 and (weakest_ratio is None or weakest_ratio < 0.9):
               losing = True
           if self.mode == "ffa" and hpf < 0.45:
               losing = True
           # finishing override: target nearly dead and not too crowded
           if target is not None and target.vis and n_close <= 2:
               burst = self.burst_damage(W, dmg_mult, tdist)
               if target.hp < burst:
                   losing = False
       if P["duel"] > 0 and target is not None and target.vis and not losing:
           # time for us to kill target vs time for visible threats to kill us
           my_dps = self.est_dps(W, dmg_mult, tdist)
           their = 0.0
           for tr in threats:
               if tr.vis and t - tr.seen_t < 0.5:
                   their += self.est_dps({k: None for k in (tr.weapons or ["gun"])}, 1 + 0.04 * min(tr.cubes or 0, 10), hyp(tr.pos, pos), enemy=True)
           if their > 0 and my_dps > 0:
               ttk = target.hp / my_dps
               ttd = hp / their
               if ttd < ttk * P["duel"]:
                   burst = self.burst_damage(W, dmg_mult, tdist)
                   if target.hp > burst:
                       losing = True
                       if P["disengage_s"] > 0:
                           self.lose_until = max(self.lose_until, t + P["disengage_s"])
       if losing:
           self.lose_until = max(self.lose_until, t + P["lose_pulse"])
       retreating = t < self.lose_until


       # ---------------- decide movement
       move = (0.0, 0.0)
       boost = False
       state = "idle"
       speed_scale = 1.0
       if zone_urgent and not (target is not None and t_vis and tdist < 60 and d_zc < zr - 3):
           state = "zone"
           goal = self.zone_goal(pos, zone, zc, zr, nc, nr)
           move = self.follow(pos, z, goal, t)
           boost = d_zc > zr - 2.0 and energy > 15
       elif target is not None and (t_vis or t - target.seen_t < 3.0) and tdist < P["engage_r"]:
           state = "retreat" if retreating else "fight"
           move = self.fight_move(pos, z, eye_z, target, t_center, tdist, threats, W, retreating, fc, fr, t, me, obs)
           if retreating and energy > 30:
               boost = True
       else:
           # calm: heal, loot, farm, drops, position
           goal, state = self.calm_goal(obs, pos, z, hpf, W, fc, fr, zc, zr, t, ghosts, npcs, me)
           if goal is not None:
               if hyp(pos, goal) > 1.2:
                   move = self.follow(pos, z, goal, t)
                   if P["quiet"] and state in ("roam", "heal"):
                       speed_scale = 4.6 / max(walk, 1.0)
               else:
                   move = (0.0, 0.0)
           if state == "farm_shoot":
               move = (0.0, 0.0)


       # dodge projectiles (mortar shells and close bullets)
       dv = self.dodge(pos, obs.get("projectiles", []))
       if dv[0] or dv[1]:
           mx, my = move[0] + dv[0], move[1] + dv[1]
           l = math.hypot(mx, my)
           move = (mx / l, my / l) if l > 1 else (mx, my)
           speed_scale = 1.0


       # stuck detection / unstick
       move = self.unstick(move, me, t, charging)
       jump = False
       if t < self.unstick_until:
           move = self.unstick_dir
           jump = bool(me.get("grounded")) and self.stuck_n % 2 == 0
           boost = energy > 30


       if speed_scale < 1.0:
           move = (move[0] * speed_scale, move[1] * speed_scale)
       self.prev_move = move
       action["move"] = [round(move[0], 4), round(move[1], 4)]
       action["boost"] = bool(boost and not charging and energy > 10)
       if jump:
           action["jump"] = True


       # ---------------- aim & fire
       aim, fire, reload = self.aim_fire(obs, me, pos, z, muzzle_z, target, t_center, t_vis, t_los, tdist, W,
                                         dmg_mult, state, npcs, threats, ghosts, move, t, retreating)
       action["aim"] = [round(wrap(aim[0]), 4), round(aim[1], 4)]
       action["fire"] = fire
       if reload:
           action["reload"] = True


       # ---------------- pickups (take)
       if DEBUG:
           action["debug"] = {"label": "%s t=%s d=%d n=%d vis=%d los=%d f=%s w=%s aim=%.2f/%.2f cur=%.2f P=%s T=%s le=%d" % (state, target.id if target else None, min(tdist, 999), len(threats), t_vis, t_los, "".join("1" if f else "0" for f in fire), ",".join("%s:%.2f:%s" % (w["kind"], w.get("cooldown", 0), w.get("ammo", w.get("charge", ""))) for w in weapons if w), aim[0], aim[1], cur_yaw, tuple(int(v) for v in pos), tuple(int(v) for v in target.pos) if target else None, (self.los(pos, z+8, t_center, t_center[2]+4.5) if target else 0))}
       tk = self.want_take(obs, pos, z, W, weapons, target, tdist)
       if tk is not None:
           action["take"] = tk[0]
           if tk[1] is not None:
               action["drop"] = tk[1]
       return action


   # ------------------------------------------------------------------ targeting
   def pick_target(self, enemies, ghosts, pos, z, muzzle_z, hp, dmg_mult, W, t):
       best, bs = None, -1e18
       for tr in enemies + ghosts:
           d = hyp(pos, tr.pos)
           if d > 300:
               continue
           frac = tr.hp / max(1.0, tr.max_hp)
           s = -d * 0.35 - tr.hp * 0.02
           if tr.vis:
               s += 40.0
           else:
               s -= (t - tr.seen_t) * 8.0
           if tr.id == self.target_id and t - self.target_t < 4.0:
               s += 35.0
           if frac < 0.3:
               s += 30.0
           if tr.dmg_to_me > 0:
               s += 10.0
           if s > bs:
               bs, best = s, tr
       if best is not None:
           self.target_id = best.id
           self.target_t = t
       return best


   def est_dps(self, W, mult, d, enemy=False):
       dps = 0.0
       if "gun" in W:
           acc = clamp(1.1 - d / 150.0, 0.25, 0.85) if not enemy else clamp(0.9 - d / 200.0, 0.2, 0.7)
           dps += 45 * 12 / (12 * 0.24 + 2.4) * acc
       if "laser" in W:
           dps += 315 / 2.4 * (0.9 if d < 200 else 0.5)
       if "lightning" in W and d < 45:
           dps += 60 / 0.25 * 0.6
       if "mortar" in W:
           dps += 15
       return dps * mult


   def burst_damage(self, W, dmg_mult, d):
       dmg = 0.0
       if "laser" in W:
           lw = W["laser"][1]
           if lw.get("cooldown", 0) <= 0.3:
               dmg += lw["stats"].get("damage", 315) * dmg_mult
       if "gun" in W:
           gw = W["gun"][1]
           gs = gw["stats"]
           acc = 0.8 if d < 80 else 0.55
           shots = min(gw.get("ammo", 0), 1.5 / max(0.05, gs.get("cooldown", 0.24)))
           dmg += shots * gs.get("damage", 45) * dmg_mult * acc
       if "lightning" in W and d < 45:
           dmg += 4 * W["lightning"][1]["stats"].get("damage", 60) * dmg_mult
       return dmg


   # ------------------------------------------------------------------ movement helpers
   def zone_goal(self, pos, zone, zc, zr, nc, nr):
       # point inside next circle along the line from us to its center
       d = hyp(pos, nc)
       want = nr * 0.55
       if d <= want:
           return (nc[0], nc[1])
       return (nc[0] + (pos[0] - nc[0]) / d * want, nc[1] + (pos[1] - nc[1]) / d * want)


   def follow(self, pos, z, goal, t):
       """Path-follow to goal; returns unit move."""
       dx, dy = goal[0] - pos[0], goal[1] - pos[1]
       dd = math.hypot(dx, dy)
       if dd < 1.0:
           return (0.0, 0.0)
       nav = self.nav
       on_roof = self.world.ok and (z - self.world.terrain(pos[0], pos[1]) > 2.5)
       if nav is None or on_roof:
           return self.steer(pos, z, unit(dx, dy))
       # direct if short and clear
       if dd < 12.0 and self.clear_line(pos, z, goal):
           return unit(dx, dy)
       need = (self.path is None or self.path_goal is None or hyp(goal, self.path_goal) > 8.0 or
               t - self.path_t > 3.0 or self.path_i >= len(self.path))
       if need and self.time_left() > 0.012:
           p = nav.path(pos[0], pos[1], goal[0], goal[1], budget_s=min(0.012, self.time_left() - 0.006))
           self.path = p
           self.path_goal = (goal[0], goal[1])
           self.path_t = t
           self.path_i = 0
       p = self.path
       if not p:
           return self.steer(pos, z, unit(dx, dy))
       # advance along path; look ahead to furthest visible waypoint (up to 4)
       while self.path_i < len(p) - 1 and hyp(p[self.path_i], pos) < 3.0:
           self.path_i += 1
       j = self.path_i
       best = j
       for k in range(j + 1, min(len(p), j + 5)):
           if self.clear_line(pos, z, p[k]):
               best = k
           else:
               break
       tx, ty = p[best]
       return unit(tx - pos[0], ty - pos[1])


   def clear_line(self, pos, z, goal):
       if not self.world.ok:
           return True
       w = self.world
       dx, dy = goal[0] - pos[0], goal[1] - pos[1]
       d = math.hypot(dx, dy)
       n = int(d / 1.5) + 1
       lim = z + 1.2
       ux, uy = dx / max(d, 1e-9), dy / max(d, 1e-9)
       px, py = -uy * 2.0, ux * 2.0
       for i in range(1, n + 1):
           f = min(1.0, i * 1.5 / max(d, 1e-9))
           cx, cy = pos[0] + dx * f, pos[1] + dy * f
           if w.surface(cx, cy) > lim or w.surface(cx + px, cy + py) > lim or w.surface(cx - px, cy - py) > lim:
               return False
           lim = max(lim, w.surface(cx, cy) + 1.2) if w.surface(cx, cy) > lim - 1.2 else lim
       return True


   def steer(self, pos, z, d):
       if not self.world.ok or (d[0] == 0 and d[1] == 0):
           return d
       base = math.atan2(d[1], d[0])
       for off in (0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.7, -1.7, 2.3, -2.3, PI):
           a = base + off * self.side
           g = (pos[0] + math.cos(a) * 7.0, pos[1] + math.sin(a) * 7.0)
           if self.clear_line(pos, z, g):
               return (math.cos(a), math.sin(a))
       return d


   def unstick(self, move, me, t, charging):
       vel = me.get("vel", [0, 0, 0])
       sp = math.hypot(vel[0], vel[1])
       want = math.hypot(move[0], move[1])
       if want > 0.5 and sp < 1.5 and not charging and t > 1.0:
           self.stuck_n += 1
       else:
           self.stuck_n = max(0, self.stuck_n - 1)
       if self.stuck_n >= 4 and t >= self.unstick_until:
           a = math.atan2(move[1], move[0]) + self.rng.choice((-1, 1)) * self.rng.uniform(1.2, 2.2)
           self.unstick_dir = (math.cos(a), math.sin(a))
           self.unstick_until = t + 0.8
           self.stuck_n = 0
           self.path = None
       return move


   def dodge(self, pos, projectiles):
       dvx = dvy = 0.0
       x, y, z = pos
       g = self.wc.get("mortar", {}).get("gravity", 22.0)
       for p in projectiles:
           pp, vv = p.get("pos"), p.get("vel")
           if not pp or not vv:
               continue
           if p.get("kind") == "mortar":
               # step the arc to impact with our height level
               px, py, pz = pp
               vx, vy, vz = vv
               land = None
               for _ in range(80):
                   px += vx * 0.05
                   py += vy * 0.05
                   vz -= g * 0.05
                   pz += vz * 0.05
                   if vz < 0 and pz <= self.ground_z(px, py) + 0.3:
                       land = (px, py)
                       break
               if land is None:
                   continue
               d = math.hypot(x - land[0], y - land[1])
               if d < 10.0:
                   ax, ay = unit(x - land[0], y - land[1])
                   if ax == 0 and ay == 0:
                       ax, ay = 1.0, 0.0
                   dvx += ax * 2.0
                   dvy += ay * 2.0
               continue
           rx, ry = pp[0] - x, pp[1] - y
           vx, vy = vv[0], vv[1]
           v2 = vx * vx + vy * vy
           if v2 < 1.0:
               continue
           tc = -(rx * vx + ry * vy) / v2
           if tc < 0.05 or tc > 0.6:
               continue
           pz = pp[2] + vv[2] * tc
           if abs(pz - (z + 4.5)) > 6.0:
               continue
           cx, cy = rx + vx * tc, ry + vy * tc
           d = math.hypot(cx, cy)
           if d < 3.5:
               vl = math.sqrt(v2)
               nx, ny = -vy / vl, vx / vl
               if nx * (-cx) + ny * (-cy) < 0:
                   nx, ny = -nx, -ny
               dvx += nx * 0.6
               dvy += ny * 0.6
       return (dvx, dvy)


   # ------------------------------------------------------------------ fight movement
   def pref_range(self, W, target):
       P = self.P()
       tw = target.weapons or []
       if "laser" in W:
           r = P["laser_pref"]
           if "lightning" in tw:
               r = max(r, 70.0)
           return r
       if "lightning" in W and "lightning" not in tw and "mortar" not in tw:
           return P["lightning_pref"]
       r = P["gun_pref"]
       if "lightning" in tw:
           r = max(r, 70.0)
       return r


   def fight_move(self, pos, z, eye_z, target, tc, tdist, threats, W, retreating, fc, fr, t, me, obs):
       P = self.P()
       world = self.world
       nav = self.nav
       pref = self.pref_range(W, target)
       # do we want to be seen by the target right now?
       laser = W.get("laser")
       gun = W.get("gun")
       lightning = W.get("lightning")
       shot_ready = False
       if laser and (laser[1].get("cooldown", 0) < 0.5 or laser[1].get("charge", 0) > 0):
           shot_ready = True
       if gun and gun[1].get("reload", 0) <= 0.3 and gun[1].get("ammo", 0) > 0:
           shot_ready = True
       if lightning and tdist < 50 and me.get("energy", 0) > 10:
           shot_ready = True
       want_los = shot_ready and not retreating
       hpf = me["hp"] / max(1.0, me["max_hp"])
       others = [tr for tr in threats if tr.id != target.id]
       # candidate points
       cands = [(pos[0], pos[1])]
       r_list = (6.0, 11.0) if not retreating else (8.0, 14.0)
       nd = 16
       base = self.rng.random() * TAU / nd
       for r in r_list:
           for k in range(nd):
               a = base + k * TAU / nd
               cands.append((pos[0] + math.cos(a) * r, pos[1] + math.sin(a) * r))
       tx, ty = target.pos[0], target.pos[1]
       tz_eye = tc[2]
       to_t = unit(tx - pos[0], ty - pos[1])
       cubes = [c[0] for c in self.cubemem.values()
                if hyp(c[0], pos) < P["cube_fight_r"] and abs(c[0][2] - z) < 3.5]
       if me.get("cubes", 0) >= 10:
           cubes = [c for c in cubes if hyp(c, pos) < 15]
       best, bs = None, -1e18
       pm = self.prev_move
       for (cx, cy) in cands:
           if self.time_left() < 0.008:
               break
           moved = (cx != pos[0] or cy != pos[1])
           if moved:
               if not world.inside(cx, cy, 4.0):
                   continue
               gz = world.surface(cx, cy) if world.ok else z
               if gz > z + 1.2 or gz < z - 6.0:
                   continue
               if not self.clear_line(pos, z, (cx, cy)):
                   continue
           else:
               gz = z
           s = 0.0
           dt_ = math.hypot(cx - tx, cy - ty)
           s -= abs(dt_ - pref) * 0.06
           if dt_ < 20:
               s -= (20 - dt_) * 0.3
           ez = gz + (eye_z - z)
           seen = world.los(cx, cy, ez, tx, ty, tz_eye) if world.ok else True
           if want_los:
               s += 3.0 if seen else -2.5
           else:
               s += 3.0 if not seen else -2.0
           for o in others:
               if hyp(o.pos, (cx, cy)) < 200 and (not world.ok or world.los(cx, cy, ez, o.pos[0], o.pos[1], o.pos[2] + 6.0)):
                   s -= P["exposure_pen"]
           # zone
           dz = math.hypot(cx - fc[0], cy - fc[1]) - (fr - 10.0)
           if dz > 0:
               s -= dz * 0.25
           # cubes
           if cubes:
               cd0 = min(hyp(c, pos) for c in cubes)
               cd1 = min(math.hypot(c[0] - cx, c[1] - cy) for c in cubes)
               s += (cd0 - cd1) * 0.35
           if moved:
               mx, my = unit(cx - pos[0], cy - pos[1])
               # strafe: lateral wrt target
               lat = (-to_t[1] * mx + to_t[0] * my) * self.side
               s += lat * 1.2 * P["strafe"]
               s += (mx * pm[0] + my * pm[1]) * 0.6
               if retreating:
                   away = -(to_t[0] * mx + to_t[1] * my)
                   s += away * 1.5
           else:
               s -= 1.0  # standing still is an easy target
           if s > bs:
               bs, best = s, (cx, cy)
       if best is None or (best[0] == pos[0] and best[1] == pos[1]):
           return (0.0, 0.0)
       return unit(best[0] - pos[0], best[1] - pos[1])


   # ------------------------------------------------------------------ calm behaviour
   def calm_goal(self, obs, pos, z, hpf, W, fc, fr, zc, zr, t, ghosts, npcs, me):
       P = self.P()
       x, y = pos[0], pos[1]


       def in_safe(p, m=0.85):
           return math.hypot(p[0] - fc[0], p[1] - fc[1]) < fr * m


       def reachable_z(p):
           return abs(p[2] - z) < 3.0 or (self.world.ok and p[2] - self.world.terrain(p[0], p[1]) < 2.5)


       # 1) supply drop
       if P["drops"] and hpf >= P["drop_hp"] or (P["drops"] and hpf < 0.5):
           for dr in obs.get("drops", []):
               dp = dr["pos"]
               dd = hyp(pos, dp)
               if dd > P["drop_r"] or not in_safe(dp, 1.0):
                   continue
               holder = dr.get("holder")
               if holder is not None and holder != me["id"] and dr.get("progress", 0) > 0:
                   continue
               if dr.get("lands_in", 0) > dd / 11.0 + 10.0:
                   continue
               if any(hyp(g.pos, dp) < 50 for g in ghosts):
                   continue
               if dd < dr.get("radius", 8.0) - 2.5 and dr.get("lands_in", 0) <= 0:
                   return (x, y), "drop_hold"
               return (dp[0], dp[1]), "drop"
       # 2) heal when hurt (stay out of sight)
       if hpf < P["heal_below"] or (self.heal_mode and hpf < P["heal_to"]):
           self.heal_mode = True
       else:
           self.heal_mode = False
       # 3) weapons
       held = set(W.keys())
       want_kinds = self.wanted_kinds(W)
       best, bs = None, 1e18
       for pid, (kind, pp, ts) in self.pickmem.items():
           if self.bad_pick.get(pid, -1) > t or kind not in want_kinds:
               continue
           if not reachable_z(pp):
               continue
           if not in_safe(pp, 0.95):
               continue
           d = hyp(pos, pp)
           lim = P["laser_seek_r"] if kind == "laser" else P["loot_r"]
           if d > lim:
               continue
           c = d - want_kinds[kind]
           if c < bs:
               bs, best = c, pp
       if best is not None and not (self.heal_mode and hpf < 0.4):
           return (best[0], best[1]), "loot"
       # 4) cubes
       if me.get("cubes", 0) < 15:
           bc, bd = None, 1e18
           for cid, (cp, ts) in self.cubemem.items():
               d = hyp(pos, cp)
               if d < P["cube_calm_r"] and reachable_z(cp) and in_safe(cp, 0.95) and d < bd:
                   bc, bd = cp, d
           if bc is not None:
               return (bc[0], bc[1]), "cube"
       if self.heal_mode:
           return self.hide_spot(pos, z, ghosts, fc, fr, t), "heal"
       # 5) farm drones / crates
       if P["farm"] and hpf > P["farm_hp"]:
           ft = self.farm_target(obs, pos, z, fc, fr, npcs)
           if ft is not None:
               kind, fp, d = ft
               self.farm = ft
               if d > 40.0 if kind == "npc" else d > 30.0:
                   return (fp[0], fp[1]), "farm"
               return (x, y), "farm_shoot"
       self.farm = None
       # 6) position: drift toward a spot inside the next circle
       goal = self.roam_spot(pos, fc, fr, t)
       return goal, "roam"


   def wanted_kinds(self, W):
       """kinds we would pick up -> bonus (m of detour we accept)."""
       out = {}
       sec = None
       for k in W:
           if k != "gun":
               sec = k
       if sec != "laser":
           out = {"laser": 220.0}
       if "gun" not in W:
           out["gun"] = 0.0
       for k in list(out.keys()):
           if k in W:
               del out[k]
       return out


   def hide_spot(self, pos, z, ghosts, fc, fr, t):
       if not ghosts and self.last_dmg_bearing is None:
           return (pos[0], pos[1])
       refs = [g.pos for g in ghosts]
       if not refs and self.last_dmg_bearing is not None and t - self.last_hit_t < 8:
           b = self.last_dmg_bearing
           refs = [(pos[0] + math.cos(b) * 80, pos[1] + math.sin(b) * 80, pos[2])]
       if not refs:
           return (pos[0], pos[1])
       best, bs = (pos[0], pos[1]), -1e9
       for r in (0.0, 8.0, 16.0, 26.0):
           for k in range(12 if r > 0 else 1):
               if self.time_left() < 0.01:
                   break
               a = k * TAU / 12
               cx, cy = pos[0] + math.cos(a) * r, pos[1] + math.sin(a) * r
               if r > 0:
                   if not self.world.inside(cx, cy, 4) or abs(self.ground_z(cx, cy) - z) > 1.2:
                       continue
                   if self.nav is not None and not self.nav.walkable(cx, cy):
                       continue
               s = -r * 0.1
               for g in refs:
                   if self.world.los(cx, cy, z + 8, g[0], g[1], g[2] + 6):
                       s -= 5
                   s += min(hyp((cx, cy), g), 150) * 0.01
               if math.hypot(cx - fc[0], cy - fc[1]) > fr * 0.9:
                   s -= 8
               if s > bs:
                   bs, best = s, (cx, cy)
       return best


   def farm_target(self, obs, pos, z, fc, fr, npcs):
       best, bs = None, 1e18
       for n in npcs:
           d = hyp(pos, n["pos"])
           if d < 140 and abs(n["pos"][2] - z) < 6 and math.hypot(n["pos"][0] - fc[0], n["pos"][1] - fc[1]) < fr:
               if d < bs:
                   bs, best = d, ("npc", n["pos"], d)
       for cid, (cp, chp, ts) in self.crates.items():
           d = hyp(pos, cp)
           if d < 110 and abs(cp[2] - z) < 3 and math.hypot(cp[0] - fc[0], cp[1] - fc[1]) < fr * 0.9:
               if d + 20 < bs:
                   bs, best = d + 20, ("crate", cp, d)
       return best


   def roam_spot(self, pos, fc, fr, t):
       d = math.hypot(pos[0] - fc[0], pos[1] - fc[1])
       if d < fr * 0.5:
           return (pos[0], pos[1])
       k = (fr * 0.4) / max(d, 1e-6)
       return (fc[0] + (pos[0] - fc[0]) * k, fc[1] + (pos[1] - fc[1]) * k)


   # ------------------------------------------------------------------ aiming
   def lead_point(self, origin, c, vel, speed, extra=0.075):
       px, py, pz = c
       vx, vy = vel[0], vel[1]
       vz = vel[2] if len(vel) > 2 else 0.0
       tt = 0.0
       for _ in range(4):
           dx, dy, dz = px - origin[0], py - origin[1], pz - origin[2]
           tt = math.sqrt(dx * dx + dy * dy + dz * dz) / speed + extra
           px = c[0] + vx * tt
           py = c[1] + vy * tt
           pz = c[2] + (vz * tt if abs(vz) > 0.5 else 0.0)
       return (px, py, pz)


   def angles(self, origin, p):
       dx, dy = p[0] - origin[0], p[1] - origin[1]
       return (math.atan2(dy, dx), math.atan2(p[2] - origin[2], max(0.1, math.hypot(dx, dy))))


   def aim_fire(self, obs, me, pos, z, muzzle_z, target, tc, t_vis, t_los, tdist, W, dmg_mult, state, npcs,
                threats, ghosts, move, t, retreating):
       cur_yaw, cur_pitch = me.get("aim", [0.0, 0.0])
       fire = [False, False]
       reload = False
       origin = (pos[0], pos[1], muzzle_z)
       gun, laser, lightning, mortar = W.get("gun"), W.get("laser"), W.get("lightning"), W.get("mortar")
       aim = None
       mstats = me.get("stats", {})


       def err(a):
           return max(abs(wrap(cur_yaw - a[0])), abs(cur_pitch - a[1]) * 0.9)


       # ---- pick what to shoot at
       aim_obj = None
       obj_kind = None
       if target is not None and (t_vis or t - target.seen_t < 1.0):
           aim_obj = (tc, target.vel if t_vis else [0, 0, 0], 2.5 * (1 + 0.05 * (target.cubes or 0)))
           obj_kind = "mech"
       elif state in ("farm", "farm_shoot") and getattr(self, "farm", None):
           kind, fp, d = self.farm
           h = 1.75 if kind == "npc" else 1.2
           aim_obj = ((fp[0], fp[1], fp[2] + h), [0, 0, 0], 1.6 if kind == "npc" else 1.3)
           obj_kind = kind
       else:
           # npc that is shooting us
           for n in npcs:
               if t - self.npc_aggro.get(n["id"], -99) < 3.0 and hyp(n["pos"], pos) < 100:
                   aim_obj = ((n["pos"][0], n["pos"][1], n["pos"][2] + 1.75), n.get("vel", [0, 0, 0]), 1.6)
                   obj_kind = "npc"
                   break
       laser_charge = laser[1].get("charge", 0.0) if laser else 0.0
       if aim_obj is not None:
           c, vel, rad = aim_obj
           d = math.hypot(c[0] - pos[0], c[1] - pos[1])
           los_ok = t_los if obj_kind == "mech" else self.los(pos, muzzle_z, c, c[2])
           if obj_kind == "mech" and not t_vis:
               los_ok = False
           gun_aim = None
           if gun:
               gp = self.lead_point(origin, c, vel, gun[1]["stats"].get("speed", 252.0))
               gun_aim = self.angles(origin, gp)
           laser_aim = None
           if laser:
               lp = (c[0] + vel[0] * 0.06, c[1] + vel[1] * 0.06, c[2])
               laser_aim = self.angles(origin, lp)
           tol = clamp(math.atan2(rad * 0.75, max(d, 1.0)), 0.006, 0.15)
           # choose steering aim: laser if charging/ready else gun
           use = gun_aim
           if laser and (laser_charge > 0 or (laser[1].get("cooldown", 0) < 0.25 and los_ok)):
               use = laser_aim
           if use is None:
               use = laser_aim or self.angles(origin, c)
           aim = use
           # ---- gun
           if gun and gun_aim is not None:
               gw = gun[1]
               if gw.get("reload", 0) <= 0 and gw.get("ammo", 0) <= 0:
                   reload = True
               elif los_ok and d < gw["stats"].get("range", 230.0) * 0.97 and err(gun_aim) < tol * 1.3:
                   fire[gun[0]] = True
           # ---- laser
           if laser:
               li, lw = laser
               ls = lw["stats"]
               full = ls.get("charge", 0.75)
               lrange = ls.get("range", 280.0)
               if laser_charge > 0:
                   # holding
                   hold = True
                   if los_ok and err(laser_aim) < tol:
                       if laser_charge >= full - 0.051:
                           hold = False
                       elif obj_kind == "mech" and target is not None:
                           frac = ls.get("min_frac", 0.35) + (1 - ls.get("min_frac", 0.35)) * min(1.0, laser_charge / full)
                           dmg = ls.get("damage", 315) * frac * dmg_mult
                           if "mortar" in (target.weapons or []):
                               dmg *= 1 + ls.get("cookoff", 0.6)
                           if dmg * 0.92 >= target.hp:
                               hold = False
                   if not los_ok:
                       # keep the charge for a while waiting for LOS
                       if laser_charge >= full and t - self.laser_hold_t > 1.2:
                           hold = True
                   fire[li] = hold
                   if los_ok:
                       self.laser_hold_t = t
               else:
                   if lw.get("cooldown", 0) <= 0.0 and los_ok and d < lrange * 0.95 and not (retreating and d > 60):
                       if obj_kind == "mech" or (obj_kind == "npc" and d < 120):
                           fire[li] = True
                           self.laser_hold_t = t
           # ---- lightning
           if lightning:
               ni, nw = lightning
               ns = nw["stats"]
               if (d <= ns.get("range", 45.0) - 1.5 and nw.get("cooldown", 0) <= 0.05 and me.get("energy", 0) > ns.get("energy", 3) + 2
                       and abs(wrap(cur_yaw - math.atan2(c[1] - pos[1], c[0] - pos[0]))) < math.radians(ns.get("cone_deg", 30)) * 0.8
                       and (los_ok or obj_kind == "crate")):
                   fire[ni] = True
           # ---- mortar: lob when no LOS, or at mid range when nothing better
           if mortar and obj_kind == "mech":
               mi, mw = mortar
               ms = mw["stats"]
               if mw.get("cooldown", 0) <= 0.0 and 35 < d < 220 and (not los_ok or not (gun or laser)):
                   m_aim = self.mortar_aim(pos, z, mstats, c, vel, ms)
                   if m_aim is not None:
                       if not (laser and laser_charge > 0):
                           aim = m_aim
                           if err(m_aim) < 0.04:
                               fire[mi] = True
       else:
           # no target: look around (toward threats / sounds / move dir)
           aim = self.idle_aim(obs, pos, cur_yaw, move, ghosts, t)
           if laser and laser_charge > 0:
               fire[laser[0]] = laser_charge < laser[1]["stats"].get("charge", 0.75) + 1.0 and t - self.laser_hold_t < 1.5
           if gun:
               gw = gun[1]
               mag = gw["stats"].get("mag", 12)
               if gw.get("reload", 0) <= 0 and gw.get("ammo", mag) < mag * 0.6 and not threats and t - self.last_hit_t > 2:
                   reload = True
       if aim is None:
           aim = (cur_yaw, 0.0)
       return aim, fire, reload


   def mortar_aim(self, pos, z, mstats, c, vel, ms):
       spd = ms.get("speed", 72.0)
       g = ms.get("gravity", 22.0)
       oz = z + mstats.get("height", 9.0) + 0.5
       tt = 1.5
       best = None
       for _ in range(3):
           px = c[0] + vel[0] * tt
           py = c[1] + vel[1] * tt
           hz = math.hypot(px - pos[0], py - pos[1])
           dz = (c[2] - 4.0) - oz
           v2 = spd * spd
           disc = v2 * v2 - g * (g * hz * hz + 2 * dz * v2)
           if disc < 0:
               return None
           # high arc if wall in the way, else low
           pit = math.atan((v2 - math.sqrt(disc)) / (g * hz))
           if not self.los(pos, oz, (px, py), c[2]):
               pit = math.atan((v2 + math.sqrt(disc)) / (g * hz))
           tt = hz / max(1.0, spd * math.cos(pit))
           best = (math.atan2(py - pos[1], px - pos[0]), pit)
       if best and best[1] > self.mc.get("max_pitch", 1.3):
           return None
       return best


   def idle_aim(self, obs, pos, cur_yaw, move, ghosts, t):
       if ghosts:
           g = min(ghosts, key=lambda g: hyp(g.pos, pos))
           return (math.atan2(g.pos[1] - pos[1], g.pos[0] - pos[0]), 0.0)
       if self.last_dmg_bearing is not None and t - self.last_hit_t < 4.0:
           return (self.last_dmg_bearing, 0.0)
       snd = None
       for s in obs.get("sounds", []):
           if s.get("kind") in ("gun", "laser", "laser_charge", "lightning", "step", "boost", "jet", "mortar", "reload"):
               if snd is None or s.get("dist") == "near":
                   snd = s
       if snd is not None:
           return (snd["bearing"], 0.0)
       base = math.atan2(move[1], move[0]) if (move[0] or move[1]) else cur_yaw
       return (base + 1.1 * math.sin(t * 0.9 + self.sweep_phase), 0.0)


   # ------------------------------------------------------------------ loot
   def want_take(self, obs, pos, z, W, weapons, target, tdist):
       want = self.wanted_kinds(W)
       if not want:
           return None
       rad = self.cfg.get("loot", {}).get("pickup_radius", 4.0) - 0.3
       best = None
       for p in obs.get("pickups", []):
           if p["kind"] not in want:
               continue
           if self.bad_pick.get(p["id"], -1) > self.t:
               continue
           if hyp(p["pos"], pos) > rad or abs(p["pos"][2] - z) > 3.8:
               continue
           if best is None or want[p["kind"]] > want[best["kind"]]:
               best = p
       if best is None:
           return None
       # slot to drop
       if weapons[1] is None:
           return (best["id"], None)
       if weapons[0] is None:
           return (best["id"], None)
       if best["kind"] == "gun":
           return None
       # replace the non-gun secondary (never the gun), unless laser is charging
       for i, w in enumerate(weapons):
           if w and w["kind"] != "gun":
               if w["kind"] == "laser" and w.get("charge", 0) > 0:
                   return None
               return (best["id"], i)
       return None


   # ------------------------------------------------------------------ upgrades
   ORDER = ["plating", "armor", "laser_damage", "laser_charge", "nanites", "gun_damage", "gun_rate",
            "lightning_efficiency", "lightning_range", "gun_mag", "servos", "gyros", "gun_velocity", "reactor",
            "capacitor", "lightning_chain", "jets", "sensors", "mortar_reload", "mortar_velocity", "mortar_splash",
            "laser_pierce"]


   def choose_upgrade(self, offer, W):
       ch = offer.get("choices", [])
       if not ch:
           return 0
       order = self.ORDER
       if self.mode == "ffa":
           order = ["plating", "armor", "nanites", "laser_damage", "laser_charge"] + [o for o in self.ORDER if o not in ("plating", "armor", "nanites", "laser_damage", "laser_charge")]
       best, bi = 1e9, 0
       for i, c in enumerate(ch):
           cid = c.get("id", "")
           r = order.index(cid) if cid in order else 50
           if r < best:
               best, bi = r, i
       return bi




if __name__ == "__main__":
   mf.run(Bot)