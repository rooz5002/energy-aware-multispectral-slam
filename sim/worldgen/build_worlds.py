#!/usr/bin/env python3
"""Generate the three simulation worlds and their fixed routes (guide section 4).

    python3 -m worldgen.build_worlds          (run from sim/)

Outputs: sim/textures/*.png, sim/worlds/<world>.sdf, sim/routes/<sequence>.yaml.
Everything is seeded and deterministic. Texture paths in the SDF are absolute,
so rerun this after moving the repository.
"""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from . import textures

SIM = Path(__file__).resolve().parents[1]
AMBIENT_K = 293.15
WALL_H = 2.6
WALL_T = 0.12
SEG = 2.0  # max wall segment length (one texture per segment)
FLOOR_TILE = 4.0
# gz-rendering passes every heat source (uniform temperature or heat signature) through an 8-bit buffer
# that spans 0 .. 65535 * resolution kelvin, so rendered temperatures are quantised to 257 * resolution.
# With the sensor resolution of 0.005 K/count that is a 1.285 K step and a 327.7 K ceiling. Scene
# temperatures are therefore chosen on that grid (so the SDF states what is rendered) and kept below 327 K.
THERMAL_RESOLUTION_K = 0.005
T_STEP = 257 * THERMAL_RESOLUTION_K
T_MAX = 327.0


def snap(kelvin):
    """Nearest temperature the thermal camera can represent."""
    return round(min(kelvin, T_MAX) / T_STEP) * T_STEP


# Thermal decals are thin exact-temperature patches on walls/floor/ceiling (studs, pipe traces behind the
# surface, patches, hot spots). They carry this visibility flag and the RGB and depth cameras mask it out,
# so they exist only in the thermal image - like sub-surface structure that is invisible in RGB.
DECAL_FLAG = 0x2
# Thermal scene richness (DECISIONS D2.1, fallback rule): chosen so that clear-condition thermal ORB keypoints
# are at most about half of RGB's on the tuning sequence. Calibrated on image statistics only
# (harness/image_stats.py), never on SLAM results. Frozen before tuning.
THERMAL_DENSITY = 0.45          # multiplier on the number of patches and spots per surface
STUD_SPACING_M = 1.2
PATCH_STEPS = (1, 3)           # temperature offset of patches, in quantisation steps
SPOT_STEPS = (2, 6)
DECAL_LAYERS = {"stud": 0.003, "trace": 0.005, "patch": 0.007, "spot": 0.009}  # stand-off from the surface (m)


class World:
    def __init__(self, name, seed, tex, crossover_zone=None, unique_walls=False):
        # crossover_zone(x, y) -> True where thermal contrast collapses (DECISIONS D2.4)
        self.crossover_zone = crossover_zone
        self.name, self.rng, self.tex = name, np.random.default_rng(seed), tex
        # thermal content has its own generator, so changing it never alters the RGB scene or the layout
        self.trng = np.random.default_rng([seed, 1])
        self.models, self.n = [], 0
        # DECISIONS v5 D22: every wall segment gets its own texture image. With the 12 shared wall textures the
        # same posters occur at several places and ORB-SLAM3 closes false loops between them.
        self.unique_walls, self.seed, self.n_walls = unique_walls, seed, 0

    # ------------------------------------------------------------ primitives
    def _pick(self, family):
        paths = self.tex[family]
        shared = paths[int(self.rng.integers(len(paths)))]  # always drawn, so the layout stream is unchanged
        if family == "rgb_wall" and self.unique_walls:
            path = paths[0].parent / f"rgb_wall_unique_s{self.seed}_{self.n_walls}.png"
            if not path.exists():
                cv2.imwrite(str(path), textures.rgb_wall(np.random.default_rng([self.seed, 2, self.n_walls])))
            self.n_walls += 1
            return path
        return shared

    def _material(self, rgb):
        if isinstance(rgb, str):
            return (f"<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0.05 0.05 0.05 1</specular>"
                    f"<pbr><metal><albedo_map>{self._pick(rgb)}</albedo_map><roughness>0.9</roughness>"
                    f"<metalness>0.0</metalness></metal></pbr></material>")
        r, g, b = rgb
        return (f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
                f"<specular>0.1 0.1 0.1 1</specular></material>")

    def _thermal(self, thermal, x, y):
        """thermal: kelvin | ("uniform", lo, hi), drawn once. Always a uniform temperature on the grid.

        Heat-signature textures are not used: in gz-sim 10 they ignore a non-default sensor resolution
        (objects render at the minimum temperature) and are quantised like everything else anyway.
        Inside the crossover zone every temperature is pulled to within one quantisation step of ambient.
        """
        kelvin = thermal if isinstance(thermal, (int, float)) else self.trng.uniform(thermal[1], thermal[2])
        if self.crossover_zone is not None and self.crossover_zone(x, y):
            ambient = snap(AMBIENT_K)
            kelvin = min(max(kelvin, ambient - T_STEP), ambient + T_STEP)
        body = f"<temperature>{snap(kelvin):.3f}</temperature>"
        return f'<plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal">{body}</plugin>'

    def add(self, kind, geom, pose, rgb, thermal, collide=True):
        """kind: label used in the model name; geom: SDF geometry body; pose: (x y z r p yaw)."""
        self.n += 1
        geometry = f"<geometry>{geom}</geometry>"
        collision = f'<collision name="c">{geometry}</collision>' if collide else ""
        self.models.append(
            f'    <model name="{kind}_{self.n}"><static>true</static>'
            f'<pose>{" ".join(f"{v:.4f}" for v in pose)}</pose><link name="link">{collision}'
            f'<visual name="v"><cast_shadows>false</cast_shadows>{geometry}{self._material(rgb)}'
            f"{self._thermal(thermal, pose[0], pose[1])}</visual></link></model>")

    def decals(self, origin, u, v, w_len, h_len, normal, base_k, density=1.0, hot_traces=True):
        """Thermal-only decals on a planar surface.

        origin: a corner of the surface; u, v: unit vectors spanning it; w_len, h_len: its size along u and v;
        normal: unit normal pointing at the viewer; base_k: temperature of the surface underneath.
        """
        rng, items = self.trng, []
        step = lambda lo, hi: T_STEP * int(rng.integers(lo, hi + 1))  # temperature offsets in whole grid steps
        for a in np.arange(rng.uniform(0.2, STUD_SPACING_M), w_len - 0.05, STUD_SPACING_M):   # studs / joists
            items.append(("stud", a, h_len / 2, 0.05, h_len, base_k + step(1, 1)))
        for _ in range(int(rng.integers(0, 2))):                                              # pipe / cable runs
            t = base_k + step(6, 14) if (hot_traces and rng.random() < 0.6) else base_k - step(4, 7)
            items.append(("trace", w_len / 2, rng.uniform(0.15, h_len - 0.15), w_len, rng.uniform(0.04, 0.09), t))
        for _ in range(int(round(16 * density * THERMAL_DENSITY))):                                             # panels, damp patches
            sw, sh = min(rng.uniform(0.12, 0.5), 0.6 * w_len), min(rng.uniform(0.12, 0.5), 0.6 * h_len)
            dt = step(*PATCH_STEPS) * (1 if rng.random() < 0.6 else -1)
            items.append(("patch", rng.uniform(sw / 2, w_len - sw / 2), rng.uniform(sh / 2, h_len - sh / 2), sw, sh,
                          base_k + dt))
        for _ in range(int(round(80 * density * THERMAL_DENSITY))):                                             # fixings, sockets
            sz = rng.uniform(0.02, 0.08)
            items.append(("spot", rng.uniform(sz, w_len - sz), rng.uniform(sz, h_len - sz), sz, sz,
                          base_k + step(*SPOT_STEPS)))
        o, u, v, nrm = (np.asarray(x, float) for x in (origin, u, v, normal))
        # rotation whose columns are (u, v, normal): the decal box is sized (su, sv, thickness) in that frame
        R = np.column_stack([u, v, nrm])
        yaw, pitch, roll = (math.atan2(R[1, 0], R[0, 0]), math.asin(-max(-1.0, min(1.0, R[2, 0]))),
                            math.atan2(R[2, 1], R[2, 2]))
        visuals = []
        for i, (layer, a, b, su, sv, temp) in enumerate(items):
            c = o + u * a + v * b + nrm * DECAL_LAYERS[layer]
            visuals.append(
                f'<visual name="d{i}"><pose>{c[0]:.4f} {c[1]:.4f} {c[2]:.4f} {roll:.5f} {pitch:.5f} {yaw:.5f}</pose>'
                f"<cast_shadows>false</cast_shadows><visibility_flags>{DECAL_FLAG}</visibility_flags>"
                f"<geometry><box><size>{su:.4f} {sv:.4f} 0.002</size></box></geometry>{self._thermal(float(temp), c[0], c[1])}</visual>")
        self.n += 1
        self.models.append(f'    <model name="decals_{self.n}"><static>true</static><link name="link">'
                           + "".join(visuals) + "</link></model>")

    def box(self, kind, x, y, z, sx, sy, sz, rgb, thermal, yaw=0.0, collide=True):
        self.add(kind, f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>", (x, y, z, 0, 0, yaw), rgb, thermal, collide)

    def hot_box(self, kind, x, y, z, sx, sy, sz, rgb, base_range, yaw=0.0, collide=True, density=1.5):
        """Box with its own thermal structure: uniform base temperature plus decals on the vertical faces."""
        base = snap(float(self.trng.uniform(*base_range)))
        self.box(kind, x, y, z, sx, sy, sz, rgb, base, yaw, collide)
        c, s_ = math.cos(yaw), math.sin(yaw)
        centre, up = np.array([x, y, z]), np.array([0.0, 0.0, 1.0])
        for nx, ny, half, width in ((1, 0, sx / 2, sy), (-1, 0, sx / 2, sy), (0, 1, sy / 2, sx), (0, -1, sy / 2, sx)):
            if width < 0.3:
                continue
            n = np.array([c * nx - s_ * ny, s_ * nx + c * ny, 0.0])
            u = np.array([-n[1], n[0], 0.0])
            origin = centre + n * half - u * width / 2 - up * sz / 2
            self.decals(origin, u, up, width, sz, n, base, density=density * width * sz / 5.2)

    def cylinder(self, kind, x, y, z, radius, length, rgb, thermal, roll=0.0, pitch=0.0, yaw=0.0, collide=True):
        self.add(kind, f"<cylinder><radius>{radius:.4f}</radius><length>{length:.4f}</length></cylinder>",
                 (x, y, z, roll, pitch, yaw), rgb, thermal, collide)

    def sphere(self, kind, x, y, z, radius, rgb, thermal):
        self.add(kind, f"<sphere><radius>{radius:.4f}</radius></sphere>", (x, y, z, 0, 0, 0), rgb, thermal, False)

    # ------------------------------------------------------------- structure
    def floor_and_ceiling(self, x0, y0, x1, y1, floor_k=(293.5, 295.5)):
        for cx in np.arange(x0 + FLOOR_TILE / 2, x1, FLOOR_TILE):
            for cy in np.arange(y0 + FLOOR_TILE / 2, y1, FLOOR_TILE):
                base = snap(float(self.trng.uniform(*floor_k)))
                self.box("floor", cx, cy, -0.05, FLOOR_TILE, FLOOR_TILE, 0.1, "rgb_floor", base)
                self.decals((cx - FLOOR_TILE / 2, cy - FLOOR_TILE / 2, 0.0), (1, 0, 0), (0, 1, 0), FLOOR_TILE,
                            FLOOR_TILE, (0, 0, 1), base, density=2.5)
                self.box("ceiling", cx, cy, WALL_H + 0.05, FLOOR_TILE, FLOOR_TILE, 0.1, "rgb_ceiling", base + T_STEP,
                         collide=False)
                self.decals((cx - FLOOR_TILE / 2, cy + FLOOR_TILE / 2, WALL_H), (1, 0, 0), (0, -1, 0), FLOOR_TILE,
                            FLOOR_TILE, (0, 0, -1), base + T_STEP, density=0.25)

    def wall(self, x0, y0, x1, y1, rgb="rgb_wall", wall_k=(292.0, 296.0), inside=None):
        """Axis-aligned wall from (x0,y0) to (x1,y1), split into textured segments.

        inside: unit normal (nx, ny) of the face that is seen; thermal decals go on that face (None = both).
        """
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(1, math.ceil(length / SEG))
        along_x = abs(x1 - x0) > abs(y1 - y0)
        seg = length / n
        for i in range(n):
            t = (i + 0.5) / n
            cx, cy = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            sx, sy = (seg, WALL_T) if along_x else (WALL_T, seg)
            base = snap(float(self.trng.uniform(*wall_k)))
            self.box("wall", cx, cy, WALL_H / 2, sx, sy, WALL_H, rgb, base)
            normals = [inside] if inside else ([(0, 1), (0, -1)] if along_x else [(1, 0), (-1, 0)])
            for nx, ny in normals:
                u = np.array([-ny, nx, 0.0])  # along the wall, right-handed with v = up and the normal
                origin = np.array([cx, cy, 0.0]) + np.array([nx, ny, 0.0]) * WALL_T / 2 - u * seg / 2
                self.decals(origin, u, (0, 0, 1), seg, WALL_H, (nx, ny, 0), base)

    # --------------------------------------------------------------- objects
    # Each takes the wall point (x, y) and the unit normal (nx, ny) pointing into the room.
    def radiator(self, x, y, nx, ny):
        yaw = math.atan2(ny, nx)
        self.hot_box("radiator", x + 0.08 * nx, y + 0.08 * ny, 0.45, 0.1, 1.0, 0.6, (0.92, 0.92, 0.9),
                     (314.0, 320.0), yaw, density=6.0)

    def person(self, x, y, nx, ny):
        px, py = x + 0.45 * nx, y + 0.45 * ny
        shirt = tuple(round(float(v), 2) for v in self.rng.uniform(0.1, 0.9, 3))
        self.cylinder("person_body", px, py, 0.75, 0.19, 1.5, shirt, ("uniform", 300.0, 308.0))
        self.sphere("person_head", px, py, 1.62, 0.12, (0.85, 0.65, 0.55), 308.5)

    def desk(self, x, y, nx, ny):
        yaw = math.atan2(ny, nx)
        px, py = x + 0.4 * nx, y + 0.4 * ny
        self.box("desk", px, py, 0.37, 0.7, 1.3, 0.74, "rgb_wood", ("uniform", 292.0, 297.0), yaw)
        self.box("computer", px, py, 0.92, 0.25, 0.5, 0.36, "rgb_machine", ("uniform", 300.0, 318.0), yaw,
                 collide=False)

    def cabinet(self, x, y, nx, ny):
        yaw = math.atan2(ny, nx)
        self.box("cabinet", x + 0.3 * nx, y + 0.3 * ny, 0.9, 0.5, 0.9, 1.8, "rgb_door", ("uniform", 291.0, 298.0), yaw)

    def window(self, x, y, nx, ny):
        yaw = math.atan2(ny, nx)
        self.box("window", x + 0.07 * nx, y + 0.07 * ny, 1.5, 0.03, 1.2, 1.0, (0.55, 0.7, 0.85), 284.0, yaw, collide=False)
        self.box("window_frame", x + 0.08 * nx, y + 0.08 * ny, 1.5, 0.03, 0.06, 1.0, (0.2, 0.2, 0.2), 288.0, yaw,
                 collide=False)

    def door(self, x, y, nx, ny):
        yaw = math.atan2(ny, nx)
        self.box("door", x + 0.07 * nx, y + 0.07 * ny, 1.02, 0.04, 0.9, 2.04, "rgb_door", ("uniform", 290.0, 296.0),
                 yaw, collide=False)

    def electrical_box(self, x, y, nx, ny):
        yaw = math.atan2(ny, nx)
        self.hot_box("elec_box", x + 0.12 * nx, y + 0.12 * ny, 1.2, 0.2, 0.5, 0.7, "rgb_machine",
                     (300.0, 312.0), yaw, collide=False, density=6.0)

    def lamp(self, x, y, nx, ny):
        self.sphere("lamp", x + 0.25 * nx, y + 0.25 * ny, 2.0, 0.09, (1.0, 0.95, 0.7), 326.0)
        self.cylinder("lamp_pole", x + 0.25 * nx, y + 0.25 * ny, 0.96, 0.02, 1.92, (0.2, 0.2, 0.2), AMBIENT_K + 2)

    def crate(self, x, y, nx, ny):
        s = float(self.rng.uniform(0.5, 0.9))
        self.box("crate", x + (s / 2 + 0.05) * nx, y + (s / 2 + 0.05) * ny, s / 2, s, s, s, "rgb_crate",
                 ("uniform", 290.0, 297.0), float(self.rng.uniform(-0.3, 0.3)))

    def barrel(self, x, y, nx, ny):
        self.cylinder("barrel", x + 0.4 * nx, y + 0.4 * ny, 0.45, 0.3, 0.9, "rgb_machine", ("uniform", 293.0, 312.0))

    def machine(self, x, y, sx, sy, sz, yaw=0.0):
        self.hot_box("machine", x, y, sz / 2, sx, sy, sz, "rgb_machine", (300.0, 312.0), yaw, density=3.0)

    def pipe(self, x0, y0, x1, y1, z, temp, radius=0.05):
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        colour = (0.7, 0.25, 0.2) if temp > AMBIENT_K else (0.2, 0.35, 0.7)
        self.cylinder("pipe", (x0 + x1) / 2, (y0 + y1) / 2, z, radius, length, colour, temp, pitch=math.pi / 2, yaw=yaw,
                      collide=False)

    def furnish(self, x0, y0, x1, y1, normal, kinds, spacing, margin=0.8):
        """Place objects along a wall line at regular slots, type drawn from `kinds` (seeded)."""
        length = math.hypot(x1 - x0, y1 - y0)
        for d in np.arange(margin, length - margin + 1e-6, spacing):
            t = d / length
            getattr(self, str(self.rng.choice(kinds)))(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, *normal)

    # ------------------------------------------------------------------ SDF
    def sdf(self, spawn):
        x, y, yaw = spawn
        models = "\n".join(self.models)
        return f"""<?xml version="1.0" ?>
<!-- GENERATED by sim/worldgen/build_worlds.py - do not edit by hand. -->
<sdf version="1.9">
  <world name="{self.name}">
    <physics name="2ms" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <scene><ambient>0.75 0.75 0.75 1</ambient><background>0.6 0.6 0.6 1</background><shadows>false</shadows><grid>false</grid></scene>
    <atmosphere type="adiabatic"><temperature>{AMBIENT_K}</temperature><temperature_gradient>0</temperature_gradient></atmosphere>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.7 0.7 0.7 1</diffuse><specular>0.1 0.1 0.1 1</specular><direction>0.35 0.2 -0.9</direction></light>
    <light type="directional" name="fill"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.35 0.35 0.35 1</diffuse><specular>0 0 0 1</specular><direction>-0.4 -0.3 -0.6</direction></light>
{models}
    <include><uri>model://tb3_waffle_ms</uri><name>tb3</name><pose>{x:.3f} {y:.3f} 0.01 0 0 {yaw:.4f}</pose></include>
  </world>
</sdf>
"""


# ------------------------------------------------------------------- worlds
def office_loop(tex, name="office_loop", crossover_zone=None):
    """Ring corridor (3 m wide) around a central core; rooms' content along the walls. Closed loop."""
    w = World(name, 11, tex, crossover_zone, unique_walls=True)
    w.floor_and_ceiling(-8, -6, 8, 6)
    ox, oy, ix, iy = 7.0, 5.0, 4.0, 2.0
    outer = [(-ox, -oy, ox, -oy, (0, 1)), (ox, -oy, ox, oy, (-1, 0)), (ox, oy, -ox, oy, (0, -1)), (-ox, oy, -ox, -oy, (1, 0))]
    inner = [(-ix, -iy, ix, -iy, (0, -1)), (ix, -iy, ix, iy, (1, 0)), (ix, iy, -ix, iy, (0, 1)), (-ix, iy, -ix, -iy, (-1, 0))]
    for x0, y0, x1, y1, n in outer:
        w.wall(x0, y0, x1, y1, inside=n)
        w.furnish(x0, y0, x1, y1, n, ["desk", "cabinet", "radiator", "person", "window", "lamp", "door"], 1.7)
    for x0, y0, x1, y1, n in inner:
        w.wall(x0, y0, x1, y1, inside=n)
        w.furnish(x0, y0, x1, y1, n, ["cabinet", "radiator", "door", "electrical_box", "person", "crate"], 2.1)
    w.pipe(-ox + 0.15, -oy + 0.15, ox - 0.15, -oy + 0.15, 2.3, 322.0)
    w.pipe(ox - 0.15, -oy + 0.15, ox - 0.15, oy - 0.15, 2.3, 322.0)
    w.pipe(-ox + 0.15, oy - 0.15, ox - 0.15, oy - 0.15, 2.3, 283.0)
    w.pipe(-ox + 0.15, -oy + 0.15, -ox + 0.15, oy - 0.15, 2.3, 283.0)
    rx, ry = 5.5, 3.5
    loop = dict(start=[-rx, -ry, 0.0], waypoints=[[rx, -ry], [rx, ry], [-rx, ry], [-rx, -ry]], final_yaw=0.0,
                role="evaluation")
    if crossover_zone is not None:
        return w, {"office_crossover_01": loop}
    routes = {
        "office_loop_01": loop,
        # tuning sequence: short, opposite direction, other start corner (guide 4.1)
        "office_loop_tune": dict(start=[rx, ry, -math.pi / 2], waypoints=[[rx, -ry], [-rx, -ry]], role="tuning"),
    }
    return w, routes


def office_crossover(tex):
    """office_loop with a thermal-crossover zone: for x > 1 m (the right-hand third of the loop: about 14 m of
    the 36 m route) every surface, decal and object is within one quantisation step of ambient, so thermal
    contrast collapses there while the RGB scene is unchanged (DECISIONS D2.4)."""
    return office_loop(tex, name="office_crossover", crossover_zone=lambda x, y: x > 1.0)


def corridor_tunnel(tex):
    """Long, low-texture corridor (RGB stress test): bare walls, sparse doors and signs, service pipes."""
    w = World("corridor_tunnel", 22, tex)
    L, half = 13.0, 1.2
    w.floor_and_ceiling(-16, -4, 16, 4)
    for sgn, n in ((-1, (0, 1)), (1, (0, -1))):
        w.wall(-L, sgn * half, L, sgn * half, rgb="rgb_plain", wall_k=(291.0, 295.0), inside=n)
        w.furnish(-L, sgn * half, L, sgn * half, n, ["door", "electrical_box", "door", "radiator"], 4.3, margin=1.5 + sgn * 0.6)
        w.pipe(-L, sgn * (half - 0.12), L, sgn * (half - 0.12), 2.2, 322.0 if sgn > 0 else 284.0)
        w.pipe(-L, sgn * (half - 0.1), L, sgn * (half - 0.1), 0.25, 310.0, radius=0.03)
    w.wall(-L, -half, -L, half, rgb="rgb_wall", inside=(1, 0))
    w.wall(L, -half, L, half, rgb="rgb_wall", inside=(-1, 0))
    w.person(4.0, -half, 0, 1)
    w.person(-6.5, half, 0, -1)
    routes = {"corridor_tunnel_01": dict(start=[-11.0, 0.0, 0.0], waypoints=[[11.0, 0.0], [-11.0, 0.0]],
                                         role="evaluation")}
    return w, routes


def industrial_hall(tex):
    """Larger hall: machine island in the middle, clutter and services along the walls."""
    w = World("industrial_hall", 33, tex)
    hx, hy = 10.0, 7.0
    w.floor_and_ceiling(-12, -8, 12, 8, floor_k=(291.0, 294.0))
    sides = [(-hx, -hy, hx, -hy, (0, 1)), (hx, -hy, hx, hy, (-1, 0)), (hx, hy, -hx, hy, (0, -1)), (-hx, hy, -hx, -hy, (1, 0))]
    for x0, y0, x1, y1, n in sides:
        w.wall(x0, y0, x1, y1, rgb="rgb_wall", wall_k=(289.0, 294.0), inside=n)
        w.furnish(x0, y0, x1, y1, n, ["crate", "barrel", "electrical_box", "cabinet", "person", "crate", "lamp"], 1.9)
    for i, x in enumerate(np.linspace(-4.8, 4.8, 5)):  # machine island
        w.machine(float(x), 0.0, 1.6, float(w.rng.uniform(1.6, 2.6)), float(w.rng.uniform(1.2, 2.2)),
                  yaw=float(w.rng.uniform(-0.15, 0.15)))
    for x, y in ((-8.6, -5.6), (8.6, -5.6), (8.6, 5.6), (-8.6, 5.6)):  # corner machines
        w.machine(x, y, 1.4, 1.4, 1.8, yaw=float(w.rng.uniform(0, 0.6)))
    for x in (-6.0, 0.0, 6.0):  # pillars
        for y in (-2.6, 2.6):
            w.box("pillar", x, y, WALL_H / 2, 0.35, 0.35, WALL_H, "rgb_plain", ("uniform", 290.0, 296.0))
    w.pipe(-hx, -hy + 0.2, hx, -hy + 0.2, 2.2, 326.0, radius=0.08)
    w.pipe(-hx, hy - 0.2, hx, hy - 0.2, 2.2, 282.0, radius=0.08)
    w.pipe(-hx, 0.0, hx, 0.0, 2.4, 324.0, radius=0.06)
    rx, ry = 7.2, 4.4
    routes = {"industrial_hall_01": dict(start=[-rx, -ry, 0.0], waypoints=[[rx, -ry], [rx, ry], [-rx, ry], [-rx, -ry]],
                                         final_yaw=0.0, role="evaluation")}
    return w, routes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sequences", nargs="+", default=None,
                    help="write only these sequences' worlds; nothing is deleted, and a world file that would "
                         "change is first moved to sim/worlds/archive/")
    args = ap.parse_args()
    tex = textures.generate(SIM / "textures", keep_existing=args.sequences is not None)
    (SIM / "worlds").mkdir(exist_ok=True)
    (SIM / "routes").mkdir(exist_ok=True)
    for builder in (office_loop, office_crossover, corridor_tunnel, industrial_hall):
        world, routes = builder(tex)
        # one world file per sequence, because the spawn pose is part of the world
        for seq, route in routes.items():
            path = SIM / "worlds" / f"{seq}.sdf"
            if args.sequences is not None:
                if seq not in args.sequences:
                    continue
                text = world.sdf(route["start"])
                if path.exists() and path.read_text() != text:
                    archive = SIM / "worlds" / "archive" / f"{seq}_shared_wall_textures.sdf"
                    archive.parent.mkdir(exist_ok=True)
                    if archive.exists():
                        raise SystemExit(f"{archive} exists: not overwriting")
                    path.rename(archive)
            path.write_text(world.sdf(route["start"]))
            route.update(world=world.name, linear_speed=0.22, angular_speed=0.6, sequence=seq)
            (SIM / "routes" / f"{seq}.yaml").write_text(yaml.safe_dump(route, sort_keys=False))
            wp = [route["start"][:2]] + route["waypoints"]
            dist = sum(math.dist(a, b) for a, b in zip(wp, wp[1:]))
            print(f"{seq}: {len(world.models)} models, route {dist:.1f} m, ~{dist / 0.22:.0f} s + turns -> {path.name}")


if __name__ == "__main__":
    main()
