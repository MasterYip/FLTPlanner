#!/usr/bin/env python3
"""
Generate terrain_ground.png for 7_prj201_nav demo — v3.

The robot walks along the +x direction (left → right).
The map is elongated in x to give room for navigation.

Layout:
  - 5 large pillar obstacles (>1m each side, height 1.2m) placed ASYMMETRICALLY
    so the robot must find a path that avoids them.
  - 1 long thin barrier wall (height 0.2m, width 0.2m, length >2m) running
    ACROSS the x-axis (spanning y) so it FACES the robot walking along x.

  The combination forces the RRT to plan a path that avoids the pillars
  while still crossing the barrier — a combined obstacle-avoidance +
  barrier-crossing task triggered by a single /move_base_simple/goal.
"""

import numpy as np
import cv2
import os

# === Parameters — tweak these to adjust the terrain ===
RES = 0.05                  # grid resolution [m]
W = 16.0                    # map width  [m] — long in x (robot direction)
H = 6.0                     # map height [m] — y direction
MIN_H = 0.0                 # min encoded height [m]
MAX_H = 1.5                 # max encoded height [m]
SHIFT_X = 3.0               # shift all features right so robot at (0,0) starts on clear ground

IMG_W = int(round(W / RES))   # image width  [pixels]  → world x
IMG_H = int(round(H / RES))   # image height [pixels]  → world y

origin_x = -W / 2.0
origin_y = -H / 2.0

GROUND_H = 0.0
BARRIER_H = 0.20
OBSTACLE_H = 1.20


def encode_height(height):
    norm = (height - MIN_H) / (MAX_H - MIN_H)
    return np.clip(norm * 65535, 0, 65535).astype(np.uint16)


def add_rect(arr, x1, x2, y1, y2, h):
    """Set height in rectangle [x1,x2]×[y1,y2] to max(h, current)."""
    ix1 = max(0, int((x1 - origin_x) / RES))
    ix2 = min(IMG_W, int((x2 - origin_x) / RES) + 1)
    iy1 = max(0, int((y1 - origin_y) / RES))
    iy2 = min(IMG_H, int((y2 - origin_y) / RES) + 1)
    arr[iy1:iy2, ix1:ix2] = np.maximum(arr[iy1:iy2, ix1:ix2], h)


def add_rounded_rect(arr, x1, x2, y1, y2, h, corner_r=0.2):
    """Rect with rounded corners by adding circles at corners."""
    add_rect(arr, x1, x2, y1, y2, h)
    r = corner_r
    for cx, cy in [(x1+r, y1+r), (x2-r, y1+r),
                   (x1+r, y2-r), (x2-r, y2-r)]:
        add_circle(arr, cx, cy, r, h)


def add_circle(arr, cx, cy, radius, h):
    """Set height in circle to max(h, current), smooth falloff at edge."""
    r_px = int(radius / RES)
    c_px = int((cx - origin_x) / RES)
    c_py = int((cy - origin_y) / RES)
    for dy in range(-r_px, r_px + 1):
        for dx in range(-r_px, r_px + 1):
            px = c_px + dx
            py = c_py + dy
            if 0 <= px < IMG_W and 0 <= py < IMG_H:
                dist = np.hypot(dx * RES, dy * RES)
                if dist <= radius:
                    falloff = 1.0
                    if dist > radius * 0.7:
                        falloff = 1.0 - (dist - radius*0.7) / (radius * 0.3)
                    arr[py, px] = max(arr[py, px], h * falloff)


# === Build heightmap (flat ground = 0) ===
height_map = np.zeros((IMG_H, IMG_W), dtype=np.float64)

S = SHIFT_X

# =====================================================================
# 1. BARRIER — spans ACROSS the x-axis so the robot walks into it
#    Length > 2m (along y), width 0.2m (along x), height 0.2m
#    Placed at x=SHIFT_X, from y=-1.4 to y=+1.4  → 2.8m long
#    Robot at (0,0) has ~3m of clear ground before the barrier.
# =====================================================================
add_rect(height_map, S - 0.1, S + 0.1, -1.4, 1.4, BARRIER_H)

# =====================================================================
# 2. LARGE OBSTACLES (>1m each dimension, height 1.2m)
#    Placed ASYMMETRICALLY to force a snaking path
# =====================================================================

# Obstacle A — far left, top (blocks upper-left path)
add_rounded_rect(height_map, S - 5.5, S - 4.1,  1.5,  2.9, OBSTACLE_H)

# Obstacle B — left side, bottom (blocks lower-left path, asymmetric relative to A)
add_rounded_rect(height_map, S - 4.0, S - 2.6, -2.9, -1.5, OBSTACLE_H, 0.15)

# Obstacle C — centre-right, mostly top (blocks upper path after barrier)
add_rounded_rect(height_map, S + 1.0, S + 2.4,  1.2,  2.8, OBSTACLE_H)

# Obstacle D — right side, bottom (blocks lower-right path)
add_rounded_rect(height_map, S + 3.0, S + 4.4, -2.8, -1.4, OBSTACLE_H, 0.15)

# Obstacle E — near goal, centre (blocks direct path, forces final detour)
add_rounded_rect(height_map, S + 4.5, S + 5.7, -0.6,  0.6, OBSTACLE_H, 0.10)

# =====================================================================
# Resulting navigation scenario (robot starts at world (0,0) walks +x):
#
#   1. Clear ground from start (0,0) to first obstacles (~x=-2.5)
#   2. Obstacle A blocks upper-left, Obstacle B blocks lower-left
#      → path funnels toward centre
#   3. Barrier at x=SHIFT_X across y=-1.4~1.4 → must step over 0.2m step
#   4. Obstacle C blocks upper-right path after barrier
#   5. Obstacle D blocks lower-right path after barrier
#      → path weaves between C and D
#   6. Obstacle E blocks the centre-right end → final detour to goal
#
#   The RRT naturally weaves through this maze using the traversability
#   layer: obstacles are non-traversable (height > 0.3m), the barrier
#   is traversable (0.2m < 0.3m).
# =====================================================================

# === Output as 3-channel 16-bit PNG ===
img_16bit = encode_height(height_map)
img_out = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint16)
for c in range(3):
    img_out[:, :, c] = img_16bit

out_path = os.path.join(os.path.dirname(__file__), "terrain_ground.png")
cv2.imwrite(out_path, img_out)

# === Verification ===
print(f"Generated: {out_path}")
print(f"  Image: {IMG_W} x {IMG_H} px  |  World: {W}m x {H}m  |  Res: {RES}m")
print(f"  Barrier: 0.2m high, 0.2m wide (x), 2.8m long (y) @ x={SHIFT_X}")
print(f"  Obstacles: 1.2m high, asymmetric placement")

h_img = height_map
bar_px = int(np.sum((h_img >= 0.15) & (h_img <= 0.30)))
obs_px = int(np.sum(h_img > 0.30))
print(f"  Barrier cells: {bar_px}  |  Obstacle cells: {obs_px}")
print("Done.")
