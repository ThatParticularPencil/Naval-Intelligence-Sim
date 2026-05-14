"""
Only change will be ddv and dh
acceleration and heading
three rays
- front ray slows down
- right and left rays push heading
"""

from __future__ import annotations

import math as m
from typing import TYPE_CHECKING, Tuple
from utils.geometry import ray_object_intersection_distance as roid
from utils import vec2
from utils.config import SimulationConfig
from utils.vec2 import Vec2

if TYPE_CHECKING:
    from entities.obstacle import Obstacle


def resolve_penetrations(
    position: Vec2,
    velocity: Vec2,
    agent_radius: float,
    obstacles: Tuple["Obstacle", ...],
    *,
    resolve_padding: float = 2.0,
    max_resolve_iters: int = 8,
) -> Tuple[Vec2, Vec2]:
    if not obstacles:
        return position, velocity
    p = position
    vel2 = velocity
    for _ in range(max_resolve_iters):
        moved = False
        for o in obstacles:
            r_sum = o.radius + agent_radius + resolve_padding * 0.5
            dvec = p - o.center
            dist = dvec.length()
            if dist >= r_sum or dist < 1e-12:
                continue
            n = dvec * (1.0 / max(dist, 1e-12))
            p = o.center + n * r_sum
            vn = vel2.dot(n)
            if vn < 0.0:
                vel2 = vel2 - n * vn
            moved = True
        if not moved:
            break
    return p, vel2

def avoid(
    position: Vec2,
    velocity: Vec2,
    agent_radius: float,
    obstacles: Tuple["Obstacle", ...], cfg: SimulationConfig,
) -> tuple[float, float]: # Returns (ddv, dh)
    
    if not obstacles or velocity.length() < 0.1:
        return 0.0, 0.0

    # 1. Setup Rays (using your config for length and spread)
    heading_vec:Vec2 = velocity.normalize()
    ray_dist:float = cfg.ray_length
    side_angle:float = m.pi / 4        # 45 degrees spread
    
    rays = {
        "front": heading_vec * ray_dist,
        "left":  heading_vec.rotate_rad(-side_angle) * ray_dist * 0.7,
        "right": heading_vec.rotate_rad(side_angle) * ray_dist * 0.7
    }

    # 2. Check Intersections
    # Store the closest distance for each ray (initialize to ray_dist)
    hits = {key: ray_dist for key in rays}
    
    for o in obstacles:
        for key, ray_vec in rays.items():
            dist:float = roid(position, ray_vec, o)
            if dist != m.inf:
                hits[key] = min(hits[key], dist)

    dv = 0.0
    dh = 0.0
    
    # Front ray: Proportional braking
    if hits["front"] < ray_dist:
        braking_force = 1.0 - (hits["front"] / ray_dist)
        dv = -cfg.max_accel * braking_force *.1
    else: dv = 0.0

    side_dist: float = 0.0
    direction: int = 0
        
    left_hit  = hits["left"]  < ray_dist
    right_hit = hits["right"] < ray_dist
    if left_hit or right_hit:
        if left_hit and not right_hit:
            side_dist = hits["left"]
            direction = 1
        elif right_hit and not left_hit:
            side_dist = hits["right"]
            direction = -1
        else:
            # both hit — steer toward whichever side is clearer
            if hits["front"] <= ray_dist:
                side_dist = hits["left"]-hits["right"] * 2
                direction = 1
            else:
                if hits["left"] > hits["right"]:
                    side_dist = hits["left"]
                    direction = 1
                else:
                    side_dist = hits["right"]
                    direction = -1
    # else:
    #     return dv, 0.0

    steer_weight = (1 - (side_dist / ray_dist)) #**2
    dh = steer_weight * cfg.max_turn * direction

    return dv, dh
