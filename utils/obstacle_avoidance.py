"""
Kinematic obstacle avoidance for circular agents vs circular obstacles.

Uses a short lookahead repulsion plus iterative penetration resolution and
velocity reflection — no full path planner; suitable for smooth sandbox motion.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Tuple

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
    """Push ``position`` outside obstacle disks; reflect inward ``velocity`` across contact normals."""
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
                vel2 = vel2 - n * (2.0 * vn)
            moved = True
        if not moved:
            break
    return p, vel2


def integrate_with_obstacle_avoidance(
    position: Vec2,
    velocity: Vec2,
    agent_radius: float,
    obstacles: Tuple["Obstacle", ...],
    dt: float,
    *,
    lookahead: float,
    repulsion: float,
    resolve_padding: float = 2.0,
    max_resolve_iters: int = 8,
) -> Tuple[Vec2, Vec2]:
    """
    Advance (position, velocity) by dt while steering clear of obstacle disks.

    ``repulsion`` scales a corrective acceleration (world units / s² flavor)
    built from clearance inside ``lookahead`` beyond (obstacle.radius + agent_radius).
    """
    if not obstacles or dt <= 0.0:
        return position + velocity * dt, velocity

    rep = Vec2(0.0, 0.0)
    for o in obstacles:
        delta = position - o.center
        dist = delta.length()
        r_contact = o.radius + agent_radius
        if dist < 1e-9:
            delta = Vec2(resolve_padding * 4.0, 0.0)
            dist = delta.length()
        n = delta * (1.0 / dist)
        clearance = dist - r_contact
        if clearance < lookahead:
            t = max(0.0, min(1.0, (lookahead - clearance) / max(lookahead, 1e-6)))
            rep = rep + n * (repulsion * t * t)

    vel2 = velocity + rep * dt
    p = position + vel2 * dt

    return resolve_penetrations(p, vel2, agent_radius, obstacles, resolve_padding=resolve_padding, max_resolve_iters=max_resolve_iters)
