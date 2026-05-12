from __future__ import annotations

import math

from utils.vec2 import Vec2


def ray_circle_intersect(origin: Vec2, direction: Vec2, center: Vec2, radius: float) -> bool:
    """
    Return True if the ray from `origin` along `direction` hits the open disk
    (center, radius) strictly in front of the origin (t > 0).

    Used for line-of-sight occlusion: obstacles between boat and target block sensing.
    """
    d = direction.normalized()
    oc = origin - center
    b = oc.dot(d)
    c = oc.dot(oc) - radius * radius
    # No real roots -> miss
    disc = b * b - c
    if disc < 0.0:
        return False
    sqrt_disc = math.sqrt(max(0.0, disc))
    t0 = -b - sqrt_disc
    t1 = -b + sqrt_disc
    # We care about intersection in forward ray direction
    t_hit = None
    if t0 > 1e-6:
        t_hit = t0
    elif t1 > 1e-6:
        t_hit = t1
    return t_hit is not None


def segment_circle_intersect(a: Vec2, b: Vec2, center: Vec2, radius: float) -> bool:
    """True if segment AB intersects or ends inside circle (center, radius)."""
    ab = b - a
    t = ((center - a).dot(ab)) / max(ab.dot(ab), 1e-12)
    t = min(1.0, max(0.0, t))
    closest = a + ab * t
    return (closest - center).length() <= radius + 1e-6
