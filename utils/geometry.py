from __future__ import annotations

import math

from utils.vec2 import Vec2


def ray_circle_intersection_distance(
    origin: Vec2,
    direction: Vec2,
    center: Vec2,
    radius: float,
) -> float:
    L = direction.length()
    if L < 1e-12:  
        return math.inf
    
    d = direction / L  # Normalized direction
    oc = origin - center
    b = oc.dot(d)
    c = oc.dot(oc) - radius**2
    disc = b**2 - c

    if disc < 0:
        return math.inf
    
    sqrt_disc = math.sqrt(disc)
    t0 = -b - sqrt_disc
    t1 = -b + sqrt_disc

    # t0 is the entry point, t1 is the exit point
    # prefer t0 if it's in front of us
    if t0 > 0:
        t_hit = t0
    elif t1 > 0:
        # This handles the case where the 'origin' is actually INSIDE the circle
        t_hit = t1
    else:
        return math.inf

    # if t_hit is greater than the original direction length, it's not a hit.
    if t_hit > L:
        return math.inf
        
    return t_hit
def ray_object_intersection_distance(origin: Vec2, direction: Vec2, obj: object) -> float:
    """
    Same as :func:`ray_circle_intersection_distance` for circle-like ``obj`` with
    ``center`` (:class:`~utils.vec2.Vec2`) and ``radius`` (numeric).
    """
    center = getattr(obj, "center", None)
    radius = getattr(obj, "radius", None)
    if center is None or radius is None:
        return math.inf
    return ray_circle_intersection_distance(origin, direction, center, float(radius))


def ray_circle_intersect(origin: Vec2, direction: Vec2, center: Vec2, radius: float) -> bool:
    """
    Return True if the ray from `origin` along `direction` hits the open disk
    (center, radius) strictly in front of the origin (t > 0).

    Used for line-of-sight occlusion: obstacles between boat and target block sensing.
    """
    return math.isfinite(ray_circle_intersection_distance(origin, direction, center, radius))


def segment_circle_intersect(a: Vec2, b: Vec2, center: Vec2, radius: float) -> bool:
    """True if segment AB intersects or ends inside circle (center, radius)."""
    ab = b - a
    t = ((center - a).dot(ab)) / max(ab.dot(ab), 1e-12)
    t = min(1.0, max(0.0, t))
    closest = a + ab * t
    return (closest - center).length() <= radius + 1e-6
