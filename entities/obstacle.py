from __future__ import annotations

from dataclasses import dataclass

from utils.vec2 import Vec2


@dataclass
class Obstacle:
    """Circular rocky obstacle for boundary effects and sensor occlusion."""

    center: Vec2
    radius: float
