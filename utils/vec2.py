from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Vec2:
    """Immutable 2D vector for simulation state"""

    x: float
    y: float

    def __add__(self, other: Vec2) -> Vec2:
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vec2) -> Vec2:
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, s: float) -> Vec2:
        return Vec2(self.x * s, self.y * s)

    def __rmul__(self, s: float) -> Vec2:
        return self.__mul__(s)

    def dot(self, other: Vec2) -> float:
        return self.x * other.x + self.y * other.y

    def length(self) -> float:
        return math.hypot(self.x, self.y)

    def normalized(self) -> Vec2:
        L = self.length()
        if L < 1e-9:
            return Vec2(0.0, 0.0)
        return Vec2(self.x / L, self.y / L)

    def rotate(self, radians: float) -> Vec2:
        c, s = math.cos(radians), math.sin(radians)
        return Vec2(c * self.x - s * self.y, s * self.x + c * self.y)
