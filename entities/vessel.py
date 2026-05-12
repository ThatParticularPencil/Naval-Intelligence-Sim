from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from interfaces.autonomy import MissionContext, NavigationPolicy
from utils.vec2 import Vec2


@dataclass
class Vessel:
    """
    Autonomous platform with kinematic state
    """

    position: Vec2
    velocity: Vec2
    heading_rad: float
    max_speed: float
    sensor_radius: float
    navigation: NavigationPolicy = field(repr=False)
    _rng: random.Random = field(default_factory=random.Random, repr=False, compare=False)

    def step(self, dt: float, ctx: MissionContext, station_noise: float) -> None:
        desired = self.navigation.desired_velocity(ctx)
        # Passive station-keeping: blend tiny noise so heading/speed panels are alive
        n = Vec2(self._rng.gauss(0.0, 1.0), self._rng.gauss(0.0, 1.0)) * station_noise
        cmd = desired + n
        speed = cmd.length()
        if speed > self.max_speed and speed > 1e-9:
            cmd = cmd * (self.max_speed / speed)
        self.velocity = cmd
        self.position = self.position + self.velocity * dt
        if self.velocity.length() > 0.05:
            self.heading_rad = math.atan2(self.velocity.y, self.velocity.x)

    def wrap_or_clamp(self, w: float, h: float) -> None:
        """Keep vessel inside map (soft clamp)."""
        m = 12.0
        x = min(max(self.position.x, m), w - m)
        y = min(max(self.position.y, m), h - m)
        self.position = Vec2(x, y)
