from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Tuple

from interfaces.autonomy import MissionContext, NavigationPolicy
from utils.config import SimulationConfig
from utils.obstacle_avoidance import integrate_surface_step, resolve_penetrations
from utils.waypoint_nav import Waypoint
from utils.vec2 import Vec2

if TYPE_CHECKING:
    from entities.obstacle import Obstacle


@dataclass
class Vessel:
    """Autonomous platform: same waypoint + ray-avoidance kinematics as targets."""

    position: Vec2
    velocity: Vec2
    heading_rad: float
    max_speed: float
    sensor_radius: float
    waypoint: Waypoint = field(repr=False)
    navigation: NavigationPolicy = field(repr=False)
    _rng: random.Random = field(default_factory=random.Random, repr=False, compare=False)

    def step(
        self,
        dt: float,
        ctx: MissionContext,
        cfg: SimulationConfig,
        obstacles: Tuple["Obstacle", ...],
    ) -> None:
        self.waypoint.tick(cfg)
        pos, vel, hdg = integrate_surface_step(
            self.position,
            self.velocity,
            self.heading_rad,
            self.waypoint.pos,
            self.max_speed,
            cfg.vessel_collision_radius,
            obstacles,
            cfg,
            dt,
        )
        self.position, self.velocity, self.heading_rad = pos, vel, hdg
        if self.velocity.length() > 0.05:
            self.heading_rad = math.atan2(self.velocity.y, self.velocity.x)

    def wrap_or_clamp(self, w: float, h: float) -> None:
        m = 12.0
        x = min(max(self.position.x, m), w - m)
        y = min(max(self.position.y, m), h - m)
        self.position = Vec2(x, y)
