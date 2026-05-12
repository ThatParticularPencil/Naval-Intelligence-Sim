from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Tuple

from interfaces.autonomy import MissionContext, NavigationPolicy
from utils.config import SimulationConfig
from utils.obstacle_avoidance import integrate_with_obstacle_avoidance
from utils.waypoint_nav import WaypointNav
from utils.vec2 import Vec2

if TYPE_CHECKING:
    from entities.obstacle import Obstacle


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
    waypoint_nav: WaypointNav = field(repr=False)
    navigation: NavigationPolicy = field(repr=False)
    _rng: random.Random = field(default_factory=random.Random, repr=False, compare=False)

    def step(
        self,
        dt: float,
        ctx: MissionContext,
        cfg: SimulationConfig,
        obstacles: Tuple["Obstacle", ...],
    ) -> None:
        desired = self.waypoint_nav.desired_velocity(
            self.position,
            ctx.sim_time_s,
            cfg,
            obstacles,
            self.max_speed,
            cfg.vessel_collision_radius,
        )
        # Small noise on top of waypoint seek (keeps heading readout from freezing)
        n = Vec2(self._rng.gauss(0.0, 1.0), self._rng.gauss(0.0, 1.0)) * cfg.vessel_station_keeping_noise
        cmd = desired + n
        speed = cmd.length()
        if speed > self.max_speed and speed > 1e-9:
            cmd = cmd * (self.max_speed / speed)
        self.position, self.velocity = integrate_with_obstacle_avoidance(
            self.position,
            cmd,
            cfg.vessel_collision_radius,
            obstacles,
            dt,
            lookahead=cfg.obstacle_avoid_lookahead,
            repulsion=cfg.obstacle_avoid_repulsion,
        )
        if self.velocity.length() > 0.05:
            self.heading_rad = math.atan2(self.velocity.y, self.velocity.x)

    def wrap_or_clamp(self, w: float, h: float) -> None:
        """Keep vessel inside map (soft clamp)."""
        m = 12.0
        x = min(max(self.position.x, m), w - m)
        y = min(max(self.position.y, m), h - m)
        self.position = Vec2(x, y)
