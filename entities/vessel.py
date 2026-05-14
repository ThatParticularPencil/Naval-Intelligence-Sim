from __future__ import annotations

import math as m
import random
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Tuple

# from interfaces.autonomy import MissionContext, NavigationPolicy
from entities.obstacle import Obstacle
from utils import SimulationConfig
from utils.obstacle_avoidance import avoid, resolve_penetrations
from utils.waypoint_nav import Waypoint 
from utils.vec2 import Vec2


@dataclass
class Vessel:
    """Autonomous platform: same waypoint + ray-avoidance kinematics as targets."""

    id: str
    position: Vec2
    velocity: Vec2
    heading: float #radians
    radius: float
    world_w: float
    world_h: float
    cruise_speed: float # unused for now
    waypoint: Waypoint 

    max_accel: float = 2 #pixels per frame^2
    max_turn: float = m.pi/12
    heading_delta: float = 0.0

    def nav_vector(
        self,
        cfg: SimulationConfig,
        obstacles: tuple[Obstacle, ...],
        chase_position: Vec2 | None = None,
    ) -> tuple[float,float]: #vector contains acceleration and change in heading
        """
        Chase an externally supplied predicted position, or coast down if idle.
        """

        avoid_vec = avoid(self.position,
              self.velocity,
              self.radius,
              obstacles,
              cfg,)
        
        target_pos = chase_position if chase_position is not None else self.position

        #turn
        direction = target_pos - self.position
        if direction.length() < 1e-6:
            desired_heading = self.heading
        else:
            desired_heading = m.atan2(direction.y, direction.x)
        heading_error = desired_heading - self.heading
        # Normalize to [-pi, pi]
        while heading_error > m.pi:
            heading_error -= 2 * m.pi
        while heading_error < -m.pi:
            heading_error += 2 * m.pi
        aug_max = cfg.max_turn * (self.velocity.length()/50)
        dh = cfg.p_turn * heading_error
        dh = max(-aug_max, min(aug_max, dh))

        #accel
        position_error = direction.length()
        if chase_position is None:
            dv = -cfg.p_accel
        else:
            dv = position_error/100 * cfg.p_accel

        if avoid_vec != (0,0):
            obs_const: float = 0
            dh = (avoid_vec[1] * (1-obs_const))  + dh * obs_const
            dv = (avoid_vec[0] * (1-obs_const))  + dv * obs_const


        return dv,dh

    def step_physics(
        self,
        dt: float,
        cfg: SimulationConfig,
        obstacles: tuple[Obstacle, ...],
        chase_position: Vec2 | None = None,
    ) -> None:
        dv, dh = self.nav_vector(cfg, obstacles, chase_position)

        new_speed = self.velocity.length() + dv
        min_speed = cfg.vessel_speed_min if chase_position is not None else 0.0
        new_speed = min(max(new_speed, min_speed), cfg.vessel_speed_max)
        
        self.velocity = Vec2(new_speed, 0).rotate_rad(self.heading)*(1-cfg.velocity_decay) + (self.velocity * cfg.velocity_decay)
        self.position += self.velocity * dt

        dh_filtered = dh * (1 - cfg.heading_decay) + self.heading_delta * cfg.heading_decay
        self.heading += dh_filtered
        self.heading_delta = dh_filtered

        p, v = self.position, self.velocity
        if p.x < self.radius:
            p = Vec2(self.radius, p.y)
            v = Vec2(abs(v.x), v.y)
        elif p.x > self.world_w - self.radius:
            p = Vec2(self.world_w - self.radius, p.y)
            v = Vec2(-abs(v.x), v.y)
        if p.y < self.radius:
            p = Vec2(p.x, self.radius)
            v = Vec2(v.x, abs(v.y))
        elif p.y > self.world_h - self.radius:
            p = Vec2(p.x, self.world_h - self.radius)
            v = Vec2(v.x, -abs(v.y))

        self.position, self.velocity = resolve_penetrations(self.position, self.velocity, self.radius, obstacles)

