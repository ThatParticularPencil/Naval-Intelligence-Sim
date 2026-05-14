from __future__ import annotations

import math as m
import random
from dataclasses import dataclass, field

from entities.obstacle import Obstacle
from utils import SimulationConfig
from utils.obstacle_avoidance import avoid, resolve_penetrations
from utils.waypoint_nav import Waypoint 
from utils.vec2 import Vec2


@dataclass
class Target:
    """
    Moving surface contact. Integrates velocity, reflects off world AABB,
    """

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

    # Occlusion state: when hidden_until > sim_time, sensors cannot see this target
    hidden_until: float = 0.0
    _rng: random.Random = field(default_factory=random.Random, repr=False, compare=False)

    def nav_vector(
        self,
        cfg: SimulationConfig,
        obstacles: tuple[Obstacle, ...],
    ) -> tuple[float,float]: #vector contains acceleration and change in heading
        """
        Only change will be dv and dh
        acceleration and heading
        three rays
        """

        avoid_vec = avoid(self.position,
              self.velocity,
              self.radius,
              obstacles,
              cfg,)
        
        #turn
        direction = self.waypoint.pos - self.position
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
        dh = cfg.p_turn * heading_error
        dh = max(-cfg.max_turn, min(cfg.max_turn, dh))

        #accel
        position_error = direction.length()
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
    ) -> None:
        dv, dh = self.nav_vector(cfg, obstacles)

        new_speed = self.velocity.length() + dv
        new_speed = min(max(new_speed,cfg.target_speed_min), cfg.target_speed_max)
        
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

    def clear_line_of_sight(self, observer: Vec2, obstacles: tuple[Obstacle, ...]) -> bool:
        """True if no obstacle disk intersects the segment observer -> target center."""
        from utils.geometry import segment_circle_intersect

        end = self.position
        for o in obstacles:
            if segment_circle_intersect(observer, end, o.center, o.radius):
                return False
        return True
