from __future__ import annotations

import math as m
import random
from dataclasses import dataclass, field
from turtle import position

from entities.obstacle import Obstacle
from utils.obstacle_avoidance import integrate_with_obstacle_avoidance, resolve_penetrations
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
    cruise_speed: float
    waypoint: Waypoint 

    max_accel: float = 2 #pixels per frame^2
    max_turn: float = m.pi/12

    # Occlusion state: when hidden_until > sim_time, sensors cannot see this target
    hidden_until: float = 0.0
    _rng: random.Random = field(default_factory=random.Random, repr=False, compare=False)

    def nav_vector(
        self,
        sim_time: float,
        cfg,
        obstacles: tuple[Obstacle, ...],
    ) -> Vec2: #vector contains acceleration and change in heading
        """
        Only change will be dv and dh
        acceleration and heading
        three rays
        """
        
        #turn
        heading_error: float = m.radians(self.position.angle_to((self.waypoint.pos - self.position))) - self.heading
        dh: float = cfg.p_turn * heading_error
        dh = max(-cfg.max_turn, min(cfg.max_turn, dh))

        #accel
        dv = cfg.max_accel



    def step_physics(
        self,
        dt: float,
        sim_time: float,
        cfg,
        obstacles: tuple[Obstacle, ...],
    ) -> None:
        """
        Only change will be dv and dh
        acceleration and heading
        three rays
        """







    def is_temporarily_hidden(self, sim_time: float) -> bool:
        return sim_time < self.hidden_until

    def clear_line_of_sight(self, observer: Vec2, obstacles: tuple[Obstacle, ...]) -> bool:
        """True if no obstacle disk intersects the segment observer -> target center."""
        from utils.geometry import segment_circle_intersect

        end = self.position
        for o in obstacles:
            if segment_circle_intersect(observer, end, o.center, o.radius):
                return False
        return True
