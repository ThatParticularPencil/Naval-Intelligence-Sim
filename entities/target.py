from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from entities.obstacle import Obstacle
from utils.vec2 import Vec2


@dataclass
class Target:
    """
    Moving surface contact. Integrates velocity, reflects off world AABB,
    """

    id: str
    position: Vec2
    velocity: Vec2
    radius: float
    world_w: float
    world_h: float

    # Occlusion state: when hidden_until > sim_time, sensors cannot see this target
    hidden_until: float = 0.0
    _rng: random.Random = field(default_factory=random.Random, repr=False, compare=False)

    def step_physics(self, dt: float, sim_time: float, cfg) -> None:
        """Advance motion; bounce on rectangular bounds."""
        p = self.position + self.velocity * dt
        v = self.velocity

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

        self.position = p
        self.velocity = v

        # Optional dropout while otherwise visible (handled in visibility check)
        if sim_time >= self.hidden_until:
            # Probability scales with dt for time-scale invariance
            p_drop = 1.0 - math.exp(-cfg.occlusion_dropout_per_s * dt)
            if self._rng.random() < p_drop:
                span = self._rng.uniform(cfg.occlusion_min_hidden_s, cfg.occlusion_max_hidden_s)
                self.hidden_until = sim_time + span

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
