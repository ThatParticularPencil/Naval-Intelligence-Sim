from __future__ import annotations

import random as r
from dataclasses import dataclass, field

from utils.config import SimulationConfig
from utils.vec2 import Vec2


@dataclass
class Waypoint:
    """
    Random navigable goal and a tick-based leg timer (in simulation steps).

    ``tick`` should be called once per engine step with the active ``SimulationConfig``.
    """

    pos: Vec2 = field(default_factory=lambda: Vec2(0.0, 0.0))
    life_time: int = -1

    def create_new(self, sim_cfg: SimulationConfig) -> None:
        marg = float(sim_cfg.nav_grid_margin)
        ww, wh = float(sim_cfg.world_width), float(sim_cfg.world_height)
        self.pos = Vec2(
            r.uniform(marg, max(marg + 1e-6, ww - marg)),
            r.uniform(marg, max(marg + 1e-6, wh - marg)),
        )
        self.life_time = r.randint(sim_cfg.nav_duration_min, sim_cfg.nav_duration_max)

    def tick(self, sim_cfg: SimulationConfig) -> None:
        if self.life_time < 0:
            self.create_new(sim_cfg)
        else:
            self.life_time -= 1
            if self.life_time <= 0:
                self.create_new(sim_cfg)
