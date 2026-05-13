from __future__ import annotations

import random as r
from dataclasses import dataclass, field

from utils.config import SimulationConfig as cfg
from utils.vec2 import Vec2


@dataclass
class Waypoint:
    pos: Vec2 = Vec2(0,0) #short lived starting orientation
    life_time: int = -1 

    def give_cruise_speed(self, min: float, max: float):
        return r.uniform(min,max)

    def create_new(self): #return unused for now
        marg: int = cfg.nav_grid_margin
        self.pos = Vec2(
            r.randint(marg, 900 - marg), 
            r.randint(marg, 900 - marg)
        )
        self.life_time = r.randint(cfg.nav_duration_min, cfg.nav_duration_max)
        # return give_cruise_speed() 


    def tick(self):
        if self.life_time == -1:
            self.create_new()
        else:
            self.life_time -= 1


