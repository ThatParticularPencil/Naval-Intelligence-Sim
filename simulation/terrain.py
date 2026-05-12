import random as r
import math as m
from entities import Obstacle
from utils.config import SimulationConfig as sc
    # num_obstacle_seeds: int = 7
    # min_cluster_size: int = 3
    # max_cluster_size: int = 10
    # obstacle_radius: int = 50.0
    # size_falloff: float = .8
    # size_variation: int = 30

from utils.vec2 import Vec2

"""interesting terrain gen"""
def noisy_radius(rad:int) -> int:
    return int(r.gauss(rad, 10))

def terrain_gen() -> list[Obstacle]:
    #generating seed locations
    seed_locations: list[Vec2] = [Vec2(r.randint(0,900), r.randint(0,900)) for i in range(sc.num_obstacle_seeds)]
    seeds: list[Obstacle] = [Obstacle(seed_locations[i], noisy_radius(sc.obstacle_radius)) for i in range(sc.num_obstacle_seeds)]
    
    return seeds
    

