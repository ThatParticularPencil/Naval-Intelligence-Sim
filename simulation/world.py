from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Tuple

from entities.obstacle import Obstacle
from entities.target import Target
from entities.vessel import Vessel
from interfaces.autonomy import MissionContext, PassiveNavigationPolicy
from tracking.sensor_model import SensorModel
from tracking.tracker import ContactTracker
from utils.config import SimulationConfig
from utils.vec2 import Vec2


def _spawn_targets(cfg: SimulationConfig, rng: random.Random) -> List[Target]:
    targets: List[Target] = []
    margin = 40.0
    for i in range(cfg.num_targets):
        x = rng.uniform(margin, cfg.world_width - margin)
        y = rng.uniform(margin, cfg.world_height - margin)
        speed = rng.uniform(cfg.target_speed_min, cfg.target_speed_max)
        ang = rng.uniform(0.0, 6.28318)
        v = Vec2(speed * math.cos(ang), speed * math.sin(ang))
        tid = f"T{i+1:02d}"
        targets.append(
            Target(
                id=tid,
                position=Vec2(x, y),
                velocity=v,
                radius=cfg.target_radius,
                world_w=cfg.world_width,
                world_h=cfg.world_height,
                hidden_until=0.0,
                _rng=random.Random(rng.randint(0, 2**30)),
            )
        )
    return targets


def _default_obstacles(cfg: SimulationConfig) -> Tuple[Obstacle, ...]:
    return tuple(Obstacle(Vec2(cx, cy), r) for cx, cy, r in cfg.default_obstacles)


@dataclass
class WorldState:
    """Authoritative simulation state: entities + time + config."""

    cfg: SimulationConfig
    sim_time: float
    vessel: Vessel
    targets: List[Target]
    obstacles: Tuple[Obstacle, ...]
    tracker: ContactTracker
    sensor: SensorModel
    rng: random.Random = field(default_factory=random.Random, repr=False)

    @classmethod
    def bootstrap(cls, cfg: SimulationConfig, seed: int | None = 42) -> WorldState:
        rng = random.Random(seed)
        nav = PassiveNavigationPolicy()
        vessel = Vessel(
            position=Vec2(cfg.world_width * 0.5, cfg.world_height * 0.5),
            velocity=Vec2(0.0, 0.0),
            heading_rad=0.0,
            max_speed=cfg.vessel_max_speed,
            sensor_radius=cfg.vessel_sensor_radius,
            navigation=nav,
            _rng=random.Random(rng.randint(0, 2**30)),
        )
        tracker = ContactTracker(
            alpha_pos=cfg.track_alpha_pos,
            beta_vel=cfg.track_beta_vel,
            conf_gain_hit=cfg.track_confidence_gain_on_hit,
            conf_decay_per_s=cfg.track_confidence_decay_per_s,
            prune_after_s=cfg.track_prune_after_s,
        )
        return cls(
            cfg=cfg,
            sim_time=0.0,
            vessel=vessel,
            targets=_spawn_targets(cfg, rng),
            obstacles=_default_obstacles(cfg),
            tracker=tracker,
            sensor=SensorModel(random.Random(rng.randint(0, 2**30))),
            rng=rng,
        )

    def mission_context(self) -> MissionContext:
        return MissionContext(
            sim_time_s=self.sim_time,
            vessel=self.vessel,
            tracks=self.tracker.all_tracks(),
        )

    def true_target_ids(self) -> set[str]:
        return {t.id for t in self.targets}
