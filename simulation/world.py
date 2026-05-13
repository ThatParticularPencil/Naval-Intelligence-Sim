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
from utils.waypoint_nav import Waypoint
from .terrain import terrain_gen


def _spawn_targets(cfg: SimulationConfig, rng: random.Random) -> List[Target]:
    targets: List[Target] = []
    margin = 40.0
    for i in range(cfg.num_targets):
        x = rng.uniform(margin, cfg.world_width - margin)
        y = rng.uniform(margin, cfg.world_height - margin)
        speed = rng.uniform(cfg.target_speed_min, cfg.target_speed_max)
        ang = rng.uniform(0.0, 6.28318)
        heading0 = float(ang)
        v = Vec2(speed, 0.0).rotate_rad(heading0)
        tid = f"T{i+1:02d}"
        t_rng = random.Random(rng.randint(0, 2**30))
        wp = Waypoint()
        wp.create_new()
        targets.append(
            Target(
                id=tid,
                position=Vec2(x, y),
                velocity=v,
                heading=heading0,
                radius=cfg.target_radius,
                world_w=cfg.world_width,
                world_h=cfg.world_height,
                cruise_speed=speed,
                waypoint=wp,
                hidden_until=0.0,
                _rng=t_rng,
            )
        )
    return targets


def _default_obstacles() -> list[Obstacle]:
    return terrain_gen()


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
        vr = random.Random(rng.randint(0, 2**30))
        vstart = Vec2(cfg.world_width * 0.5, cfg.world_height * 0.5)
        v_wp = Waypoint()
        v_wp.create_new()
        h0 = math.atan2(v_wp.pos.y - vstart.y, v_wp.pos.x - vstart.x)
        vessel = Vessel(
            id="Vessel_1",
            position=vstart,
            velocity=Vec2(min(8.0, cfg.vessel_speed_max * 0.25), 0.0).rotate_rad(h0),
            heading=h0,
            radius=cfg.vessel_radius,
            # sensor_radius=cfg.vessel_sensor_radius,
            world_w=cfg.world_width,
            world_h=cfg.world_height,
            cruise_speed=cfg.vessel_speed_max,
            waypoint=v_wp,
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
            obstacles=tuple(_default_obstacles()),
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
