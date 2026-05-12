from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List

from simulation.metrics import TrackingMetrics
from simulation.world import WorldState
from tracking.observation import Observation
from utils.vec2 import Vec2


@dataclass
class SimulationEngine:
    """
    Fixed-timestep orchestration: physics, passive navigation, sensing, tracking, metrics.

    Exposes ``last_observations`` for the dashboard (noisy hits this frame).
    """

    world: WorldState
    metrics: TrackingMetrics = field(default_factory=TrackingMetrics)
    true_trails: Dict[str, Deque[Vec2]] = field(default_factory=lambda: defaultdict(lambda: deque(maxlen=120)))
    pred_trails: Dict[str, Deque[Vec2]] = field(default_factory=lambda: defaultdict(lambda: deque(maxlen=120)))
    last_observations: List[Observation] = field(default_factory=list)

    def reset(self, seed: int | None = None) -> None:
        cfg = self.world.cfg
        self.world = WorldState.bootstrap(cfg, seed=seed if seed is not None else self.world.rng.randint(0, 10_000_000))
        self.metrics.reset()
        self.true_trails.clear()
        self.pred_trails.clear()
        self.last_observations.clear()

    def step(self) -> None:
        w = self.world
        cfg = w.cfg
        dt = cfg.dt
        w.sim_time += dt

        ctx = w.mission_context()
        w.vessel.step(dt, ctx, cfg.vessel_station_keeping_noise)
        w.vessel.wrap_or_clamp(cfg.world_width, cfg.world_height)

        for t in w.targets:
            t.step_physics(dt, w.sim_time, cfg)

        w.tracker.predict(dt, w.sim_time)

        self.last_observations.clear()
        for tgt in w.targets:
            obs = w.sensor.observe(
                w.vessel,
                tgt,
                w.sim_time,
                cfg.observation_position_noise_std,
                w.obstacles,
            )
            if obs is not None:
                w.tracker.update(obs)
                self.last_observations.append(obs)
                self.metrics.on_observation(
                    w.sim_time,
                    tgt.id,
                    tgt.position,
                    tgt.velocity,
                    w.tracker.all_tracks(),
                    cfg.reacquisition_gap_s,
                )

        w.tracker.prune_stale(w.sim_time)

        self.metrics.record_frame(
            w.tracker.all_tracks(),
            w.true_target_ids(),
            cfg.track_maintained_conf_threshold,
        )

        for tgt in w.targets:
            self.true_trails[tgt.id].append(tgt.position)
        for tr in w.tracker.all_tracks():
            self.pred_trails[tr.contact_id].append(tr.estimated_position)
