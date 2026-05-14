from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Tuple

from simulation.metrics import TrackingMetrics
from simulation.world import WorldState
from tracking.observation import Observation
from utils.obstacle_avoidance import resolve_penetrations
from utils.vec2 import Vec2


@dataclass
class SimulationEngine:
    """
    Fixed-timestep orchestration: physics, passive navigation, sensing, tracking, metrics.

    Exposes ``last_observations`` for the dashboard
    """

    world: WorldState
    metrics: TrackingMetrics = field(default_factory=TrackingMetrics)
    true_trails: Dict[str, Deque[Tuple[Vec2, float]]] = field(default_factory=lambda: defaultdict(lambda: deque()))
    pred_trails: Dict[str, Deque[Tuple[Vec2, float]]] = field(default_factory=lambda: defaultdict(lambda: deque()))
    last_observations: List[Observation] = field(default_factory=list)
    active_chase_assignments: Dict[str, str] = field(default_factory=dict)
    tracking_score: float = 0.0

    def reset(self, seed: int | None = None) -> None:
        cfg = self.world.cfg
        self.world = WorldState.bootstrap(cfg, seed=seed if seed is not None else self.world.rng.randint(0, 10_000_000))
        self.metrics.reset()
        self.true_trails.clear()
        self.pred_trails.clear()
        self.last_observations.clear()
        self.active_chase_assignments.clear()
        self.tracking_score = 0.0

    def step(self) -> None:
        w = self.world
        cfg = w.cfg
        dt = cfg.dt
        w.sim_time += dt

        for t in w.targets:
            t.step_physics(dt, cfg, w.obstacles)

        w.tracker.predict(dt, w.sim_time)

        self.last_observations.clear()
        # All vessels observe all targets
        for vessel in w.vessels:
            for tgt in w.targets:
                obs = w.sensor.observe(
                    vessel,
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

        chase_positions = self._assign_prediction_chasers()
        for vessel in w.vessels:
            chase_list = [chase_positions[vessel.id]] if vessel.id in chase_positions else None
            vessel.step_physics(dt, cfg, w.obstacles, chase_list)
            vessel.position, vessel.velocity = resolve_penetrations(
                vessel.position,
                vessel.velocity,
                cfg.vessel_radius,
                w.obstacles,
            )

        self.metrics.record_frame(
            w.tracker.all_tracks(),
            w.true_target_ids(),
            cfg.track_maintained_conf_threshold,
        )
        self.tracking_score += sum(track.confidence for track in w.tracker.all_tracks()) * dt

        for tgt in w.targets:
            self.true_trails[tgt.id].append((tgt.position.copy(), w.sim_time))
        for tr in w.tracker.all_tracks():
            self.pred_trails[tr.contact_id].append((tr.estimated_position, w.sim_time))

        # Remove trail entries older than 2 seconds
        for trail in self.true_trails.values():
            while trail and w.sim_time - trail[0][1] > 2.0:
                trail.popleft()
        for trail in self.pred_trails.values():
            while trail and w.sim_time - trail[0][1] > 2.0:
                trail.popleft()

    def _assign_prediction_chasers(self) -> Dict[str, Vec2]:
        self.active_chase_assignments.clear()
        if not self.world.vessels:
            return {}

        chase_positions: Dict[str, Vec2] = {}
        assigned_vessels: set[str] = set()
        tracks = sorted(
            self.world.tracker.all_tracks(),
            key=lambda tr: tr.confidence,
            reverse=True,
        )
        for track in tracks:
            available = [v for v in self.world.vessels if v.id not in assigned_vessels]
            if not available:
                break
            closest = min(
                available,
                key=lambda v: (track.estimated_position - v.position).length(),
            )
            assigned_vessels.add(closest.id)
            chase_positions[closest.id] = track.estimated_position
            self.active_chase_assignments[track.contact_id] = closest.id
        return chase_positions
