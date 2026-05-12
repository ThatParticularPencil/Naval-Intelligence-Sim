from __future__ import annotations

import random

from entities.obstacle import Obstacle
from entities.target import Target
from entities.vessel import Vessel
from tracking.observation import Observation
from utils.vec2 import Vec2


class SensorModel:
    """
    Range-disk sensor with additive Gaussian position noise.

    Noise is applied in a boat-aligned frame (along/across LOS) then rotated
    to world — crude anisotropic proxy without full sensor simulation.
    """

    def __init__(self, rng: random.Random | None = None) -> None:
        self._rng = rng or random.Random()

    def observe(
        self,
        vessel: Vessel,
        target: Target,
        sim_time: float,
        noise_std: float,
        obstacles: tuple[Obstacle, ...],
    ) -> Observation | None:
        rel = target.position - vessel.position
        dist = rel.length()
        if dist > vessel.sensor_radius + 1e-6:
            return None
        if target.is_temporarily_hidden(sim_time):
            return None
        if not target.clear_line_of_sight(vessel.position, obstacles):
            return None

        if dist < 1e-6:
            los = Vec2(1.0, 0.0)
        else:
            los = rel * (1.0 / dist)
        perp = Vec2(-los.y, los.x)
        n_los = self._rng.gauss(0.0, noise_std * 0.85)
        n_cross = self._rng.gauss(0.0, noise_std * 1.15)
        noise_world = los * n_los + perp * n_cross
        measured = target.position + noise_world
        return Observation(contact_id=target.id, measured_position=measured, time=sim_time, range_m=dist)
