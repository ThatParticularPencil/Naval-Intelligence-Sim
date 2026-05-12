from __future__ import annotations

from dataclasses import dataclass

from utils.vec2 import Vec2


@dataclass(frozen=True)
class Observation:
    """Single noisy measurement of a contact in world frame."""

    contact_id: str
    measured_position: Vec2
    time: float
    range_m: float
