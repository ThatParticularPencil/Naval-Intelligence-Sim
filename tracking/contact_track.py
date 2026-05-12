from __future__ import annotations

from dataclasses import dataclass

from utils.vec2 import Vec2


@dataclass
class ContactTrack:
    """
    Internal track state for one contact id.

    Uses a lightweight alpha-beta style correction on position with
    inferred velocity — sufficient for a sandbox; swap for EKF/UKF later.
    """

    contact_id: str
    estimated_position: Vec2
    estimated_velocity: Vec2
    confidence: float
    last_observation_time: float
    created_time: float
    # Debug / observability
    last_innovation_norm: float = 0.0

    def seconds_since_observation(self, sim_time: float) -> float:
        return max(0.0, sim_time - self.last_observation_time)
