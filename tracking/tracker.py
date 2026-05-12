from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict

from tracking.contact_track import ContactTrack
from tracking.observation import Observation
from utils.vec2 import Vec2


@dataclass
class ContactTracker:
    """
    Multi-contact tracker: prediction + observation update.

    Prediction: constant-velocity coast.
    Update: alpha-beta correction on position; confidence rises on hits and decays on misses.
    """

    alpha_pos: float
    beta_vel: float
    conf_gain_hit: float
    conf_decay_per_s: float
    prune_after_s: float
    tracks: Dict[str, ContactTrack] = field(default_factory=dict)

    def predict(self, dt: float, sim_time: float) -> None:
        for tr in self.tracks.values():
            tr.estimated_position = tr.estimated_position + tr.estimated_velocity * dt
            age = max(0.0, sim_time - tr.last_observation_time)
            # Faster decay the longer we have coasted without measurement
            tr.confidence *= math.exp(-self.conf_decay_per_s * dt * (1.0 + 0.08 * age))

    def update(self, obs: Observation) -> None:
        tr = self.tracks.get(obs.contact_id)
        if tr is None:
            self.tracks[obs.contact_id] = ContactTrack(
                contact_id=obs.contact_id,
                estimated_position=obs.measured_position,
                estimated_velocity=Vec2(0.0, 0.0),
                confidence=min(1.0, 0.35 + self.conf_gain_hit),
                last_observation_time=obs.time,
                created_time=obs.time,
            )
            return

        innov = obs.measured_position - tr.estimated_position
        innov_norm = innov.length()
        tr.last_innovation_norm = innov_norm

        tr.estimated_position = tr.estimated_position + innov * self.alpha_pos
        if obs.time > tr.last_observation_time + 1e-6:
            dt_obs = obs.time - tr.last_observation_time
            measured_vel = innov * (1.0 / max(dt_obs, 1e-3))
            tr.estimated_velocity = tr.estimated_velocity * (1.0 - self.beta_vel) + measured_vel * self.beta_vel
        tr.last_observation_time = obs.time
        tr.confidence = min(1.0, tr.confidence + self.conf_gain_hit)

    def prune_stale(self, sim_time: float) -> None:
        dead = [cid for cid, t in self.tracks.items() if sim_time - t.last_observation_time > self.prune_after_s]
        for cid in dead:
            del self.tracks[cid]

    def all_tracks(self) -> tuple[ContactTrack, ...]:
        return tuple(self.tracks.values())
