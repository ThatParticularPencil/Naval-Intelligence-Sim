from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict

from tracking.contact_track import ContactTrack
from utils.vec2 import Vec2


@dataclass
class TrackingMetrics:
    """
    Runtime observability: position error, maintenance counts, reacquisition timing.

    Reacquisition sample: when a contact is observed again after a gap longer than
    ``gap_threshold_s`` since the previous observation, we record that gap duration
    (time blind / time to re-hit the sensor).
    """

    position_errors: Deque[float] = field(default_factory=lambda: deque(maxlen=600))
    velocity_errors: Deque[float] = field(default_factory=lambda: deque(maxlen=600))
    reacquisition_samples: Deque[float] = field(default_factory=lambda: deque(maxlen=200))
    maintained_fraction_window: Deque[float] = field(default_factory=lambda: deque(maxlen=240))

    _last_obs_time: Dict[str, float] = field(default_factory=dict)

    def reset(self) -> None:
        self.position_errors.clear()
        self.velocity_errors.clear()
        self.reacquisition_samples.clear()
        self.maintained_fraction_window.clear()
        self._last_obs_time.clear()

    def on_observation(
        self,
        sim_time: float,
        contact_id: str,
        true_pos: Vec2,
        true_vel: Vec2,
        tracks: tuple[ContactTrack, ...],
        gap_threshold_s: float,
    ) -> None:
        prev = self._last_obs_time.get(contact_id)
        if prev is not None:
            gap = sim_time - prev
            if gap > gap_threshold_s:
                self.reacquisition_samples.append(gap)
        self._last_obs_time[contact_id] = sim_time

        tr = next((t for t in tracks if t.contact_id == contact_id), None)
        if tr is not None:
            pe = (tr.estimated_position - true_pos).length()
            ve = (tr.estimated_velocity - true_vel).length()
            self.position_errors.append(pe)
            self.velocity_errors.append(ve)

    def record_frame(
        self,
        tracks: tuple[ContactTrack, ...],
        true_ids: set[str],
        conf_threshold: float,
    ) -> None:
        if not true_ids:
            return
        maintained = sum(
            1
            for tid in true_ids
            if any(t.contact_id == tid and t.confidence >= conf_threshold for t in tracks)
        )
        self.maintained_fraction_window.append(maintained / max(1, len(true_ids)))

    def mean_position_error(self) -> float:
        return sum(self.position_errors) / max(1, len(self.position_errors))

    def mean_velocity_error(self) -> float:
        return sum(self.velocity_errors) / max(1, len(self.velocity_errors))

    def mean_reacquisition_s(self) -> float:
        if not self.reacquisition_samples:
            return 0.0
        return sum(self.reacquisition_samples) / len(self.reacquisition_samples)

    def mean_maintained_fraction(self) -> float:
        if not self.maintained_fraction_window:
            return 0.0
        return sum(self.maintained_fraction_window) / len(self.maintained_fraction_window)
