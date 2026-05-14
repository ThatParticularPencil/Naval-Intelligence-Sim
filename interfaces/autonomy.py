from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol, Sequence

if TYPE_CHECKING:
    from entities.vessel import Vessel
    from entities.target import Target
    from tracking.contact_track import ContactTrack
    from utils.vec2 import Vec2


@dataclass(frozen=True)
class MissionContext:
    """Snapshot passed to mission modules (expand fields as missions grow)."""

    sim_time_s: float
    vessels: tuple[Vessel, ...]
    primary_vessel: Vessel | None
    tracks: tuple[ContactTrack, ...]

    @property
    def vessel(self) -> Vessel | None:
        """Backward-compatible alias for older single-vessel autonomy code."""
        return self.primary_vessel


class TargetPrioritizer(ABC):
    """Rank contacts for attention (future: threat, COLREGS, intel value)."""

    @abstractmethod
    def prioritize(
        self, ctx: MissionContext, targets: Sequence[Target]
    ) -> Sequence[str]:
        """Return target ids in descending priority order."""


class InterceptPlanner(ABC):
    """Generate intercept geometry / time-on-target (future)."""

    @abstractmethod
    def plan(self, ctx: MissionContext, target_id: str) -> Any:
        """Return planner-specific plan object (path, ETA, feasibility)."""


class MissionExecutor(ABC):
    """Execute discrete mission tasks on top of navigation (future)."""

    @abstractmethod
    def step(self, ctx: MissionContext, dt: float) -> None:
        """Advance mission state machine."""


class NavigationPolicy(ABC):
    """Convert high-level intent into vessel commands (future pursuit, station-keep)."""

    @abstractmethod
    def desired_velocity(self, ctx: MissionContext) -> Vec2:
        """World-frame desired velocity for the vessel this tick."""


class TaskScorer(ABC):
    """Score candidate tasks for planning / allocation (future multi-agent)."""

    @abstractmethod
    def score(self, ctx: MissionContext, task: Any) -> float:
        """Higher is better."""


class TelemetrySink(Protocol):
    """Optional hook for logging / replay streams."""

    def emit(self, record: dict[str, Any]) -> None: ...


class NoOpTargetPrioritizer(TargetPrioritizer):
    def prioritize(self, ctx: MissionContext, targets: Sequence[Target]) -> Sequence[str]:
        return tuple(t.id for t in targets)


class PassiveNavigationPolicy(NavigationPolicy):
    """Default: station-keeping micro-drift handled in vessel; zero commanded motion here."""

    def desired_velocity(self, ctx: MissionContext) -> Vec2:
        from utils.vec2 import Vec2

        return Vec2(0.0, 0.0)
