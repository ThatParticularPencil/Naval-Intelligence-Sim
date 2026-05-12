"""
Random waypoint legs with timed horizons and detour waypoints when the
straight segment to the primary goal intersects an obstacle disk.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Optional, Tuple

from entities.obstacle import Obstacle
from utils.config import SimulationConfig
from utils.geometry import segment_circle_intersect
from utils.vec2 import Vec2


def random_goal_in_bounds(
    rng: random.Random,
    world_w: float,
    world_h: float,
    margin: float,
) -> Vec2:
    """Uniform random point inside the navigable rectangle."""
    x = rng.uniform(margin, max(margin + 1e-6, world_w - margin))
    y = rng.uniform(margin, max(margin + 1e-6, world_h - margin))
    return Vec2(x, y)


def _first_blocker(
    a: Vec2,
    b: Vec2,
    obstacles: Tuple[Obstacle, ...],
    inflate: float,
) -> Optional[Obstacle]:
    """Obstacle whose inflated disc first blocks segment A→B (by closest projection t)."""
    ab = b - a
    lab2 = ab.dot(ab)
    if lab2 < 1e-18:
        return None
    best_o: Optional[Obstacle] = None
    best_t = 1e18
    for o in obstacles:
        r = o.radius + inflate
        if not segment_circle_intersect(a, b, o.center, r):
            continue
        t = (o.center - a).dot(ab) / lab2
        t = min(1.0, max(0.0, t))
        if t < best_t:
            best_t = t
            best_o = o
    return best_o


def _path_blocked(
    a: Vec2,
    b: Vec2,
    obstacles: Tuple[Obstacle, ...],
    inflate: float,
) -> bool:
    for o in obstacles:
        if segment_circle_intersect(a, b, o.center, o.radius + inflate):
            return True
    return False


def pick_detour_waypoint(
    position: Vec2,
    primary_goal: Vec2,
    blocker: Obstacle,
    agent_radius: float,
    pad: float,
    side: float,
    world_w: float,
    world_h: float,
    margin: float,
) -> Vec2:
    """
    Secondary point offset around ``blocker`` (side = +1 or -1) then nudged toward open water.

    ``pad`` is extra clearance beyond combined radii.
    """
    c = blocker.center
    toward = primary_goal - c
    if toward.length() < 1e-9:
        toward = position - c
    if toward.length() < 1e-9:
        toward = Vec2(1.0, 0.0)
    g = toward.normalized()
    perp = Vec2(-g.y, g.x) * side
    standoff = blocker.radius + agent_radius + pad
    wp = c + perp * standoff + g * (standoff * 0.35)
    # Clamp into navigable bounds
    x = min(max(wp.x, margin), world_w - margin)
    y = min(max(wp.y, margin), world_h - margin)
    return Vec2(x, y)


@dataclass
class WaypointNav:
    """Per-agent random waypoint legs + optional detour around blocking rocks."""

    _rng: random.Random = field(repr=False)
    primary_goal: Vec2
    leg_end_time: float
    detour_goal: Optional[Vec2] = None
    detour_end_time: Optional[float] = None

    @classmethod
    def bootstrap(
        cls,
        rng: random.Random,
        sim_time: float,
        position: Vec2,
        cfg: SimulationConfig,
    ) -> WaypointNav:
        g = random_goal_in_bounds(rng, cfg.world_width, cfg.world_height, cfg.nav_grid_margin)
        dur = rng.uniform(cfg.nav_leg_duration_min_s, cfg.nav_leg_duration_max_s)
        return cls(
            _rng=rng,
            primary_goal=g,
            leg_end_time=sim_time + dur,
        )

    def _roll_new_primary(self, sim_time: float, position: Vec2, cfg: SimulationConfig) -> None:
        self.primary_goal = random_goal_in_bounds(
            self._rng, cfg.world_width, cfg.world_height, cfg.nav_grid_margin
        )
        self.leg_end_time = sim_time + self._rng.uniform(
            cfg.nav_leg_duration_min_s, cfg.nav_leg_duration_max_s
        )
        self.detour_goal = None
        self.detour_end_time = None

    def desired_velocity(
        self,
        position: Vec2,
        sim_time: float,
        cfg: SimulationConfig,
        obstacles: Tuple[Obstacle, ...],
        max_speed: float,
        agent_radius: float,
    ) -> Vec2:
        """
        Seek current active waypoint; advance legs and assign detours when the path is blocked.
        """
        ww, wh = cfg.world_width, cfg.world_height
        margin = cfg.nav_grid_margin
        inflate = agent_radius + cfg.nav_path_inflate
        arrive = cfg.nav_arrival_radius

        # Expire / renew primary leg
        if sim_time >= self.leg_end_time or (position - self.primary_goal).length() < arrive:
            self._roll_new_primary(sim_time, position, cfg)

        # Manage active detour
        if self.detour_goal is not None and self.detour_end_time is not None:
            arrived_d = (position - self.detour_goal).length() < arrive
            time_up = sim_time >= self.detour_end_time
            clear_path = not _path_blocked(position, self.primary_goal, obstacles, inflate)
            if time_up or (arrived_d and clear_path):
                self.detour_goal = None
                self.detour_end_time = None

        active = self.primary_goal
        if self.detour_goal is not None:
            active = self.detour_goal
        elif _path_blocked(position, self.primary_goal, obstacles, inflate):
            blk = _first_blocker(position, self.primary_goal, obstacles, inflate)
            if blk is not None:
                side = 1.0 if self._rng.random() < 0.5 else -1.0
                self.detour_goal = pick_detour_waypoint(
                    position,
                    self.primary_goal,
                    blk,
                    agent_radius,
                    cfg.nav_detour_pad,
                    side,
                    ww,
                    wh,
                    margin,
                )
                self.detour_end_time = sim_time + self._rng.uniform(
                    cfg.nav_detour_duration_min_s, cfg.nav_detour_duration_max_s
                )
                active = self.detour_goal

        d = active - position
        L = d.length()
        if L < 1e-9:
            return Vec2(0.0, 0.0)
        return d * (max_speed / L)
