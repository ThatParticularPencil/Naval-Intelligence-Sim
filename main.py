#!/usr/bin/env python3
"""
Autonomous Contact Tracker — entry point.

Passive maritime contact tracking sandbox with pygame dashboard.
"""

from __future__ import annotations

from simulation.engine import SimulationEngine
from simulation.world import WorldState
from ui.dashboard import Dashboard
from utils.config import SimulationConfig


def main() -> None:
    cfg = SimulationConfig()
    world = WorldState.bootstrap(cfg, seed=42)
    engine = SimulationEngine(world)
    Dashboard(engine, scale=1.0).run(max_fps=60.0)


if __name__ == "__main__":
    main()
