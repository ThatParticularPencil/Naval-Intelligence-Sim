from __future__ import annotations
import math as m

from dataclasses import dataclass, field


@dataclass
class SimulationConfig:
    """Central tuning knobs for the sandbox. Loaded once at startup."""

    world_width: float = 900.0
    world_height: float = 900.0

    # Random waypoint legs: life_time counts simulation ticks (per dt step), not wall seconds
    nav_grid_margin: int = 20
    nav_duration_min: int = 3 * 60
    nav_duration_max: int = 7 * 60
    nav_path_inflate: float = 10.0  # inflate obstacle discs for LOS test to primary goal

    # Simulation timing
    dt: float = 1.0 / 60.0
    fixed_substeps: int = 1

    #navigation constants
    p_turn: float = 0.5
    p_accel: float = 2
    max_turn: float = m.pi/100
    max_accel: float = 10 #pixels per frame^2
    velocity_decay: float = .9
    ray_length: float = 100


    # Vessel (passive: holds station with optional micro-drift)
    vessel_speed_max: float = 50.0
    vessel_speed_min: float = 10.0
    vessel_sensor_radius: float = 130.0
    vessel_radius: float = 1.0  # disk used for obstacle avoidance / resolution

    # Targets
    num_targets: int = 2
    target_speed_min: float = 10.0
    target_speed_max: float = 70.0
    target_radius: float = 6.0 #

    # Sensor noise (std dev in meters, applied in boat frame then mapped to world)
    observation_position_noise_std: float = 0.1 #zero for now

    # Tracking filter
    track_alpha_pos: float = 0.45
    track_beta_vel: float = 0.08
    track_confidence_gain_on_hit: float = 0.22
    track_confidence_decay_per_s: float = 0.35
    track_prune_after_s: float = 5.0

    # Metrics
    track_maintained_conf_threshold: float = 0.45
    reacquisition_gap_s: float = 1.2  # no obs above this counts as "lost" for metric

    num_obstacle_seeds: int = 20
    # min_cluster_size: int = 3
    # max_cluster_size: int = 10
    obstacle_radius: int = 50
    size_variation: float = 30

