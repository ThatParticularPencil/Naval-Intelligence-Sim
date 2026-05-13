from __future__ import annotations
import math as m

from dataclasses import dataclass, field


@dataclass
class SimulationConfig:
    """Central tuning knobs for the sandbox. Loaded once at startup."""

    world_width: float = 900.0
    world_height: float = 900.0

    # Random waypoint legs (seconds) for vessel + targets
    nav_grid_margin: int = 20
    nav_duration_min: int = 5 * 60
    nav_duration_max: int = 15 * 60
    nav_path_inflate: float = 10.0  # inflate obstacle discs for LOS test to primary goal

    # Simulation timing
    dt: float = 1.0 / 60.0
    fixed_substeps: int = 1

    #navigation constants
    p_turn: float = 1.1
    p_accel: float = 1.1
    max_turn: float = m.pi/12
    max_accel: float = 2 #pixels per frame^2
    ray_length: float = 100


    # Vessel (passive: holds station with optional micro-drift)
    vessel_max_speed: float = 30.0
    vessel_sensor_radius: float = 130.0
    vessel_collision_radius: float = 9.0  # disk used for obstacle avoidance / resolution

    # Targets
    num_targets: int = 2
    target_speed_min: float = 30.0
    target_speed_max: float = 50.0
    target_radius: float = 5.0 #

    # Occlusion / sensor loss (probabilistic drop-out when otherwise visible)
    occlusion_dropout_per_s: float = 0.1
    occlusion_min_hidden_s: float = 0.35
    occlusion_max_hidden_s: float = 1.8

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

    num_obstacle_seeds: int = 14
    min_cluster_size: int = 3
    max_cluster_size: int = 10
    obstacle_radius: int = 50
    size_falloff: float = .8

