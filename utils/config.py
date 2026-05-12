from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class SimulationConfig:
    """Central tuning knobs for the sandbox. Loaded once at startup."""

    world_width: float = 900.0
    world_height: float = 900.0

    # Random waypoint legs (seconds) for vessel + targets
    nav_grid_margin: float = 28.0
    nav_leg_duration_min_s: float = 4.0
    nav_leg_duration_max_s: float = 14.0
    nav_detour_duration_min_s: float = 1.5
    nav_detour_duration_max_s: float = 6.5
    nav_arrival_radius: float = 26.0
    nav_path_inflate: float = 12.0  # inflate obstacle discs for LOS test to primary goal
    nav_detour_pad: float = 22.0  # clearance when placing secondary waypoint around a rock

    # Simulation timing
    dt: float = 1.0 / 60.0
    fixed_substeps: int = 1

    # Vessel (passive: holds station with optional micro-drift)
    vessel_max_speed: float = 30.0
    vessel_sensor_radius: float = 130.0
    vessel_station_keeping_noise: float = 0.001  # tiny velocity jitter for realism
    vessel_collision_radius: float = 9.0  # disk used for obstacle avoidance / resolution

    # Shared obstacle avoidance (kinematic repulsion + penetration resolve)
    obstacle_avoid_lookahead: float = 10.0
    obstacle_avoid_repulsion: float = 100.0

    # Targets
    num_targets: int = 2
    target_speed_min: float = 30.0
    target_speed_max: float = 50.0
    target_radius: float = 5.0 #

    # Occlusion / sensor loss (probabilistic drop-out when otherwise visible)
    occlusion_dropout_per_s: float = 0.001 #zero for now
    occlusion_min_hidden_s: float = 0.35
    occlusion_max_hidden_s: float = 1.8

    # Sensor noise (std dev in meters, applied in boat frame then mapped to world)
    observation_position_noise_std: float = 0.0 #zero for now

    # Tracking filter
    track_alpha_pos: float = 0.45
    track_beta_vel: float = 0.08
    track_confidence_gain_on_hit: float = 0.22
    track_confidence_decay_per_s: float = 0.35
    track_prune_after_s: float = 5.0

    # Metrics
    track_maintained_conf_threshold: float = 0.45
    reacquisition_gap_s: float = 1.2  # no obs above this counts as "lost" for metric

    # Obstacles (center x, y, radius)
    default_obstacles: tuple[tuple[float, float, float], ...] = field(
        default_factory=lambda: (
            (180.0, 420.0, 45.0),
            (520.0, 160.0, 55.0),
            (720.0, 380.0, 38.0),
        )
    )

    num_obstacle_seeds: int = 14
    min_cluster_size: int = 3
    max_cluster_size: int = 10
    obstacle_radius: int = 50
    size_falloff: float = .8

