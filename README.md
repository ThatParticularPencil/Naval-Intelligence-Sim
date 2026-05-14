# Autonomous Contact Tracker

A **Python** maritime autonomy **experimentation sandbox**: multiple active autonomous vessels, moving contacts, rocky obstacles, range–bearing–style sensing with noise, and an internal **multi-contact tracker** with confidence and timing metadata. Visualization uses **pygame** as a **dashboard** (not a game loop focused on polish).

## Quick start

```bash
cd "/path/to/Naval Intelligence Sim"
python3 -m pip install -r requirements.txt
python3 main.py
```

- **R** — reset world (new RNG seed draw for targets)
- **Esc** — quit

## System architecture

| Package / area | Role |
|----------------|------|
| `utils/` | `Vec2`, `SimulationConfig`, geometry (LOS vs obstacles). |
| `entities/` | `Vessel`, `Target`, `Obstacle` — kinematic state and environment interaction. |
| `tracking/` | `SensorModel`, `Observation`, `ContactTracker`, `ContactTrack` — measurements and filter state. |
| `simulation/` | `WorldState` (bootstrap), `SimulationEngine` (fixed step), `TrackingMetrics` (observability). |
| `interfaces/` | ABCs / protocols for **future autonomy** (prioritization, intercept, mission exec, nav policy, scoring, telemetry). |
| `ui/` | `Dashboard` — tactical map + HUD metrics. |
| `main.py` | Wire config → world → engine → UI. |

Data flows **one way** each tick: propagate targets → `tracker.predict` → generate fleet `Observation`s → `tracker.update` → assign the closest vessel to each global prediction → propagate vessels → prune → metrics → trails for plotting.

## Tracking approach

Each known contact id maps to a `ContactTrack` with:

- **Estimated position / velocity** — constant-velocity **prediction** each step; **alpha–beta**-style correction on position when a measurement arrives, with velocity nudged toward an implied measurement velocity using `track_beta_vel`.
- **Confidence** — increases on successful hits (`track_confidence_gain_on_hit`), decays exponentially while coasting without measurements (`track_confidence_decay_per_s` scaled by time since last obs).
- **Time since last observation** — `sim_time - last_observation_time` for HUD and gating.

This is intentionally **lightweight and inspectable**; swap the internals of `ContactTracker` for an EKF/UKF, particle filter, or track-oriented MHT without changing entity or UI contracts.

The default fleet spawns six vessels. Any vessel can create or update the shared predicted-location marker through observation, but vessels do not chase ground truth; only the closest available vessel chases a global prediction.

## Uncertainty and noise modeling

1. **Additive Gaussian position noise** in a **boat-aligned frame** (along / across line of sight with different scales) then mapped to world — simple **anisotropic** stand-in for radar/video jitter (`observation_position_noise_std` in `SimulationConfig`).
2. **Range disks** — each vessel can observe contacts within `vessel_sensor_radius`.
3. **Line of sight** — each vessel-to-target segment must not intersect obstacle disks (`entities/target.py` + `utils/geometry.py`).
4. **Stochastic occlusion / drop-out** — targets may enter a hidden window (`occlusion_dropout_per_s`, `occlusion_min_hidden_s` / `occlusion_max_hidden_s`) to emulate brief sensor loss unrelated to geometry.

## Extension points (future autonomy)

Defined in `interfaces/autonomy.py`:

- `TargetPrioritizer` — rank contacts (threat, COLREGS, intel).
- `InterceptPlanner` — geometry / time-on-target plans.
- `MissionExecutor` — discrete mission state machines.
- `NavigationPolicy` — map high-level intent to `desired_velocity` (today: `PassiveNavigationPolicy` → zero; vessel motion is tiny station-keeping noise in `Vessel.step`).
- `TaskScorer` — multi-agent task allocation.
- `TelemetrySink` — structured logs / replay bus.

`MissionContext` carries `sim_time_s`, the active `vessels` tuple, a backward-compatible `primary_vessel`, and the current `tracks` tuple — extend the dataclass as richer world snapshots are needed.

**Suggested integration:** construct a small `AutonomyStack` in `simulation/engine.py` that holds optional instances and calls `navigation.desired_velocity`, `mission.step`, etc., after building `MissionContext` each tick — keep the passive demo by default.

## Metrics (HUD)

- **Mean position / velocity error** — rolling window over observation instants where a track exists (estimated vs ground truth for that id).
- **Mean reacquisition gap** — mean of inter-observation intervals **longer than** `reacquisition_gap_s` (proxy for blind time between hits).
- **Average maintained** — rolling mean fraction of true targets with at least one track above `track_maintained_conf_threshold`.

## Future improvements

- Replace alpha–beta with **CV-EKF** and explicit measurement Jacobians.
- **Data association** when contact ids are not baked in (measurement clusters vs tracks).
- `TaskScorer`-driven assignment across the active vessel fleet.
- **Replay** — implement `TelemetrySink` writing JSONL frames; add deterministic seed control per subsystem.
- **COLREGS / domain constraints** in `NavigationPolicy` once pursuit is enabled.

## Repository layout

```
main.py
requirements.txt
simulation/
tracking/
entities/
interfaces/
ui/
utils/
```

---

This project is structured as **internal robotics / autonomy tooling**: readable modules, explicit parameters, and a smooth **60 Hz** stepping loop suitable for future algorithm hooks.
