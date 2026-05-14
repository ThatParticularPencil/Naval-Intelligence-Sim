# Naval Intelligence Sim
https://github.com/user-attachments/assets/57832c82-b4b1-4d27-800e-7e1fdbe0bfcb

A maritime autonomy sandbox for experimenting with multi-vessel search, sensing, tracking, and pursuit behavior; **Inspired by Saronic autonomous boats**. The sim models a small fleet of autonomous surface vessels searching for moving contacts in an obstacle-filled ocean map. They build predicted target locations from noisy observations, and algorithmically decide which vessel should pursue.

I built this project to practice tools that might be useful for real-world robotics applications: simulation loops, sensor models, navigation decisions, tracking confidence, observability metrics, etc.

## What is it

This is not a graphics-first game.

- **Multi-vessel coordination** 
- **Prediction-only pursuit** 
- **Noisy sensing and line of sight** 
- **Tracking confidence over time** 
- **Real-time dashboard** 
- **Configurable simulation parameters** — ~50 editable constants in `SimulationConfig`.

vessels integrate velocity, heading, waypoint drift, obstacle avoidance, and pursuit targets inside a fixed-step simulation loop.
A `SensorModel` turns world state into noisy observations with range and line-of-sight gating.
`ContactTracker` maintains estimates, confidence, stale-track pruning, and metrics over time.
All vessels can contribute observations to shared global tracks, while assignment logic decides which vessel should act.
The HUD exposes position error, velocity error, reacquisition gaps, maintained-track fraction, current assignments, and cumulative tracking score.
The code separates entities, tracking, simulation orchestration, autonomy interfaces, UI, geometry, and configuration.

The result is a compact environment where I can change autonomy logic and immediately see the effect on fleet behavior, reacquisition time, track quality, and mission score.


## Quick Start

```bash
python3 -m pip install -r requirements.txt
python3 main.py
```

Controls:

- `R` — reset the world with a new random seed
- `Esc` — quit

## System Architecture

| Area | Role |
| --- | --- |
| `entities/` | `Vessel`, `Target`, and `Obstacle` kinematics and world interaction |
| `tracking/` | Sensor observations, contact tracks, confidence, and prediction updates |
| `simulation/` | `WorldState`, fixed-step `SimulationEngine`, and tracking metrics |
| `interfaces/` | Autonomy extension points for prioritization, planning, task scoring, and telemetry |
| `utils/` | Configuration, vector math, geometry, waypoint navigation, and obstacle avoidance |
| `ui/` | Real-time tactical dashboard and HUD |

Each tick follows a simple pipeline:

```text
move targets
predict existing tracks
generate observations from every vessel
update global contact tracks
assign closest vessels to predicted targets
move vessels toward assigned predictions or idle waypoints
record metrics, trails, and score
draw dashboard
```

## Core Behavior

### Fleet Search

Targets spawn outside the starting sensor coverage of the vessels, so the fleet begins without immediate perfect knowledge. Idle vessels drift using their waypoint navigation until a sensor observation creates a global predicted marker.

### Shared Tracking

Any vessel can observe a target. Observations update a shared `ContactTracker`, so the prediction belongs to the fleet rather than to one vessel.

Each contact track stores:

- estimated position
- estimated velocity
- confidence
- creation time
- last observation time
- last innovation magnitude

### Assignment And Pursuit

Vessels only chase predictions, not true target positions. For each active track, the engine assigns the closest available vessel to pursue that predicted location. Other vessels continue waypoint drift until they are the best candidate for a prediction.

### Scoring

Every tick, the engine adds the current sum of track confidences multiplied by `dt`. In practice, this rewards the fleet for maintaining confident tracks over time.
A perfect score means that the score is 3 times the sim time.

When all targets are being tracked above the configured maintained-confidence threshold, the HUD score panel pulses

## Metrics Shown In The HUD

- **Mean position error** — estimated position vs. true target position when observed
- **Mean velocity error** — estimated velocity vs. true target velocity
- **Mean reacquisition gap** — average blind time before a target is reacquired
- **Average maintained fraction** — share of true targets with confident tracks over time
- **Tracking score** — cumulative confidence-weighted tracking time
- **Track assignment** — which vessel is currently chasing each contact prediction

## What I Would Build Next

- Replace the alpha-beta tracker with a constant-velocity EKF.
- Add data association for observations without baked-in contact IDs.
- Add telemetry logging and replay with JSONL or ROS-style bagging.
- Model communications latency or packet loss between vessels.
- Add health/state reporting for vessels and sensors.
- Add COLREGS-inspired navigation constraints.
- Split the dashboard from the engine so the sim can run headless in CI.
 
