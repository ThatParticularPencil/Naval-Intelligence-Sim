# Naval Intelligence Sim

A maritime autonomy sandbox for experimenting with multi-vessel search, sensing, tracking, and pursuit behavior. The sim models a small fleet of autonomous surface vessels looking for moving contacts in an obstacle-filled ocean map, building shared predicted target locations from noisy observations, and assigning the closest vessel to pursue each prediction.

I built this project to practice the kind of software that sits between autonomy algorithms and real vehicle behavior: simulation loops, sensor models, navigation decisions, tracking confidence, observability metrics, and a live operator-style dashboard.

## Why This Project Is Interesting

This is not a graphics-first game. It is a small systems/autonomy testbed with inspectable modules and real-time behavior:

- **Multi-vessel coordination** — multiple vessels are active at once, but they share global predicted target tracks instead of each vessel acting on isolated knowledge.
- **Prediction-only pursuit** — vessels do not chase ground-truth target positions. Sensor hits update global predicted markers, and only the closest available vessel chases each prediction.
- **Noisy sensing and line of sight** — observations are range-limited, noisy, and blocked by obstacles, so tracking depends on visibility rather than omniscient state.
- **Tracking confidence over time** — tracks gain confidence on observations, decay when stale, and feed a cumulative score representing how long the fleet has been tracking something.
- **Real-time dashboard** — pygame renders vessels, targets, sensor rings, observations, predicted tracks, assignment state, and health/quality metrics at interactive speed.
- **Configurable simulation parameters** — world size, vessel count, target count, sensor range, target speed, noise, and tracker behavior are centralized in `SimulationConfig`.

The result is a compact environment where I can change autonomy logic and immediately see the effect on fleet behavior, reacquisition time, track quality, and mission score.

## Relevance To Autonomous Surface Vessel Software

Saronic Technologies works on autonomous vessels where software has to connect perception, control, telemetry, and real-world constraints. This project is intentionally aligned with that kind of work:

- **Navigation and control:** vessels integrate velocity, heading, waypoint drift, obstacle avoidance, and pursuit targets inside a fixed-step simulation loop.
- **Sensor integration:** a `SensorModel` turns world state into noisy observations with range and line-of-sight gating.
- **Tracking and data processing:** `ContactTracker` maintains estimates, confidence, stale-track pruning, and metrics over time.
- **Distributed autonomy concepts:** all vessels can contribute observations to shared global tracks, while assignment logic decides which vessel should act.
- **System observability:** the HUD exposes position error, velocity error, reacquisition gaps, maintained-track fraction, current assignments, and cumulative tracking score.
- **Modular architecture:** the code separates entities, tracking, simulation orchestration, autonomy interfaces, UI, geometry, and configuration.

For a Systems Software Engineer Intern role, the project demonstrates that I enjoy building the glue between autonomy concepts and running systems: state propagation, interfaces, telemetry, debugging views, and behavior that can be inspected frame by frame.

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

The tracking score is cumulative and never decreases. Every tick, the engine adds the current sum of track confidences multiplied by `dt`. In practice, this rewards the fleet for maintaining confident tracks over time.

When all targets are being tracked above the configured maintained-confidence threshold, the HUD score panel pulses to make that state obvious.

## Metrics Shown In The HUD

- **Mean position error** — estimated position vs. true target position when observed
- **Mean velocity error** — estimated velocity vs. true target velocity
- **Mean reacquisition gap** — average blind time before a target is reacquired
- **Average maintained fraction** — share of true targets with confident tracks over time
- **Tracking score** — cumulative confidence-weighted tracking time
- **Track assignment** — which vessel is currently chasing each contact prediction

## Design Choices

The implementation is intentionally lightweight. I wanted the autonomy behavior to be easy to inspect and change without hiding core logic inside a large framework.

- The tracker uses an alpha-beta style correction rather than a full EKF, so the data flow stays readable.
- The sensor model is simple but captures the important systems idea: observations are constrained by range, noise, and geometry.
- The dashboard favors debugging and observability over visual polish.
- Configuration is centralized so scenarios can be changed quickly.
- Interfaces exist for future mission logic, task scoring, telemetry, prioritization, and planning.

## What I Would Build Next

- Replace the alpha-beta tracker with a constant-velocity EKF.
- Add data association for observations without baked-in contact IDs.
- Add telemetry logging and replay with JSONL or ROS-style bagging.
- Model communications latency or packet loss between vessels.
- Add health/state reporting for vessels and sensors.
- Add COLREGS-inspired navigation constraints.
- Add unit tests for assignment, tracking confidence, and spawn visibility.
- Split the dashboard from the engine so the sim can run headless in CI.

## Repository Layout

```text
main.py
requirements.txt
entities/
interfaces/
simulation/
tracking/
ui/
utils/
```

## What This Shows About Me

This project reflects the way I like to work: make the system run, expose the state, then iterate on behavior with fast feedback. I am comfortable moving between simulation logic, autonomy interfaces, noisy sensor data, metrics, and UI instrumentation. I also enjoy the practical debugging loop of seeing an issue in the running system, tracing it through the code, and tightening the architecture until the behavior matches the mission intent.

That is the kind of work I am excited to do on real autonomous maritime systems.
