# Fuzzy Robot Coordinator

An explainable fuzzy-logic task allocator connected to a small autonomous warehouse simulation. The **primary demo is MuJoCo**: three motorized differential-drive robots move under MuJoCo physics, choose feasible jobs, and return to a visible charging dock when their battery is low. The browser dashboard is an optional local companion.

## Run the MuJoCo simulation on macOS

From the repository root, create the environment once if you do not already have one:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Launch the primary demo with MuJoCo's macOS viewer launcher:

```bash
.venv/bin/mjpython mujoco_sim.py
```

Or double-click `run_mujoco.command` in Finder. It creates the project virtual environment and installs MuJoCo if needed, then opens the same viewer.

The MuJoCo window opens on the warehouse. R2 starts with a low battery and autonomously drives to the cyan charging pad; the other robots begin assigning themselves feasible jobs. Yellow markers are open jobs and green markers are completed jobs. The terminal prints every job's full description and live labels in the 3D view show the robot's current destination or charging state. After the starter jobs are complete, the system queues follow-up delivery runs so the demo keeps moving.

While the viewer is open, enter commands in the same terminal:

```text
task 7 8 3 90 Deliver a medicine kit to the north shelf
status
help
quit
```

Task coordinates are in the 0-10 m warehouse. Payload must be positive and urgency ranges from 0 to 100. The allocator respects robot payload capacity and gives each open task to at most one robot. A robot at or below 25% battery suspends its job, charges to 85%, then returns to the pending work queue. The MuJoCo scene reserves 24 task markers per run; reset the simulation to start a fresh run after reaching that limit.

To verify MuJoCo physics and autonomous movement without opening a window:

```bash
.venv/bin/mjpython mujoco_sim.py --smoke-test 500
```

## Optional browser dashboard

The dashboard is an additional, lightweight visualization and task-entry surface:

```bash
.venv/bin/python simulator.py
```

Open http://localhost:8000. Enter the job description, coordinates, payload, and urgency, or click the warehouse map to select its coordinates before submitting.

This companion server is for local development and has no authentication; do not expose it directly to the public internet.

## Decision engine

The Python prototype computes fuzzy memberships for battery, distance, workload, payload load ratio, and task urgency. Eight Sugeno-style rules produce an interpretable suitability score, and a weighted mathematical baseline provides a comparison. The allocator maximizes total urgency-weighted suitability subject to one-task-per-robot, one-robot-per-task, and payload-capacity constraints. See [MATHEMATICAL_ENGINE.md](./MATHEMATICAL_ENGINE.md) for the membership formulas, rule consequents, objective, constraints, and MuJoCo controller equations.

## Why I built this

This weekend, I wanted to try something new: I used MuJoCo for the first time and applied ideas from my theoretical subjects to a small robotics project. I am exploring how fuzzy logic and mathematical decision-making can help coordinate robots, then using simulation to see those ideas in motion.

## Tests

```bash
.venv/bin/python -m unittest discover -s tests -v
```
