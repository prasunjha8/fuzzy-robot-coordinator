# Fuzzy Robot Coordinator

A fuzzy-logic decision engine for evaluating how suitable each robot is for each task. The project aims to grow into a multi-robot coordination system, with robot behavior visualized in MuJoCo.

## Project Aim

Build an explainable system that uses robot and task conditions to estimate robot-task suitability. The longer-term goal is to use these scores to guide task assignments and simulate the resulting robot behavior.

## Current Status

The current prototype is a Python Sugeno-style fuzzy decision engine. It:

- Calculates fuzzy memberships for battery, distance, workload, payload load ratio, and task urgency.
- Evaluates eight hand-written Sugeno rules and combines their outputs into a suitability score from 0 to 1.
- Rejects assignments whose task payload exceeds the robot's capacity (`INF` in the printed matrix).
- Prints a robot-task suitability matrix.

The current rule base is intentionally small and does not cover every input combination. A score of `0.000` can mean no rule fired; it does not necessarily mean the assignment is physically impossible. MuJoCo simulation and automatic task assignment are not implemented yet.

## Run

Requires Python 3. Run from the repository root:

```bash
python fuzzy_robot/main.py
```

The project currently uses only Python's standard library.

## Planned Direction

1. Improve and validate fuzzy rule coverage and suitability behavior.
2. Add an assignment method that respects infeasible robot-task pairs.
3. Connect the decision engine to a robot simulation, with MuJoCo as the intended option.
4. Visualize robots, tasks, and the resulting assignments.
