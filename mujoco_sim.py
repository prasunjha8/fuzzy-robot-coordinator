import argparse
import math
import queue
import shlex
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

from simulator import CHARGE_THRESHOLD, SimulationState


MAX_TASKS = 24
WORLD_SIZE = 10.0
SIMULATION_DT = 0.01
WHEEL_RADIUS = 0.115
WHEEL_BASE = 0.38


def build_world_xml(state):
    robots = []
    colors = {
        "R1": "0.96 0.25 0.28 1",
        "R2": "0.22 0.62 1 1",
        "R3": "0.16 0.85 0.56 1",
    }
    for name, robot in state.robots.items():
        robots.append(
            f"""
      <body name="{name}" pos="{robot['x']} {robot['y']} 0.115">
        <freejoint name="{name}_free"/>
        <geom name="{name}_chassis" type="box" pos="0 0 0.045"
          size="0.23 0.16 0.05" mass="3" rgba="{colors[name]}"/>
        <geom name="{name}_top" type="cylinder" pos="0 0 0.105"
          size="0.09 0.015" contype="0" conaffinity="0" rgba="0.10 0.14 0.20 1"/>
        <body name="{name}_left_wheel" pos="0 0.19 0">
          <joint name="{name}_left_wheel_joint" type="hinge" axis="0 1 0" damping="0.02"/>
          <geom type="cylinder" quat="0.707107 0.707107 0 0"
            size="{WHEEL_RADIUS} 0.035" mass="0.3" rgba="0.08 0.10 0.13 1"/>
        </body>
        <body name="{name}_right_wheel" pos="0 -0.19 0">
          <joint name="{name}_right_wheel_joint" type="hinge" axis="0 1 0" damping="0.02"/>
          <geom type="cylinder" quat="0.707107 0.707107 0 0"
            size="{WHEEL_RADIUS} 0.035" mass="0.3" rgba="0.08 0.10 0.13 1"/>
        </body>
        <site name="{name}_front" pos="0.18 0 0.09" size="0.035" rgba="1 1 1 1"/>
      </body>"""
        )

    task_markers = []
    for slot in range(MAX_TASKS):
        task_markers.append(
            f"""
    <geom name="task_marker_{slot}" type="sphere" pos="-20 -20 -20"
      size="0.19" contype="0" conaffinity="0" rgba="0.96 0.68 0.12 0"/>"""
        )

    charger = state.charger
    return f"""<mujoco model="fuzzy_warehouse">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{SIMULATION_DT}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual>
    <global offwidth="1440" offheight="900"/>
    <quality shadowsize="2048"/>
    <headlight diffuse="0.75 0.78 0.82" ambient="0.25 0.27 0.31" specular="0.15 0.15 0.15"/>
  </visual>
  <default>
    <geom friction="1.2 0.01 0.001" solref="0.01 1" solimp="0.9 0.95 0.001"/>
  </default>
  <worldbody>
    <light directional="true" diffuse="0.85 0.88 0.95" pos="4 4 9" dir="-0.3 -0.3 -1"/>
    <geom name="warehouse_floor" type="plane" size="12 12 0.1" rgba="0.12 0.16 0.21 1"/>
    <geom name="north_wall" type="box" pos="5 10.15 0.55" size="5.2 0.12 0.55" contype="0" conaffinity="0" rgba="0.18 0.24 0.32 1"/>
    <geom name="south_wall" type="box" pos="5 -0.15 0.55" size="5.2 0.12 0.55" contype="0" conaffinity="0" rgba="0.18 0.24 0.32 1"/>
    <geom name="west_wall" type="box" pos="-0.15 5 0.55" size="0.12 5.2 0.55" contype="0" conaffinity="0" rgba="0.18 0.24 0.32 1"/>
    <geom name="east_wall" type="box" pos="10.15 5 0.55" size="0.12 5.2 0.55" contype="0" conaffinity="0" rgba="0.18 0.24 0.32 1"/>

    <geom name="charger_pad" type="box" pos="{charger['x']} {charger['y']} 0.035"
      size="0.58 0.58 0.035" contype="0" conaffinity="0" rgba="0.05 0.73 0.96 1"/>
    <geom name="charger_core" type="cylinder" pos="{charger['x']} {charger['y']} 0.08"
      size="0.22 0.04" contype="0" conaffinity="0" rgba="0.42 0.91 1 1"/>
    <site name="charger_label" pos="{charger['x']} {charger['y']} 0.25"
      type="box" size="0.42 0.08 0.01" rgba="0.1 0.9 1 1"/>

    <geom name="rack_1" type="box" pos="3.3 8.2 0.35" size="0.65 0.8 0.35" contype="0" conaffinity="0" rgba="0.28 0.36 0.46 1"/>
    <geom name="rack_2" type="box" pos="7.8 5.8 0.35" size="0.7 0.65 0.35" contype="0" conaffinity="0" rgba="0.28 0.36 0.46 1"/>
    <geom name="rack_3" type="box" pos="4.5 3.1 0.35" size="0.75 0.6 0.35" contype="0" conaffinity="0" rgba="0.28 0.36 0.46 1"/>
    {''.join(robots)}
    {''.join(task_markers)}
  </worldbody>
  <actuator>
    {''.join(
        f'<velocity name="{robot_name}_left_motor" joint="{robot_name}_left_wheel_joint" '
        'kv="1.5" ctrllimited="true" ctrlrange="-12 12" '
        'forcelimited="true" forcerange="-2 2"/>'
        f'<velocity name="{robot_name}_right_motor" joint="{robot_name}_right_wheel_joint" '
        'kv="1.5" ctrllimited="true" ctrlrange="-12 12" '
        'forcelimited="true" forcerange="-2 2"/>'
        for robot_name in state.robots
    )}
  </actuator>
</mujoco>"""


def _clamp(value, magnitude):
    return max(-magnitude, min(magnitude, value))


def _wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def set_robot_controls(model, data, state):
    for robot_name in state.robots:
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot_name}_free"
        )
        qpos_address = model.jnt_qposadr[joint_id]
        target = state.target_for_robot(robot_name)

        position = data.qpos[qpos_address : qpos_address + 3]
        left_motor = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{robot_name}_left_motor"
        )
        right_motor = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{robot_name}_right_motor"
        )

        if target is not None:
            error_x = target["x"] - position[0]
            error_y = target["y"] - position[1]
            desired_heading = math.atan2(error_y, error_x)
            qw, qx, qy, qz = data.qpos[qpos_address + 3 : qpos_address + 7]
            heading = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
            heading_error = _wrap_angle(desired_heading - heading)
            distance = math.hypot(error_x, error_y)
            linear_speed = min(0.8, distance * 1.2) * max(0.0, math.cos(heading_error))
            angular_speed = _clamp(2.5 * heading_error, 1.8)
            left_speed = (linear_speed - angular_speed * WHEEL_BASE / 2.0) / WHEEL_RADIUS
            right_speed = (linear_speed + angular_speed * WHEEL_BASE / 2.0) / WHEEL_RADIUS
            data.ctrl[left_motor] = _clamp(left_speed, 12.0)
            data.ctrl[right_motor] = _clamp(right_speed, 12.0)
        else:
            data.ctrl[left_motor] = 0.0
            data.ctrl[right_motor] = 0.0


def sync_state_from_physics(model, data, state, previous_positions):
    for robot_name, robot in state.robots.items():
        joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, f"{robot_name}_free"
        )
        qpos_address = model.jnt_qposadr[joint_id]
        x, y = data.qpos[qpos_address : qpos_address + 2]
        old_x, old_y = previous_positions[robot_name]
        traveled = math.hypot(x - old_x, y - old_y)
        previous_positions[robot_name] = (float(x), float(y))
        state.set_robot_pose(robot_name, x, y, battery_used=traveled * 1.5)


def sync_task_markers(model, state):
    marker_colors = {
        "pending": (0.98, 0.68, 0.12, 1.0),
        "done": (0.20, 0.86, 0.48, 0.72),
    }
    for slot, (task_name, task) in enumerate(state.tasks.items()):
        if slot >= MAX_TASKS:
            break
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"task_marker_{slot}"
        )
        model.geom_pos[geom_id] = (task["x"], task["y"], 0.22)
        model.geom_rgba[geom_id] = marker_colors["done" if task["done"] else "pending"]

    for slot in range(len(state.tasks), MAX_TASKS):
        geom_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_GEOM, f"task_marker_{slot}"
        )
        model.geom_rgba[geom_id, 3] = 0.0


def sync_viewer_labels(viewer, state):
    scene = viewer.user_scn
    labels = [
        ("CHARGER - recharge dock", (state.charger["x"], state.charger["y"], 0.58), (0.2, 0.95, 1.0, 1.0))
    ]
    for robot_name, robot in state.robots.items():
        destination = state.target_for_robot(robot_name)
        if destination is None:
            detail = "idle"
        elif robot["charging"]:
            detail = f"charging {robot['battery']:.0f}%"
        else:
            detail = f"to {robot['task']}"
        labels.append(
            (
                f"{robot_name} - {detail}",
                (robot["x"], robot["y"], 0.65),
                (0.92, 0.96, 1.0, 1.0),
            )
        )
    for task_name, task in state.tasks.items():
        if task["done"]:
            continue
        labels.append(
            (
                f"{task_name} - {task['description']}",
                (task["x"], task["y"], 0.48),
                (1.0, 0.86, 0.45, 1.0),
            )
        )

    scene.ngeom = 0
    for label, position, color in labels[: scene.maxgeom]:
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_LABEL,
            np.ones(3, dtype=np.float64),
            np.asarray(position, dtype=np.float64),
            np.eye(3, dtype=np.float64).ravel(),
            np.asarray(color, dtype=np.float32),
        )
        geom.label = label[:100]
        scene.ngeom += 1


def print_task_legend(state):
    print("\nFUZZY WAREHOUSE - MuJoCo")
    print("Yellow spheres are open tasks; green spheres are completed tasks.")
    print("Cyan pad is the charging station. Robots autonomously choose feasible tasks.")
    print("\nInitial jobs:")
    for task_name, task in state.tasks.items():
        print(
            f"  {task_name}: {task['description']} "
            f"(load {task['payload']} kg, urgency {task['urgency']}/100, "
            f"location {task['x']:.1f}, {task['y']:.1f})"
        )
    print(f"\nR2 starts at low battery and will demonstrate charging at {state.charger}.")
    print("Interactive terminal commands while MuJoCo runs:")
    print("  task <x> <y> [payload_kg] [urgency] [description...]")
    print("  status | help | quit")


def _input_loop(commands, stop_event):
    while not stop_event.is_set():
        try:
            line = input("warehouse> ")
        except EOFError:
            return
        commands.put(line.strip())
        if line.strip().lower() == "quit":
            return


def handle_command(line, state):
    try:
        parts = shlex.split(line)
    except ValueError as error:
        print(f"Could not parse command: {error}")
        return False
    if not parts:
        return False

    command = parts[0].lower()
    if command in {"quit", "exit"}:
        return True
    if command == "help":
        print("Use: task <x 0-10> <y 0-10> [payload kg] [urgency 0-100] [description...]")
        print("Other commands: status, help, quit")
        return False
    if command == "status":
        for name, robot in state.robots.items():
            assignment = robot["task"] or "none"
            print(
                f"  {name}: {robot['status']}, battery={robot['battery']:.1f}%, "
                f"task={assignment}, position=({robot['x']:.1f}, {robot['y']:.1f})"
            )
        for name, task in state.tasks.items():
            state_label = "done" if task["done"] else "open"
            print(f"  {name}: {state_label} — {task['description']}")
        return False
    if command != "task":
        print("Unknown command. Type 'help' for commands.")
        return False

    if len(parts) < 3:
        print("Task command needs a location: task <x> <y> [payload] [urgency] [description...]")
        return False
    if len(state.tasks) >= MAX_TASKS:
        print(f"Task limit reached ({MAX_TASKS}); reset to create more.")
        return False
    try:
        x, y = float(parts[1]), float(parts[2])
    except ValueError:
        print("Task coordinates must be numeric.")
        return False
    remainder = parts[3:]
    payload = 5
    urgency = 70
    if remainder:
        try:
            payload = int(remainder[0])
        except ValueError:
            pass
        else:
            remainder = remainder[1:]
            if remainder:
                try:
                    urgency = int(remainder[0])
                except ValueError:
                    pass
                else:
                    remainder = remainder[1:]
    if not (0.0 <= x <= WORLD_SIZE and 0.0 <= y <= WORLD_SIZE):
        print("Task coordinates must be within the 0–10 m warehouse.")
        return False
    if payload <= 0 or urgency < 0 or urgency > 100:
        print("Payload must be positive and urgency must be between 0 and 100.")
        return False
    description = " ".join(remainder) if remainder else "User-created delivery task"
    task_name = state.add_task(x, y, payload, urgency, description)
    print(f"Added {task_name}: {description} at ({x:.1f}, {y:.1f}); robots will allocate it.")
    return False


def run(smoke_steps=0):
    state = SimulationState()
    model = mujoco.MjModel.from_xml_string(build_world_xml(state))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    previous_positions = {
        name: (robot["x"], robot["y"]) for name, robot in state.robots.items()
    }
    sync_task_markers(model, state)
    mujoco.mj_forward(model, data)

    if smoke_steps:
        initial_positions = {
            name: (robot["x"], robot["y"]) for name, robot in state.robots.items()
        }
        for _ in range(smoke_steps):
            state.tick(dt=SIMULATION_DT, move_robots=False)
            set_robot_controls(model, data, state)
            mujoco.mj_step(model, data)
            sync_state_from_physics(model, data, state, previous_positions)
        moved = any(
            math.hypot(
                state.robots[name]["x"] - initial_positions[name][0],
                state.robots[name]["y"] - initial_positions[name][1],
            )
            > 0.05
            for name in initial_positions
        )
        if not moved:
            raise RuntimeError("MuJoCo smoke test did not move any robot.")
        print(f"MuJoCo smoke test passed: robots moved in {smoke_steps} physics steps.")
        return

    print_task_legend(state)
    commands = queue.Queue()
    stop_event = threading.Event()
    command_thread = threading.Thread(
        target=_input_loop, args=(commands, stop_event), daemon=True
    )
    command_thread.start()

    viewer = mujoco.viewer.launch_passive(model, data, show_left_ui=False)
    viewer.cam.lookat[:] = (5.0, 5.0, 0.0)
    viewer.cam.distance = 15.0
    viewer.cam.azimuth = 90.0
    viewer.cam.elevation = -48.0
    try:
        while viewer.is_running() and not stop_event.is_set():
            while not commands.empty():
                if handle_command(commands.get_nowait(), state):
                    stop_event.set()
                    break

            state.tick(dt=SIMULATION_DT, move_robots=False)
            sync_task_markers(model, state)
            mujoco.mj_forward(model, data)
            set_robot_controls(model, data, state)
            mujoco.mj_step(model, data)
            sync_state_from_physics(model, data, state, previous_positions)
            sync_viewer_labels(viewer, state)
            viewer.sync()
            time.sleep(SIMULATION_DT)
    finally:
        stop_event.set()
        viewer.close()
        if sys.stdin.isatty():
            print("MuJoCo warehouse simulation closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Autonomous fuzzy warehouse in MuJoCo")
    parser.add_argument(
        "--smoke-test",
        type=int,
        metavar="STEPS",
        help="run headless physics steps and verify robot motion without opening a viewer",
    )
    args = parser.parse_args()
    run(smoke_steps=args.smoke_test or 0)
