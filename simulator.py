import json
import math
import os
import threading
import time
from collections import OrderedDict
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

from fuzzy_robot.main import combined_suitability


CHARGER = {"x": 1.0, "y": 9.0}
CHARGE_THRESHOLD = 25.0
CHARGE_RESUME_LEVEL = 85.0
MAX_TASKS = 24
WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")
WEB_PORT = 8000


class SimulationState:
    """Shared task allocation and battery policy for both simulator front ends."""

    def __init__(self):
        self.lock = threading.RLock()
        self.charger = dict(CHARGER)
        self.robots = OrderedDict(
            [
                ("R1", self._robot(1.5, 1.0, 82, 10, 1)),
                ("R2", self._robot(8.5, 2.0, 22, 15, 2)),
                ("R3", self._robot(5.0, 8.2, 62, 8, 0)),
            ]
        )
        self.tasks = OrderedDict(
            [
                (
                    "T1",
                    self._task(8.0, 8.0, 5, 90, "Deliver a package to the north shelf", "B"),
                ),
                (
                    "T2",
                    self._task(2.5, 6.5, 7, 55, "Restock the west aisle", "A"),
                ),
                (
                    "T3",
                    self._task(5.5, 5.0, 12, 100, "Move a heavy crate to the central bay", "C"),
                ),
                (
                    "T4",
                    self._task(2.0, 2.5, 4, 70, "Collect a return from the south station", "A"),
                ),
            ]
        )
        self.inventory = OrderedDict(
            [
                ("A", {"stock": 20, "demand": 50, "status": "low"}),
                ("B", {"stock": 45, "demand": 36, "status": "normal"}),
                ("C", {"stock": 62, "demand": 24, "status": "high"}),
            ]
        )
        self.completed = []
        self.task_counter = 5
        self.autonomous_cycle = 0
        self.last_tick = 0

    @staticmethod
    def _robot(x, y, battery, capacity, workload):
        return {
            "x": float(x),
            "y": float(y),
            "battery": float(battery),
            "payload_capacity": capacity,
            "workload": workload,
            "speed": 1.1,
            "task": None,
            "charging": battery <= CHARGE_THRESHOLD,
            "status": "charging" if battery <= CHARGE_THRESHOLD else "idle",
        }

    @staticmethod
    def _task(x, y, payload, urgency, description, inventory_item=None):
        return {
            "x": float(x),
            "y": float(y),
            "payload": int(payload),
            "urgency": int(urgency),
            "description": description,
            "inventory_item": inventory_item,
            "done": False,
        }

    def add_task(self, x, y, payload=5, urgency=70, description="Deliver a package"):
        with self.lock:
            label = f"T{self.task_counter}"
            self.task_counter += 1
            self.tasks[label] = self._task(x, y, payload, urgency, description)
            return label

    def _score(self, robot, task):
        return combined_suitability(
            {
                "position": [robot["x"], robot["y"]],
                "battery": robot["battery"],
                "payload_capacity": robot["payload_capacity"],
                "workload": robot["workload"],
            },
            {
                "position": [task["x"], task["y"]],
                "payload": task["payload"],
                "urgency": task["urgency"],
            },
        )

    def _dispatch_tasks(self):
        claimed = {robot["task"] for robot in self.robots.values() if robot["task"]}
        available = [
            name
            for name, robot in self.robots.items()
            if robot["task"] is None
            and not robot["charging"]
            and robot["battery"] > CHARGE_THRESHOLD
        ]
        pending = [
            name
            for name, task in self.tasks.items()
            if not task["done"] and name not in claimed
        ]

        priorities = {}
        for robot_name in available:
            robot = self.robots[robot_name]
            for task_name in pending:
                task = self.tasks[task_name]
                suitability = self._score(robot, task)
                if suitability is not None:
                    priorities[(robot_name, task_name)] = suitability * (
                        0.5 + task["urgency"] / 200.0
                    )

        best_priority = -1.0
        best_assignment = {}

        def search(robot_index, used_tasks, assignment, total_priority):
            nonlocal best_priority, best_assignment
            if robot_index == len(available):
                if total_priority > best_priority:
                    best_priority = total_priority
                    best_assignment = assignment.copy()
                return

            robot_name = available[robot_index]
            for task_name in pending:
                priority = priorities.get((robot_name, task_name))
                if priority is None or task_name in used_tasks:
                    continue
                assignment[robot_name] = task_name
                used_tasks.add(task_name)
                search(
                    robot_index + 1,
                    used_tasks,
                    assignment,
                    total_priority + priority,
                )
                used_tasks.remove(task_name)
                del assignment[robot_name]

            search(robot_index + 1, used_tasks, assignment, total_priority)

        search(0, set(), {}, 0.0)
        for robot_name, task_name in best_assignment.items():
            self.robots[robot_name]["task"] = task_name
            self.robots[robot_name]["status"] = "working"

    def _queue_autonomous_task(self):
        if len(self.tasks) >= MAX_TASKS or any(not task["done"] for task in self.tasks.values()):
            return
        destinations = [
            (8.4, 7.8, 4, 68, "Autonomous delivery run"),
            (2.0, 5.8, 3, 76, "Autonomous west-aisle restock"),
            (7.5, 2.0, 5, 60, "Autonomous south-bay pickup"),
            (5.0, 4.8, 6, 82, "Autonomous central-bay transfer"),
        ]
        x, y, payload, urgency, description = destinations[
            self.autonomous_cycle % len(destinations)
        ]
        self.autonomous_cycle += 1
        self.add_task(
            x,
            y,
            payload,
            urgency,
            f"{description} #{self.autonomous_cycle}",
        )

    def target_for_robot(self, robot_name):
        robot = self.robots[robot_name]
        if robot["charging"]:
            return dict(self.charger)
        task_name = robot["task"]
        if task_name is None or task_name not in self.tasks:
            return None
        task = self.tasks[task_name]
        return None if task["done"] else {"x": task["x"], "y": task["y"]}

    def set_robot_pose(self, robot_name, x, y, battery_used=0.0):
        with self.lock:
            robot = self.robots[robot_name]
            robot["x"] = float(x)
            robot["y"] = float(y)
            robot["battery"] = max(0.0, robot["battery"] - max(0.0, battery_used))

    def tick(self, dt=0.04, move_robots=True):
        with self.lock:
            self.last_tick += 1

            for robot_name, robot in self.robots.items():
                if robot["battery"] <= CHARGE_THRESHOLD:
                    robot["charging"] = True
                    robot["task"] = None

                if robot["charging"]:
                    robot["status"] = "charging"
                    distance_to_dock = math.hypot(
                        self.charger["x"] - robot["x"],
                        self.charger["y"] - robot["y"],
                    )
                    if move_robots and distance_to_dock > 0.3:
                        step = min(robot["speed"] * dt, distance_to_dock)
                        robot["x"] += (
                            (self.charger["x"] - robot["x"]) / distance_to_dock * step
                        )
                        robot["y"] += (
                            (self.charger["y"] - robot["y"]) / distance_to_dock * step
                        )
                        robot["battery"] = max(0.0, robot["battery"] - step * 0.5)
                    elif distance_to_dock <= 0.45:
                        robot["battery"] = min(100.0, robot["battery"] + 12.0 * dt)
                        if robot["battery"] >= CHARGE_RESUME_LEVEL:
                            robot["charging"] = False
                            robot["status"] = "idle"
                    continue

                task_name = robot["task"]
                if task_name is not None:
                    task = self.tasks[task_name]
                    distance_to_task = math.hypot(task["x"] - robot["x"], task["y"] - robot["y"])
                    if distance_to_task <= 0.45:
                        task["done"] = True
                        self.completed.append({"task": task_name, "robot": robot_name})
                        inventory_item = task["inventory_item"]
                        if inventory_item is not None:
                            item = self.inventory[inventory_item]
                            item["stock"] = min(100, item["stock"] + task["payload"])
                        robot["task"] = None
                        robot["status"] = "idle"
                        robot["battery"] = max(0.0, robot["battery"] - 4.0)
                        continue
                    robot["status"] = "working"
                else:
                    robot["status"] = "idle"

            self._queue_autonomous_task()
            self._dispatch_tasks()

            if move_robots:
                for robot_name, robot in self.robots.items():
                    target = self.target_for_robot(robot_name)
                    if target is None or robot["charging"]:
                        continue
                    dx = target["x"] - robot["x"]
                    dy = target["y"] - robot["y"]
                    distance_to_target = math.hypot(dx, dy)
                    if distance_to_target <= 0.45:
                        continue
                    step = min(robot["speed"] * dt, distance_to_target)
                    robot["x"] += dx / distance_to_target * step
                    robot["y"] += dy / distance_to_target * step
                    robot["battery"] = max(0.0, robot["battery"] - step * 0.5)

            for item in self.inventory.values():
                if item["stock"] <= 30:
                    item["status"] = "low"
                elif item["stock"] < 60:
                    item["status"] = "normal"
                else:
                    item["status"] = "high"

    def snapshot(self):
        with self.lock:
            robots = {
                name: {
                    **robot,
                    "x": round(robot["x"], 3),
                    "y": round(robot["y"], 3),
                    "battery": round(robot["battery"], 2),
                }
                for name, robot in self.robots.items()
            }
            tasks = {
                name: {**task, "x": round(task["x"], 3), "y": round(task["y"], 3)}
                for name, task in self.tasks.items()
            }
            return {
                "robots": robots,
                "tasks": tasks,
                "inventory": self.inventory,
                "charger": self.charger,
                "completed": self.completed[-10:],
                "tick": self.last_tick,
            }


SIM = SimulationState()


class WebHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=WEB_DIR, **kwargs)

    def do_GET(self):
        if urlparse(self.path).path == "/api/state":
            self._send_json(SIM.snapshot())
            return
        super().do_GET()

    def do_POST(self):
        global SIM
        endpoint = urlparse(self.path).path
        if endpoint == "/api/reset":
            SIM = SimulationState()
            self._send_json({"status": "reset"})
            return
        if endpoint != "/api/task":
            self.send_error(404)
            return

        try:
            length = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            x = float(body["x"])
            y = float(body["y"])
            payload = int(body.get("payload", 5))
            urgency = int(body.get("urgency", 70))
            description = str(body.get("description", "User-created delivery task"))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self._send_json({"error": "Task requires numeric x/y and valid optional payload/urgency."}, 400)
            return

        if not (0 <= x <= 10 and 0 <= y <= 10 and payload > 0 and 0 <= urgency <= 100):
            self._send_json({"error": "Coordinates must be 0-10, payload positive, urgency 0-100."}, 400)
            return
        if len(SIM.tasks) >= 24:
            self._send_json({"error": "Task limit reached; reset the simulation to add more."}, 409)
            return
        task_name = SIM.add_task(x, y, payload, urgency, description[:100])
        self._send_json({"status": "ok", "task": task_name})

    def _send_json(self, data, status=200):
        payload = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, format, *args):
        return


def run_web_server(host="0.0.0.0", port=WEB_PORT):
    def advance_simulation():
        while True:
            SIM.tick(dt=0.04)
            time.sleep(0.04)

    threading.Thread(target=advance_simulation, daemon=True).start()
    server = ThreadingHTTPServer((host, port), WebHandler)
    print(f"Browser dashboard running at http://{host}:{port}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping browser dashboard.")
    finally:
        server.server_close()


if __name__ == "__main__":
    run_web_server()
