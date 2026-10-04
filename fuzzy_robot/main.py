import math

robots = {
    "R1": {
        "position": [0, 0],
        "battery": 80,
        "payload_capacity": 10,
        "workload": 1
    },
    "R2": {
        "position": [8, 2],
        "battery": 45,
        "payload_capacity": 15,
        "workload": 2
    },
    "R3": {
        "position": [5, 5],
        "battery": 60,
        "payload_capacity": 8,
        "workload": 0
    }
}

tasks = {
    "T1": {
        "position": [2, 3],
        "payload": 5,
        "urgency": 90
    },
    "T2": {
        "position": [7, 1],
        "payload": 8,
        "urgency": 40
    },
    "T3": {
        "position": [4, 4],
        "payload": 12,
        "urgency": 100
    },
    "T4": {
        "position": [1, 6],
        "payload": 3,
        "urgency": 70
}
}

def distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

def triangular(x,a,b,c):
    if x <= a or x >= c:
        return 0
    elif a <= x < b:
        return (x-a)/(b-a)
    else:
        return (c-x)/(c-b)

def left_shoulder(x, a, b):
    if x <= a:
        return 1.0
    elif x >= b:
        return 0.0
    else:
        return (b - x) / (b - a)


def right_shoulder(x, a, b):
    if x <= a:
        return 0.0
    elif x >= b:
        return 1.0
    else:
        return (x - a) / (b - a)

def battery_low(x):
    return left_shoulder(x, 30, 50)


def battery_medium(x):
    return triangular(x, 30, 50, 70)


def battery_high(x):
    return right_shoulder(x, 50, 70)

def distance_near(x):
    return left_shoulder(x, 2, 5)


def distance_medium(x):
    return triangular(x, 3, 6, 9)


def distance_far(x):
    return right_shoulder(x, 7, 11)

def workload_low(x):
    return left_shoulder(x, 0.5, 1.5)

def workload_medium(x):
    return triangular(x, 0.5, 1.5, 2.5)

def workload_high(x):
    return right_shoulder(x, 1.5, 2.5)


def load_low(x):
    return left_shoulder(x, 0.2, 0.4)


def load_medium(x):
    return triangular(x, 0.3, 0.5, 0.7)


def load_high(x):
    return right_shoulder(x, 0.6, 0.8)

def urgency_low(x):
    return left_shoulder(x, 20, 40)


def urgency_medium(x):
    return triangular(x, 25, 50, 75)


def urgency_high(x):
    return triangular(x, 60, 80, 95)


def urgency_critical(x):
    return right_shoulder(x, 85, 100)


def fuzzify_robot_task(robot, task):

    distance_value = distance(
        robot["position"],
        task["position"]
    )

    load_ratio = task["payload"] / robot["payload_capacity"]

    if load_ratio > 1:
        return None

    result = {

        "battery": {
            "low": battery_low(robot["battery"]),
            "medium": battery_medium(robot["battery"]),
            "high": battery_high(robot["battery"])
        },

        "distance": {
            "near": distance_near(distance_value),
            "medium": distance_medium(distance_value),
            "far": distance_far(distance_value)
        },

        "workload": {
            "light": workload_low(robot["workload"]),
            "moderate": workload_medium(robot["workload"]),
            "heavy": workload_high(robot["workload"])
        },

        "load": {
            "low": load_low(load_ratio),
            "medium": load_medium(load_ratio),
            "high": load_high(load_ratio)
        },

        "urgency": {
            "low": urgency_low(task["urgency"]),
            "medium": urgency_medium(task["urgency"]),
            "high": urgency_high(task["urgency"]),
            "critical": urgency_critical(task["urgency"])
        }
    }

    return result

rules = [
    # Favor a capable, nearby robot carrying a light load.
    {
        "conditions": [("battery", "high"), ("distance", "near"), ("load", "low")],
        "output": {"bias": 0.70, "battery": 0.20, "distance": -0.10, "load": -0.10, "workload": -0.05, "urgency": 0.10}
    },
    # Favor a capable robot at medium distance with a typical load.
    {
        "conditions": [("battery", "high"), ("distance", "medium"), ("load", "medium")],
        "output": {"bias": 0.60, "battery": 0.20, "distance": -0.10, "load": -0.10, "workload": -0.05, "urgency": 0.10}
    },
    # Allow a moderately charged, nearby robot to take a typical load.
    {
        "conditions": [("battery", "medium"), ("distance", "near"), ("load", "medium")],
        "output": {"bias": 0.55, "battery": 0.15, "distance": -0.05, "load": -0.10, "workload": -0.05, "urgency": 0.15}
    },
    # Penalize a low-battery robot facing a far trip.
    {
        "conditions": [("battery", "low"), ("distance", "far")],
        "output": {"bias": 0.20, "battery": 0.10, "distance": -0.20, "load": -0.10, "workload": -0.10, "urgency": 0.10}
    },
    # Penalize a low-battery robot assigned a high load.
    {
        "conditions": [("load", "high"), ("battery", "low")],
        "output": {"bias": 0.15, "battery": 0.10, "distance": -0.05, "load": -0.20, "workload": -0.10, "urgency": 0.10}
    },
    # Prioritize urgent work when a capable robot is nearby.
    {
        "conditions": [("urgency", "critical"), ("battery", "high"), ("distance", "near")],
        "output": {"bias": 0.75, "battery": 0.15, "distance": -0.05, "load": -0.05, "workload": -0.05, "urgency": 0.20}
    },
    # Penalize a heavily loaded robot when the task is far away.
    {
        "conditions": [("workload", "heavy"), ("distance", "far")],
        "output": {"bias": 0.20, "battery": 0.10, "distance": -0.15, "load": -0.05, "workload": -0.20, "urgency": 0.10}
    },
    # Favor medium-distance urgent work for a moderately charged robot.
    {
        "conditions": [("urgency", "high"), ("battery", "medium"), ("distance", "medium")],
        "output": {"bias": 0.55, "battery": 0.15, "distance": -0.10, "load": -0.05, "workload": -0.05, "urgency": 0.20}
    }
]


def rule_strength(rule, fuzzy_data):
    return min(
        fuzzy_data[variable][label]
        for variable, label in rule["conditions"]
    )


def normalized_inputs(robot, task):
    distance_value = distance(robot["position"], task["position"])

    return {
        "battery": robot["battery"] / 100.0,
        "distance": min(distance_value / 14.0, 1.0),
        "load": task["payload"] / robot["payload_capacity"],
        "workload": min(robot["workload"] / 3.0, 1.0),
        "urgency": task["urgency"] / 100.0
    }


def rule_output(rule, inputs):
    parameters = rule["output"]
    score = parameters["bias"] + sum(
        parameters[variable] * value
        for variable, value in inputs.items()
    )
    return max(0.0, min(1.0, score))


def fuzzy_suitability(robot, task):
    load_ratio = task["payload"] / robot["payload_capacity"]
    if load_ratio > 1.0:
        return None

    fuzzy_data = fuzzify_robot_task(robot, task)
    if fuzzy_data is None:
        return None

    inputs = normalized_inputs(robot, task)
    weighted_score = 0.0
    total_strength = 0.0

    for rule in rules:
        strength = rule_strength(rule, fuzzy_data)
        if strength == 0:
            continue

        weighted_score += strength * rule_output(rule, inputs)
        total_strength += strength

    if total_strength == 0:
        return 0.0

    return weighted_score / total_strength


def build_suitability_matrix():
    matrix = {}

    for robot_name, robot in robots.items():
        matrix[robot_name] = {}
        for task_name, task in tasks.items():
            score = fuzzy_suitability(robot, task)
            matrix[robot_name][task_name] = None if score is None else score

    return matrix


def print_matrix(matrix):
    print("\nFuzzy robot-task suitability (0 to 1)")
    print(f"{'Robot':<10}" + "".join(f"{task:>10}" for task in tasks))
    print("-" * (10 + 10 * len(tasks)))

    for robot_name, scores in matrix.items():
        values = "".join(
            f"{'INF' if scores[task_name] is None else f'{scores[task_name]:.3f}':>10}"
            for task_name in tasks
        )
        print(f"{robot_name:<10}{values}")


if __name__ == "__main__":
    print_matrix(build_suitability_matrix())