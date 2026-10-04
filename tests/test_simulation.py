import unittest

from simulator import CHARGE_THRESHOLD, SimulationState


class SimulationStateTests(unittest.TestCase):
    def test_seed_tasks_have_descriptions(self):
        state = SimulationState()
        self.assertTrue(all(task["description"] for task in state.tasks.values()))

    def test_dispatch_assigns_distinct_feasible_tasks(self):
        state = SimulationState()
        state.tick(move_robots=False)

        assigned_tasks = [
            robot["task"] for robot in state.robots.values() if robot["task"] is not None
        ]
        self.assertEqual(len(assigned_tasks), len(set(assigned_tasks)))
        self.assertNotIn("T3", assigned_tasks)
        self.assertIsNone(state.robots["R2"]["task"])
        self.assertTrue(state.robots["R2"]["charging"])

    def test_dispatch_maximizes_total_pair_score(self):
        state = SimulationState()
        for task in state.tasks.values():
            task["done"] = True

        state.robots["R1"]["charging"] = False
        state.robots["R2"]["charging"] = False
        state.robots["R2"]["battery"] = 80
        state.robots["R3"]["charging"] = True
        state.add_task(3, 3, urgency=100, description="A")
        state.add_task(6, 6, urgency=100, description="B")

        pair_scores = {
            (1, "A"): 0.90,
            (1, "B"): 0.80,
            (2, "A"): 0.85,
            (2, "B"): 0.10,
        }
        state._score = lambda robot, task: pair_scores.get(
            (robot["workload"], task["description"])
        )

        state._dispatch_tasks()

        self.assertEqual(state.robots["R1"]["task"], "T6")
        self.assertEqual(state.robots["R2"]["task"], "T5")

    def test_low_battery_robot_targets_the_charger(self):
        state = SimulationState()
        robot = state.robots["R2"]

        self.assertLessEqual(robot["battery"], CHARGE_THRESHOLD)
        self.assertEqual(state.target_for_robot("R2"), state.charger)

    def test_custom_task_is_added_with_its_description(self):
        state = SimulationState()
        task_name = state.add_task(4.0, 6.0, 3, 88, "Deliver a first-aid kit")

        self.assertEqual(task_name, "T5")
        self.assertEqual(state.tasks[task_name]["description"], "Deliver a first-aid kit")
        self.assertEqual(state.tasks[task_name]["urgency"], 88)

    def test_browser_simulation_moves_robot_towards_its_task(self):
        state = SimulationState()
        state.tick(dt=0.04)
        initial_position = (state.robots["R1"]["x"], state.robots["R1"]["y"])

        for _ in range(30):
            state.tick(dt=0.04)

        final_position = (state.robots["R1"]["x"], state.robots["R1"]["y"])
        self.assertNotEqual(initial_position, final_position)

    def test_autonomous_followup_job_is_queued_when_seed_jobs_finish(self):
        state = SimulationState()
        for task in state.tasks.values():
            task["done"] = True

        state.tick(move_robots=False)

        self.assertEqual(state.tasks["T5"]["description"], "Autonomous delivery run #1")
        self.assertFalse(state.tasks["T5"]["done"])


if __name__ == "__main__":
    unittest.main()
