import unittest

from fuzzy_robot.main import (
    assign_tasks_to_robots,
    build_suitability_matrix,
    combined_suitability,
    load_high,
    load_low,
    load_medium,
    mathematical_suitability,
    robots,
    select_best_robot_for_task,
    tasks,
    triangular,
)


class MembershipFunctionTests(unittest.TestCase):
    def test_triangular_membership_peaks_at_center(self):
        self.assertEqual(triangular(0.5, 0.3, 0.5, 0.7), 1.0)

    def test_load_memberships_match_reference_ratios(self):
        self.assertEqual(load_low(0.2), 1.0)
        self.assertEqual(load_medium(0.5), 1.0)
        self.assertEqual(load_high(0.8), 1.0)


class SuitabilityTests(unittest.TestCase):
    def test_over_capacity_pairs_are_infeasible(self):
        matrix = build_suitability_matrix()

        self.assertIsNone(matrix["R1"]["T3"])
        self.assertIsNone(matrix["R3"]["T3"])

    def test_feasible_scores_stay_between_zero_and_one(self):
        matrix = build_suitability_matrix()
        scores = [
            score
            for robot_scores in matrix.values()
            for score in robot_scores.values()
            if score is not None
        ]

        self.assertTrue(scores)
        self.assertTrue(all(0.0 <= score <= 1.0 for score in scores))

    def test_matrix_contains_every_robot_and_task(self):
        matrix = build_suitability_matrix()

        self.assertEqual(set(matrix), set(robots))
        for robot_scores in matrix.values():
            self.assertEqual(set(robot_scores), set(tasks))

    def test_mathematical_suitability_handles_feasible_pairs(self):
        score = mathematical_suitability(robots["R1"], tasks["T1"])
        self.assertIsNotNone(score)
        self.assertTrue(0.0 <= score <= 1.0)

    def test_best_robot_selection_uses_highest_feasible_score(self):
        robot_name, score = select_best_robot_for_task("T1")
        self.assertEqual(robot_name, "R1")
        self.assertTrue(0.0 <= score <= 1.0)

    def test_combined_suitability_stays_within_bounds(self):
        score = combined_suitability(robots["R2"], tasks["T2"])
        self.assertIsNotNone(score)
        self.assertTrue(0.0 <= score <= 1.0)

    def test_assignment_planner_returns_one_choice_per_task(self):
        assignments = assign_tasks_to_robots()
        self.assertEqual(set(assignments), set(tasks))
        self.assertTrue(all("robot" in info for info in assignments.values()))
        assigned = [
            info["robot"] for info in assignments.values() if info["robot"] is not None
        ]
        self.assertEqual(len(assigned), len(set(assigned)))

    def test_assignment_planner_maximizes_total_score(self):
        robot_map = {"R1": {"id": 1}, "R2": {"id": 2}}
        task_map = {
            "A": {"urgency": 100, "id": "A"},
            "B": {"urgency": 100, "id": "B"},
        }
        scores = {
            (1, "A"): 0.90,
            (1, "B"): 0.80,
            (2, "A"): 0.85,
            (2, "B"): 0.10,
        }

        assignments = assign_tasks_to_robots(
            robot_map,
            task_map,
            scorer=lambda robot, task: scores[(robot["id"], task["id"])],
        )

        self.assertEqual(assignments["A"]["robot"], "R2")
        self.assertEqual(assignments["B"]["robot"], "R1")


if __name__ == "__main__":
    unittest.main()
