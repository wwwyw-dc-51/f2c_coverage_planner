from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest

from batch_result_gate import main, validate_report


def valid_report():
    return {
        "scenario": "S1",
        "path": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
        "swaths": [{"points": [[0.0, 0.0], [1.0, 0.0]]}],
        "eval": {
            "coverage_rate": 99.0,
            "single_score": 80.0,
            "uncovered_area": 0.1,
            "total_distance": 10.0,
            "work_ratio": 90.0,
            "turn_count": 1,
            "overlap_rate": 5.0,
            "planning_time_ms": 100.0,
            "net_area": 100.0,
        },
        "cspace": {
            "valid": True,
            "original_area": 100.0,
            "reachable_area": 99.5,
            "excluded_area": 0.5,
            "excluded_ratio": 0.005,
            "max_ratio": 0.05,
            "component_count": 1,
            "requires_repositioning": False,
            "gate": "PASS",
        },
        "batch_status": {
            "plan_received": True,
            "evaluation_completed": True,
            "visualization_artifact_created": True,
            "grid_artifact_created": True,
        },
    }


def valid_multi_component_report():
    report = valid_report()
    report["path"] = []
    report["component_paths"] = [
        [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 0.0}],
        [{"x": 5.0, "y": 0.0}, {"x": 6.0, "y": 0.0}],
    ]
    report["component_evals"] = [
        dict(report["eval"]),
        dict(report["eval"]),
    ]
    report["eval"] = {}
    report["cspace"]["component_count"] = 2
    report["cspace"]["requires_repositioning"] = True
    return report


class BatchResultGateTest(unittest.TestCase):
    def test_accepts_complete_report_at_coverage_threshold(self):
        self.assertEqual(validate_report(valid_report(), coverage_threshold=0.99), [])

    def test_accepts_independent_multi_component_paths_and_evaluations(self):
        self.assertEqual(validate_report(valid_multi_component_report()), [])

    def test_rejects_missing_multi_component_path_or_evaluation(self):
        for field_name in ("component_paths", "component_evals"):
            with self.subTest(field_name=field_name):
                report = valid_multi_component_report()
                report[field_name].pop()

                errors = validate_report(report)

                self.assertTrue(any(field_name in error for error in errors))

    def test_rejects_a_flattened_top_level_multi_component_path(self):
        report = valid_multi_component_report()
        report["path"] = [
            {"x": 0.0, "y": 0.0},
            {"x": 6.0, "y": 0.0},
        ]

        errors = validate_report(report)

        self.assertTrue(any("禁止跨区展平" in error for error in errors))

    def test_rejects_one_failing_component_evaluation(self):
        report = valid_multi_component_report()
        report["component_evals"][0]["coverage_rate"] = 98.0

        errors = validate_report(report)

        self.assertTrue(any(
            "component_evals[0].coverage_rate" in error for error in errors))

    def test_rejects_report_below_coverage_threshold(self):
        report = valid_report()
        report["eval"]["coverage_rate"] = 98.99

        errors = validate_report(report, coverage_threshold=0.99)

        self.assertTrue(any("coverage_rate" in error for error in errors))

    def test_rejects_incomplete_capture_or_stale_artifacts(self):
        for status_name in valid_report()["batch_status"]:
            with self.subTest(status_name=status_name):
                report = valid_report()
                report["batch_status"][status_name] = False

                errors = validate_report(report)

                self.assertTrue(any(status_name in error for error in errors))

    def test_rejects_missing_path_or_swath_evidence(self):
        for field_name in ("path", "swaths"):
            with self.subTest(field_name=field_name):
                report = valid_report()
                report[field_name] = []

                errors = validate_report(report)

                self.assertTrue(any(field_name in error for error in errors))

    def test_rejects_missing_or_non_finite_evaluation_metrics(self):
        metric_names = (
            "coverage_rate", "single_score", "uncovered_area", "total_distance",
            "work_ratio", "turn_count", "overlap_rate", "planning_time_ms", "net_area",
        )
        for metric_name in metric_names:
            with self.subTest(metric_name=metric_name):
                report = valid_report()
                report["eval"][metric_name] = float("nan")

                errors = validate_report(report)

                self.assertTrue(any(metric_name in error for error in errors))

    def test_rejects_metrics_outside_their_semantic_ranges(self):
        invalid_metrics = {
            "coverage_rate": 100.01,
            "single_score": -0.01,
            "uncovered_area": -0.01,
            "total_distance": -0.01,
            "work_ratio": -0.01,
            "turn_count": -1,
            "overlap_rate": -0.01,
            "planning_time_ms": -0.01,
            "net_area": 0.0,
        }
        for metric_name, invalid_value in invalid_metrics.items():
            with self.subTest(metric_name=metric_name):
                report = valid_report()
                report["eval"][metric_name] = invalid_value

                errors = validate_report(report)

                self.assertTrue(any(metric_name in error for error in errors))

    def test_rejects_fractional_turn_count(self):
        report = valid_report()
        report["eval"]["turn_count"] = 1.5

        errors = validate_report(report)

        self.assertTrue(any("turn_count" in error for error in errors))

    def test_rejects_missing_or_failed_cspace_report(self):
        for mutation in ("missing", "invalid", "failed_gate"):
            with self.subTest(mutation=mutation):
                report = valid_report()
                if mutation == "missing":
                    report.pop("cspace")
                elif mutation == "invalid":
                    report["cspace"]["valid"] = False
                else:
                    report["cspace"]["gate"] = "FAIL"

                errors = validate_report(report)

                self.assertTrue(any("cspace" in error for error in errors))

    def test_rejects_cspace_area_or_ratio_inconsistency(self):
        mutations = {
            "area": ("reachable_area", 98.0),
            "ratio": ("excluded_ratio", 0.50),
            "limit": ("max_ratio", 0.001),
        }
        for mutation, (field, value) in mutations.items():
            with self.subTest(mutation=mutation):
                report = valid_report()
                report["cspace"][field] = value

                errors = validate_report(report)

                self.assertTrue(any("cspace" in error for error in errors))

    def test_requires_repositioning_exactly_for_multiple_components(self):
        for component_count, repositioning in ((1, True), (2, False)):
            with self.subTest(
                    component_count=component_count,
                    repositioning=repositioning):
                report = valid_report()
                report["cspace"]["component_count"] = component_count
                report["cspace"]["requires_repositioning"] = repositioning

                errors = validate_report(report)

                self.assertTrue(any("requires_repositioning" in error
                                    for error in errors))

    def test_cli_returns_nonzero_and_explains_rejected_report(self):
        report = valid_report()
        report["batch_status"]["evaluation_completed"] = False
        with tempfile.TemporaryDirectory() as temp_dir:
            report_path = Path(temp_dir) / "S1_data.json"
            report_path.write_text(json.dumps(report), encoding="utf-8")
            output = io.StringIO()

            with redirect_stdout(output):
                exit_code = main([str(report_path), "--coverage-threshold", "0.99"])

        self.assertEqual(exit_code, 1)
        self.assertIn("evaluation_completed", output.getvalue())

    def test_rejects_malformed_report_shapes_without_crashing(self):
        for report in ([], {"batch_status": [], "eval": []}):
            with self.subTest(report=report):
                errors = validate_report(report)

                self.assertTrue(errors)


if __name__ == "__main__":
    unittest.main()
