"""Unit tests for mission dry-run orchestration helpers."""

from lunabot_bringup.mission_dry_run import dry_run_exit_code
from lunabot_bringup.mission_dry_run import execute_dry_run
from lunabot_bringup.mission_dry_run import summary_lines


def test_execute_dry_run_runs_each_step_once_on_success():
    calls = []

    def _step(name):
        def _runner():
            calls.append(name)
            return True, f"{name} ok"

        return _runner

    results = execute_dry_run(
        [
            ("travel", _step("travel")),
            ("excavate", _step("excavate")),
            ("deposit", _step("deposit")),
        ]
    )

    assert calls == ["travel", "excavate", "deposit"]
    assert [result.passed for result in results] == [True, True, True]


def test_execute_dry_run_marks_remaining_steps_failed_after_first_failure():
    calls = []

    def _travel():
        calls.append("travel")
        return True, "travel ok"

    def _excavate():
        calls.append("excavate")
        return False, "excavate failed"

    def _deposit():
        calls.append("deposit")
        return True, "deposit ok"

    results = execute_dry_run(
        [
            ("travel", _travel),
            ("excavate", _excavate),
            ("deposit", _deposit),
        ]
    )

    assert calls == ["travel", "excavate"]
    assert [(result.name, result.passed) for result in results] == [
        ("travel", True),
        ("excavate", False),
        ("deposit", False),
    ]
    assert results[-1].detail == "not run because excavate failed"


def test_summary_lines_include_all_steps_and_overall():
    results = execute_dry_run(
        [
            ("travel", lambda: (True, "travel ok")),
            ("excavate", lambda: (True, "excavate ok")),
            ("deposit", lambda: (False, "deposit failed")),
        ]
    )

    assert summary_lines(results) == [
        "travel: pass",
        "excavate: pass",
        "deposit: fail",
        "overall: fail",
    ]


def test_dry_run_exit_code_is_non_zero_on_failure():
    results = execute_dry_run(
        [
            ("travel", lambda: (True, "travel ok")),
            ("excavate", lambda: (False, "excavate failed")),
            ("deposit", lambda: (True, "deposit ok")),
        ]
    )

    assert dry_run_exit_code(results) == 1
