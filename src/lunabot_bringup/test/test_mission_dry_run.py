"""Unit tests for mission dry-run orchestration helpers."""

from types import SimpleNamespace

import lunabot_bringup.mission_dry_run as dry_run_module
from lunabot_bringup.mission_dry_run import (
    MissionDryRunHarness,
    dry_run_exit_code,
    execute_dry_run,
    summary_lines,
)


def _fake_logger():
    return SimpleNamespace(
        info=lambda _msg: None,
        error=lambda _msg: None,
        warn=lambda _msg: None,
    )


def _failed_runtime_preflight(self):
    del self
    return False, "runtime preflight failed"


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


def test_execute_dry_run_marks_all_steps_failed_when_preflight_fails():
    results = execute_dry_run(
        [
            ("travel", lambda: (True, "travel ok")),
            ("excavate", lambda: (True, "excavate ok")),
            ("deposit", lambda: (True, "deposit ok")),
        ],
        preflight_runner=lambda: (False, "runtime preflight failed"),
    )

    assert [(result.name, result.passed) for result in results] == [
        ("travel", False),
        ("excavate", False),
        ("deposit", False),
    ]


def test_run_returns_non_zero_when_runtime_preflight_fails(monkeypatch):
    harness = object.__new__(MissionDryRunHarness)
    harness.get_logger = _fake_logger
    monkeypatch.setattr(
        dry_run_module.MissionDryRunHarness,
        "_run_runtime_preflight",
        _failed_runtime_preflight,
    )

    status = MissionDryRunHarness.run(harness)

    assert status == 1


def test_run_prints_flat_summary_when_runtime_preflight_fails(
    monkeypatch, capsys
):
    harness = object.__new__(MissionDryRunHarness)
    harness.get_logger = _fake_logger
    monkeypatch.setattr(
        dry_run_module.MissionDryRunHarness,
        "_run_runtime_preflight",
        _failed_runtime_preflight,
    )

    status = MissionDryRunHarness.run(harness)

    assert status == 1
    assert capsys.readouterr().out.splitlines() == [
        "travel: fail",
        "excavate: fail",
        "deposit: fail",
        "overall: fail",
    ]
