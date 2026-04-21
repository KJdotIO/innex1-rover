#!/usr/bin/env python3
"""Regression checks for interface contract drift enforcement."""

from __future__ import annotations

import importlib.util
import sys
import tempfile
from pathlib import Path
from typing import Protocol, cast

REPO_ROOT = Path(__file__).resolve().parents[2]
CHECKER_PATH = REPO_ROOT / ".github/scripts/check_interface_contracts.py"


class ContractCheckerModule(Protocol):
    REPO_ROOT: Path

    def _check_declared_interfaces(
        self,
        contract: dict,
        section_name: str,
    ) -> list[str]: ...

    def _check_uncontracted_interfaces(self, contract: dict) -> list[str]: ...


def _load_checker_module() -> ContractCheckerModule:
    spec = importlib.util.spec_from_file_location(
        "check_interface_contracts",
        CHECKER_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load checker module from {CHECKER_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return cast(ContractCheckerModule, module)


def _write_source(
    root: Path,
    relative_path: str,
    *,
    extra_type: str = "Trigger",
) -> None:
    path = root / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(
            [
                "from rclpy.action import ActionServer",
                "from rclpy.node import Node",
                "from std_srvs.srv import Empty, Trigger",
                "from lunabot_interfaces.action import Excavate",
                "",
                "class Example(Node):",
                "    def __init__(self):",
                "        super().__init__('example')",
                f"        self.create_client({extra_type}, srv_name='/excavation/home')",
                "        self._server = ActionServer(",
                "            self,",
                "            Excavate,",
                "            action_name='/mission/excavate',",
                "            execute_callback=lambda _goal: None,",
                "        )",
            ]
        ),
        encoding="utf-8",
    )


def _base_contract(source_rel: str) -> dict:
    return {
        "source_policies": [
            {"source": source_rel, "strict_interfaces": True},
        ],
        "topics": [
            {
                "name": "/mission/excavate",
                "type": "lunabot_interfaces/action/Excavate",
                "kind": "action_server",
                "source": source_rel,
            }
        ],
        "services": [],
    }


def _assert_contains(errors: list[str], expected: str) -> list[str]:
    if any(error.startswith(expected) for error in errors):
        return []
    return [f"expected {expected!r}, got {errors}"]


def _run_regressions(checker: ContractCheckerModule, root: Path) -> list[str]:
    source_rel = "src/lunabot_excavation/lunabot_excavation/example.py"
    checker.REPO_ROOT = root
    _write_source(root, source_rel)

    failures: list[str] = []
    contract = _base_contract(source_rel)
    errors = checker._check_uncontracted_interfaces(contract)
    failures.extend(
        _assert_contains(
            errors,
            "Uncontracted interface: service_client /excavation/home",
        )
    )

    zero_entry_contract = {
        "source_policies": [{"source": source_rel, "strict_interfaces": True}],
        "topics": [],
        "services": [],
    }
    errors = checker._check_uncontracted_interfaces(zero_entry_contract)
    failures.extend(
        _assert_contains(
            errors,
            "Uncontracted interface: action_server /mission/excavate",
        )
    )

    contract["services"].append(
        {
            "name": "/excavation/home",
            "type": "std_srvs/srv/Trigger",
            "kind": "service_client",
            "source": source_rel,
        }
    )
    errors = checker._check_declared_interfaces(contract, "services")
    errors.extend(checker._check_uncontracted_interfaces(contract))
    failures.extend(errors)

    _write_source(root, source_rel, extra_type="Empty")
    errors = checker._check_uncontracted_interfaces(contract)
    failures.extend(
        _assert_contains(
            errors,
            "Uncontracted interface: service_client /excavation/home",
        )
    )

    malformed = checker._check_declared_interfaces(
        {"services": [{"name": "/bad"}]},
        "services",
    )
    failures.extend(
        _assert_contains(
            malformed,
            "Invalid services entry in contract file",
        )
    )
    return failures


def main() -> int:
    checker = _load_checker_module()

    with tempfile.TemporaryDirectory() as tmp:
        failures = _run_regressions(checker, Path(tmp))
        if failures:
            print("Interface contract drift regression checks failed:")
            for failure in failures:
                print(f"- {failure}")
            return 1

    print("Interface contract drift regression checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
