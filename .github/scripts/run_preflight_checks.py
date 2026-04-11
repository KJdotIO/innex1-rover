#!/usr/bin/env python3
"""Run cheap repository checks that do not need a ROS workspace build."""

from __future__ import annotations

import py_compile
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCAN_ROOTS = (REPO_ROOT / ".github", REPO_ROOT / "src")
PYTHON_SUFFIXES = {".py"}
YAML_SUFFIXES = {".yaml", ".yml"}
SKIP_DIR_NAMES = {
    "__pycache__",
    ".git",
    "build",
    "install",
    "log",
}


def _iter_files(suffixes: set[str]) -> list[Path]:
    files: list[Path] = []
    for root in SCAN_ROOTS:
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if path.suffix not in suffixes:
                continue
            if any(part in SKIP_DIR_NAMES for part in path.parts):
                continue
            files.append(path)
    return sorted(files)


def _check_python(files: list[Path]) -> list[str]:
    errors: list[str] = []
    for path in files:
        try:
            py_compile.compile(str(path), doraise=True)
        except py_compile.PyCompileError as exc:
            errors.append(
                f"Python syntax error in {path.relative_to(REPO_ROOT)}: {exc.msg}"
            )
    return errors


def _check_yaml(files: list[Path]) -> list[str]:
    errors: list[str] = []
    for path in files:
        try:
            yaml.safe_load(path.read_text(encoding="utf-8"))
        except yaml.YAMLError as exc:
            errors.append(f"YAML parse error in {path.relative_to(REPO_ROOT)}: {exc}")
    return errors


def main() -> int:
    python_files = _iter_files(PYTHON_SUFFIXES)
    yaml_files = _iter_files(YAML_SUFFIXES)

    errors = []
    errors.extend(_check_python(python_files))
    errors.extend(_check_yaml(yaml_files))

    print(
        "Preflight scanned "
        f"{len(python_files)} Python files and {len(yaml_files)} YAML files."
    )

    if errors:
        print("Preflight checks failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("Preflight checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
