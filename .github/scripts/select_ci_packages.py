#!/usr/bin/env python3
"""Select a conservative ROS package set for CI."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SRC_ROOT = REPO_ROOT / "src"
BROAD_PATH_PREFIXES = (
    ".github/workflows/",
    ".github/actions/",
    ".github/scripts/",
    ".github/contracts/",
)
BROAD_PACKAGES = {
    "lunabot_bringup",
    "lunabot_control",
    "lunabot_interfaces",
    "lunabot_navigation",
    "lunabot_simulation",
}
SAFE_SELECT_PACKAGES = {
    "lunabot_excavation",
    "lunabot_perception",
    "lunabot_teleop",
}
DEPENDENCY_TAGS = {
    "depend",
    "build_depend",
    "build_export_depend",
    "exec_depend",
    "test_depend",
}


@dataclass(frozen=True)
class Selection:
    mode: str
    packages: tuple[str, ...]
    reason: str


def _git_changed_files(base_sha: str, head_sha: str) -> list[str]:
    if not base_sha or set(base_sha) == {"0"}:
        return []

    try:
        result = subprocess.run(
            ["git", "diff", "--name-only", base_sha, head_sha],
            check=True,
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        print(
            f"Falling back to full CI because git diff failed for {base_sha}..{head_sha}: "
            f"{exc.stderr.strip()}",
            file=sys.stderr,
        )
        return []

    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def _discover_packages() -> tuple[dict[str, str], dict[str, set[str]]]:
    package_roots: dict[str, str] = {}
    dependencies: dict[str, set[str]] = {}

    for package_xml in sorted(SRC_ROOT.rglob("package.xml")):
        root = package_xml.parent
        tree = ET.parse(package_xml)
        package = tree.getroot()
        name = package.findtext("name")
        if not name:
            continue

        root_rel = root.relative_to(REPO_ROOT).as_posix()
        package_roots[root_rel] = name
        dependencies[name] = {
            element.text.strip()
            for element in package
            if element.tag in DEPENDENCY_TAGS and element.text
        }

    return package_roots, dependencies


def _owning_package(changed_file: str, package_roots: dict[str, str]) -> str | None:
    matching_root = None
    for root in package_roots:
        prefix = f"{root}/"
        if changed_file == root or changed_file.startswith(prefix):
            if matching_root is None or len(root) > len(matching_root):
                matching_root = root

    if matching_root is None:
        return None
    return package_roots[matching_root]


def _reverse_dependencies(
    changed_packages: set[str],
    dependencies: dict[str, set[str]],
) -> set[str]:
    selected = set(changed_packages)
    added = True
    while added:
        added = False
        for package, package_deps in dependencies.items():
            if package in selected:
                continue
            if package_deps & selected:
                selected.add(package)
                added = True
    return selected


def _select(
    changed_files: list[str],
    package_roots: dict[str, str],
    dependencies: dict[str, set[str]],
) -> Selection:
    if not changed_files:
        return Selection("full", (), "No reliable diff range available")

    if any(
        any(path.startswith(prefix) for prefix in BROAD_PATH_PREFIXES)
        for path in changed_files
    ):
        return Selection("full", (), "Shared CI or contract files changed")

    changed_packages: set[str] = set()
    for path in changed_files:
        if not path.startswith("src/"):
            return Selection("full", (), f"Top-level repo file changed: {path}")

        package = _owning_package(path, package_roots)
        if package is None:
            return Selection("full", (), f"Unowned source path changed: {path}")

        if package.startswith("leo_"):
            return Selection("full", (), f"External package changed: {package}")

        if package in BROAD_PACKAGES:
            return Selection("full", (), f"Broad package changed: {package}")

        if package not in SAFE_SELECT_PACKAGES:
            return Selection("full", (), f"Package not whitelisted for selective CI: {package}")

        changed_packages.add(package)

    selected = _reverse_dependencies(changed_packages, dependencies)
    return Selection(
        "packages",
        tuple(sorted(selected)),
        "Selective CI is safe for this package set",
    )


def _write_github_output(path: Path, selection: Selection) -> None:
    with path.open("a", encoding="utf-8") as handle:
        handle.write(f"mode={selection.mode}\n")
        handle.write(f"packages={' '.join(selection.packages)}\n")
        handle.write(f"reason={selection.reason}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-sha", required=True)
    parser.add_argument("--head-sha", required=True)
    parser.add_argument("--github-output")
    args = parser.parse_args()

    package_roots, dependencies = _discover_packages()
    changed_files = _git_changed_files(args.base_sha, args.head_sha)
    selection = _select(changed_files, package_roots, dependencies)

    summary = {
        "mode": selection.mode,
        "packages": list(selection.packages),
        "reason": selection.reason,
        "changed_files": changed_files,
    }
    print(json.dumps(summary, indent=2))

    if args.github_output:
        _write_github_output(Path(args.github_output), selection)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
