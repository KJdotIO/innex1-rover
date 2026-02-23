#!/usr/bin/env python3
"""Check required ROS interfaces declared in repository source files."""

from __future__ import annotations

import ast
import json
import re
import sys
from typing import Any
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
CONTRACT_PATH = REPO_ROOT / ".github/contracts/interface_contracts.json"


def _constant_string(node: ast.AST | None) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _resolve_type(node: ast.AST | None, import_aliases: dict[str, str]) -> str:
    if isinstance(node, ast.Name):
        return import_aliases.get(node.id, node.id)
    if isinstance(node, ast.Attribute):
        parts = []
        current = node
        while isinstance(current, ast.Attribute):
            parts.append(current.attr)
            current = current.value
        if isinstance(current, ast.Name):
            parts.append(current.id)
        return ".".join(reversed(parts))
    return "<unknown>"


def _extract_python_interfaces(path: Path) -> list[dict[str, str]]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    import_aliases: dict[str, str] = {}

    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            module = node.module
            for alias in node.names:
                local_name = alias.asname or alias.name
                if module.endswith(".msg"):
                    package = module.rsplit(".", 1)[0]
                    import_aliases[local_name] = f"{package}/msg/{alias.name}"
                elif module.endswith(".action"):
                    package = module.rsplit(".", 1)[0]
                    import_aliases[local_name] = f"{package}/action/{alias.name}"
                else:
                    import_aliases[local_name] = f"{module}.{alias.name}"

    interfaces: list[dict[str, str]] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue

        func_name = None
        if isinstance(node.func, ast.Name):
            func_name = node.func.id
        elif isinstance(node.func, ast.Attribute):
            func_name = node.func.attr

        if (
            func_name in {"create_publisher", "create_subscription"}
            and len(node.args) >= 2
        ):
            topic = _constant_string(node.args[1])
            if topic:
                interfaces.append(
                    {
                        "kind": "publisher"
                        if func_name == "create_publisher"
                        else "subscription",
                        "name": topic,
                        "type": _resolve_type(node.args[0], import_aliases),
                    }
                )

        if func_name in {"ActionServer", "ActionClient"} and len(node.args) >= 3:
            action_name = _constant_string(node.args[2])
            if action_name:
                interfaces.append(
                    {
                        "kind": "action_server"
                        if func_name == "ActionServer"
                        else "action_client",
                        "name": action_name,
                        "type": _resolve_type(node.args[1], import_aliases),
                    }
                )

    return interfaces


def _extract_simple_yaml_params(path: Path, node_name: str) -> dict[str, str]:
    lines = path.read_text(encoding="utf-8").splitlines()
    in_node = False
    in_params = False
    params_indent = None
    values: dict[str, str] = {}

    for raw_line in lines:
        line = raw_line.rstrip("\n")
        stripped = line.strip()
        indent = len(line) - len(line.lstrip(" "))

        if not stripped or stripped.startswith("#"):
            continue

        if not in_node and re.match(rf"^{re.escape(node_name)}:\s*$", stripped):
            in_node = True
            continue

        if in_node and indent == 0 and not stripped.startswith(node_name):
            break

        if in_node and not in_params and stripped == "ros__parameters:":
            in_params = True
            params_indent = indent
            continue

        if in_params and params_indent is not None:
            if indent <= params_indent:
                break
            match = re.match(r"^\s*([A-Za-z0-9_]+):\s*(.+?)\s*$", line)
            if match:
                key = match.group(1)
                value = match.group(2).split("#", 1)[0].strip().strip('"')
                values[key] = value

    return values


def _check_topics(contract: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    by_source: dict[Path, list[dict[str, str]]] = {}
    topics = contract.get("topics", [])
    if not isinstance(topics, list):
        return ["Invalid contract format: 'topics' must be a list"]

    for expected in topics:
        if not isinstance(expected, dict):
            errors.append("Invalid topic entry in contract file")
            continue

        source_rel = expected["source"]
        source = REPO_ROOT / source_rel
        if source not in by_source:
            if not source.exists():
                errors.append(f"Missing topic source file: {source_rel}")
                continue
            by_source[source] = _extract_python_interfaces(source)

        found = [
            item
            for item in by_source[source]
            if item["name"] == expected["name"] and item["kind"] == expected["kind"]
        ]

        if not found:
            errors.append(
                "Missing interface: "
                f"{expected['kind']} {expected['name']} in {source_rel}"
            )
            continue

        actual_type = found[0]["type"]
        if actual_type != expected["type"]:
            errors.append(
                "Type mismatch for "
                f"{expected['name']}: expected {expected['type']}, got {actual_type}"
            )

    return errors


def _check_tf_links(contract: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    tf_links = contract.get("tf_links", [])
    if not isinstance(tf_links, list):
        return ["Invalid contract format: 'tf_links' must be a list"]

    for tf_link in tf_links:
        if not isinstance(tf_link, dict):
            errors.append("Invalid tf_links entry in contract file")
            continue

        source_rel = tf_link["source"]
        source = REPO_ROOT / source_rel
        if not source.exists():
            errors.append(f"Missing TF source file: {source_rel}")
            continue

        kind = tf_link.get("kind")
        if kind == "ekf_tf":
            node_name = tf_link["node"]
            params = _extract_simple_yaml_params(source, node_name)
            parent_value = params.get(tf_link["parent_param"])
            child_value = params.get(tf_link["child_param"])
            if parent_value != tf_link["parent"] or child_value != tf_link["child"]:
                errors.append(
                    "Missing TF link in EKF config: "
                    f"{tf_link['parent']} -> {tf_link['child']} from {source_rel}"
                )
        elif kind == "xacro_joint":
            text = source.read_text(encoding="utf-8")
            parent_pattern = re.escape(tf_link["parent_pattern"])
            child_pattern = re.escape(tf_link["child_pattern"])
            first_order = re.search(
                rf"<joint[\s\S]*?<parent link=\"{parent_pattern}\"\s*/>[\s\S]*?<child link=\"{child_pattern}\"\s*/>[\s\S]*?</joint>",
                text,
            )
            second_order = re.search(
                rf"<joint[\s\S]*?<child link=\"{child_pattern}\"\s*/>[\s\S]*?<parent link=\"{parent_pattern}\"\s*/>[\s\S]*?</joint>",
                text,
            )
            if not first_order and not second_order:
                errors.append(
                    "Missing TF link in xacro joint: "
                    f"{tf_link['parent']} -> {tf_link['child']} from {source_rel}"
                )
        else:
            errors.append(f"Unknown tf_links kind '{kind}' in contract file")

    return errors


def main() -> int:
    if not CONTRACT_PATH.exists():
        print(f"Contract file not found: {CONTRACT_PATH}")
        return 1

    contract = json.loads(CONTRACT_PATH.read_text(encoding="utf-8"))
    errors = []
    errors.extend(_check_topics(contract))
    errors.extend(_check_tf_links(contract))

    if errors:
        print("Interface contract check failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("Interface contract check passed")
    deferred = contract.get("deferred_topics", [])
    if deferred:
        print("Deferred mission-state topics (not enforced yet):")
        for topic in deferred:
            print(f"- {topic}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
