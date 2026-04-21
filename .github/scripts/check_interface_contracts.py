#!/usr/bin/env python3
"""Check required ROS interfaces declared in repository source files."""

from __future__ import annotations

import ast
import json
import re
import sys
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
CONTRACT_PATH = REPO_ROOT / ".github/contracts/interface_contracts.json"
PACKAGE_CONTRACT_PATH = (
    REPO_ROOT / "src/lunabot_bringup/contracts/interface_contracts.json"
)


def _constant_string(node: ast.AST | None) -> str | None:
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    return None


def _constant_arg(
    node: ast.Call,
    index: int,
    keyword_names: set[str],
) -> str | None:
    if len(node.args) > index:
        value = _constant_string(node.args[index])
        if value is not None:
            return value
    for keyword in node.keywords:
        if keyword.arg in keyword_names:
            return _constant_string(keyword.value)
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
                elif module.endswith(".srv"):
                    package = module.rsplit(".", 1)[0]
                    import_aliases[local_name] = f"{package}/srv/{alias.name}"
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

        if func_name in {"create_publisher", "create_subscription"} and node.args:
            topic = _constant_arg(node, 1, {"topic"})
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

        if func_name in {"ActionServer", "ActionClient"} and len(node.args) >= 2:
            action_name = _constant_arg(node, 2, {"action_name"})
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

        if func_name in {"create_client", "create_service"} and node.args:
            service_name = _constant_arg(node, 1, {"srv_name"})
            if service_name:
                interfaces.append(
                    {
                        "kind": "service_client"
                        if func_name == "create_client"
                        else "service_server",
                        "name": service_name,
                        "type": _resolve_type(node.args[0], import_aliases),
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


def _check_metadata(contract: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if contract.get("schema_version") != 1:
        errors.append("Invalid contract metadata: schema_version must be 1")
    if not contract.get("contract_version"):
        errors.append("Invalid contract metadata: contract_version is required")
    return errors


def _check_packaged_contract_copy(contract: dict[str, Any]) -> list[str]:
    if not PACKAGE_CONTRACT_PATH.exists():
        return [f"Packaged contract copy not found: {PACKAGE_CONTRACT_PATH}"]

    packaged_contract = json.loads(PACKAGE_CONTRACT_PATH.read_text(encoding="utf-8"))
    if packaged_contract != contract:
        return [
            "Packaged contract copy is out of sync with "
            f"{CONTRACT_PATH.relative_to(REPO_ROOT)}"
        ]
    return []


def _interface_identity(item: dict[str, str]) -> tuple[str, str, str]:
    return (item["kind"], item["name"], item["type"])


def _source_policies(contract: dict[str, Any]) -> dict[str, dict[str, Any]]:
    policies: dict[str, dict[str, Any]] = {}
    source_policies = contract.get("source_policies", [])
    if not isinstance(source_policies, list):
        return policies

    for policy in source_policies:
        if not isinstance(policy, dict):
            continue
        source = policy.get("source")
        if isinstance(source, str):
            policies[source] = policy
    return policies


def _missing_fields(
    entry: dict[str, Any],
    section_name: str,
    required_fields: set[str],
) -> list[str]:
    missing = sorted(
        field for field in required_fields if not isinstance(entry.get(field), str)
    )
    if not missing:
        return []
    return [
        f"Invalid {section_name} entry in contract file: "
        f"missing string fields {', '.join(missing)}"
    ]


def _contract_interfaces_by_source(
    contract: dict[str, Any],
) -> dict[str, list[dict[str, str]]]:
    by_source: dict[str, list[dict[str, str]]] = {}
    for section_name in ("topics", "services"):
        section = contract.get(section_name, [])
        if not isinstance(section, list):
            continue
        for item in section:
            if not isinstance(item, dict):
                continue
            source = item.get("source")
            if isinstance(source, str):
                by_source.setdefault(source, []).append(item)
    return by_source


def _check_uncontracted_interfaces(contract: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    expected_by_source = _contract_interfaces_by_source(contract)
    policies = _source_policies(contract)

    for source_rel, policy in sorted(policies.items()):
        if not bool(policy.get("strict_interfaces", False)):
            continue

        source = REPO_ROOT / source_rel
        if not source.exists():
            errors.append(f"Missing strict interface source file: {source_rel}")
            continue

        expected = expected_by_source.get(source_rel, [])
        expected_identities = {
            _interface_identity(item)
            for item in expected
            if isinstance(item.get("kind"), str)
            and isinstance(item.get("name"), str)
            and isinstance(item.get("type"), str)
        }
        for actual in _extract_python_interfaces(source):
            if _interface_identity(actual) in expected_identities:
                continue
            errors.append(
                "Uncontracted interface: "
                f"{actual['kind']} {actual['name']} ({actual['type']}) "
                f"in {source_rel}"
            )

    return errors


def _check_declared_interfaces(
    contract: dict[str, Any],
    section_name: str,
) -> list[str]:
    errors: list[str] = []
    by_source: dict[Path, list[dict[str, str]]] = {}
    entries = contract.get(section_name, [])
    if not isinstance(entries, list):
        return [f"Invalid contract format: '{section_name}' must be a list"]

    for expected in entries:
        if not isinstance(expected, dict):
            errors.append(f"Invalid {section_name} entry in contract file")
            continue
        missing_errors = _missing_fields(
            expected,
            section_name,
            {"source", "kind", "name", "type"},
        )
        if missing_errors:
            errors.extend(missing_errors)
            continue

        source_rel = expected["source"]
        source = REPO_ROOT / source_rel
        if source not in by_source:
            if not source.exists():
                errors.append(f"Missing {section_name} source file: {source_rel}")
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

        actual_types = sorted({item["type"] for item in found})
        if expected["type"] not in actual_types:
            errors.append(
                "Type mismatch for "
                f"{expected['name']}: expected {expected['type']}, "
                f"got {actual_types}"
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
        missing_errors = _missing_fields(
            tf_link,
            "tf_links",
            {"source", "kind", "parent", "child"},
        )
        if missing_errors:
            errors.extend(missing_errors)
            continue

        source_rel = tf_link["source"]
        source = REPO_ROOT / source_rel
        if not source.exists():
            errors.append(f"Missing TF source file: {source_rel}")
            continue

        kind = tf_link.get("kind")
        if kind == "ekf_tf":
            missing_errors = _missing_fields(
                tf_link,
                "tf_links ekf_tf",
                {"node", "parent_param", "child_param"},
            )
            if missing_errors:
                errors.extend(missing_errors)
                continue
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
            missing_errors = _missing_fields(
                tf_link,
                "tf_links xacro_joint",
                {"parent_pattern", "child_pattern"},
            )
            if missing_errors:
                errors.extend(missing_errors)
                continue
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
    errors.extend(_check_metadata(contract))
    errors.extend(_check_packaged_contract_copy(contract))
    errors.extend(_check_declared_interfaces(contract, "topics"))
    errors.extend(_check_declared_interfaces(contract, "services"))
    errors.extend(_check_uncontracted_interfaces(contract))
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
