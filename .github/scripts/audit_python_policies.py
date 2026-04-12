#!/usr/bin/env python3
"""Report policy findings that generic linters do not capture well."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


class PolicyAudit(ast.NodeVisitor):
    """Collect broad exception handlers in production code."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.findings: list[str] = []

    def visit_Try(self, node: ast.Try) -> None:
        for handler in node.handlers:
            if handler.type is None:
                self.findings.append(
                    f"{self.path}:{handler.lineno}: bare except is not allowed"
                )
                continue

            if isinstance(handler.type, ast.Name) and handler.type.id == "Exception":
                self.findings.append(
                    f"{self.path}:{handler.lineno}: broad except Exception in "
                    "production path"
                )

        self.generic_visit(node)


def _python_files(paths: list[str]) -> list[Path]:
    files: list[Path] = []
    for raw_path in paths:
        path = REPO_ROOT / raw_path
        if path.is_file() and path.suffix == ".py":
            files.append(path)
            continue
        files.extend(sorted(path.rglob("*.py")))
    return files


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--paths", nargs="+", required=True)
    args = parser.parse_args()

    findings: list[str] = []
    for path in _python_files(args.paths):
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source, filename=str(path))
        audit = PolicyAudit(path)
        audit.visit(tree)
        findings.extend(audit.findings)

    if findings:
        print("Policy audit findings:")
        for finding in findings:
            print(f"  {finding}")
        return 1

    print("Policy audit passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
