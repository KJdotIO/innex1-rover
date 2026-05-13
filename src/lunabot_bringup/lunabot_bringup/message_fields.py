"""Helpers for dotted ROS message field paths."""

from __future__ import annotations

from typing import Any

FieldToken = str | int


def parse_field_path(path: str) -> list[FieldToken]:
    """Split a field path such as ``header.stamp.sec`` or ``values[0]``."""
    tokens: list[FieldToken] = []
    for part in path.split("."):
        if not part:
            raise ValueError(f"Invalid empty field in path '{path}'")
        name, bracket, rest = part.partition("[")
        if name:
            tokens.append(name)
        while bracket:
            index_text, close, rest = rest.partition("]")
            if not close or not index_text.isdigit():
                raise ValueError(f"Invalid list index in field path '{path}'")
            tokens.append(int(index_text))
            bracket, rest = rest[:1], rest[1:]
    return tokens


def read_message_field(message: Any, path: str) -> Any:
    """Read a nested ROS message field using dotted and indexed path syntax."""
    value: Any = message
    for token in parse_field_path(path):
        value = value[token] if isinstance(token, int) else getattr(value, token)
    return value


def write_message_field(message: Any, path: str, value: Any) -> None:
    """Set a nested ROS message field using dotted and indexed path syntax."""
    tokens = parse_field_path(path)
    if not tokens:
        raise ValueError("field path must not be empty")

    target = message
    for token in tokens[:-1]:
        target = target[token] if isinstance(token, int) else getattr(target, token)

    last = tokens[-1]
    if isinstance(last, int):
        target[last] = value
    else:
        setattr(target, last, value)
