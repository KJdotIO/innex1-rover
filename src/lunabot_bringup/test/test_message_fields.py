"""Tests for shared ROS message field-path helpers."""

from types import SimpleNamespace

import pytest

from lunabot_bringup.message_fields import (
    parse_field_path,
    read_message_field,
    write_message_field,
)


def test_parse_field_path_supports_nested_attributes_and_indices():
    assert parse_field_path("header.stamp.sec") == ["header", "stamp", "sec"]
    assert parse_field_path("controller_online[1]") == ["controller_online", 1]


def test_read_and_write_message_field_supports_nested_paths():
    message = SimpleNamespace(
        controller_online=[False, False],
        nested=SimpleNamespace(state="idle"),
    )

    write_message_field(message, "controller_online[0]", True)
    write_message_field(message, "nested.state", "ready")

    assert read_message_field(message, "controller_online[0]") is True
    assert read_message_field(message, "nested.state") == "ready"


def test_parse_field_path_rejects_invalid_paths():
    with pytest.raises(ValueError, match="Invalid empty field"):
        parse_field_path("header..stamp")

    with pytest.raises(ValueError, match="Invalid list index"):
        parse_field_path("controller_online[first]")
