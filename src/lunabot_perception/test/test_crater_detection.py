# Copyright 2026 University of Leicester
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Unit tests for crater detection timing behaviour."""

from unittest.mock import Mock

import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import ExtrapolationException, LookupException

from lunabot_perception.crater_detection import (
    CraterDetectionNode,
    _binary_dilation,
    _rotation_matrix_from_quaternion,
)


def _node_with_buffer(tf_buffer: Mock) -> CraterDetectionNode:
    node = object.__new__(CraterDetectionNode)
    node.tf_buffer = tf_buffer
    node.target_frame = "odom"
    node.get_logger = Mock(return_value=Mock())
    return node


def _point_cloud() -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = "camera_front_rgb_camera_optical_frame"
    msg.header.stamp.sec = 42
    msg.header.stamp.nanosec = 123
    return msg


def test_rotation_matrix_from_quaternion_applies_yaw():
    """Quaternion transforms do not depend on SciPy at runtime."""
    rot = _rotation_matrix_from_quaternion(0.0, 0.0, 2**0.5 / 2.0, 2**0.5 / 2.0)

    np.testing.assert_allclose(
        rot @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-6
    )


def test_binary_dilation_expands_without_wrapping_edges():
    """Inflation expands into neighbours without wrapping across the grid."""
    mask = np.zeros((3, 3), dtype=bool)
    mask[0, 0] = True

    dilated = _binary_dilation(mask, 1)

    assert dilated.tolist() == [
        [True, True, False],
        [True, True, False],
        [False, False, False],
    ]


def test_lookup_cloud_transform_uses_cloud_stamp_first():
    """Stamped TF lookup is used when the buffer has the exact transform."""
    expected_tf = object()
    tf_buffer = Mock()
    tf_buffer.lookup_transform.return_value = expected_tf
    node = _node_with_buffer(tf_buffer)
    msg = _point_cloud()

    result = node._lookup_cloud_transform(msg)

    assert result is expected_tf
    tf_buffer.lookup_transform.assert_called_once()
    _, _, lookup_time = tf_buffer.lookup_transform.call_args.args
    assert lookup_time.nanoseconds == Time.from_msg(msg.header.stamp).nanoseconds


def test_lookup_cloud_transform_falls_back_to_latest_for_future_extrapolation():
    """Small future extrapolation uses the latest available TF instead."""
    exact_error = ExtrapolationException(
        "Lookup would require extrapolation into the future"
    )
    latest_tf = object()
    tf_buffer = Mock()
    tf_buffer.lookup_transform.side_effect = [exact_error, latest_tf]
    node = _node_with_buffer(tf_buffer)
    msg = _point_cloud()

    result = node._lookup_cloud_transform(msg)

    assert result is latest_tf
    assert tf_buffer.lookup_transform.call_count == 2
    _, _, fallback_time = tf_buffer.lookup_transform.call_args.args
    assert fallback_time.nanoseconds == Time().nanoseconds


def test_lookup_cloud_transform_does_not_fallback_for_past_extrapolation():
    """Past extrapolation is not silently converted into a latest-TF lookup."""
    tf_buffer = Mock()
    tf_buffer.lookup_transform.side_effect = ExtrapolationException(
        "Lookup would require extrapolation into the past"
    )
    node = _node_with_buffer(tf_buffer)

    result = node._lookup_cloud_transform(_point_cloud())

    assert result is None
    tf_buffer.lookup_transform.assert_called_once()


def test_lookup_cloud_transform_returns_none_when_tf_missing():
    """Missing frame data drops the update cleanly."""
    tf_buffer = Mock()
    tf_buffer.lookup_transform.side_effect = LookupException("frame does not exist")
    node = _node_with_buffer(tf_buffer)

    result = node._lookup_cloud_transform(_point_cloud())

    assert result is None
    tf_buffer.lookup_transform.assert_called_once()
