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

"""Unit tests for the simulation CameraInfo stamp aligner."""

from unittest.mock import Mock

from lunabot_localisation.camera_info_stamp_aligner import CameraInfoStampAligner
from sensor_msgs.msg import CameraInfo, Image


def _aligner_with_publisher() -> tuple[CameraInfoStampAligner, Mock]:
    node = object.__new__(CameraInfoStampAligner)
    publisher = Mock()
    node.aligned_pub = publisher
    node.latest_camera_info = None
    return node, publisher


def test_image_callback_waits_for_camera_info():
    """Image messages are ignored until calibration data is available."""
    node, publisher = _aligner_with_publisher()

    node._image_callback(Image())

    publisher.publish.assert_not_called()


def test_image_callback_republishes_camera_info_with_image_header():
    """The aligned CameraInfo uses the image stamp and frame."""
    node, publisher = _aligner_with_publisher()
    camera_info = CameraInfo()
    camera_info.header.stamp.sec = 1
    camera_info.header.frame_id = "old_frame"
    camera_info.width = 1280
    camera_info.height = 720
    image = Image()
    image.header.stamp.sec = 42
    image.header.stamp.nanosec = 123
    image.header.frame_id = "camera_front_rgb_camera_optical_frame"

    node._camera_info_callback(camera_info)
    node._image_callback(image)

    publisher.publish.assert_called_once()
    published = publisher.publish.call_args.args[0]
    assert published.header.stamp == image.header.stamp
    assert published.header.frame_id == image.header.frame_id
    assert published.width == 1280
    assert published.height == 720
    assert camera_info.header.stamp.sec == 1
    assert camera_info.header.frame_id == "old_frame"
