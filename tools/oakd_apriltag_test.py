#!/usr/bin/env python3
"""Native OAK-D AprilTag telemetry visualizer for macOS/Linux."""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import depthai as dai
import numpy as np
from pupil_apriltags import Detector

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None


@dataclass
class Queues:
    rgb: Any
    depth: Any | None = None


@dataclass
class CameraModel:
    matrix: np.ndarray
    distortion: np.ndarray
    usb_speed: str = "unknown"


@dataclass
class ExperimentState:
    phase: str = "baseline"
    event: str = ""
    last_key: str = ""


def _gray_from_bgr(frame: np.ndarray) -> np.ndarray:
    return (
        0.114 * frame[:, :, 0] + 0.587 * frame[:, :, 1] + 0.299 * frame[:, :, 2]
    ).astype(np.uint8)


def _lighting_metrics(gray: np.ndarray, detection: Any | None = None) -> dict[str, float | str]:
    metrics: dict[str, float | str] = {
        "brightness_mean": float(np.mean(gray)),
        "brightness_std": float(np.std(gray)),
        "brightness_p05": float(np.percentile(gray, 5)),
        "brightness_p95": float(np.percentile(gray, 95)),
        "tag_mean": "",
        "tag_std": "",
        "tag_contrast": "",
    }
    if detection is None or cv2 is None:
        return metrics

    corners = detection.corners.astype(np.int32)
    mask = np.zeros(gray.shape, dtype=np.uint8)
    cv2.fillConvexPoly(mask, corners, 255)
    pixels = gray[mask > 0]
    if pixels.size:
        metrics["tag_mean"] = float(np.mean(pixels))
        metrics["tag_std"] = float(np.std(pixels))
        metrics["tag_contrast"] = float(np.percentile(pixels, 95) - np.percentile(pixels, 5))
    return metrics


def _build_pipeline(args: argparse.Namespace) -> tuple[dai.Pipeline, Queues]:
    pipeline = dai.Pipeline()
    if hasattr(dai.node, "XLinkOut"):
        color = pipeline.create(dai.node.ColorCamera)
        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName("preview")
        color.setPreviewSize(args.width, args.height)
        color.setInterleaved(False)
        color.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        color.setFps(args.fps)
        color.preview.link(xout.input)
        return pipeline, Queues(rgb=None)

    color = pipeline.create(dai.node.Camera).build()
    rgb_out = color.requestOutput((args.width, args.height), type=dai.ImgFrame.Type.BGR888i)
    rgb_queue = rgb_out.createOutputQueue()
    depth_queue = None

    if args.enable_depth:
        left = pipeline.create(dai.node.MonoCamera)
        right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        left.setFps(args.fps)
        right.setFps(args.fps)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        depth_queue = stereo.depth.createOutputQueue()

    return pipeline, Queues(rgb=rgb_queue, depth=depth_queue)


def _read_camera_model(device: Any, width: int, height: int) -> CameraModel:
    matrix = np.array([[width, 0, width / 2], [0, width, height / 2], [0, 0, 1]], dtype=np.float64)
    distortion = np.zeros((5, 1), dtype=np.float64)
    usb_speed = "unknown"
    try:
        usb_speed = str(device.getUsbSpeed()).split(".")[-1]
    except Exception:
        pass
    try:
        calib = device.readCalibration()
        matrix = np.array(
            calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, width, height),
            dtype=np.float64,
        )
        coeffs = calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        distortion = np.array(coeffs, dtype=np.float64).reshape(-1, 1)
    except Exception as exc:
        print(f"warning: using approximate intrinsics: {exc}")
    return CameraModel(matrix=matrix, distortion=distortion, usb_speed=usb_speed)


def _pose_from_detection(
    detection: Any,
    camera: CameraModel,
    tag_size_m: float,
) -> tuple[np.ndarray, np.ndarray, tuple[float, float, float]] | None:
    if cv2 is None:
        return None
    half = tag_size_m / 2.0
    object_points = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    image_points = detection.corners.astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera.matrix,
        camera.distortion,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        return None
    rot, _ = cv2.Rodrigues(rvec)
    sy = math.sqrt(rot[0, 0] ** 2 + rot[1, 0] ** 2)
    if sy < 1e-6:
        roll = math.atan2(-rot[1, 2], rot[1, 1])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = 0.0
    else:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        pitch = math.atan2(-rot[2, 0], sy)
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    return rvec, tvec, (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def _tag_pixel_metrics(detection: Any) -> tuple[float, float]:
    corners = detection.corners.astype(np.float64)
    sides = [np.linalg.norm(corners[i] - corners[(i + 1) % 4]) for i in range(4)]
    area = 0.5 * abs(
        np.dot(corners[:, 0], np.roll(corners[:, 1], 1))
        - np.dot(corners[:, 1], np.roll(corners[:, 0], 1))
    )
    return float(np.mean(sides)), float(area)


def _depth_at(depth_frame: np.ndarray | None, center: tuple[float, float], rgb_shape: tuple[int, ...]) -> float | None:
    if depth_frame is None:
        return None
    y_scale = depth_frame.shape[0] / rgb_shape[0]
    x_scale = depth_frame.shape[1] / rgb_shape[1]
    x = int(center[0] * x_scale)
    y = int(center[1] * y_scale)
    if y < 2 or x < 2 or y >= depth_frame.shape[0] - 2 or x >= depth_frame.shape[1] - 2:
        return None
    patch = depth_frame[y - 2 : y + 3, x - 2 : x + 3].astype(np.float64)
    patch = patch[(patch > 0) & np.isfinite(patch)]
    if patch.size == 0:
        return None
    return float(np.median(patch) / 1000.0)


def _draw_text_panel(frame: np.ndarray, lines: list[str]) -> None:
    if cv2 is None:
        return
    x, y = 10, 22
    for line in lines:
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
        cv2.putText(frame, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        y += 22


def _open_csv(path: str | None):
    if not path:
        return None, None
    csv_path = Path(path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    handle = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        handle,
        fieldnames=[
            "time_s",
            "phase",
            "event",
            "fps",
            "detected",
            "tag_id",
            "decision_margin",
            "center_x",
            "center_y",
            "pixel_side",
            "pixel_area",
            "pose_x_m",
            "pose_y_m",
            "pose_z_m",
            "pose_range_m",
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "depth_center_m",
            "brightness_mean",
            "brightness_std",
            "brightness_p05",
            "brightness_p95",
            "tag_mean",
            "tag_std",
            "tag_contrast",
        ],
    )
    writer.writeheader()
    return handle, writer


def _set_phase_from_key(key: int, state: ExperimentState) -> None:
    phase_keys = {
        ord("0"): "baseline",
        ord("1"): "bright_light",
        ord("2"): "dim_light",
        ord("3"): "distance_sweep",
        ord("4"): "angle_sweep",
        ord("5"): "motion_blur",
        ord("6"): "edge_of_frame",
        ord("7"): "occlusion",
        ord("8"): "far_range",
        ord("9"): "near_range",
    }
    event_keys = {
        ord("m"): "manual_marker",
        ord("b"): "begin_segment",
        ord("e"): "end_segment",
    }
    if key in phase_keys:
        state.phase = phase_keys[key]
        state.event = f"phase:{state.phase}"
        state.last_key = chr(key)
        print(f"phase -> {state.phase}")
    elif key in event_keys:
        state.event = event_keys[key]
        state.last_key = chr(key)
        print(f"event -> {state.event} in phase {state.phase}")


def _base_row(
    now: float,
    started: float,
    fps: float,
    state: ExperimentState,
    lighting: dict[str, float | str],
    detected: bool,
) -> dict[str, float | int | str | bool]:
    return {
        "time_s": now - started,
        "phase": state.phase,
        "event": state.event,
        "fps": fps,
        "detected": detected,
        "tag_id": "",
        "decision_margin": "",
        "center_x": "",
        "center_y": "",
        "pixel_side": "",
        "pixel_area": "",
        "pose_x_m": "",
        "pose_y_m": "",
        "pose_z_m": "",
        "pose_range_m": "",
        "roll_deg": "",
        "pitch_deg": "",
        "yaw_deg": "",
        "depth_center_m": "",
        **lighting,
    }


def _run_loop(queues: Queues, camera: CameraModel, args: argparse.Namespace) -> None:
    if cv2 is None and args.display:
        raise RuntimeError("--display requires opencv-python")
    cv = cv2

    detector = Detector(
        families=args.tag_family,
        nthreads=2,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
    )
    csv_handle, csv_writer = _open_csv(args.csv)
    started = time.monotonic()
    frames = 0
    last_print = 0.0
    last_depth = None
    show_depth = args.enable_depth
    state = ExperimentState(phase=args.phase)

    try:
        while True:
            msg = queues.rgb.get()
            frame = msg.getCvFrame()
            if queues.depth is not None and queues.depth.has():
                last_depth = queues.depth.get().getFrame()

            gray = _gray_from_bgr(frame)
            detections = detector.detect(gray)
            frames += 1
            now = time.monotonic()
            fps = frames / max(now - started, 1e-6)
            frame_lighting = _lighting_metrics(gray)
            panel = [
                (
                    f"FPS {fps:.1f} | USB {camera.usb_speed} | "
                    f"depth {'on' if queues.depth else 'off'} | phase {state.phase}"
                ),
                (
                    f"light mean {frame_lighting['brightness_mean']:.1f} "
                    f"std {frame_lighting['brightness_std']:.1f} "
                    f"p05/p95 {frame_lighting['brightness_p05']:.0f}/"
                    f"{frame_lighting['brightness_p95']:.0f}"
                ),
                "keys: 0 base 1 bright 2 dim 3 dist 4 angle 5 motion 6 edge 7 occ 8 far 9 near",
                "keys: b begin e end m marker d depth q/esc quit",
            ]

            rows = []
            if not detections:
                panel.append("no tag detected")
                if args.log_all_frames:
                    rows.append(
                        _base_row(
                            now,
                            started,
                            fps,
                            state,
                            frame_lighting,
                            detected=False,
                        )
                    )
                if now - last_print >= 1.0:
                    print(
                        f"phase={state.phase} no tag detected | "
                        f"light={frame_lighting['brightness_mean']:.1f} fps={fps:.1f}"
                    )
                    last_print = now

            for detection in detections:
                side_px, area_px = _tag_pixel_metrics(detection)
                pose = _pose_from_detection(detection, camera, args.tag_size)
                depth_m = _depth_at(last_depth, tuple(detection.center), frame.shape)
                lighting = _lighting_metrics(gray, detection)
                pose_text = "pose unavailable"
                row = _base_row(now, started, fps, state, lighting, detected=True)
                row.update(
                    {
                        "tag_id": detection.tag_id,
                        "decision_margin": detection.decision_margin,
                        "center_x": detection.center[0],
                        "center_y": detection.center[1],
                        "pixel_side": side_px,
                        "pixel_area": area_px,
                        "pose_x_m": "",
                        "pose_y_m": "",
                        "pose_z_m": "",
                        "pose_range_m": "",
                        "roll_deg": "",
                        "pitch_deg": "",
                        "yaw_deg": "",
                        "depth_center_m": "" if depth_m is None else depth_m,
                    }
                )
                if pose is not None:
                    _, tvec, angles = pose
                    x_m, y_m, z_m = tvec.flatten().tolist()
                    range_m = float(np.linalg.norm(tvec))
                    roll, pitch, yaw = angles
                    pose_text = (
                        f"xyz=({x_m:+.2f},{y_m:+.2f},{z_m:.2f})m "
                        f"range={range_m:.2f}m rpy=({roll:+.0f},{pitch:+.0f},{yaw:+.0f})deg"
                    )
                    row.update(
                        {
                            "pose_x_m": x_m,
                            "pose_y_m": y_m,
                            "pose_z_m": z_m,
                            "pose_range_m": range_m,
                            "roll_deg": roll,
                            "pitch_deg": pitch,
                            "yaw_deg": yaw,
                        }
                    )
                marker = "OK" if detection.tag_id == args.expected_id else "UNEXPECTED"
                depth_text = "" if depth_m is None else f" depth={depth_m:.2f}m"
                message = (
                    f"phase={state.phase} {marker} id={detection.tag_id} "
                    f"margin={detection.decision_margin:.1f} side={side_px:.0f}px "
                    f"light={lighting['brightness_mean']:.1f} "
                    f"tagC={lighting['tag_contrast']} {pose_text}{depth_text}"
                )
                panel.append(message[:105])
                rows.append(row)
                if now - last_print >= args.print_period:
                    print(message + f" fps={fps:.1f}")
                    last_print = now

                if args.display:
                    if cv is None:
                        raise RuntimeError("--display requires opencv-python")
                    corners = detection.corners.astype(int)
                    for index in range(4):
                        cv.line(frame, tuple(corners[index]), tuple(corners[(index + 1) % 4]), (0, 255, 0), 2)
                    center = tuple(detection.center.astype(int))
                    cv.circle(frame, center, 5, (0, 255, 0), 2)
                    cv.putText(frame, str(detection.tag_id), center, cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if csv_writer is not None:
                for row in rows:
                    csv_writer.writerow(row)
                if csv_handle is not None:
                    csv_handle.flush()
                state.event = ""

            if args.display:
                if cv is None:
                    raise RuntimeError("--display requires opencv-python")
                _draw_text_panel(frame, panel)
                display = frame
                if show_depth and last_depth is not None:
                    norm = cv.normalize(last_depth, None, 0, 255, cv.NORM_MINMAX).astype(np.uint8)
                    depth_color = cv.applyColorMap(norm, cv.COLORMAP_TURBO)
                    depth_color = cv.resize(depth_color, (frame.shape[1], frame.shape[0]))
                    display = np.hstack([frame, depth_color])
                cv.imshow("OAK-D AprilTag telemetry", display)
                key = cv.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    return
                if key == ord("d"):
                    show_depth = not show_depth
                _set_phase_from_key(key, state)
    finally:
        if csv_handle is not None:
            csv_handle.close()


def _run_pipeline(pipeline: dai.Pipeline, queues: Queues, args: argparse.Namespace) -> None:
    if hasattr(dai.node, "XLinkOut"):
        with dai.Device(pipeline) as device:
            camera = _read_camera_model(device, args.width, args.height)
            print(f"Connected: {device.getDeviceName()} ({device.getMxId()})")
            print(f"USB speed: {camera.usb_speed}")
            queues.rgb = device.getOutputQueue("preview", maxSize=4, blocking=False)
            _run_loop(queues, camera, args)
    else:
        device = pipeline.getDefaultDevice()
        camera = _read_camera_model(device, args.width, args.height)
        pipeline.start()
        with pipeline:
            print("Connected to OAK-D")
            print(f"USB speed: {camera.usb_speed}")
            print("Point the camera at tag36h11 ID 0. Press q/Esc to stop.")
            _run_loop(queues, camera, args)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag-family", default="tag36h11")
    parser.add_argument("--expected-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.19, help="Printed black square edge in metres")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--display", action="store_true")
    parser.add_argument("--enable-depth", action="store_true", help="Also request stereo depth; press d to show/hide")
    parser.add_argument("--csv", help="Optional telemetry CSV output path")
    parser.add_argument("--print-period", type=float, default=0.25)
    parser.add_argument("--phase", default="baseline", help="Initial experiment phase label")
    parser.add_argument(
        "--log-all-frames",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Write CSV rows even when no tag is detected",
    )
    args = parser.parse_args()

    pipeline, queues = _build_pipeline(args)
    _run_pipeline(pipeline, queues, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
