"""Live 6-DoF Vive Tracker visualization through OpenVR and Rerun.

This client does not need an HMD.  It does require a running OpenVR runtime
(normally SteamVR), because OpenVR is the client API exposed by that runtime.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import numpy as np
import openvr
import rerun as rr


AXIS_COLORS = [[255, 70, 70], [70, 255, 70], [70, 120, 255]]
IDENTITY_3X3 = np.eye(3)


@dataclass(frozen=True)
class Device:
    index: int
    serial: str
    transform: np.ndarray


def openvr_matrix_to_numpy(matrix: object) -> np.ndarray:
    """Convert an OpenVR HmdMatrix34_t to a homogeneous 4x4 transform."""
    return np.array(
        [
            [matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3]],
            [matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3]],
            [matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3]],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def device_serial(vr_system: object, index: int) -> str:
    """Return a stable hardware identifier, falling back to the device index."""
    try:
        serial = vr_system.getStringTrackedDeviceProperty(
            index, openvr.Prop_SerialNumber_String
        )
        return serial or f"device-{index}"
    except openvr.OpenVRError:
        return f"device-{index}"


def valid_devices(vr_system: object, poses: object, device_class: int) -> list[Device]:
    devices: list[Device] = []
    for index, pose in enumerate(poses):
        if not pose.bPoseIsValid:
            continue
        if vr_system.getTrackedDeviceClass(index) != device_class:
            continue
        devices.append(
            Device(index, device_serial(vr_system, index), openvr_matrix_to_numpy(pose.mDeviceToAbsoluteTracking))
        )
    return devices


def choose_reference(base_stations: list[Device], requested_serial: str | None) -> Device | None:
    if requested_serial:
        return next((base for base in base_stations if base.serial == requested_serial), None)
    return min(base_stations, key=lambda base: base.index, default=None)


def log_axes(path: str, length: float) -> None:
    rr.log(
        path,
        rr.Arrows3D(
            vectors=[[length, 0, 0], [0, length, 0], [0, 0, length]],
            colors=AXIS_COLORS,
            radii=length / 25,
        ),
    )


def log_reference_frame() -> None:
    rr.log("world/base_station", rr.Transform3D(translation=[0, 0, 0], mat3x3=IDENTITY_3X3))
    log_axes("world/base_station/axes", 0.2)
    rr.log(
        "world/base_station/body",
        rr.Boxes3D(half_sizes=[[0.06, 0.03, 0.03]], colors=[[230, 190, 30]]),
    )


def log_tracker(serial: str, transform: np.ndarray) -> None:
    # Serial numbers are stable across restarts, unlike OpenVR device indices.
    entity = f"world/trackers/{serial}"
    rr.log(entity, rr.Transform3D(translation=transform[:3, 3], mat3x3=transform[:3, :3]))
    rr.log(
        f"{entity}/body",
        rr.Boxes3D(half_sizes=[[0.04, 0.025, 0.015]], colors=[[80, 180, 255]]),
    )
    log_axes(f"{entity}/axes", 0.1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--reference-serial",
        help="Serial number of the base station that defines the world frame. "
        "If omitted, the lowest OpenVR device index is used.",
    )
    parser.add_argument("--hz", type=float, default=90.0, help="Polling frequency (default: 90).")
    parser.add_argument("--no-spawn", action="store_true", help="Connect to an existing Rerun viewer instead of opening one.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.hz <= 0:
        raise ValueError("--hz must be positive")

    rr.init("vive_tracker_6dof", spawn=not args.no_spawn)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)

    try:
        vr_system = openvr.init(openvr.VRApplication_Background)
    except openvr.OpenVRError as error:
        print(
            f"Unable to initialize OpenVR: {error}. Start the OpenVR runtime (typically SteamVR) and check tracker dongles.",
            file=sys.stderr,
        )
        return 2

    print("OpenVR connected. Waiting for valid tracker and base-station poses. Press Ctrl+C to stop.")
    announced_reference: str | None = None
    period = 1.0 / args.hz

    try:
        while True:
            started = time.monotonic()
            poses = vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
            )
            base_stations = valid_devices(vr_system, poses, openvr.TrackedDeviceClass_TrackingReference)
            trackers = valid_devices(vr_system, poses, openvr.TrackedDeviceClass_GenericTracker)
            reference = choose_reference(base_stations, args.reference_serial)

            if reference is None:
                if args.reference_serial and base_stations:
                    seen = ", ".join(base.serial for base in base_stations)
                    print(f"Requested base station '{args.reference_serial}' is not valid. Available: {seen}", file=sys.stderr)
                time.sleep(period)
                continue

            if reference.serial != announced_reference:
                print(f"World frame: base station {reference.serial} (OpenVR index {reference.index})")
                announced_reference = reference.serial
                log_reference_frame()

            # T_base_tracker = inverse(T_universe_base) @ T_universe_tracker.
            universe_to_base = np.linalg.inv(reference.transform)
            for tracker in trackers:
                log_tracker(tracker.serial, universe_to_base @ tracker.transform)

            time.sleep(max(0.0, period - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print("Stopping tracker visualization.")
    finally:
        openvr.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
