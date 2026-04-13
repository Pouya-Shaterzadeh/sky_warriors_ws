from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np
from geometry_msgs.msg import Quaternion


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Create a 3x3 rotation matrix from roll/pitch/yaw (radians)."""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    r = np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )
    return r


def rotation_matrix_from_quaternion_wxyz(
    w: float, x: float, y: float, z: float
) -> np.ndarray:
    """Create a 3x3 rotation matrix from a Hamilton quaternion in w,x,y,z order."""
    q = np.array([w, x, y, z], dtype=float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.eye(3, dtype=float)
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def normalize_quaternion(q: Quaternion) -> Quaternion:
    """Normalize a quaternion, falling back to identity if degenerate."""
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-12:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return Quaternion(x=q.x / norm, y=q.y / norm, z=q.z / norm, w=q.w / norm)


def rotate_vector_by_quaternion(q: Quaternion, v: Iterable[float]) -> np.ndarray:
    """Rotate vector `v` by quaternion `q`.

    Uses the standard quaternion rotation formula: v' = q * v * q_conj
    (implemented via equivalent matrix multiplication).
    """
    q = normalize_quaternion(q)
    x, y, z, w = q.x, q.y, q.z, q.w
    vx, vy, vz = v

    # Rotation matrix derived from quaternion components.
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return r @ np.array([vx, vy, vz], dtype=float)


def estimate_depth_from_diameter_px(
    diameter_px: float, fx_px: float, pad_real_size_m: float
) -> float:
    """Estimate depth Z (meters) from apparent pad diameter in pixels.

    Pinhole approximation:
      Z = (pad_real_size_m * fx_px) / diameter_px
    """
    diameter_px = max(float(diameter_px), 1e-6)
    fx_px = max(float(fx_px), 1e-6)
    return (pad_real_size_m * fx_px) / diameter_px


def pixel_to_camera_xyz(
    u_px: float,
    v_px: float,
    z_m: float,
    fx_px: float,
    fy_px: float,
    cx_px: float,
    cy_px: float,
) -> np.ndarray:
    """Convert pixel (u,v) + depth to camera optical-frame coordinates (X right, Y down, Z forward)."""
    x = (u_px - cx_px) * z_m / fx_px
    y = (v_px - cy_px) * z_m / fy_px
    return np.array([x, y, z_m], dtype=float)


def pixel_to_camera_ray(
    u_px: float,
    v_px: float,
    fx_px: float,
    fy_px: float,
    cx_px: float,
    cy_px: float,
) -> np.ndarray:
    """Return a normalized camera-optical ray for a pixel (X right, Y down, Z forward)."""
    ray = np.array(
        [
            (u_px - cx_px) / max(float(fx_px), 1e-6),
            (v_px - cy_px) / max(float(fy_px), 1e-6),
            1.0,
        ],
        dtype=float,
    )
    ray /= max(np.linalg.norm(ray), 1e-6)
    return ray


def intersect_ray_with_horizontal_plane(
    origin_xyz: Iterable[float],
    direction_xyz: Iterable[float],
    plane_z: float,
) -> np.ndarray | None:
    """Intersect a ray with a horizontal plane z=plane_z in the same frame."""
    origin = ensure_numpy_vec3(origin_xyz)
    direction = ensure_numpy_vec3(direction_xyz)
    denom = float(direction[2])
    if abs(denom) < 1e-9:
        return None
    scale = (float(plane_z) - float(origin[2])) / denom
    if scale < 0.0:
        return None
    return origin + scale * direction


def ned_to_world_xyz(ned_xyz: Iterable[float], spawn_xy: Iterable[float]) -> np.ndarray:
    """Convert local PX4 NED coordinates into Gazebo ENU/world coordinates."""
    ned = ensure_numpy_vec3(ned_xyz)
    spawn = np.asarray(list(spawn_xy), dtype=float).reshape(-1)
    if spawn.shape[0] != 2:
        raise ValueError(f"Expected 2D spawn offset, got shape {spawn.shape}")
    return np.array(
        [
            ned[1] + spawn[0],
            ned[0] + spawn[1],
            -ned[2],
        ],
        dtype=float,
    )


def ensure_numpy_vec3(v: Iterable[float]) -> np.ndarray:
    arr = np.asarray(list(v), dtype=float).reshape(-1)
    if arr.shape[0] != 3:
        raise ValueError(f"Expected 3-vector, got shape {arr.shape}")
    return arr


@dataclass(frozen=True)
class Pose3D:
    """Simple pose container for 3D transforms."""

    position_xyz: np.ndarray  # (x,y,z) in target frame
    orientation_quat: Quaternion  # orientation of the body in the target frame
