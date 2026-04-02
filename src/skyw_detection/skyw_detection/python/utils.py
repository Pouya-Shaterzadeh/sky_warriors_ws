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

