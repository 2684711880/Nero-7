"""Coordinate transforms for a D405 depth observation; metres and radians."""
import numpy as np
from scipy.spatial.transform import Rotation


def transform_depth_observation(base_flange, flange_color, color_depth, center, normal):
    matrices = [np.asarray(x, dtype=float) for x in (base_flange, flange_color, color_depth)]
    for matrix in matrices:
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError('Invalid transform')
        r = matrix[:3, :3]
        if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7) or not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1, atol=1e-5):
            raise ValueError('Transform is not rigid')
    center = np.asarray(center, dtype=float)
    normal = np.asarray(normal, dtype=float)
    if center.shape != (3,) or normal.shape != (3,) or not np.isfinite(center).all() or not np.isfinite(normal).all() or np.linalg.norm(normal) < 1e-8:
        raise ValueError('Invalid hole observation')
    base_depth = matrices[0] @ matrices[1] @ matrices[2]
    return (base_depth @ np.r_[center, 1])[:3], base_depth[:3, :3] @ (normal / np.linalg.norm(normal))


def flange_goal(center, outward_normal, current_flange_rotation, tip_in_flange, axis_in_flange, clearance_m):
    """Align the measured tool axis into the hole, keeping its remaining roll minimal."""
    center, normal, tip, axis = [np.array(x, dtype=float, copy=True) for x in (center, outward_normal, tip_in_flange, axis_in_flange)]
    rotation = np.asarray(current_flange_rotation, dtype=float)
    if any(x.shape != (3,) or not np.isfinite(x).all() for x in (center, normal, tip, axis)) or rotation.shape != (3, 3):
        raise ValueError('Invalid tool or target geometry')
    if np.linalg.norm(normal) < 1e-8 or np.linalg.norm(axis) < 1e-8 or not np.isfinite(clearance_m):
        raise ValueError('Invalid normal, tool axis or clearance')
    normal /= np.linalg.norm(normal)
    axis /= np.linalg.norm(axis)
    current_axis = rotation @ axis
    desired_axis = -normal
    cross = np.cross(current_axis, desired_axis)
    dot = np.clip(current_axis @ desired_axis, -1., 1.)
    if np.linalg.norm(cross) < 1e-9:
        if dot < 0:
            basis = np.eye(3)[np.argmin(np.abs(current_axis))]
            perpendicular = np.cross(current_axis, basis)
            adjustment = Rotation.from_rotvec(np.pi * perpendicular / np.linalg.norm(perpendicular)).as_matrix()
        else:
            adjustment = np.eye(3)
    else:
        adjustment = Rotation.from_rotvec(np.arctan2(np.linalg.norm(cross), dot) * cross / np.linalg.norm(cross)).as_matrix()
    goal = np.eye(4)
    goal[:3, :3] = adjustment @ rotation
    goal[:3, 3] = center + normal * clearance_m - goal[:3, :3] @ tip
    return goal
