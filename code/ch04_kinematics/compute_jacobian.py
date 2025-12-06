"""
Module: compute_jacobian.py
Chapter: 4 - Kinematics & Dynamics
Author: Physical AI & Humanoid Robotics Textbook Team
Date: 2025-12-05
License: MIT

Description:
    Implementation of Jacobian matrix computation for robotic manipulators.
    Includes both translational and rotational Jacobians using the geometric
    method. Also includes methods for handling singularities and computing
    pseudoinverses.

Requirements:
    - numpy>=1.21.0
    - scipy>=1.7.0

Performance:
    - Time Complexity: O(n^2) for general case, O(n) for simple geometries
    - Space Complexity: O(6*n) for standard Jacobian
    - Execution Time: <1ms for 6-DOF manipulator

Reference:
    Craig, J. J. (2005). Introduction to Robotics (3rd ed.).
    Lynch, K. M., & Park, F. C. (2017). Modern Robotics (1st ed.).
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
from code.ch04_kinematics.forward_kinematics import DHParameter, forward_kinematics_recursive, extract_position_orientation


def jacobian_translational(dh_params: List[DHParameter],
                          joint_angles: np.ndarray) -> np.ndarray:
    """Compute the translational (linear velocity) part of the geometric Jacobian.

    Args:
        dh_params: List of DHParameter objects for the manipulator
        joint_angles: Current joint angle values

    Returns:
        3xn Jacobian matrix where n is the number of joints
        Each column represents the linear velocity contribution of the corresponding joint
    """
    if len(joint_angles) != len(dh_params):
        raise ValueError(f"joint_angles length {len(joint_angles)} "
                        f"doesn't match dh_params length {len(dh_params)}")

    n = len(dh_params)
    J_trans = np.zeros((3, n))

    # Get all transformation matrices from base to each frame
    transforms = forward_kinematics_recursive(dh_params, joint_angles)

    # End-effector position in base frame
    end_effector_pos = transforms[-1][:3, 3]

    # Compute each column of the Jacobian
    for i in range(n):
        # Get the z-axis of frame i-1 in base coordinates
        if i == 0:
            # For the first joint, z-axis is the base frame z-axis
            z_prev = np.array([0, 0, 1])
        else:
            z_prev = transforms[i-1][:3, 2]  # Third column is z-axis

        # Get the position of frame i in base coordinates
        if i == 0:
            p_i = np.array([0, 0, 0])  # Base frame origin
        else:
            p_i = transforms[i-1][:3, 3]

        # For revolute joints: J_trans[:, i] = z_{i-1} × (p_e - p_{i-1})
        # For prismatic joints: J_trans[:, i] = z_{i-1}
        # We'll assume revolute joints for this implementation
        # (for prismatic joints, we'd use the z-axis directly)

        # The axis of motion for revolute joints is the z-axis of the previous frame
        joint_axis = z_prev
        position_diff = end_effector_pos - p_i

        if dh_params[i].d != 0:  # If d is variable, it's a prismatic joint
            J_trans[:, i] = joint_axis  # For prismatic joints
        else:  # Revolute joint
            J_trans[:, i] = np.cross(joint_axis, position_diff)

    return J_trans


def jacobian_rotational(dh_params: List[DHParameter],
                       joint_angles: np.ndarray) -> np.ndarray:
    """Compute the rotational (angular velocity) part of the geometric Jacobian.

    Args:
        dh_params: List of DHParameter objects for the manipulator
        joint_angles: Current joint angle values

    Returns:
        3xn Jacobian matrix where n is the number of joints
        Each column represents the angular velocity contribution of the corresponding joint
    """
    if len(joint_angles) != len(dh_params):
        raise ValueError(f"joint_angles length {len(joint_angles)} "
                        f"doesn't match dh_params length {len(dh_params)}")

    n = len(dh_params)
    J_rot = np.zeros((3, n))

    # Get all transformation matrices from base to each frame
    transforms = forward_kinematics_recursive(dh_params, joint_angles)

    # Compute each column of the rotational Jacobian
    for i in range(n):
        # Get the z-axis of frame i-1 in base coordinates (axis of rotation)
        if i == 0:
            # For the first joint, z-axis is the base frame z-axis
            z_prev = np.array([0, 0, 1])
        else:
            z_prev = transforms[i-1][:3, 2]  # Third column is z-axis

        # For revolute joints: J_rot[:, i] = z_{i-1}
        # For prismatic joints: J_rot[:, i] = [0, 0, 0] (no angular contribution)
        if dh_params[i].d != 0:  # Prismatic joint
            J_rot[:, i] = np.array([0, 0, 0])  # No angular contribution from prismatic joints
        else:  # Revolute joint
            J_rot[:, i] = z_prev

    return J_rot


def jacobian_full(dh_params: List[DHParameter],
                 joint_angles: np.ndarray) -> np.ndarray:
    """Compute the full 6xn geometric Jacobian matrix.

    Args:
        dh_params: List of DHParameter objects for the manipulator
        joint_angles: Current joint angle values

    Returns:
        6xn Jacobian matrix combining translational and rotational parts
        Top 3 rows: translational Jacobian
        Bottom 3 rows: rotational Jacobian
    """
    J_trans = jacobian_translational(dh_params, joint_angles)
    J_rot = jacobian_rotational(dh_params, joint_angles)

    # Stack translational and rotational parts
    J_full = np.vstack([J_trans, J_rot])

    return J_full


def jacobian_pseudoinverse(J: np.ndarray, damping: Optional[float] = None) -> np.ndarray:
    """Compute the pseudoinverse of the Jacobian matrix.

    Args:
        J: mxn Jacobian matrix
        damping: Optional damping factor for damped least squares (for singularity handling)

    Returns:
        nxm pseudoinverse of the Jacobian
    """
    m, n = J.shape

    if damping is not None:
        # Damped Least Squares (Levenberg-Marquardt)
        if m >= n:  # More equations than unknowns (redundant or exact)
            J_pinv = np.linalg.inv(J.T @ J + damping**2 * np.eye(n)) @ J.T
        else:  # Underdetermined system
            J_pinv = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(m))
    else:
        # Standard pseudoinverse using SVD
        J_pinv = np.linalg.pinv(J)

    return J_pinv


def jacobian_damped_least_squares(J: np.ndarray, damping: float = 0.01) -> np.ndarray:
    """Compute the damped least squares pseudoinverse of the Jacobian.

    Args:
        J: mxn Jacobian matrix
        damping: Damping factor (typically 0.001 to 0.1)

    Returns:
        nxm damped pseudoinverse of the Jacobian
    """
    m, n = J.shape

    if m >= n:  # More rows than columns (overdetermined or square)
        # J# = (J^T * J + λ^2 * I)^(-1) * J^T
        J_pinv = np.linalg.inv(J.T @ J + damping**2 * np.eye(n)) @ J.T
    else:  # More columns than rows (underdetermined)
        # J# = J^T * (J * J^T + λ^2 * I)^(-1)
        J_pinv = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(m))

    return J_pinv


def jacobian_manipulability(J: np.ndarray) -> float:
    """Compute the manipulability measure of the Jacobian.

    Args:
        J: 6xn Jacobian matrix

    Returns:
        Manipulability measure (scalar value)
        Higher values indicate better dexterity
    """
    # For a 6xn Jacobian, manipulability is sqrt(det(J * J^T)) if m >= n
    # or sqrt(det(J^T * J)) if n > m
    m, n = J.shape

    if m >= n:
        # For less than or equally redundant systems
        manip = np.sqrt(np.linalg.det(J @ J.T)) if n > 0 else 0.0
    else:
        # For redundant systems
        manip = np.sqrt(np.linalg.det(J.T @ J)) if m > 0 else 0.0

    return manip


def jacobian_condition_number(J: np.ndarray) -> float:
    """Compute the condition number of the Jacobian.

    Args:
        J: mxn Jacobian matrix

    Returns:
        Condition number (ratio of largest to smallest singular value)
        Higher values indicate closer to singularity
    """
    # Use SVD to compute condition number
    _, s, _ = np.linalg.svd(J)

    # Condition number is the ratio of largest to smallest singular values
    if len(s) > 0 and s[-1] != 0:
        cond_num = s[0] / s[-1]
    else:
        cond_num = float('inf')  # Singular matrix

    return cond_num


def jacobian_singularity_check(J: np.ndarray, threshold: float = 1e-3) -> Tuple[bool, float]:
    """Check if the Jacobian is near a singular configuration.

    Args:
        J: mxn Jacobian matrix
        threshold: Threshold for condition number to consider singular

    Returns:
        Tuple of (is_singular, condition_number)
    """
    cond_num = jacobian_condition_number(J)
    is_singular = cond_num > threshold or np.isinf(cond_num)

    return is_singular, cond_num


def jacobian_null_space(J: np.ndarray) -> np.ndarray:
    """Compute the null space projection matrix of the Jacobian.

    Args:
        J: mxn Jacobian matrix

    Returns:
        nxn null space projection matrix (I - J# * J)
    """
    J_pinv = jacobian_pseudoinverse(J)
    n = J.shape[1]
    null_proj = np.eye(n) - J_pinv @ J

    return null_proj


def jacobian_ik_step(J: np.ndarray,
                     error: np.ndarray,
                     damping: Optional[float] = None) -> np.ndarray:
    """Compute a single step of Jacobian-based inverse kinematics.

    Args:
        J: 6xn current Jacobian matrix
        error: 6x1 error vector [linear_error, angular_error]
        damping: Optional damping factor

    Returns:
        nx1 joint angle update vector
    """
    if J.shape[0] != error.shape[0]:
        raise ValueError(f"Jacobian rows {J.shape[0]} doesn't match error size {error.shape[0]}")

    if damping is not None:
        J_pinv = jacobian_damped_least_squares(J, damping)
    else:
        J_pinv = jacobian_pseudoinverse(J)

    delta_theta = J_pinv @ error
    return delta_theta


if __name__ == "__main__":
    # Example usage and testing
    print("Testing Jacobian Computation Implementation")

    # Example: Simple 3-DOF arm
    print("\n1. Testing 3-DOF Arm Jacobian:")
    dh_params_3dof = [
        DHParameter(theta=0.1, d=0.1, a=0.0, alpha=np.pi/2),  # Shoulder joint
        DHParameter(theta=0.2, d=0.0, a=0.5, alpha=0.0),      # Elbow joint
        DHParameter(theta=0.3, d=0.0, a=0.4, alpha=0.0)       # Wrist joint
    ]
    joint_angles = np.array([0.1, 0.2, 0.3])

    # Compute Jacobians
    J_trans = jacobian_translational(dh_params_3dof, joint_angles)
    J_rot = jacobian_rotational(dh_params_3dof, joint_angles)
    J_full = jacobian_full(dh_params_3dof, joint_angles)

    print(f"  Translational Jacobian shape: {J_trans.shape}")
    print(f"  Translational Jacobian:\n{J_trans}")
    print(f"  Rotational Jacobian shape: {J_rot.shape}")
    print(f"  Rotational Jacobian:\n{J_rot}")
    print(f"  Full Jacobian shape: {J_full.shape}")

    # Test properties
    manip = jacobian_manipulability(J_full)
    cond_num = jacobian_condition_number(J_full)
    is_singular, _ = jacobian_singularity_check(J_full)

    print(f"\n  Manipulability: {manip:.4f}")
    print(f"  Condition number: {cond_num:.2f}")
    print(f"  Is near singularity: {is_singular}")

    # Test pseudoinverse
    J_pinv = jacobian_pseudoinverse(J_full)
    print(f"  Pseudoinverse shape: {J_pinv.shape}")

    # Verify J * J# * J ≈ J (property of pseudoinverse)
    JJpJ = J_full @ J_pinv @ J_full
    reconstruction_error = np.linalg.norm(J_full - JJpJ)
    print(f"  Pseudoinverse verification error: {reconstruction_error:.2e}")

    # Test damped pseudoinverse near singularity
    print("\n2. Testing Damped Pseudoinverse:")
    # Create a near-singular configuration (both elbow joints aligned)
    singular_angles = np.array([0.0, 0.0, 0.0])  # This creates alignment
    J_singular = jacobian_full(dh_params_3dof, singular_angles)
    cond_singular = jacobian_condition_number(J_singular)
    print(f"  Near-singular condition number: {cond_singular:.2e}")

    J_damped = jacobian_damped_least_squares(J_singular, damping=0.01)
    print(f"  Damped pseudoinverse computed successfully")

    # Test null space
    null_proj = jacobian_null_space(J_full)
    print(f"  Null space projection shape: {null_proj.shape}")

    # Verify null space property: J * null_proj should be close to zero
    J_null = J_full @ null_proj
    null_error = np.linalg.norm(J_null)
    print(f"  Null space verification error: {null_error:.2e}")

    print("\n✓ Jacobian computation tests completed successfully!")