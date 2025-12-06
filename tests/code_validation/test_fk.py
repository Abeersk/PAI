"""
Test module for forward kinematics implementation.
Tests the forward_kinematics.py module from Chapter 4.
"""

import numpy as np
import pytest
from code.ch04_kinematics.forward_kinematics import (
    DHParameter, dh_transform, forward_kinematics,
    forward_kinematics_recursive, extract_position_orientation
)


def test_dh_parameter_creation():
    """Test creation of DHParameter objects with valid values."""
    dh = DHParameter(theta=0.1, d=0.2, a=0.3, alpha=0.4)
    assert dh.theta == 0.1
    assert dh.d == 0.2
    assert dh.a == 0.3
    assert dh.alpha == 0.4


def test_dh_parameter_validation():
    """Test DHParameter validation."""
    with pytest.raises(TypeError):
        DHParameter(theta="invalid", d=0.2, a=0.3, alpha=0.4)


def test_dh_transform_identity():
    """Test DH transform with zero parameters returns identity matrix."""
    dh = DHParameter(theta=0, d=0, a=0, alpha=0)
    T = dh_transform(dh)
    expected = np.eye(4)
    np.testing.assert_allclose(T, expected, rtol=1e-10)


def test_dh_transform_rotation_only():
    """Test DH transform with rotation only (non-zero theta)."""
    theta = np.pi / 2
    dh = DHParameter(theta=theta, d=0, a=0, alpha=0)
    T = dh_transform(dh)

    expected = np.array([
        [0, -1, 0, 0],
        [1,  0, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ])
    np.testing.assert_allclose(T, expected, rtol=1e-6, atol=1e-10)


def test_dh_transform_translation_only():
    """Test DH transform with translation only (non-zero d)."""
    d = 5.0
    dh = DHParameter(theta=0, d=d, a=0, alpha=0)
    T = dh_transform(dh)

    expected = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, d],
        [0, 0, 0, 1]
    ])
    np.testing.assert_allclose(T, expected, rtol=1e-10)


def test_forward_kinematics_empty_params():
    """Test forward kinematics with empty DH parameters list."""
    with pytest.raises(ValueError):
        forward_kinematics([])


def test_forward_kinematics_single_joint():
    """Test forward kinematics with a single joint."""
    dh_params = [DHParameter(theta=0, d=0, a=1, alpha=0)]
    T = forward_kinematics(dh_params)

    # Should be identity since theta=0 and alpha=0
    expected = np.array([
        [1, 0, 0, 1],  # a*cos(theta) = 1*1 = 1
        [0, 1, 0, 0],  # a*sin(theta) = 1*0 = 0
        [0, 0, 1, 0],  # d = 0
        [0, 0, 0, 1]
    ])
    np.testing.assert_allclose(T, expected, rtol=1e-10)


def test_forward_kinematics_2dof_planar():
    """Test forward kinematics for a simple 2-DOF planar arm."""
    dh_params = [
        DHParameter(theta=0, d=0, a=1, alpha=0),  # First link
        DHParameter(theta=0, d=0, a=1, alpha=0)   # Second link
    ]
    T = forward_kinematics(dh_params)

    # Both joints at 0, end effector should be at (2, 0, 0)
    expected = np.array([
        [1, 0, 0, 2],  # a1*cos(0) + a2*cos(0) = 1 + 1 = 2
        [0, 1, 0, 0],  # a1*sin(0) + a2*sin(0) = 0 + 0 = 0
        [0, 0, 1, 0],  # d1 + d2 = 0 + 0 = 0
        [0, 0, 0, 1]
    ])
    np.testing.assert_allclose(T, expected, rtol=1e-10)


def test_forward_kinematics_with_joint_angles():
    """Test forward kinematics with explicit joint angles."""
    dh_params = [
        DHParameter(theta=np.pi/2, d=0, a=1, alpha=0),
        DHParameter(theta=np.pi/2, d=0, a=1, alpha=0)
    ]
    # Override with different angles
    joint_angles = np.array([0, 0])
    T = forward_kinematics(dh_params, joint_angles=joint_angles)

    # Both angles set to 0 regardless of initial theta values
    expected = np.array([
        [1, 0, 0, 2],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    np.testing.assert_allclose(T, expected, rtol=1e-10)


def test_forward_kinematics_joint_angles_length_mismatch():
    """Test forward kinematics with mismatched joint angles length."""
    dh_params = [DHParameter(theta=0, d=0, a=1, alpha=0)]
    joint_angles = np.array([0, 1])  # Wrong length

    with pytest.raises(ValueError):
        forward_kinematics(dh_params, joint_angles=joint_angles)


def test_extract_position_orientation():
    """Test extraction of position and orientation from transformation matrix."""
    T = np.array([
        [1, 0, 0, 5],
        [0, 1, 0, 6],
        [0, 0, 1, 7],
        [0, 0, 0, 1]
    ])

    pos, rot = extract_position_orientation(T)
    expected_pos = np.array([5, 6, 7])
    expected_rot = np.eye(3)

    np.testing.assert_allclose(pos, expected_pos)
    np.testing.assert_allclose(rot, expected_rot)


def test_forward_kinematics_recursive():
    """Test forward kinematics for all intermediate frames."""
    dh_params = [
        DHParameter(theta=0, d=0, a=1, alpha=0),
        DHParameter(theta=0, d=0, a=1, alpha=0)
    ]

    transforms = forward_kinematics_recursive(dh_params)

    assert len(transforms) == 2  # Should return 2 transforms

    # First transform (from base to frame 1)
    T1 = transforms[0]
    expected_T1 = np.array([
        [1, 0, 0, 1],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    np.testing.assert_allclose(T1, expected_T1, rtol=1e-10)

    # Second transform (from base to frame 2, which is end-effector)
    T2 = transforms[1]
    expected_T2 = np.array([
        [1, 0, 0, 2],  # 1 + 1 = 2
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    np.testing.assert_allclose(T2, expected_T2, rtol=1e-10)


def test_fundamental_transformation_properties():
    """Test that transformation matrices maintain fundamental properties."""
    dh = DHParameter(theta=np.pi/4, d=0.1, a=0.5, alpha=np.pi/6)
    T = dh_transform(dh)

    # Check that T is 4x4
    assert T.shape == (4, 4)

    # Check that the rotation part is orthonormal
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), rtol=1e-6, atol=1e-10)

    # Check that the bottom row is [0, 0, 0, 1]
    np.testing.assert_allclose(T[3, :], [0, 0, 0, 1], rtol=1e-6, atol=1e-10)


if __name__ == "__main__":
    # Run tests manually if executed as script
    test_dh_parameter_creation()
    test_dh_parameter_validation()
    test_dh_transform_identity()
    test_dh_transform_rotation_only()
    test_dh_transform_translation_only()
    test_forward_kinematics_empty_params()
    test_forward_kinematics_single_joint()
    test_forward_kinematics_2dof_planar()
    test_forward_kinematics_with_joint_angles()
    test_forward_kinematics_joint_angles_length_mismatch()
    test_extract_position_orientation()
    test_forward_kinematics_recursive()
    test_fundamental_transformation_properties()
    print("All forward kinematics tests passed!")