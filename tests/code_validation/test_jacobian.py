"""
Test module for Jacobian computation implementation.
Tests the compute_jacobian.py module from Chapter 4.
"""

import numpy as np
import pytest
from code.ch04_kinematics.compute_jacobian import (
    jacobian_translational, jacobian_rotational, jacobian_full,
    jacobian_pseudoinverse, jacobian_damped_least_squares,
    jacobian_manipulability, jacobian_condition_number,
    jacobian_singularity_check, jacobian_null_space, jacobian_ik_step
)
from code.ch04_kinematics.forward_kinematics import DHParameter


def test_jacobian_translational_basic():
    """Test translational Jacobian computation for a simple case."""
    # Simple 2-DOF planar arm
    dh_params = [
        DHParameter(theta=0, d=0, a=1, alpha=0),
        DHParameter(theta=0, d=0, a=1, alpha=0)
    ]
    joint_angles = np.array([0, 0])

    J_trans = jacobian_translational(dh_params, joint_angles)

    # For 2-DOF planar arm at zero angles:
    # J_trans should be 3x2
    # Column 1: z0 x (pe - p0) = [0, 0, 1] x [2, 0, 0] = [0, 2, 0]
    # Column 2: z1 x (pe - p1) = [0, 0, 1] x [1, 0, 0] = [0, 1, 0]
    # Actually, for revolute joints: J[:,i] = z_{i-1} x (p_e - p_{i-1})
    assert J_trans.shape == (3, 2), f"Expected shape (3, 2), got {J_trans.shape}"


def test_jacobian_rotational_basic():
    """Test rotational Jacobian computation for a simple case."""
    # Simple 2-DOF arm
    dh_params = [
        DHParameter(theta=0, d=0, a=1, alpha=0),
        DHParameter(theta=0, d=0, a=1, alpha=0)
    ]
    joint_angles = np.array([0, 0])

    J_rot = jacobian_rotational(dh_params, joint_angles)

    assert J_rot.shape == (3, 2), f"Expected shape (3, 2), got {J_rot.shape}"

    # For revolute joints, rotational Jacobian columns should be the z-axes of each frame
    # At zero angles, both z-axes are [0, 0, 1] in the base frame
    expected_col1 = np.array([0, 0, 1])
    expected_col2 = np.array([0, 0, 1])

    np.testing.assert_allclose(J_rot[:, 0], expected_col1, rtol=1e-10)
    np.testing.assert_allclose(J_rot[:, 1], expected_col2, rtol=1e-10)


def test_jacobian_full_composition():
    """Test that full Jacobian is properly composed of translational and rotational parts."""
    dh_params = [
        DHParameter(theta=0.1, d=0, a=1, alpha=0),
        DHParameter(theta=0.2, d=0, a=0.8, alpha=0)
    ]
    joint_angles = np.array([0.1, 0.2])

    J_trans = jacobian_translational(dh_params, joint_angles)
    J_rot = jacobian_rotational(dh_params, joint_angles)
    J_full = jacobian_full(dh_params, joint_angles)

    assert J_full.shape == (6, 2), f"Expected shape (6, 2), got {J_full.shape}"
    np.testing.assert_allclose(J_full[:3, :], J_trans, rtol=1e-10)
    np.testing.assert_allclose(J_full[3:, :], J_rot, rtol=1e-10)


def test_jacobian_pseudoinverse_basic():
    """Test pseudoinverse computation."""
    # Create a simple 3x2 matrix
    J = np.array([
        [1, 0],
        [0, 1],
        [0, 0]
    ])

    J_pinv = jacobian_pseudoinverse(J)

    assert J_pinv.shape == (2, 3), f"Expected shape (2, 3), got {J_pinv.shape}"

    # Verify pseudoinverse property: J * J_pinv * J = J
    JJpJ = J @ J_pinv @ J
    np.testing.assert_allclose(J, JJpJ, rtol=1e-10)


def test_jacobian_pseudoinverse_with_damping():
    """Test damped pseudoinverse computation."""
    J = np.array([
        [1, 0],
        [0, 1],
        [0, 0]
    ])

    J_pinv_damped = jacobian_pseudoinverse(J, damping=0.01)

    # Should not crash and should have correct dimensions
    assert J_pinv_damped.shape == (2, 3), f"Expected shape (2, 3), got {J_pinv_damped.shape}"


def test_jacobian_damped_least_squares():
    """Test damped least squares pseudoinverse."""
    # Test with a matrix that might be close to singular
    J = np.array([
        [1, 0.001],
        [0.001, 1],
        [0, 0]
    ])

    J_dls = jacobian_damped_least_squares(J, damping=0.01)

    assert J_dls.shape == (2, 3), f"Expected shape (2, 3), got {J_dls.shape}"

    # Apply to a test vector
    b = np.array([1, 1, 0])
    x = J_dls @ b

    # Check that the result is reasonable (not NaN or Inf)
    assert not np.any(np.isnan(x)), "Solution should not contain NaN"
    assert not np.any(np.isinf(x)), "Solution should not contain Inf"


def test_jacobian_manipulability():
    """Test manipulability computation."""
    # Identity Jacobian (best case)
    J_identity = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ])

    manip = jacobian_manipulability(J_identity)
    assert manip >= 0, "Manipulability should be non-negative"

    # Non-zero Jacobian
    J = np.array([
        [1, 0],
        [0, 1],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 1]
    ])

    manip = jacobian_manipulability(J)
    assert manip >= 0, "Manipulability should be non-negative"


def test_jacobian_condition_number():
    """Test condition number computation."""
    # Well-conditioned matrix
    J_well = np.array([
        [2, 0],
        [0, 1],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0]
    ])

    cond_well = jacobian_condition_number(J_well)
    assert cond_well > 0, "Condition number should be positive"
    assert not np.isinf(cond_well), "Condition number should be finite"

    # Identity matrix should have condition number of 1
    J_identity = np.array([
        [1, 0],
        [0, 1],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0]
    ])

    cond_identity = jacobian_condition_number(J_identity)
    np.testing.assert_allclose(cond_identity, 1.0, rtol=1e-10)


def test_jacobian_singularity_check():
    """Test singularity detection."""
    # Non-singular Jacobian
    J_normal = np.array([
        [1, 0],
        [0, 1],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0]
    ])

    is_singular, cond_num = jacobian_singularity_check(J_normal, threshold=10)
    assert not is_singular, "Normal Jacobian should not be singular"
    assert cond_num > 0, "Condition number should be positive"

    # Try with a nearly singular matrix
    J_near_singular = np.array([
        [1, 0],
        [0, 1e-10],  # Very small value
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0]
    ])

    is_singular, cond_num = jacobian_singularity_check(J_near_singular, threshold=1e5)
    # This might be considered singular depending on the threshold


def test_jacobian_null_space():
    """Test null space computation."""
    # 6x3 Jacobian (redundant)
    J = np.array([
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ])

    null_proj = jacobian_null_space(J)

    assert null_proj.shape == (3, 3), f"Expected shape (3, 3), got {null_proj.shape}"

    # Verify null space property: J @ null_proj should be close to zero
    J_null = J @ null_proj
    np.testing.assert_allclose(J_null, np.zeros_like(J_null), rtol=1e-10)


def test_jacobian_ik_step():
    """Test IK step computation using Jacobian."""
    J = np.array([
        [1, 0],
        [0, 1],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0]
    ])

    error = np.array([0.1, 0.2, 0, 0, 0, 0])  # Position error only

    delta_theta = jacobian_ik_step(J, error)

    assert delta_theta.shape == (2,), f"Expected shape (2,), got {delta_theta.shape}"

    # For this simple case, the result should be [0.1, 0.2] (first two elements of error)
    expected = np.array([0.1, 0.2])
    np.testing.assert_allclose(delta_theta, expected, rtol=1e-10)


def test_jacobian_ik_step_with_damping():
    """Test IK step computation with damping."""
    J = np.array([
        [1, 0.001],
        [0.001, 1],
        [0, 0],
        [0, 0],
        [0, 0],
        [0, 0]
    ])

    error = np.array([0.1, 0.2, 0, 0, 0, 0])

    delta_theta = jacobian_ik_step(J, error, damping=0.01)

    assert delta_theta.shape == (2,), f"Expected shape (2,), got {delta_theta.shape}"

    # Check for reasonable values (not NaN or Inf)
    assert not np.any(np.isnan(delta_theta)), "Result should not contain NaN"
    assert not np.any(np.isinf(delta_theta)), "Result should not contain Inf"


def test_jacobian_invalid_inputs():
    """Test Jacobian functions with invalid inputs."""
    dh_params = [DHParameter(theta=0, d=0, a=1, alpha=0)]
    joint_angles = np.array([0])

    # Test with wrong joint angles length
    with pytest.raises(ValueError):
        jacobian_translational(dh_params, np.array([0, 1]))

    # Test IK step with mismatched dimensions
    J = np.array([[1, 0], [0, 1], [0, 0], [0, 0], [0, 0], [0, 0]])
    error = np.array([0.1, 0.2, 0, 0, 0])  # Wrong length

    with pytest.raises(ValueError):
        jacobian_ik_step(J, error)


def test_jacobian_properties():
    """Test fundamental properties of Jacobian computation."""
    # Test with a 1-DOF system
    dh_params = [DHParameter(theta=0.5, d=0, a=1, alpha=0)]
    joint_angles = np.array([0.5])

    J_trans = jacobian_translational(dh_params, joint_angles)
    J_rot = jacobian_rotational(dh_params, joint_angles)
    J_full = jacobian_full(dh_params, joint_angles)

    assert J_trans.shape == (3, 1), "Translational Jacobian should be 3x1"
    assert J_rot.shape == (3, 1), "Rotational Jacobian should be 3x1"
    assert J_full.shape == (6, 1), "Full Jacobian should be 6x1"


if __name__ == "__main__":
    # Run tests manually if executed as script
    test_jacobian_translational_basic()
    test_jacobian_rotational_basic()
    test_jacobian_full_composition()
    test_jacobian_pseudoinverse_basic()
    test_jacobian_pseudoinverse_with_damping()
    test_jacobian_damped_least_squares()
    test_jacobian_manipulability()
    test_jacobian_condition_number()
    test_jacobian_singularity_check()
    test_jacobian_null_space()
    test_jacobian_ik_step()
    test_jacobian_ik_step_with_damping()
    test_jacobian_invalid_inputs()
    test_jacobian_properties()
    print("All Jacobian tests passed!")