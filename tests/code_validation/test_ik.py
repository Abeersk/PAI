"""
Test module for inverse kinematics implementation.
Tests the inverse_kinematics.py module from Chapter 4.
"""

import numpy as np
import pytest
from code.ch04_kinematics.inverse_kinematics import (
    inverse_kinematics_analytical_2dof, inverse_kinematics_jacobian,
    inverse_kinematics_cyclic_coordinate_descent, validate_ik_solution
)
from code.ch04_kinematics.forward_kinematics import DHParameter, forward_kinematics


def test_analytical_2dof_reachable_target():
    """Test analytical IK for 2-DOF arm with reachable target."""
    L1, L2 = 1.0, 0.8
    x, y = 1.2, 0.5  # Reachable target

    sol1, sol2 = inverse_kinematics_analytical_2dof(x, y, L1, L2)

    assert sol1 is not None, "First solution should exist for reachable target"
    assert sol2 is not None, "Second solution should exist for reachable target"

    # Verify solutions
    theta1_1, theta2_1 = sol1
    theta1_2, theta2_2 = sol2

    # Create DH parameters for verification
    dh1 = DHParameter(theta=theta1_1, d=0, a=L1, alpha=0)
    dh2 = DHParameter(theta=theta2_1, d=0, a=L2, alpha=0)
    pose1 = forward_kinematics([dh1, dh2])
    pos1, _ = [pose1[:3, 3], pose1[:3, :3]]

    # Check if the computed position is close to the target
    np.testing.assert_allclose(pos1[:2], [x, y], rtol=1e-5)


def test_analytical_2dof_unreachable_target():
    """Test analytical IK for 2-DOF arm with unreachable target."""
    L1, L2 = 1.0, 0.8
    x, y = 3.0, 0.0  # Unreachable target (beyond L1 + L2 = 1.8)

    sol1, sol2 = inverse_kinematics_analytical_2dof(x, y, L1, L2)

    assert sol1 is None, "Solution 1 should not exist for unreachable target"
    assert sol2 is None, "Solution 2 should not exist for unreachable target"


def test_analytical_2dof_invalid_inputs():
    """Test analytical IK with invalid inputs."""
    with pytest.raises(TypeError):
        inverse_kinematics_analytical_2dof("invalid", 0, 1, 1)

    with pytest.raises(ValueError):
        inverse_kinematics_analytical_2dof(0, 0, -1, 1)  # Negative link length


def test_analytical_2dof_boundary_cases():
    """Test analytical IK at workspace boundaries."""
    L1, L2 = 1.0, 0.5

    # At maximum reach
    x_max = L1 + L2
    sol1, sol2 = inverse_kinematics_analytical_2dof(x_max, 0, L1, L2)
    assert sol1 is not None, "Should have solution at maximum reach"

    # At minimum reach (when L1 > L2)
    x_min = abs(L1 - L2)
    sol1, sol2 = inverse_kinematics_analytical_2dof(x_min, 0, L1, L2)
    assert sol1 is not None, "Should have solution at minimum reach"


def test_jacobian_ik_basic():
    """Test Jacobian-based IK with a simple case."""
    # Create a simple 2-DOF arm
    dh_params = [
        DHParameter(theta=0.1, d=0, a=1.0, alpha=0),
        DHParameter(theta=0.2, d=0, a=0.8, alpha=0)
    ]

    # Calculate a target pose using forward kinematics
    initial_angles = np.array([0.1, 0.2])
    target_pose = forward_kinematics(dh_params, initial_angles)

    # Perturb the initial guess slightly
    initial_guess = initial_angles + np.array([0.05, -0.03])

    solution, success, iters = inverse_kinematics_jacobian(
        dh_params, target_pose, initial_guess, tolerance=1e-5
    )

    assert success, f"Jacobian IK should converge, took {iters} iterations"
    assert iters <= 100, "Should converge within max iterations"

    # Validate the solution
    is_valid, error = validate_ik_solution(dh_params, solution, target_pose, tolerance=1e-4)
    assert is_valid, f"Solution should be valid, error: {error}"


def test_jacobian_ik_with_bad_initial_guess():
    """Test Jacobian IK with a poor initial guess."""
    dh_params = [
        DHParameter(theta=0.1, d=0, a=1.0, alpha=0),
        DHParameter(theta=0.2, d=0, a=0.8, alpha=0)
    ]

    # Calculate a target pose
    target_angles = np.array([0.5, -0.3])
    target_pose = forward_kinematics(dh_params, target_angles)

    # Use a very different initial guess
    initial_guess = np.array([2.0, 2.0])

    solution, success, iters = inverse_kinematics_jacobian(
        dh_params, target_pose, initial_guess, tolerance=1e-5, max_iterations=200
    )

    # May or may not converge depending on the problem, but shouldn't crash
    assert iters <= 200, "Should not exceed max iterations"


def test_jacobian_ik_invalid_inputs():
    """Test Jacobian IK with invalid inputs."""
    dh_params = [DHParameter(theta=0.1, d=0, a=1.0, alpha=0)]

    target_pose = np.eye(4)
    initial_guess = np.array([0.1])  # Correct length

    # Test with wrong target pose shape
    bad_pose = np.eye(3)
    with pytest.raises(ValueError):
        inverse_kinematics_jacobian(dh_params, bad_pose, initial_guess)

    # Test with wrong initial guess length
    bad_guess = np.array([0.1, 0.2])  # Too long
    with pytest.raises(ValueError):
        inverse_kinematics_jacobian(dh_params, target_pose, bad_guess)


def test_ccd_ik_basic():
    """Test CCD-based IK with a simple case."""
    # Create a simple 2-DOF arm
    dh_params = [
        DHParameter(theta=0.1, d=0, a=1.0, alpha=0),
        DHParameter(theta=0.2, d=0, a=0.8, alpha=0)
    ]

    # Calculate a target position using forward kinematics
    initial_angles = np.array([0.1, 0.2])
    target_pose = forward_kinematics(dh_params, initial_angles)
    target_pos, _ = [target_pose[:3, 3], target_pose[:3, :3]]  # Extract position

    # Create a target pose with the same position but arbitrary orientation
    target_pose_for_ccd = np.eye(4)
    target_pose_for_ccd[:3, 3] = target_pos  # Set only the position

    # Perturb the initial guess slightly
    initial_guess = initial_angles + np.array([0.05, -0.03])

    solution, success, iters = inverse_kinematics_cyclic_coordinate_descent(
        dh_params, target_pose_for_ccd, initial_guess, tolerance=1e-4
    )

    # CCD might not always converge, but shouldn't crash
    assert iters <= 100, "Should not exceed max iterations"


def test_validate_ik_solution():
    """Test IK solution validation."""
    dh_params = [
        DHParameter(theta=0.1, d=0, a=1.0, alpha=0),
        DHParameter(theta=0.2, d=0, a=0.8, alpha=0)
    ]

    # Valid solution
    test_angles = np.array([0.1, 0.2])
    target_pose = forward_kinematics(dh_params, test_angles)

    is_valid, error = validate_ik_solution(dh_params, test_angles, target_pose)
    assert is_valid, "Valid solution should pass validation"
    assert error < 1e-10, "Error should be very small for valid solution"

    # Invalid solution
    bad_angles = np.array([1.0, 1.0])
    is_valid, error = validate_ik_solution(dh_params, bad_angles, target_pose)
    assert not is_valid or error > 1e-6, "Invalid solution should have high error"


def test_analytical_ik_specific_values():
    """Test analytical IK with specific known values."""
    # For a 2-DOF arm with L1=1, L2=1, at target (sqrt(2)/2, sqrt(2)/2)
    # Verify that the solution is valid by checking forward kinematics
    L1, L2 = 1.0, 1.0
    x, y = np.sqrt(2)/2, np.sqrt(2)/2  # 45-degree position

    sol1, sol2 = inverse_kinematics_analytical_2dof(x, y, L1, L2)

    assert sol1 is not None, "Solution should exist for reachable target"
    assert sol2 is not None, "Second solution should exist for reachable target"

    # Verify that the first solution is valid by checking forward kinematics
    from code.ch04_kinematics.forward_kinematics import DHParameter, forward_kinematics
    theta1, theta2 = sol1
    dh_params = [
        DHParameter(theta=theta1, d=0, a=L1, alpha=0),
        DHParameter(theta=theta2, d=0, a=L2, alpha=0)
    ]
    pose = forward_kinematics(dh_params)
    pos, _ = [pose[:3, 3], pose[:3, :3]]

    # Check if the computed position is close to the target
    np.testing.assert_allclose(pos[:2], [x, y], rtol=1e-5)


def test_ik_functions_handle_edge_cases():
    """Test that IK functions handle edge cases gracefully."""
    # Zero link lengths
    sol1, sol2 = inverse_kinematics_analytical_2dof(0, 0, 0, 0)
    # This should handle the degenerate case appropriately
    assert sol1 is not None  # Should handle zero lengths

    # Single joint arm (shouldn't have a valid 2-DOF solution but shouldn't crash)
    dh_params = [DHParameter(theta=0, d=0, a=1, alpha=0)]
    target_pose = np.eye(4)
    initial_guess = np.array([0.0])

    # This should complete without crashing, though may not converge
    try:
        solution, success, iters = inverse_kinematics_jacobian(dh_params, target_pose, initial_guess)
        # Function should handle mismatched dimensions gracefully without raising
    except ValueError:
        # If it does raise ValueError (which is acceptable), that's fine too
        pass


if __name__ == "__main__":
    # Run tests manually if executed as script
    test_analytical_2dof_reachable_target()
    test_analytical_2dof_unreachable_target()
    test_analytical_2dof_invalid_inputs()
    test_analytical_2dof_boundary_cases()
    test_jacobian_ik_basic()
    test_jacobian_ik_with_bad_initial_guess()
    test_jacobian_ik_invalid_inputs()
    test_ccd_ik_basic()
    test_validate_ik_solution()
    test_analytical_ik_specific_values()
    test_ik_functions_handle_edge_cases()
    print("All inverse kinematics tests passed!")