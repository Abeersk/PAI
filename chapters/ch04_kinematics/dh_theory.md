# Denavit-Hartenberg Parameters

## Introduction

The Denavit-Hartenberg (DH) convention is a systematic method for defining coordinate frames on robotic manipulator links and describing the kinematic relationships between them. Developed by Jacques Denavit and Richard S. Hartenberg in 1955, this method provides a standardized approach to robot modeling that is essential for forward and inverse kinematics calculations.

## The Four DH Parameters

For each joint $i$ in a robotic manipulator, four parameters define the relationship to the previous joint:

1. **$\\theta_i$ (theta)** - Joint angle about $z_{i-1}$ axis (variable for revolute joints, constant for prismatic)
2. **$d_i$ (d)** - Joint offset along $z_{i-1}$ axis (variable for prismatic joints, constant for revolute)
3. **$a_i$ (a)** - Link length along $x_i$ axis (constant)
4. **$\\alpha_i$ (alpha)** - Link twist about $x_i$ axis (constant)

## DH Frame Assignment Rules

To properly assign DH frames, follow these systematic rules:

1. **$z_0$ axis**: Choose the first joint axis direction
2. **$x_0$ axis**: Choose any direction perpendicular to $z_0$ (or to $z_1$ if they intersect)
3. **$y_0$ axis**: Complete the right-handed coordinate system
4. **$z_i$ axis**: For $i = 1$ to $n$, $z_i$ is the axis of actuation for joint $i+1$
5. **$x_i$ axis**: Along the common normal between $z_{i-1}$ and $z_i$, pointing from $z_{i-1}$ to $z_i$
6. **$y_i$ axis**: Complete the right-handed coordinate system

## Transformation Matrix

The homogeneous transformation matrix from frame $i-1$ to frame $i$ is given by the DH convention as:

$$
^i_{i-1}T =
\\begin{bmatrix}
\\cos\\theta_i & -\\sin\\theta_i\\cos\\alpha_i & \\sin\\theta_i\\sin\\alpha_i & a_i\\cos\\theta_i \\\\
\\sin\\theta_i & \\cos\\theta_i\\cos\\alpha_i & -\\cos\\theta_i\\sin\\alpha_i & a_i\\sin\\theta_i \\\\
0 & \\sin\\alpha_i & \\cos\\alpha_i & d_i \\\\
0 & 0 & 0 & 1
\\end{bmatrix}
$$

## Types of Joints

The DH parameters handle different joint types as follows:

- **Revolute Joint**: $\\theta_i$ is variable, $d_i$ is constant
- **Prismatic Joint**: $d_i$ is variable, $\\theta_i$ is constant
- **Fixed Joint**: Both $\\theta_i$ and $d_i$ are constant

## Example: 2-DOF Planar Arm

Consider a simple 2-DOF planar arm with two revolute joints:

| Link | $\\theta_i$ | $d_i$ | $a_i$ | $\\alpha_i$ |
|------|-------------|-------|-------|-------------|
| 1    | $\\theta_1$ | 0     | $a_1$ | 0           |
| 2    | $\\theta_2$ | 0     | $a_2$ | 0           |

For this arm, the transformation matrices become:
- $^1_0T$: Position and orientation of frame 1 relative to base
- $^2_1T$: Position and orientation of frame 2 relative to frame 1
- $^2_0T = ^1_0T \\cdot ^2_1T$: End-effector pose in base frame

## Common Configurations

### PUMA 560 Robot
The PUMA 560 is a classic 6-DOF industrial robot with a spherical wrist. Its DH parameters are well-documented and commonly used as a standard example in robotics literature.

### SCARA Robot
SCARA (Selective Compliance Assembly Robot Arm) robots have parallel joint axes for the first two joints, making them ideal for pick-and-place operations.

## Advantages and Limitations

**Advantages:**
- Systematic approach to robot modeling
- Standardized notation
- Applicable to most serial manipulators
- Forms the basis for many kinematic algorithms

**Limitations:**
- Cannot model all possible kinematic structures
- Requires careful frame assignment
- Can become complex for parallel mechanisms
- Not ideal for redundant manipulators

## Implementation Considerations

When implementing DH parameter calculations:
- Validate that the parameter set represents a physically realizable mechanism
- Check for singular configurations where the DH parameters may become undefined
- Consider the modified DH convention as an alternative in some cases
- Verify that the resulting transformation matrices match expected kinematic behavior

## Reference

Denavit, J., & Hartenberg, R. S. (1955). A kinematic notation for lower-pair mechanisms based on matrices. *Journal of Applied Mechanics*, 22, 215-221.