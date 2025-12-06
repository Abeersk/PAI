# Jacobian Matrices

## Definition and Purpose

The Jacobian matrix is a fundamental tool in robotics that relates joint velocities to end-effector velocities. For a robot with $n$ joints and an end-effector with $m$ degrees of freedom (typically $m = 6$ for full pose in 3D space), the Jacobian is an $m \\times n$ matrix that linearizes the relationship between joint space and task space velocities:

$$\\dot{x} = J(q)\\dot{q}$$

Where $\\dot{x}$ is the end-effector velocity vector, $J(q)$ is the Jacobian matrix, and $\\dot{q}$ is the joint velocity vector.

## Mathematical Formulation

The relationship between joint velocities $\\dot{q}$ and end-effector velocities $\\dot{x}$ is given by:

$$\\dot{x} = J(q)\\dot{q}$$

For a 6-DOF end-effector, $\\dot{x}$ typically contains both linear and angular velocities:
$$\\dot{x} = \\begin{bmatrix} \\dot{p} \\\\ \\omega \\end{bmatrix}$$

Where $\\dot{p}$ is the linear velocity and $\\omega$ is the angular velocity.

## Types of Jacobians

### Geometric Jacobian

Derived directly from the kinematic equations using the definition of spatial velocity:

$$J_{geom}(q) = \\begin{bmatrix} J_v(q) \\\\ J_\\omega(q) \\end{bmatrix}$$

For revolute joints:
- $J_{v,i} = z_{i-1} \\times (p_e - p_{i-1})$ (linear velocity contribution)
- $J_{\\omega,i} = z_{i-1}$ (angular velocity contribution)

For prismatic joints:
- $J_{v,i} = z_{i-1}$ (linear velocity contribution)
- $J_{\\omega,i} = 0$ (no angular velocity contribution)

### Analytical Jacobian

Derived by differentiating the forward kinematics equations with respect to joint variables.

## Computing the Geometric Jacobian

For each joint $i$, the $i$-th column of the Jacobian is computed as:

**Revolute Joint:**
$$J_i = \\begin{bmatrix} z_{i-1} \\times (p_e - p_{i-1}) \\\\ z_{i-1} \\end{bmatrix}$$

**Prismatic Joint:**
$$J_i = \\begin{bmatrix} z_{i-1} \\\\ 0 \\end{bmatrix}$$

Where:
- $z_{i-1}$ is the axis of joint $i$ expressed in the base frame
- $p_e$ is the end-effector position in the base frame
- $p_{i-1}$ is the origin of frame ${i-1}$ in the base frame

## Jacobian Properties

### Singularity

The Jacobian is singular when $\\det(J^TJ) = 0$ for square matrices, or when $J$ does not have full rank. At singular configurations:
- The robot loses instantaneous mobility in certain directions
- Joint velocities can become very large for small end-effector motions
- The inverse Jacobian does not exist

### Dexterity

Dexterity measures how close the robot is to a singular configuration. The condition number $\\kappa(J) = ||J|| \\cdot ||J^+||$ indicates dexterity, where smaller values indicate better dexterity.

### Manipulability

The manipulability measure $w = \\sqrt{\\det(JJ^T)}$ indicates how "well-conditioned" the Jacobian is for force and velocity transmission.

## Applications of the Jacobian

### Velocity Control

Directly used to convert between joint and task space velocities:
$$\\dot{q} = J^+(q)\\dot{x}$$

### Force Control

Relates end-effector forces to joint torques:
$$\\tau = J^T(q)F$$

### Trajectory Planning

Used to ensure smooth motion in task space while respecting joint limits.

### Singularity Analysis

Used to identify and avoid singular configurations during motion planning.

## Pseudoinverse Methods

When the Jacobian is not square or is singular, the pseudoinverse is used:

**For $m > n$ (redundant system):**
$$J^+ = (J^TJ)^{-1}J^T$$

**For $m < n$ (underactuated system):**
$$J^+ = J^T(JJ^T)^{-1}$$

**Damped Least Squares (for singularity handling):**
$$J^+ = (J^TJ + \\lambda^2I)^{-1}J^T$$

Where $\\lambda$ is the damping factor that increases near singularities.

## Computational Complexity

**Time Complexity:**
- Computing Jacobian: $O(n^2)$ for general case
- Computing pseudoinverse: $O(n^3)$ for square matrices
- Computing damped pseudoinverse: $O(n^3)$

**Space Complexity:**
- Storing Jacobian: $O(mn)$
- Additional storage for pseudoinverse: $O(n^2)$

## Implementation Considerations

### Numerical Stability
- Use SVD (Singular Value Decomposition) for robust pseudoinverse computation
- Avoid direct matrix inversion when possible
- Monitor condition number to detect near-singular configurations

### Singularity Handling
- Implement damped least squares near singularities
- Use singularity-robust inverse techniques
- Plan trajectories to avoid singular configurations when possible

### Efficiency Improvements
- Cache Jacobian computations when possible
- Use analytical derivatives for faster computation
- Exploit robot structure for computational efficiency

## Verification Methods

### Velocity Verification
Compare computed end-effector velocity with finite difference approximation of forward kinematics.

### Force Verification
Verify that $\\tau = J^T F$ produces expected joint torques for known end-effector forces.

### Singularity Detection
Check that the Jacobian rank decreases at known singular configurations.

## Performance Considerations

### Real-time Constraints
- For 1kHz control: Jacobian computation <0.5ms
- For 100Hz control: Jacobian computation <5ms
- For trajectory planning: Jacobian computation <50ms

### Accuracy Requirements
- Position accuracy: 1e-6 m for precision tasks
- Orientation accuracy: 1e-6 rad for precision tasks
- Force accuracy: 0.1 N for compliant tasks

## Reference

Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.