# Inverse Kinematics

## Definition

Inverse kinematics (IK) is the process of determining the joint variables required to achieve a desired end-effector position and orientation. Given a desired end-effector pose $^0T_{e,desired}$, inverse kinematics computes the joint vector $q = [q_1, q_2, ..., q_n]^T$ such that $FK(q) = ^0T_{e,desired}$.

## Mathematical Formulation

The inverse kinematics problem can be expressed as a system of nonlinear equations:
$$f(q) = x_{desired}$$

Where $f(q)$ represents the forward kinematics function and $x_{desired}$ is the desired end-effector pose.

## Solution Approaches

### Analytical Solutions

Closed-form solutions exist for specific robot geometries with particular configurations:

**Conditions for Closed-Form Solutions:**
1. Three consecutive joint axes intersect (Pieper's condition)
2. Six consecutive joint axes are parallel to a common plane
3. Three consecutive joints are prismatic

**Advantages:**
- Exact solutions
- Fast computation (O(1) time)
- Multiple solutions available
- Deterministic results

**Disadvantages:**
- Limited to specific robot geometries
- Complex derivation for complex robots
- May not exist for general configurations

### Numerical Solutions

For general robot configurations, iterative numerical methods are employed:

**Jacobian-Based Methods:**
- Compute the Jacobian matrix $J(q)$
- Use iterative update: $\\Delta q = J^+(q) \\Delta x$
- Where $J^+$ is the pseudoinverse of the Jacobian

**Newton-Raphson Method:**
- Iteratively solve: $q_{k+1} = q_k - J^{-1}(q_k)f(q_k)$
- Requires square and invertible Jacobian

**Cyclic Coordinate Descent (CCD):**
- Update one joint at a time to minimize error
- Robust to singularities
- Good for reaching targets

## Computational Complexity

### Analytical Methods
- **Time Complexity**: O(1) - constant time for closed-form solutions
- **Space Complexity**: O(1) - minimal memory usage
- **Accuracy**: Exact (subject to numerical precision)

### Numerical Methods
- **Time Complexity**: O(k·n) where k is iterations (typically 10-100) and n is joints
- **Space Complexity**: O(n²) for Jacobian storage and computation
- **Accuracy**: Dependent on tolerance and convergence criteria

## Implementation Challenges

### Multiple Solutions

Most manipulators have multiple configurations that achieve the same end-effector pose:

**Example: 2-DOF Planar Arm**
For a 2-DOF arm with link lengths $L_1$ and $L_2$, given target $(x, y)$:
1. **Elbow-up solution**: $\\theta_2 = \\arccos\\left(\\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}\\right)$
2. **Elbow-down solution**: $\\theta_2 = -\\arccos\\left(\\frac{x^2 + y^2 - L_1^2 - L_2^2}{2L_1L_2}\\right)$

### Singularities

Configurations where the Jacobian loses rank, causing:
- Loss of instantaneous mobility in certain directions
- Infinite joint velocities for finite end-effector motion
- Numerical instability in inverse computation

**Singularity Handling:**
- Damped least squares: $J^+ = (J^TJ + \\lambda^2I)^{-1}J^T$
- Levenberg-Marquardt method
- Singularity-robust inverse techniques

### Joint Limits

Solutions must respect mechanical constraints:
$$q_{min} \\leq q_i \\leq q_{max}$$

## Verification Methods

### Forward Check
Apply computed joint angles to forward kinematics and verify the result matches the desired pose within tolerance.

### Boundary Cases
Test with positions at workspace boundaries, singular configurations, and unreachable positions.

### Continuity Check
Verify that small changes in desired pose result in small changes in joint angles (avoid solution jumps).

## Applications

### Trajectory Following
IK enables robots to follow desired end-effector trajectories.

### Task-Space Control
Allows control in Cartesian space rather than joint space.

### Humanoid Locomotion
Essential for foot placement and balance control in walking robots.

## Performance Considerations

### Real-time Constraints
- For 1kHz control: <1ms computation time
- For 100Hz control: <10ms computation time
- For trajectory planning: <100ms computation time

### Accuracy vs. Speed Trade-offs
- Analytical solutions: High accuracy, fast computation
- Numerical solutions: Adjustable accuracy, slower computation
- Approximate methods: Fast but less accurate

## Reference

Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2020). *Robot Modeling and Control* (2nd ed.). Wiley.