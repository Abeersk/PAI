# Rigid Body Dynamics

## Introduction

Rigid body dynamics describes the motion of bodies under the action of forces and torques. In robotics, understanding dynamics is crucial for control, simulation, and trajectory planning. The equations of motion for a robotic manipulator are more complex than for a single rigid body due to the interconnected nature of the links.

## Newton-Euler Equations

For a single rigid body, the Newton-Euler equations describe the relationship between forces/torques and motion:

**Translational Motion (Newton's equation):**
$$F = m\\ddot{p}$$

Where $F$ is the force vector, $m$ is mass, and $\\ddot{p}$ is the linear acceleration of the center of mass.

**Rotational Motion (Euler's equation):**
$$\\tau = \\frac{d}{dt}(I\\omega) = I\\dot{\\omega} + \\omega \\times (I\\omega)$$

Where $\\tau$ is the torque vector, $I$ is the inertia matrix, $\\omega$ is the angular velocity, and $\\dot{\\omega}$ is the angular acceleration.

## Lagrangian Formulation

The Lagrangian approach is more systematic for multi-body systems like robotic manipulators. The Lagrangian $L$ is defined as:

$$L = T - U$$

Where $T$ is the kinetic energy and $U$ is the potential energy.

For a robotic manipulator, the equations of motion are given by the Euler-Lagrange equations:

$$\\frac{d}{dt}\\left(\\frac{\\partial L}{\\partial \\dot{q}_i}\\right) - \\frac{\\partial L}{\\partial q_i} = \\tau_i$$

## Manipulator Equation

The dynamics of an $n$-DOF robotic manipulator can be expressed in the compact form:

$$M(q)\\ddot{q} + C(q,\\dot{q})\\dot{q} + g(q) = \\tau$$

Where:
- $M(q) \\in \\mathbb{R}^{n \\times n}$ is the inertia matrix
- $C(q,\\dot{q}) \\in \\mathbb{R}^{n \\times n}$ is the Coriolis and centrifugal matrix
- $g(q) \\in \\mathbb{R}^n$ is the gravity vector
- $\\tau \\in \\mathbb{R}^n$ is the joint torque vector
- $q \\in \\mathbb{R}^n$ is the joint position vector
- $\\dot{q} \\in \\mathbb{R}^n$ is the joint velocity vector
- $\\ddot{q} \\in \\mathbb{R}^n$ is the joint acceleration vector

### Inertia Matrix $M(q)$

The inertia matrix is symmetric and positive definite. It represents the inertial properties of the manipulator as a function of configuration. The elements are given by:

$$M_{ij} = \\sum_{k=\\max(i,j)}^n \\text{trace}(U_{ijk}J_k)$$

Where $U_{ijk}$ and $J_k$ are matrices derived from the manipulator's kinematics and link inertias.

### Coriolis and Centrifugal Matrix $C(q,\\dot{q})$

This matrix accounts for velocity-dependent forces due to the motion of the manipulator. The elements are computed from the Christoffel symbols:

$$C_{ij} = \\sum_{k=1}^n \\Gamma_{ijk}\\dot{q}_k$$

Where:
$$\\Gamma_{ijk} = \\frac{1}{2}\\left(\\frac{\\partial M_{ij}}{\\partial q_k} + \\frac{\\partial M_{ik}}{\\partial q_j} - \\frac{\\partial M_{jk}}{\\partial q_i}\\right)$$

### Gravity Vector $g(q)$

The gravity vector represents the gravitational forces acting on the manipulator as a function of configuration:

$$g_i = \\frac{\\partial U}{\\partial q_i}$$

Where $U$ is the potential energy of the manipulator.

## Dynamic Parameters

Each link of a manipulator is characterized by dynamic parameters:

- **Mass** ($m_i$): Scalar mass of link $i$
- **Center of mass** ($c_i$): 3D vector from link frame to center of mass
- **Inertia tensor** ($I_i$): 3×3 symmetric matrix representing rotational inertia

These parameters can be combined into a parameter vector $\\theta$ for identification purposes.

## Recursive Newton-Euler Algorithm

The recursive Newton-Euler (RNE) algorithm efficiently computes inverse dynamics (given $q$, $\\dot{q}$, $\\ddot{q}$, compute $\\tau$) in $O(n)$ time:

**Outward Recursion (from base to tip):**
1. Compute link velocities and accelerations
2. Compute forces and torques acting on each link

**Inward Recursion (from tip to base):**
1. Compute the joint torques required to produce the motion
2. Propagate reaction forces and torques back to the base

## Computed Torque Control

The manipulator equation forms the basis for advanced control strategies. Computed torque control linearizes and decouples the system:

$$\\tau = M(q)\\ddot{q}_d + C(q,\\dot{q})\\dot{q}_d + g(q) + K_v(\\dot{q}_d - \\dot{q}) + K_p(q_d - q)$$

Where $q_d$, $\\dot{q}_d$, $\\ddot{q}_d$ are desired trajectories and $K_p$, $K_v$ are positive definite gain matrices.

## Computational Complexity

**Inverse Dynamics (RNE Algorithm):**
- Time Complexity: $O(n)$ - linear in the number of joints
- Space Complexity: $O(1)$ - constant additional space

**Forward Dynamics (Computing $\\ddot{q}$ from $\\tau$):**
- Time Complexity: $O(n^3)$ - dominated by matrix inversion
- Space Complexity: $O(n^2)$ - for matrix storage

## Simulation Considerations

### Numerical Integration
- Use appropriate integrators (Euler, Runge-Kutta, etc.)
- Consider the trade-off between accuracy and computational cost
- Monitor energy conservation in conservative systems

### Constraint Handling
- Implement joint limits and actuator saturation
- Handle contact and collision events
- Consider friction and other non-conservative forces

### Real-time Requirements
- For 1kHz simulation: $\\leq$0.5ms per integration step
- For 100Hz simulation: $\\leq$5ms per integration step

## Verification Methods

### Energy Conservation
Verify that total energy (kinetic + potential) is conserved in conservative systems.

### Static Equilibrium
Verify that the gravity compensation term balances gravitational forces in static configurations.

### Dynamic Consistency
Check that the equations of motion satisfy physical constraints (e.g., D'Alembert's principle).

## Implementation Challenges

### Numerical Conditioning
- Inertia matrix can become ill-conditioned near singularities
- Use SVD or other robust methods for matrix inversion
- Monitor condition number during simulation

### Parameter Identification
- Accurate dynamic parameters are often difficult to obtain
- Consider system identification techniques
- Account for unmodeled dynamics and friction

### Real-time Performance
- Optimize for computational efficiency
- Consider simplified models for high-frequency control
- Use parallel computation where possible

## Applications

### Robot Control
- Feedforward compensation in model-based control
- Trajectory planning with dynamic constraints
- Force control and impedance control

### Simulation
- Physics-based simulation for training and testing
- Virtual environments for robot development
- Hardware-in-the-loop testing

### Trajectory Optimization
- Dynamic feasibility checking
- Minimum-time trajectory generation
- Energy-optimal motion planning

## Reference

Siciliano, B., & Khatib, O. (Eds.). (2009). *Robotics: Modelling, Planning and Control*. Springer-Verlag.