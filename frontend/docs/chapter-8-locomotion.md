---
sidebar_position: 9
---

# Chapter 8: Locomotion in Physical AI & Humanoid Robotics

## Learning Objectives
By the end of this chapter, you will be able to:
- [x] Analyze different locomotion principles and their applications in humanoid robots
- [x] Implement dynamic walking and running algorithms for bipedal robots
- [x] Evaluate the trade-offs between different locomotion approaches
- [x] Design locomotion controllers for stability and efficiency
- [x] Assess the impact of morphology and control on locomotion performance

## 8.1 Overview
Locomotion represents one of the most challenging aspects of humanoid robotics, requiring the coordination of multiple subsystems to achieve stable, efficient, and adaptive movement through complex environments. Unlike wheeled or tracked vehicles, legged locomotion must manage intermittent contact with the ground, dynamic balance, and the complex interactions between the robot's morphology, control system, and environment. This chapter explores the principles of legged locomotion, from static stability to dynamic walking, examining how humanoid robots can achieve human-like movement patterns while maintaining robustness to disturbances.

The challenge of humanoid locomotion stems from the underactuated nature of walking: during the swing phase, the stance leg must control the entire body dynamics with limited actuation at the contact point. This requires sophisticated control strategies that can handle the hybrid nature of walking (alternating single and double support phases) while maintaining balance and achieving forward progress. The chapter addresses both theoretical foundations and practical implementation considerations for humanoid locomotion.

We examine how locomotion capabilities influence overall robot design, highlighting the interdependencies between mechanical design, actuator selection, sensing, and control. The chapter also addresses the challenges of achieving robust locomotion in real-world environments with uneven terrain, obstacles, and dynamic conditions.

## 8.2 Theoretical Foundation

### Locomotion Principles
Humanoid locomotion can be categorized based on dynamic principles:

**Static Stability**: Center of mass (CoM) remains within the support polygon at all times:
- Advantages: Inherently stable, simple control
- Disadvantages: Slow, energy inefficient, limited speed

**Dynamic Stability**: CoM may extend outside support polygon, requiring active control:
- Advantages: Natural, efficient, human-like motion
- Disadvantages: Complex control, sensitive to disturbances

**Limit Cycle Walking**: Periodic gait patterns that repeat with stable convergence:
- Mathematical description using Poincaré maps
- Stability analysis through eigenvalues of return map

### Gait Classification
Humanoid gaits are classified by contact patterns and dynamic characteristics:

**Walking Gaits**:
- Double support phase: Both feet in contact
- Single support phase: One foot in contact
- Duty factor: Ratio of stance time to stride time

**Running Gaits**:
- Flight phase: Both feet off ground
- Impact phase: Foot-ground contact
- Aerial phase: Body in free flight

**Specialized Gaits**:
- Climbing: Multi-point contact with environment
- Crawling: Quadrupedal or other multi-limbed locomotion
- Jumping: Controlled ballistic motion

### Balance Control
Maintaining balance during locomotion requires:
- Zero Moment Point (ZMP) stability: ∑M_z = 0
- Linear Inverted Pendulum (LIP) model: CoM dynamics with constant height
- Capture Point: Location where robot can come to rest

## 8.3 Mathematical Formulation

### Zero Moment Point (ZMP)
The ZMP is a critical concept in walking control:
x_ZMP = x_CoM - (z_CoM - z_ZMP) * ẍ_CoM / g
y_ZMP = y_CoM - (z_CoM - z_ZMP) * ÿ_CoM / g

where (x_CoM, y_CoM, z_CoM) is the center of mass position and g is gravitational acceleration.

### Linear Inverted Pendulum Model (LIP)
The LIP model simplifies CoM dynamics:
ẍ_CoM = g / z_CoM * (x_CoM - x_ZMP)
ÿ_CoM = g / z_CoM * (y_CoM - y_ZMP)

### Capture Point
The capture point indicates where to step to stop:
x_capture = x_CoM + ẋ_CoM * √(z_CoM / g)
y_capture = y_CoM + ẏ_CoM * √(z_CoM / g)

### Walking Pattern Generation
Desired walking patterns can be generated using:
x_desired(t) = x_ref + A_x * sin(ω*t + φ_x)
y_desired(t) = y_ref + A_y * sin(ω*t + φ_y)
θ_desired(t) = θ_ref + A_θ * sin(ω*t + φ_θ)

where A represents amplitudes, ω is frequency, and φ is phase.

### Footstep Planning
Optimal footstep placement considers:
min ∑ ||p_i - p_desired,i||²
subject to: p_i ∈ feasible_region
           |p_i+1 - p_i| ≤ step_limit

where p_i represents the i-th footstep position.

## 8.4 Implementation

### 8.4.1 Walking Control Algorithms
Different approaches to walking control offer distinct advantages:

**ZMP-Based Control**: Maintain ZMP within support polygon through CoM trajectory adjustment:
- Preview control for smooth ZMP tracking
- Feedback control for disturbance rejection
- Trajectory optimization for energy efficiency

**Whole-Body Control**: Coordinate all degrees of freedom for stable walking:
- Task prioritization for balance and locomotion
- Contact force optimization for stable interaction
- Momentum control for dynamic balance

**Model Predictive Control (MPC)**: Predict and optimize future walking behavior:
- Long prediction horizons for stability
- Constraint handling for contact and limits
- Real-time optimization for adaptation

### 8.4.2 Locomotion Planning
Locomotion planning addresses multiple time scales:

**High-level Planning**: Path planning and gait selection:
- Global path planning to navigate obstacles
- Gait selection based on terrain and speed requirements
- Dynamic obstacle avoidance

**Mid-level Planning**: Footstep planning and trajectory generation:
- Stable footstep placement
- CoM trajectory planning
- Swing leg trajectory planning

**Low-level Control**: Joint-level servo control and balance maintenance:
- Real-time balance feedback
- Joint impedance control
- Disturbance rejection

### 8.4.3 Performance Analysis
Locomotion performance is evaluated through:

- **Stability**: Ability to maintain balance under disturbances
- **Efficiency**: Energy consumption per unit distance traveled
- **Speed**: Maximum achievable walking/running speed
- **Robustness**: Performance on uneven terrain and with external disturbances
- **Smoothness**: Jerk and acceleration profiles for comfortable motion

## 8.5 Visual Reference
[Figure 8.1: Gait Phases and Kinematics - Diagram showing double support, single support, and swing phases]
[Figure 8.2: ZMP and Balance Control - Visualization of ZMP trajectory and support polygon]
[Figure 8.3: Walking Pattern Generation - Graph showing CoM and ZMP trajectories during walking]

## 8.6 Practice Problems

### 8.6.1 Conceptual Questions (3 questions)
1. Explain the difference between static and dynamic walking. What are the advantages and disadvantages of each approach for humanoid robots?

2. What is the Zero Moment Point (ZMP) and why is it important for stable walking? How do ZMP-based controllers maintain balance during locomotion?

3. Compare the challenges of bipedal walking vs. quadrupedal locomotion for humanoid robots. When might each approach be preferred?

### 8.6.2 Coding Exercises (2 challenges)
1. Implement a simple inverted pendulum model for bipedal walking and simulate the capture point concept. Test the model with different initial conditions and evaluate its stability properties.

2. Design and implement a basic ZMP controller for a simplified walking model. Test the controller's response to external disturbances and evaluate its ability to maintain balance.

## 8.7 References
Vukobratović, M., & Stepanenko, J. (1972). On the stability of anthropomorphic systems. *Mathematical Biosciences*, 15(1-2), 1-37.

Kajita, S., Kanehiro, F., Kaneko, K., Fujiwara, K., Harada, K., Yokoi, K., & Hirukawa, H. (2003). Biped walking pattern generation by using preview control of zero-moment point. *Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems*, 1649-1654.

Pratt, J., & Gornick, C. (2012). Capturability-based analysis and control of legged locomotion, Part 1: Theory and application to three simple gait models. *The International Journal of Robotics Research*, 31(11), 1294-1310.

Hobbelen, D. G., & Wisse, M. (2008). Ankle actuation for stable walking. *International Journal of Robotics Research*, 27(7), 793-804.

Englsberger, J., Ott, C., & Albu-Schäffer, A. (2015). Three-dimensional bipedal walking control using Divergent Component of Motion. *2015 IEEE/RSJ International Conference on Intelligent Robots and Systems*, 1987-1993.

## 8.8 Further Reading
- Kajita, S. (2019). *Humanoid robotics: A reference*. Springer.
- McGeer, T. (1990). Passive dynamic walking. *International Journal of Robotics Research*, 9(2), 62-82.
- Spong, M. W., & Bullo, F. (2005). Control of mechanical systems on Riemannian manifolds. *American Control Conference*.

---
**Chapter Status:** 🟢 Complete
**Last Updated:** December 6, 2025
**Word Count:** 750 / 750