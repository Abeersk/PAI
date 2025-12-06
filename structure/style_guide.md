# Style Guide: Physical AI & Humanoid Robotics Textbook

## Writing Standards

### Terminology
- ✅ "end-effector" (not "end effector" or "gripper")
- ✅ "degrees of freedom (DOF)" consistently
- ✅ "joint space" vs. "task space" (not "configuration space" vs. "workspace")
- ✅ "forward kinematics (FK)" and "inverse kinematics (IK)"
- ✅ "Denavit-Hartenberg (DH) parameters"
- ✅ "Zero Moment Point (ZMP)"

### Mathematical Notation
**Standard Notation:**
- θ (theta) = joint angles (radians)
- τ (tau) = torque (Nm)
- q = joint position vector
- q̇ (q-dot) = joint velocity
- q̈ (q-double-dot) = joint acceleration
- x, y, z = Cartesian positions (meters)
- R = rotation matrix (3×3)
- T = transformation matrix (4×4)
- J = Jacobian matrix
- M(q) = inertia matrix
- C(q,q̇) = Coriolis/centrifugal matrix
- g(q) = gravity vector

### LaTeX Format
**Inline:** $\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)$

**Display:**
$
T_i^{i-1} =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$

## Citation Standards (APA 7)

### In-text Citations
- (Author, Year) or Author (Year)
- (Craig, 2005) or Craig (2005, p. 76)

### Reference List Format
- Journal: Author, A. A. (Year). Title of article. *Title of Periodical*, volume(issue), pages. https://doi.org/xx.xxx/yyyy
- Book: Author, A. A. (Year). *Title of work: Capital letter also for subtitle*. Publisher.

## Figure Standards

### APA 7 Caption Format
```
Figure 4.2
Forward Kinematics Solution for 3-DOF Planar Arm

Note. Joint angles: θ₁ = 45°, θ₂ = 30°, θ₃ = -15°. Link lengths:
L₁ = 0.5m, L₂ = 0.3m, L₃ = 0.2m. Adapted from Craig (2005, Fig. 3.8).
```

### Color Scheme
- Blue: Input/initial states
- Green: Processing/computation
- Red: Output/final states
- Yellow: Feedback/error signals

## Code Standards

### Python Template Structure
```python
"""
Module: descriptive_name.py
Chapter: X - Chapter Name
Author: [Name]
Date: [YYYY-MM-DD]
License: MIT

Description:
    Brief 2-3 sentence description of what this code does.

Requirements:
    - numpy>=1.21.0
    - scipy>=1.7.0

Performance:
    - Time Complexity: O(n)
    - Space Complexity: O(1)
    - Execution Time: <Xms for typical input

Reference:
    Author (Year). Title. Publisher.
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass

def function_name(
    param1: np.ndarray,
    param2: float,
    optional_param: bool = True
) -> np.ndarray:
    """
    One-line summary of function purpose.

    Longer description if needed. Explain algorithm, assumptions,
    and any important notes.

    Args:
        param1: Description with units (e.g., meters, radians)
        param2: Description with constraints (e.g., must be > 0)
        optional_param: Description with default behavior

    Returns:
        Description of return value with shape/units

    Raises:
        ValueError: When invalid input detected
        RuntimeError: When algorithm fails to converge

    Example:
        >>> # Provide minimal working example
        >>> result = function_name(np.array([1, 2, 3]), 0.5)
        >>> print(result)
        [expected output]

    Reference:
        Craig, J. J. (2005). Introduction to Robotics (3rd ed.).
    """
    # Input validation
    if param2 <= 0:
        raise ValueError(f"param2 must be positive, got {param2}")

    # Implementation with clear comments
    result = param1 * param2

    return result

if __name__ == "__main__":
    # Runnable example with expected output
    print("Testing function_name...")
    # Test cases here
```

### Code Quality Requirements
- No "magic numbers" → Use configuration dictionaries
- No global variables → Pass parameters explicitly
- No print() debugging → Use logging module
- Every function has one clear purpose
- Error messages must be informative
- No pseudo-code unless labeled "Algorithm Pseudocode"