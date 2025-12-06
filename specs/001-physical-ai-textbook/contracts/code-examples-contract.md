# Code Example API Contract: Physical AI & Humanoid Robotics Textbook

**Version**: 1.0
**Date**: 2025-12-05
**Feature**: 001-physical-ai-textbook

## Overview

This contract defines the standardized interfaces and structures for code examples in the Physical AI & Humanoid Robotics textbook. All code examples must conform to these contracts to ensure consistency, testability, and educational value.

## Code Example Interface Contract

### Standard Module Structure

Every code example module must conform to this structure:

```python
"""
Module: [module_name].py
Chapter: [Chapter Number] - [Chapter Title]
Author: [Author Name]
Date: [YYYY-MM-DD]
License: MIT

Description:
    [2-3 sentences describing the code's purpose and what it demonstrates]

Requirements:
    - numpy>=1.21.0
    - [other dependencies as needed]

Performance:
    - Time Complexity: [Big O notation]
    - Space Complexity: [Big O notation]
    - Execution Time: [<Xms for typical input]

Reference:
    [Author (Year). Title. Publisher/Conference.]
"""

import numpy as np
# ... other imports

from typing import List, Tuple, Optional, Union, Dict, Any
from dataclasses import dataclass
# ... other type imports as needed
```

### Function Interface Contract

All functions must follow this contract:

```python
def function_name(
    param1: np.ndarray,
    param2: float,
    optional_param: bool = True
) -> np.ndarray:
    """
    [One-line summary of function purpose]

    [Detailed description explaining algorithm, assumptions, and notes]

    Args:
        param1: [Description with units, e.g., "position vector in meters"]
        param2: [Description with constraints, e.g., "time step > 0"]
        optional_param: [Description with default behavior]

    Returns:
        [Description of return value with shape/units]

    Raises:
        ValueError: [When invalid input detected]
        RuntimeError: [When algorithm fails to converge]

    Example:
        >>> result = function_name(np.array([1, 2, 3]), 0.5)
        >>> print(result)
        [expected output]

    Reference:
        [Author (Year). Title. Publisher.]
    """
    # Input validation
    if param2 <= 0:
        raise ValueError(f"param2 must be positive, got {param2}")

    # Implementation with clear comments
    result = param1 * param2

    return result
```

## Validation Contract

### Input Validation Requirements

All functions must validate inputs according to this contract:

| Parameter Type | Validation Required | Error Raised |
|----------------|-------------------|--------------|
| Numeric (int/float) | Range constraints | `ValueError` |
| Arrays/matrices | Shape, type, content | `ValueError` |
| Enums | Valid value check | `ValueError` |
| Strings | Format, length, content | `ValueError` |
| Optional params | Type consistency | `TypeError` |

### Error Handling Contract

All functions must follow this error handling pattern:

```python
def example_function(param1: float) -> float:
    """
    Example function with proper error handling.

    Args:
        param1: Must be positive value

    Returns:
        Processed value

    Raises:
        ValueError: When param1 is not positive
        TypeError: When param1 is not a number
    """
    # Type validation first
    if not isinstance(param1, (int, float)):
        raise TypeError(f"param1 must be numeric, got {type(param1)}")

    # Value validation second
    if param1 <= 0:
        raise ValueError(f"param1 must be positive, got {param1}")

    # Implementation
    return param1 * 2.0
```

## Class Interface Contract

All classes must follow this structure:

```python
@dataclass
class RobotState:
    """
    Represents the state of a robot at a given time.

    Attributes:
        position: 3D position vector [x, y, z] in meters
        orientation: Quaternion [w, x, y, z] representing rotation
        joint_angles: Array of joint angles in radians
        timestamp: Time in seconds
    """
    position: np.ndarray  # shape (3,)
    orientation: np.ndarray  # shape (4,) quaternion
    joint_angles: np.ndarray  # shape (n,) where n is number of joints
    timestamp: float

    def __post_init__(self):
        """Validate the state after initialization."""
        if self.position.shape != (3,):
            raise ValueError(f"Position must be shape (3,), got {self.position.shape}")

        if self.orientation.shape != (4,):
            raise ValueError(f"Orientation must be shape (4,), got {self.orientation.shape}")

        # Normalize quaternion
        norm = np.linalg.norm(self.orientation)
        if norm != 0:
            self.orientation = self.orientation / norm
```

## Test Contract

Every code example must include validation tests that follow this contract:

```python
import numpy as np
import pytest
from [module_name] import [function_to_test]

def test_function_name_valid_inputs():
    """Test function with valid inputs."""
    # Arrange
    input_param = np.array([1.0, 2.0, 3.0])
    expected = np.array([2.0, 4.0, 6.0])

    # Act
    result = function_name(input_param, 2.0)

    # Assert
    np.testing.assert_allclose(result, expected, rtol=1e-6)

def test_function_name_edge_cases():
    """Test function with edge cases."""
    # Test zero values
    result = function_name(np.array([0.0, 0.0]), 1.0)
    expected = np.array([0.0, 0.0])
    np.testing.assert_allclose(result, expected)

def test_function_name_error_conditions():
    """Test function raises appropriate errors."""
    with pytest.raises(ValueError):
        function_name(np.array([1.0, 2.0]), -1.0)  # negative parameter
```

## Performance Contract

All algorithms must document performance characteristics:

```python
def algorithm_function(input_data: np.ndarray) -> np.ndarray:
    """
    [Description of algorithm]

    Performance:
        - Time Complexity: O(n) where n is input size
        - Space Complexity: O(1) constant extra space
        - Execution Time: <1ms for input size <= 1000 on reference hardware
        - Memory Usage: <10MB for typical inputs

    Args:
        input_data: [Description]

    Returns:
        [Description]
    """
    # Implementation that meets performance requirements
    pass
```

## Mathematical Formula Contract

When implementing mathematical formulas, follow this contract:

```python
def compute_formula(
    var1: float,  # [units, e.g., meters]
    var2: float,  # [units, e.g., radians/second]
) -> float:  # [units, e.g., Newton-meters]
    """
    Compute [formula name] using:

    τ = I * α

    Where:
    - τ (tau): Torque in Nm
    - I: Moment of inertia in kg⋅m²
    - α (alpha): Angular acceleration in rad/s²

    Derivation:
    1. [Step 1 of derivation]
    2. [Step 2 of derivation]
    3. [Step 3 of derivation]

    Reference:
        [Author (Year). Title. Publisher. Equation number if applicable]

    Args:
        var1: Moment of inertia (I) in kg⋅m²
        var2: Angular acceleration (α) in rad/s²

    Returns:
        Torque (τ) in Nm
    """
    # Validate inputs
    if var1 < 0:
        raise ValueError(f"Moment of inertia must be non-negative, got {var1}")

    # Compute formula
    torque = var1 * var2

    return torque
```

## Compliance Verification

### Automated Checks
All code examples must pass:
- `mypy --strict` for type checking
- `flake8` for style compliance
- `pytest` for unit tests
- `pylint` with textbook-specific configuration

### Manual Verification
- Mathematical formulas are dimensionally consistent
- All citations have corresponding references
- Code executes in clean Python 3.10+ environment
- Performance characteristics are documented and realistic

## Versioning

- Major version changes when interface contracts change
- Minor version changes for new examples or features
- Patch version changes for bug fixes and documentation updates