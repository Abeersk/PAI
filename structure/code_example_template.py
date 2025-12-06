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

@dataclass
class DHParameter:
    """DH parameters for a single joint."""
    theta: float  # Joint angle (rad)
    d: float      # Link offset (m)
    a: float      # Link length (m)
    alpha: float  # Link twist (rad)

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