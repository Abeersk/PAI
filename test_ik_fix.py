import numpy as np
from code.ch04_kinematics.inverse_kinematics import inverse_kinematics_analytical_2dof
from code.ch04_kinematics.forward_kinematics import DHParameter, forward_kinematics

# Test the corrected function
L1, L2 = 1.0, 0.8
x, y = 1.2, 0.5  # Reachable target

sol1, sol2 = inverse_kinematics_analytical_2dof(x, y, L1, L2)
print(f'Solutions: sol1={sol1}, sol2={sol2}')

if sol1 is not None and sol2 is not None:
    theta1_1, theta2_1 = sol1
    theta1_2, theta2_2 = sol2
    print(f'Solution 1: theta1={theta1_1:.6f}, theta2={theta2_1:.6f}')
    print(f'Solution 2: theta1={theta1_2:.6f}, theta2={theta2_2:.6f}')

    # Test both solutions
    dh_params1 = [DHParameter(theta=theta1_1, d=0, a=L1, alpha=0), DHParameter(theta=theta2_1, d=0, a=L2, alpha=0)]
    T1 = forward_kinematics(dh_params1)
    pos1 = T1[:3, 3]
    print(f'Solution 1 result: ({pos1[0]:.6f}, {pos1[1]:.6f})')

    dh_params2 = [DHParameter(theta=theta1_2, d=0, a=L1, alpha=0), DHParameter(theta=theta2_2, d=0, a=L2, alpha=0)]
    T2 = forward_kinematics(dh_params2)
    pos2 = T2[:3, 3]
    print(f'Solution 2 result: ({pos2[0]:.6f}, {pos2[1]:.6f})')

    print(f'Target: ({x}, {y})')
    print(f'Solution 1 error: ({pos1[0]-x:.10f}, {pos1[1]-y:.10f})')
    print(f'Solution 2 error: ({pos2[0]-x:.10f}, {pos2[1]-y:.10f})')