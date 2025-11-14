
from numpy import sqrt, atan2
import numpy as np

L1 = 1903.392e-3
L2 = 1351.485e-3

x_ref = 2
y_ref = 0.5

cost2 = (x_ref**2 + y_ref**2 - L1**2 - L2**2) / (2 * L1 * L2)
sint2 = -sqrt(1 - cost2**2)

theta1_ref = atan2(y_ref, x_ref) - atan2(L2 * sint2, L1 + L2 * cost2)
theta2_ref = atan2(sint2, cost2)

"""
Convert to degrees
"""

theta1_ref = theta1_ref * 180 / np.pi
theta2_ref = theta2_ref * 180 / np.pi

print(theta1_ref) 
print(theta2_ref)
