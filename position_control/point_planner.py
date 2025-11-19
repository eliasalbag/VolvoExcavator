"""
Point planner for 2DOF robotic arm.
Calculates (x_ref, y_ref) based on joystick inputs and current position.
"""

from numpy import arctan2, cos, sin

L1 = 1903.392e-3
L2 = 1351.485e-3

# Current end-effector (x, y)
y_meas = L1*sin(theta1) + L2*sin(theta1 + theta2)
x_meas = L1*cos(theta1) + L2*cos(theta1 + theta2)

tol = 20
increment = 0.20       # 20 cm
point_tolerance = 0.10 # 10 cm

# Reset reference
if reset == 1:
    x_ref = 2.0
    y_ref = 0.5

# Only add a new point when we're close to the previous one
if increment > point_tolerance:

    # --- X DIRECTION ---------------------------------------------------------
    if abs(x_ref - x_meas) < point_tolerance:
        if abs(x_stick) > tol or abs(y_stick) > tol:
            angle = arctan2(y_stick, x_stick)
            x_ref = x_meas + increment * cos(angle)

    # --- Y DIRECTION ---------------------------------------------------------
    if abs(y_ref - y_meas) < point_tolerance:
        if abs(x_stick) > tol or abs(y_stick) > tol:
            angle = arctan2(y_stick, x_stick)
            y_ref = y_meas + increment * sin(angle)

x_ref_out = x_ref
y_ref_out = y_ref