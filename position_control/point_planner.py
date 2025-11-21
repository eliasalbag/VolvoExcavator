"""
Point planner for 2DOF robotic arm.
Calculates (x_ref, y_ref) based on joystick inputs and current position.
"""

from numpy import arctan2, cos, sin, sign, abs, pi

L1 = 1903.392e-3
L2 = 1351.485e-3

theta1 = theta1 * pi / 180
theta2 = theta2 * pi / 180

# Current end-effector (x, y)
y_meas = L1*sin(theta1) + L2*sin(theta1 + theta2)
x_meas = L1*cos(theta1) + L2*cos(theta1 + theta2)

tol = 20
#increment = 0.05       # 20 cm
#point_tolerance = 0.045 # 10 cm

if abs(x_stick) < 1:
    x_stick = 0
    
if abs(y_stick) < 1:
    y_stick = 0

# Reset reference
if reset == 1:
    x_ref = 2.5
    y_ref = 0.5


if increment > point_tolerance:

    # --- X DIRECTION ---------------------------------------------------------
    if abs(x_ref - x_meas) < point_tolerance:
        if abs(x_stick) > tol:
            x_ref = x_meas + cos(arctan2(y_stick, x_stick))*increment

    # --- Y DIRECTION ---------------------------------------------------------
    if abs(y_ref - y_meas) < point_tolerance:
        if abs(y_stick) > tol:
            y_ref = y_meas + sin(arctan2(y_stick, x_stick))*increment

x_ref_out = x_ref
y_ref_out = y_ref

distance_to_target_x = abs(x_ref - x_meas)
distance_to_target_y = abs(y_ref - y_meas)