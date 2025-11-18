"""
Point planner for 2DOF robotic arm.
Calculates (x_ref, y_ref) based on joystick inputs and current position.
"""


from numpy import arctan, cos, sin, sign
L1 = 1903.392e-3
L2 = 1351.485e-3

"""
Calculate current (x,y) position based on measured joint angles
"""

y_meas = L1*sin(theta1) + L2*sin(theta1 + theta2)
x_meas = L1*cos(theta1) + L2*cos(theta1 + theta2)

tol = 20
increment = 10e-2 # 10 cm increment
point_tolerance = 5

assert increment > point_tolerance
"""
Check sticks
"""

if abs(x_stick) > tol:
    dx = increment * cos(arctan(y_stick/x_stick))
else:
    dx = 0


if abs(y_stick) > tol:
    dy = increment * sin(arctan(y_stick/x_stick))
else:
    dy = 0


"""
Calculate new (x_ref, y_ref) position
"""

if x_ref or y_ref is None:
    x_ref = x_meas
    y_ref = y_meas


if abs(x_ref - x_meas) < point_tolerance:
    x_ref = x_meas + dx


if abs(y_ref - y_meas) < point_tolerance:
    y_ref = y_meas + dy



x_ref_out = x_ref
y_ref_out = y_ref