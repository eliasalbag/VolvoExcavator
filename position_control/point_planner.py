"""
Point planner for 2DOF robotic arm.
Calculates (x_ref, y_ref) based on joystick inputs and current position.
"""

theta1, theta2, x_stick, y_stick = 1, 1, 1, 1

from numpy import cos, sin, sign
L1 = 1903.392e-3
L2 = 1351.485e-3

"""
Calculate current (x,y) position based on measured joint angles
"""

y_meas = L1*sin(theta1) + L2*sin(theta1 + theta2)
x_meas = L1*cos(theta1) + L2*cos(theta1 + theta2)

tol = 20
increment = 10e-2 # 10 cm increment

"""
Check x_stick
"""

if (x_stick < -tol) and (x_stick > tol):
    dx = increment * sign(x_stick)
else:
    dx = 0

"""
Check y_stick
"""

if (y_stick < -tol) and (y_stick > tol):
    dy = increment * sign(y_stick)
else:
    dy = 0


"""
Calculate new (x_ref, y_ref) position
"""
x_ref = x_meas + dx
y_ref = y_meas + dy

