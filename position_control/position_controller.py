"""
Controller for 2DOF robotic arm using PID control in joint space.
Calculates control inputs for the two joints based on reference (x,y) position.
"""

# Tuning parameters for the PID controller
Kp1, Ki1, Kd1 = 15, 0.01, 0.1
Kp2, Ki2, Kd2 = 15, 0.01, 0.1
Ts = 0.01

"""
Initalize variables
"""

from numpy import sin, cos, arctan2 as atan2, sqrt

# Remove the following line when integrating with point_planner
Ts, x_ref, y_ref, theta1_meas, theta2_meas, error_1_prev, error_1_integral, error_2_prev, error_2_integral = 1, 1, 1, 1,1, 0, 0

L1 = 1903.392e-3
L2 = 1351.485e-3


"""
Inverse kinematics. Takes (x,y) reference and calculates corresponding joint angles (theta1_ref, theta2_ref). 
"""

cost2 = (x_ref^2 + y_ref^2 - L1^2 - L2^2) / (2 * L1 * L2)
sint2 = -sqrt(1 - cost2^2)

theta1_ref = atan2(y_ref, x_ref) - atan2(L2 * sint2, L1 + L2 * cost2)
theta2_ref = -atan2(sint2, cost2)

"""
PID - controller
"""

error_1 = theta1_meas - theta1_ref
error_2 = theta2_meas - theta2_ref


"""
Error 1 integral
"""
if error_1_prev is None:
    error_1_prev = 0

if error_1_integral is None:
    error_1_integral = 0

error_1_integral = (error_1_prev + error_1) / 2 + error_1_integral

error_1_prev = error_1 

"""
Error 2 integral
"""

if error_2_prev is None:
    error_2_prev = 0

if error_2_integral is None:
    error_2_integral = 0

error_2_integral = (error_2_prev + error_2) / 2 + error_2_integral
error_2_prev = error_2

"""
Error derivatives
"""
error_1_derivative = error_1 - error_1_prev / Ts
error_2_derivative = error_2 - error_2_prev / Ts

"""
PID control law
"""

ctrl_1 = Kp1 * error_1 + Ki1 * error_1_integral + Kd1 * error_1_derivative
ctrl_2 = Kp2 * error_2 + Ki2 * error_2_integral + Kd2 * error_2_derivative