"""
Controller for 2DOF robotic arm using PID control in joint space.
Calculates control inputs for the two joints based on reference (x,y) position.
"""

# Tuning parameters for the PID controller
#Kp1, Ki1, Kd1 = 30, 2, 0
#Kp2, Ki2, Kd2 = 30, 2, 0
Ts = 0.01

alpha = 0.1 # 
beta = 10   # 

"""
Initalize variables
"""

from numpy import sin, cos, arctan2 as atan2, sqrt
import numpy as np
# Remove the following line when integrating with point_planner

L1 = 1903.392e-3
L2 = 1351.485e-3


"""
Inverse kinematics. Takes (x,y) reference and calculates corresponding joint angles (theta1_ref, theta2_ref). 
"""

cost2 = (x_ref**2 + y_ref**2 - L1**2 - L2**2) / (2 * L1 * L2)
sint2 = -sqrt(1 - cost2**2)

theta1_ref = atan2(y_ref, x_ref) - atan2(L2 * sint2, L1 + L2 * cost2)
theta2_ref = atan2(sint2, cost2)

"""
Convert to degrees
"""

theta1_ref = theta1_ref * 180 / np.pi
theta2_ref = theta2_ref * 180 / np.pi

theta1_meas_rad = theta1_meas / 180 * np.pi
theta2_meas_rad = theta2_meas / 180 * np.pi

"""
PID - controller
"""

error_1 = theta1_ref - theta1_meas
error_2 = theta2_ref - theta2_meas


"""
Error 1 integral
"""
if error_1_prev is None:
    error_1_prev = 0

if error_1_integral is None:
    error_1_integral = 0

error_1_integral = (error_1_prev + error_1) * Ts / 2 + error_1_integral

error_1_prev = error_1 

"""
Error 2 integral
"""

if error_2_prev is None:
    error_2_prev = 0

if error_2_integral is None:
    error_2_integral = 0

error_2_integral = (error_2_prev + error_2) * Ts / 2 + error_2_integral
error_2_prev = error_2

"""
Derivative Calculation with Filter
"""
raw_derivative_1 = (error_1 - error_1_prev) / Ts
raw_derivative_2 = (error_2 - error_2_prev) / Ts

error_1_derivative_filtered = alpha * raw_derivative_1 + (1 - alpha) * error_1_derivative_filtered
error_2_derivative_filtered = alpha * raw_derivative_2 + (1 - alpha) * error_2_derivative_filtered

""" 
PID Control Law
"""

# If boom is to move upwards
if error_1 > 0:
    ctrl_1 = clip(Kp1 * error_1 + Ki1 * error_1_integral + Kd1 * error_1_derivative_filtered + cos(theta1_meas_rad + theta2_meas_rad) * error_1 * beta, -99, 99)
    
else:
    ctrl_1 = clip(Kp1 * error_1 + Ki1 * error_1_integral + Kd1 * error_1_derivative_filtered, -99, 99)

ctrl_2 = clip(Kp2 * error_2 + Ki2 * error_2_integral + Kd2 * error_2_derivative_filtered, -99, 99)