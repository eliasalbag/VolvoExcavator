# Inputs:
#   Joystick input: xdot_ref, ydot_ref in [-1, 1]
#   Joint angles: theta1, theta2 in [rad] 
#   max_joint_vel1, max_joint_vel2 [rad/s]


# Outputs:
#   u1, u2        [-100, 100]


import numpy as np

#constants
DT = 0.01  # time step [s]

VX_MAX = 0.5  # max Cartesian speed along x [m/s]
VY_MAX = 0.5  # max Cartesian speed along y [m/s]

AXIS_DEADZONE_NORM = 0.05   # joystick deadzone in normalized units [-1, 1]

M0 = 0.08      # shaping deadzone
P_SHAPE = 2.0  # shaping exponent

# Link lengths [m]
L1 = 1903.392e-3
L2 = 1351.485e-3
L3 = 590.267e-3 #not using the bucket angle here

# PID gains (Cartesian velocity PID)
Kp = 30.0
Ki = 2.0
Kd = 0.0

# Persistent state (keep between calls)

if "theta1_prev" not in globals():
    theta1_prev = None
if "theta2_prev" not in globals():
    theta2_prev = None

if "thetadot1_error_prev" not in globals():
    thetadot1_error_prev = None
if "thetadot2_error_prev" not in globals():
    thetadot2_error_prev = None

if "thetadot1_error_integral_prev" not in globals():
    thetadot1_error_integral_prev = 0.0
if "thetadot2_error_integral_prev" not in globals():
    thetadot2_error_integral_prev = 0.0

# 1) Joystick to Cartesian velocity reference (axis-locked)
# Deadzone check
if not (abs(xdot_ref) <= AXIS_DEADZONE_NORM and abs(ydot_ref) <= AXIS_DEADZONE_NORM):

    # Decide dominant axis: X-mode or Y-mode
    if abs(xdot_ref) > abs(ydot_ref):
        # X-mode
        vel_ref = xdot_ref
        v_max = VX_MAX

        # Axis deadzone
        if abs(vel_ref) <= AXIS_DEADZONE_NORM:
            vx_ref = 0.0

            
        else:
            # Normalize magnitude into [0, 1]
            if vel_ref >= 0.0:
                sign = 1.0
            else:
                sign = -1.0

            mag_norm = (abs(vel_ref) - AXIS_DEADZONE_NORM) / (1.0 - AXIS_DEADZONE_NORM)

            # Shaping
            if mag_norm <= M0:
                f = 0.0
            else:
                r = (mag_norm - M0) / (1.0 - M0)
                f = r ** P_SHAPE

            vx_ref = sign * v_max * f
            vy_ref = 0.0

    else:
        # Y-mode
        vel_ref = ydot_ref
        v_max = VY_MAX

        if abs(vel_ref) <= AXIS_DEADZONE_NORM:
            vy_ref = 0.0
        else:
            if vel_ref >= 0.0:
                sign = 1.0
            else:
                sign = -1.0

            mag_norm = (abs(vel_ref) - AXIS_DEADZONE_NORM) / (1.0 - AXIS_DEADZONE_NORM)

            if mag_norm <= M0:
                f = 0.0
            else:
                r = (mag_norm - M0) / (1.0 - M0)
                f = r ** P_SHAPE

            vy_ref = sign * v_max * f
            vx_ref = 0.0

xydot_ref = np.array([vx_ref, vy_ref])

# 2) Jacobian and joint velocity reference (IK)
J11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
J12 = -L2 * np.sin(theta1 + theta2)
J21 =  L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
J22 =  L2 * np.cos(theta1 + theta2)

J = np.array([[J11, J12],
              [J21, J22]])

Jpinv = np.linalg.pinv(J)

theta_dot_ref_vec = Jpinv @ xydot_ref
theta1_dot_ref = float(theta_dot_ref_vec[0])
theta2_dot_ref = float(theta_dot_ref_vec[1])

theta1_dot_ref = float(np.clip(theta1_dot_ref, -max_joint_vel1, max_joint_vel1))
theta2_dot_ref = float(np.clip(theta2_dot_ref, -max_joint_vel2, max_joint_vel2))
theta_dot_ref_vec = np.array([theta1_dot_ref, theta2_dot_ref])


#LOGGINGMeasured Cartesian velocity (output to log in Simulink)
xydot_meas = J @ thetadot_meas_vec
xdot_meas = float(xydot_meas[0])
ydot_meas = float(xydot_meas[1])


#LOGGINGVelocity error in Cartesian space (output to log in Simulink)
xydot_error = xydot_ref - xydot_meas
xdot_error = float(xydot_error[0])
ydot_error = float(xydot_error[1])


# 3) Measured joint velocities (finite difference)
if theta1_prev is None:
    theta1_prev = theta1
if theta2_prev is None:
    theta2_prev = theta2

theta1dot_meas = (theta1 - theta1_prev) / DT
theta2dot_meas = (theta2 - theta2_prev) / DT

theta1_prev = theta1
theta2_prev = theta2

thetadot_meas_vec = np.array([theta1dot_meas, theta2dot_meas])

# Joint velocity error
thetadot_error = theta_dot_ref_vec - thetadot_meas_vec
theta1dot_error = float(thetadot_error[0])
theta2dot_error = float(thetadot_error[1])


#Integral
# Integral error initialization 
if theta1dot_error_prev is None:
    theta1dot_error_prev = thetadot_error
if theta2dot_error_prev is None:
    theta2dot_error_prev = thetadot_error


theta1dot_error_integral = (theta1dot_error + theta1dot_error_prev) * DT / 2.0 + theta1dot_error_integral_prev
theta2dot_error_integral = (theta2dot_error + theta2dot_error_prev) * DT / 2.0 + theta2dot_error_integral_prev

theta1dot_error_integral_prev = theta1dot_error_integral
theta2dot_error_integral_prev = theta2dot_error_integral
thetadot_error_integral = np.array([theta1dot_error_integral, theta2dot_error_integral])


# Derivative term
theta1dot_error_derivative = (theta1dot_error - theta1dot_error_prev) / DT
theta2dot_error_derivative = (theta2dot_error - theta2dot_error_prev) / DT

thetadot_error_derivative = np.array([theta1dot_error_derivative, theta2dot_error_derivative])

theta1dot_error_prev = theta1dot_error
theta2dot_error_prev = theta2dot_error


# Joint space PID to joint valve control
valve_control_norm = (
    Kp * thetadot_error
    + Ki * thetadot_error_integral
    + Kd * thetadot_error_derivative
)


valve_control_norm = np.clip(valve_control_norm, -1.0, 1.0)

control_1_norm = float(valve_control_norm[0])
control_2_norm = float(valve_control_norm[1])


# Scale normalized control to valve hardware range [-100, 100]
u1 = float(np.clip(control_1_norm * 100.0, -100.0, 100.0))
u2 = float(np.clip(control_2_norm * 100.0, -100.0, 100.0))