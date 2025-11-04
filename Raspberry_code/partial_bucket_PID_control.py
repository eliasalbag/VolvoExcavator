import numpy as np

bucket_error_prev = None
x_dot_error_prev = None
y_dot_error_prev = None
bucket_error_integral_prev = None
x_dot_error_integral_prev = None
y_dot_error = None

def control (xdot_ref, ydot_ref, theta_ref, theta1, theta2, theta3):

    #### TUNING PARAMETERS
    Ts = 0.01

    ## Velocity Control Parameters
    Kp = 1e-3
    Ki = 0
    Kd = 0

    ## Bucket Control Parameters
    Kp_bucket = 2e-3
    Ki_bucket = 0
    Kd_bucket = 1e-3

    ## Bucket controller
    calc_thetaw = theta1 + theta2 + theta3

    bucket_error = theta_ref - calc_thetaw

    if bucket_error_prev is None:
        bucket_error_prev = bucket_error
    if bucket_error_integral_prev is None:
        bucket_error_integral_prev  = 0

    # calculate integral
    bucket_error_integral = (bucket_error + bucket_error_prev) * Ts / 2 + bucket_error_integral_prev

    bucket_error_integral_prev = bucket_error_integral
    bucket_error_prev = bucket_error

    # calculate derivative
    bucket_error_derivative = (bucket_error - bucket_error_prev) / Ts

    control_3 = Kp_bucket * bucket_error +  Ki_bucket * bucket_error_integral +  Kd_bucket * bucket_error_derivative


    ## 2-link controller
    # Inputs 
    L1 = 1903.392e-3
    L2 = 1351.485e-3
    L3 = 590.267e-3

    # Compute end-effector position 
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)

    # Compute Jacobian
    J11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    J12 = -L2 * np.sin(theta1 + theta2)
    J21 =  L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    J22 =  L2 * np.cos(theta1 + theta2)

    J = np.array([[J11, J12],
                [J21, J22]])

    # Compute pseudoinverse and inverse
    Jpinv = np.linalg.pinv(J)

    # Reference velocity
    xy_dot_ref = np.array([xdot_ref, ydot_ref])

    # Joint velocities
    jointVelocities = Jpinv @ xy_dot_ref


    # Map joint velocities
    #

    # Angle velocities

        
    # Initialize on first call
    if theta1_prev is None:
        theta1_prev = theta1
    if theta2_prev is None:
        theta2_prev = theta2

    # Compute derivatives (simple finite difference)
    theta1_dot = (theta1 - theta1_prev) / Ts
    theta2_dot = (theta2 - theta2_prev) / Ts

    # Update previous values
    theta1_prev = theta1
    theta2_prev = theta2

    theta_dot_vec = np.array([theta1_dot, theta2_dot])

    xy_dot = J @ theta_dot_vec
    x_dot = xy_dot[0]
    y_dot = xy_dot[1]

    xy_dot_error = xy_dot_ref - xy_dot 
    x_dot_error = xy_dot_error[0]
    y_dot_error = xy_dot_error[1]

    # xy_dot_error integral

    if x_dot_error_prev is None:
        x_dot_error_prev = x_dot_error
    if y_dot_error_prev is None:
        y_dot_error_prev = y_dot_error

    if x_dot_error_integral_prev is None:
        x_dot_error_integral_prev = 0
    if y_dot_error_integral_prev is None:
        y_dot_error_integral_prev = 0

    # calculate integral
    x_dot_error_integral = (x_dot_error + x_dot_error_prev) * Ts / 2 + x_dot_error_integral_prev
    y_dot_error_integral = (y_dot_error + y_dot_error_prev) * Ts / 2 + y_dot_error_integral_prev

    # pass integral to next iteration
    x_dot_error_integral_prev = x_dot_error_integral
    y_dot_error_integral_prev = y_dot_error_integral

    xy_dot_error_integral = np.array([x_dot_error_integral, y_dot_error_integral])

    # xy_dot_error derivative
    x_dot_error_derivative = (x_dot_error - x_dot_error_prev) / Ts
    y_dot_error_derivative = (y_dot_error - y_dot_error_prev) / Ts
    xy_dot_error_derivative = np.array([x_dot_error_derivative, y_dot_error_derivative])


    # PID control
    valve_ctrl = Kp * xy_dot_error + Ki * xy_dot_error_integral + Kd * xy_dot_error_derivative
    valve_ctrl = np.clip(valve_ctrl, -1.0, 1.0)

    control_1, control_2 = valve_ctrl

    # pass error to next iteration
    x_dot_error_prev = x_dot_error
    y_dot_error_prev = y_dot_error

    return control_1, control_2

#makes sure control signal is within bounds. kanske kolla att vi inte har några otillåtna vinklar?
def safety_check(control_signal):
    return None
