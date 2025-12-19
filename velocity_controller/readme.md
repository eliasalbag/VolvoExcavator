# Velocity Controller
This controller is used in **master.slx**. 

' denotes velocity


## master.slx/controller/y_reference_handler
.py Python code block
Input: stick_signal, y_meas
Output: y_ref_out 
Persistent: y_ref, flag

Calculates the y-level in increments. When joystick not actuated: y-level is set to the current position + offset. The offset is a tuneable parameter and is to be seen as a time delay compensation.

## master.slx/bucket_controller
.py Python code block
Input:theta1, theta2, theta3, bom_input, stick_input, bucket_stick_signal, Kp, Ki, Kd, degpersec
Output: control_3, calc_thetaw, bucket_ref_out
Persistent: theta_ref, bucket_error_prev, bucket_error_integral_prev, flag, comp_flag_1

Calculates bucket control signal (control_3). Uses a feedforward part from the boom and stick -input. Kp, Ki, Kd are tuneable parameters. degpersec is the desired angular velocity when rotating the bucket.


## master.slx/controller/inverse_kinematics
.m Matlab fcn
Input: 2 velocities, 2 angles (x', y', theta1, theta2)
Output: angle velocity (boom, stick)

Calculates reference angle velocity from desired Cartesian velocity

## master.slx/calculate_position
.m Matlab fcn
Input: theta1, theta2
Output: x, y
Calculates x,y position from current geometry for data logging purposes

## master.slx/calculate_velocity
.m Matlab fcn
Input: theta1', theta2', theta1, theta2
Output: x', y'
Calculates x,y velocity from current geometry for data logging purposes


Built to be used in Matlab/Simulink code-blocks.
Written outside of Simulink for the convenience of a real IDE.
*Niklas Nyberg, niknyb@kth.se*
