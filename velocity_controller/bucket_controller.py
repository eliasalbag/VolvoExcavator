import numpy as np
from numpy import sign, abs

#### TUNING PARAMETERS
degpersec 
Ts = 0.01

BUCKET_STICK1 = 16
BUCKET_STICK2 = 32
if flag is None:
    flag = 1

## Bucket Control Parameters
Kp_bucket = Kp
Ki_bucket = Ki
Kd_bucket = Kd

## Bucket controller
calc_thetaw = theta1 + theta2 + theta3


if theta_ref is None:
    theta_ref = -20

if comp_flag_1 is None:
    comp_flag_1 = 0


if bucket_stick_signal == 0 and flag == 0:
    if comp_flag_1 == 1:
        theta_ref = calc_thetaw - 0.6
    else:
        theta_ref = calc_thetaw + 0.6
    bucket_error_integral_prev = 0
    bucket_error_integral = 0
    bucket_error_prev = 0
    flag = 1
    

if bucket_stick_signal == BUCKET_STICK1 or bucket_stick_signal == BUCKET_STICK2:
    if bucket_stick_signal == BUCKET_STICK1 + BUCKET_STICK2:
        pass
    else:

        flag = 0

        if bucket_stick_signal == BUCKET_STICK1:
            theta_ref -= degpersec * Ts 
            comp_flag_1 = 1

        elif bucket_stick_signal == BUCKET_STICK2:
            theta_ref += degpersec * Ts 
            comp_flag_1 = 0
    
bucket_error = theta_ref - calc_thetaw

if bucket_error_prev is None:
    bucket_error_prev = bucket_error
if bucket_error_integral_prev is None:
    bucket_error_integral_prev  = 0

# calculate integral
bucket_error_integral = (bucket_error + bucket_error_prev) * Ts / 2 + bucket_error_integral_prev

bucket_error_integral_prev = bucket_error_integral


# calculate derivative
bucket_error_derivative = (bucket_error - bucket_error_prev) / Ts
bucket_error_prev = bucket_error


control_3 = -(Kp_bucket * bucket_error +  Ki_bucket * bucket_error_integral +  Kd_bucket * bucket_error_derivative  + 0.2*stick_input + 0.2*bom_input)
control_3 = np.clip(control_3,-99, 90)
#if control_3 < 0:
#    control_3 = control_3 * 0.85 - 15
#else:
#    control_3 = control_3 * 0.85 + 15
            
bucket_ref_out = theta_ref