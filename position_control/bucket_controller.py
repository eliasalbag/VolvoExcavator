import numpy as np


#### TUNING PARAMETERS

Ts = 0.01

## Bucket Control Parameters
Kp_bucket = 30
Ki_bucket = 3
Kd_bucket = 1


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

control_3 = -(Kp_bucket * bucket_error +  Ki_bucket * bucket_error_integral +  Kd_bucket * bucket_error_derivative - bom_input*0.4 - stick_input*0.4)