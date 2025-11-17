#code for testing how the numerical derivation is affected by sampling time etc.
#change sampling time to see how the derivated output is affected

    

if theta1_prev is None:
    theta1_prev = 0.0
if theta2_prev is None:
    theta2_prev = 0.0

#raw derivative calculation
#DT is a parameter
theta1dot_meas = (theta1 - theta1_prev) / DT
theta2dot_meas = (theta2 - theta2_prev) / DT

theta1_prev = theta1
theta2_prev = theta2



## moving mean derivative calculation
#N is parameter
'''
If your sampling frequency is:

100 Hz → try N = 3, 5, or 7

50 Hz → try N = 2 or 3

200 Hz → try N = 10–15
'''
if theta1_hist is None:
    theta1_hist = [theta1] * N
if theta2_hist is None:
    theta2_hist = [theta2] * N

#moving average filter
theta1_hist.pop(0)
theta1_hist.append(theta1)

theta2_hist.pop(0)
theta2_hist.append(theta2)

theta1dot_meas_mean = (theta1_hist[-1] - theta1_hist[0]) / (N * DT)
theta2dot_meas_mean = (theta2_hist[-1] - theta2_hist[0]) / (N * DT)


#filtering with a simple low-pass filter
#alpha is parameter between 0 and 1
#α≈0: almost no filtering (fast response)
#α≈1: very strong filtering (slow response)

ALPHA = 0.7  # 0 = no filter, 1 = very strong smoothing

if theta1dot_filt_prev is None:
    theta1dot_filt_prev = theta1dot_meas

if theta2dot_filt_prev is None:
    theta2dot_filt_prev = theta2dot_meas

theta1dot_meas_filt = ALPHA*theta1dot_filt_prev + (1-ALPHA)*theta1dot_meas
theta1dot_filt_prev = theta1dot_meas_filt

theta2dot_meas_filt = ALPHA*theta2dot_filt_prev + (1-ALPHA)*theta2dot_meas
theta2dot_filt_prev = theta2dot_meas_filt