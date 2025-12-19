import numpy as np
from numpy import sign
"""
Reference handler for velocity controller.

Input: desired velocity in y-direction
Output: y-level reference 

y_ref: persistent
flag: persistent

"""

# stick tolerance depending on expected noise level of joystick
stick_tolerance = 0.01

# desired reference change in m/s when stick is fully deflected
meterspersec = np.abs(stick_signal)
Ts = 0.01

############################################################################
# Reference handler code
############################################################################

import numpy as np
from numpy import sign

if flag is None:
    flag = 1

if y_ref is None:
    y_ref = 0.5

if stick_signal < stick_tolerance and stick_signal > -stick_tolerance:
    stick_signal = 0

if stick_signal == 0 and flag == 0:
    y_ref = y_meas
    flag = 1

if stick_signal != 0:
        flag = 0
        y_ref += sign(stick_signal) * meterspersec * Ts 

y_ref_out = y_ref