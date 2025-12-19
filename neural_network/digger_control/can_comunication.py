from math import pi
import time
from canlib import canlib, Frame

# Parameters
ANGLE_OFFSET = {"theta1": 3.59607, "theta2": 4.652175, "theta3": 3.582114}
TO_RADIANS = 2 * pi / 65535


def update_angle_data(values, data_history, theta_time, sensor_data, theta, dtheta):
    angle = sensor_data * TO_RADIANS - ANGLE_OFFSET[theta]
    if data_history[theta][0] is not None:
        values[dtheta] = (angle - data_history[theta][0]) / (time.time() - theta_time[theta][0])
    else:
        values[dtheta] = 0
    values[theta] = angle
    return values


def get_sensor_data(ch_sensors, values, data_history, theta_time, cycle_time, db_sensors, time_delay):
    # Read sensor data
    try:
        try:
            frame = ch_sensors.read(timeout=0)
            msg = db_sensors.decode_message(frame.id, frame.data)
            if 'SnsrPos_33' in msg.keys():
                # Boom
                values = update_angle_data(values, data_history, theta_time, msg['SnsrPos_33'], "theta1",
                                                  "dtheta1")
            elif 'SnsrPos_20' in msg.keys():
                # Stick
                values = update_angle_data(values, data_history, theta_time, msg['SnsrPos_20'], "theta2",
                                                  "dtheta2")
            elif 'SnsrPos_01' in msg.keys():
                # Bucket
                values = update_angle_data(values, data_history, theta_time, msg['SnsrPos_01'], "theta3",
                                                  "dtheta3")
        except KeyError:
            pass
    except canlib.canNoMsg:
        pass

    # Save data if 1 / DATA_FREQUENCY number of seconds has passed
    if time.time() - cycle_time > time_delay:
        cycle_time = time.time()

        # Save all values to a list (where data is sampled to DATA_FREQUENCY Hz)
        for theta in data_history.keys():
            data_history[theta][:-1] = data_history[theta][1:]
            data_history[theta][-1] = values[theta]
            theta_time[theta][:-1] = theta_time[theta][1:]
            theta_time[theta][-1] = time.time()

    return values, data_history, theta_time, cycle_time


def send_control_signal(ch_hydraulics, msg, data_send):
    data = msg.encode(data_send)
    frame = Frame(id_=msg.frame_id, data=data)
    ch_hydraulics.write(frame)






