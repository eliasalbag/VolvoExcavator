import time
#from setup_CAN import *
from sensors import *
from kinematics import *
from control import *
from plot import *

def main_sim():
    #load sensor data from dbc files
    dbc_sensors, dbc_hydraulics = load_dbc_files()

    #create signal objects
    sensor_manager = create_sensor_objects(dbc_sensors)

    sampling_period = 0.01 # 10 ms = 100 Hz
    last_time = time.monotonic()

    sampling_period = 0.01  # 100 Hz
    for i in range(500):  # exempel med 5 sekunder data
        ts = time.monotonic()

        # Simulerade data, i verkligheten kommer detta från CAN
        imu_x_data = {"AngleXAxis_128": i*0.01, "AngularVelocityXAxis_128": 1.0}
        imu_y_data = {"AngleYAxis_128": i*0.015, "AngularVelocityYAxis_128": 1.2}
        encoder_data = {"SnsrPos_20": i*0.02}

        sensor_manager["IMU_XAxis_128"].new_data(imu_x_data, ts)
        sensor_manager["IMU_YAxis_128"].new_data(imu_y_data, ts)
        sensor_manager["Rot_01"].new_data(encoder_data, ts)

        time.sleep(sampling_period)
        now = time.monotonic()

    # Returns fused sensor data at specified time (latest)
    sensor_data = fuse_sensors(sensor_manager,now)
            
    # Kinematics
    ref = kinematics()

    # Calculate control signal
    control_signal = control(ref, sensor_data)

    # Send control signal
    safe_control = safety_check(control_signal)
    send_control(safe_control)

    plot(sensor_manager)

if __name__ == "__main__":
    main_sim()