import time
#from setup_CAN import *
from sensors import *

from plot import *

def main_sim():
    #load sensor data from dbc files
    dbc_sensors, dbc_hydraulics = load_dbc_files()

    #create signal objects
    sensor_manager = create_sensor_objects(dbc_sensors)

    sampling_period = 0.01 # 10 ms = 100 Hz
    last_time = time.monotonic()

    kalman_filter = KalmanFilter(sampling_period)

    sampling_period = 0.01  # 100 Hz
    for i in range(3):  # exempel med 5 sekunder data
        ts = time.monotonic()

        # Simulerade data, i verkligheten kommer detta från CAN
        imu_128 = {"AngleXAxis_128": 0.0, "AngularVelocityXAxis_128": 0.0}
        imu_137 = {"AngleXAxis_137": i*0.015, "AngularVelocityXAxis_137": 1.2}
        imu_138 = {"AngleXAxis_138": i*0.01, "AngularVelocityXAxis_138": 1.0}
        imu_139 = {"AngleXAxis_139": i*0.015, "AngularVelocityXAxis_139": 1.2}

        encoder_data = {"SnsrPos_20": i*0.02}

        sensor_manager["IMU_XAxis_128"].new_data(imu_128, ts)
        sensor_manager["IMU_XAxis_137"].new_data(imu_137, ts)
        sensor_manager["IMU_XAxis_138"].new_data(imu_138, ts)
        sensor_manager["IMU_XAxis_139"].new_data(imu_139, ts)

        sensor_manager["Rot_01"].new_data(encoder_data, ts)
        sensor_manager["Rot_20"].new_data(encoder_data, ts)
        sensor_manager["Rot_33"].new_data(encoder_data, ts)

        IMU_joints = IMU_to_joint_converter(sensor_manager)
        IMU_joints = IMU_to_joint_converter(sensor_manager)

        # get angle values
        fused = fuse_sensors(sensor_manager, IMU_joints, kalman_filter, ts)

        encoder = ["Rot_01", "Rot_20", "Rot_33"]
        for i in range(3):  
            print(f"--- Joint {i+1} ---")
            print(f"IMU position: {IMU_joints[i][0]}")
            print(f"IMU velocity: {IMU_joints[i][1]}")
            print(f"Encoder position: {sensor_manager[encoder[i]].position[-1]}")
            print(f"Encoder velocity: {sensor_manager[encoder[i]].vel[-1]}")
            print(f"Fused position: {fused[i][0]}")
            print(f"Fused velocity: {fused[i][1]}")
            print() 
        time.sleep(sampling_period)
        now = time.monotonic()

    plot(sensor_manager)

if __name__ == "__main__":
    main_sim()