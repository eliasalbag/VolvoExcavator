import time
from setup_CAN import *
from sensors import *
from kinematics import *
from partial_bucket_PID_control import *
from plot import *

def main():
    #open bus
    channel = open_can_channel()

    #load sensor data from dbc files
    dbc_sensors, dbc_hydraulics = load_dbc_files()

    #create signal objects
    sensor_manager = create_sensor_objects(dbc_sensors)

    sampling_period = 0.01 # 10 ms = 100 Hz
    last_time = time.monotonic()

    while True:
        now = time.monotonic()
        #read sensors
        listen_can_messages(channel, dbc_sensors, sensor_manager, now)

        if now - last_time >= sampling_period:
            last_time = now

            IMU_joints = IMU_to_joint_converter(sensor_manager)
            encoder = ["Rot_01", "Rot_20", "Rot_33"]
            for i in range(3):
                print(f"--- Joint {i+1} ---")
                print(f"IMU position: {IMU_joints[i][0]}")
                print(f"IMU velocity: {IMU_joints[i][1]}")
                print(f"Encoder position: {sensor_manager[encoder[i]].position[-1]}")
                print(f"Encoder velocity: {sensor_manager[encoder[i]].vel[-1]}")
                print() 
            #använd för att justera så att IMU och encoder är samma?
            #IMU får vinkel från början
            #Encoders måste positioneras korrekt eller hur vet man absoluta positionen?
                
            #vilka vinklar ska in i regleringen??
            #måste de kalibreras från början?
            #kalibrera sensorer?
                

            break

            #läs av joystick

            # Calculate control signal
            #control_signal = control(0.1, 0, sensor_data[0], sensor_data[1], sensor_data[2])

            # Send control signal
            #safe_control = safety_check(control_signal)
            #send_control(channel, safe_control[0], safe_control[1], dbc_hydraulics)


if __name__ == "__main__":
    main()

#TODO: säker kod! om nått går fel typ listen to can ger felmeddelande eller tar för lång tid så måste skopan stanna. Kanske enklast att få till genom att använda threads så att man kan sätta säkerhet som högst prio?