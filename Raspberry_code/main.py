import time
from setup_CAN import *
from sensors import *
from kinematics import *
from control import *
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
    main()

#TODO: säker kod! om nått går fel typ listen to can ger felmeddelande eller tar för lång tid så måste skopan stanna. Kanske enklast att få till genom att använda threads så att man kan sätta säkerhet som högst prio?
#TODO: plots to show angles and bucket tip position. Also good to be able to compare plots when we use different sampling time, velocities.