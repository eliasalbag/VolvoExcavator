import cantools
from canlib import canlib, Frame
from sensors import *

def load_dbc_files():
    dbc_sensors = cantools.database.load_file("ec20_sensors.dbc")
    dbc_hydraulics = cantools.database.load_file("ec20_hydraulics.dbc")
    return dbc_sensors, dbc_hydraulics

def create_sensor_objects(dbc_sensors):
    sensor_manager = {}
    for msg in dbc_sensors.messages:
        if msg.name not in sensor_manager.keys():
            if "IMU" in msg.name:
                sensor_manager[msg.name] = IMU(msg.name)
            else:
                sensor_manager[msg.name] = Encoder(msg.name)
    return sensor_manager


def open_can_channel():
    channel = canlib.openChannel(0, canlib.Open.EXCLUSIVE)
    channel.setBusOutputControl(canlib.Driver.NORMAL)
    channel.setBusParams(canlib.canBITRATE_500K)
    channel.busOn()

    status = channel.getBusStatus()
    if status == 0:
        print("Bus ON")
    elif status <= 2:
        print("Bus ON, but with warnings")
    else:
        print(f"Bus failed to start. Status: {status}")
    return channel

def listen_can_messages(channel, dbc_sensors, sensor_manager, time):
    try:
        frame = channel.read(timeout=0)
        msg = dbc_sensors.get_message_by_frame_id(frame.id)
        decoded = msg.decode(frame.data)
        sensor_manager[msg.name].new_data(decoded, time)
    except (KeyError, cantools.database.errors.DecodeError):
        print("Error reading data")