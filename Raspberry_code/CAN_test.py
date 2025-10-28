
"""
sensorer = []
ej_relevanta = [0xEFF0289,0xEFF0289, 0xEFF0289]
#skapar sensorobjekt
for msg in db.messages:
    if msg.id 
    sensorer.append(Sensors(msg))
"""
import cantools
from canlib import canlib, Frame
import time


def load_dbc_files():
    dbc_sensors = cantools.database.load_file("ec20_sensors.dbc")
    dbc_hydraulics = cantools.database.load_file("ec20_hydraulics.dbc")
    return dbc_sensors, dbc_hydraulics


def open_can_channel():
    # Open Kvaser CAN channel
    channel = canlib.openChannel(0, canlib.Open.EXCLUSIVE)
    channel.setBusOutputControl(canlib.Driver.NORMAL)
    channel.setBusParams(canlib.canBITRATE_500K)
    channel.busOn()

    print("CAN channel opened and bus is ON")
    return channel


def listen_can_messages(channel, dbc_sensors):
    '''
    Store sensor values (encoder values) in a dictionary
    later ? Store spacemouse-inputs in a dictionary
    '''
    sensor_values = {}
    spacemouse_inputs = {}

    try:
        while True:
            frame = channel.read(timeout=100)  # timeout in ms
            try:
                msg = dbc_sensors.get_message_by_frame_id(frame.id)
                data = msg.decode(frame.data)
                # Store all sensor values
                for key, value in data.items():
                    sensor_values[key] = value

            except (KeyError, cantools.database.errors.DecodeError):
                # Unknown message or decode error, skip
                continue
    except KeyboardInterrupt:
        print("Stopped listening to CAN messages.")

    return sensor_values


def main():
    dbc_sensors, dbc_hydraulics = load_dbc_files() #ska vi ladda in alla 3 sensorerna separat här också?
    channel = open_can_channel()

    msg_hydraulics = dbc_hydraulics.get_message_by_name("ControlMessage")

    listen_can_messages(channel, dbc_sensors)




if __name__ == "__main__":
    main()
