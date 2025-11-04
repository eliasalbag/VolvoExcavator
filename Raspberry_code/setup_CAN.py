import cantools
from canlib import canlib, Frame
from sensors import *

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

#sends control message to MC43
def send_control(channel, control_1, control_2, dbc_hydraulics):
    msg = dbc_hydraulics.get_message_by_name("ControlMessage")

    data_dict = {"UpDown": 0, "Parallelll_RL": 0, "Parallell_FR": 0, "Tilt_FR": 0,
             "Tilt_RL": 0, "Rotate_RL": 0} #Vårt CAN meddelande 
    
    # Konvertera dictionary till CAN-data enligt DBC
    data = msg.encode(data_dict)

    data_dict["UpDown"] = control_1
    data_dict["Parallell_FR"] = control_2
    
    # Skapa och skriv CAN-frame
    frame = Frame(id_=msg.frame_id, data=data)
    channel.write(frame)