import cantools
from canlib import canlib, Frame
from sensors import *
import time

def open_can_channel():
    # Öppna Kvaser-kanal
    ch = canlib.openChannel(0, 0)
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.setBusParams(canlib.canBITRATE_500K)
    ch.busOn()
    print("Välkommen till VOLVO CE projektet här kommer Viktro Traktor!")
    status = ch.readStatus()
    print("Bus status:", status)
    return ch
    
"""
def listen_can_messages(channel, dbc_sensors, sensor_manager, time):
    try:
        frame = channel.read(timeout=1)
        try:
            msg = dbc_sensors.get_message_by_frame_id(frame.id)
        except:
            return
        decoded = msg.decode(frame.data)
        sensor_manager[msg.name].new_data(decoded, time)
    except (KeyError, cantools.database.errors.DecodeError):
        print("Error reading data")
"""
def listen_can_messages(channel, dbc_sensors, sensor_manager, _ignored_ts=None):
    """Läs ur alla väntande CAN-ramar just nu (drain) utan att blockera."""
    while True:
        try:
            # timeout=0 => icke-blockerande; kastar CanNoMsg om kön är tom
            frame = channel.read(timeout=0)
        except canlib.CanNoMsg:
            break  # inget mer i kön just nu
        except canlib.CanError as e:
            print("CAN error:", e)
            break

        # Tidsstämpla varje ram när den faktiskt läses
        ts = time.monotonic()

        try:
            msg = dbc_sensors.get_message_by_frame_id(frame.id)
            decoded = msg.decode(frame.data)
            sensor_manager[msg.name].new_data(decoded, ts)
        except (KeyError, cantools.database.errors.DecodeError):
            # okänd frame eller decode-fel – hoppa vidare
            continue

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
