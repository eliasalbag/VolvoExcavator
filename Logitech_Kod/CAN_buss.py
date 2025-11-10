import cantools
from canlib import canlib, Frame
import sys, time


import threading
import time
import logitech_test as joystick

# Starta joystick koden i en separat thread/tråd
t = threading.Thread(target=joystick.main, daemon=True)
t.start()


# Ladda DBC
db_sensors = cantools.database.load_file("ec20_sensors.dbc")
db_hydraulics = cantools.database.load_file("ec20_hydraulics.dbc")

# Öppna Kvaser-kanal
ch = canlib.openChannel(0, canlib.Open.EXCLUSIVE)
ch.setBusOutputControl(canlib.Driver.NORMAL)
ch.setBusParams(canlib.canBITRATE_500K)
ch.busOn()


# Hämta meddelandedefinitionen från DBC-filen
# "msg"-objektet innehåller all information som är definierad i DBC-filen, inklusive:
# - meddelandenamnet ("ControlMessage")
# - dess CAN-ID (till exempel 123 eller 0x7B)
# - dess signaler (t.ex. "UpDown", "Tilt_FR".)
msg = db_hydraulics.get_message_by_name("ControlMessage")

t = time.time()



# Skicka CAN meddelande
# UpDown - boom
data_dict = {"UpDown": 0, "Parallelll_RL": 0, "Parallell_FR": 0, "Tilt_FR": 0,
             "Tilt_RL": 0, "Rotate_RL": 0} #Vårt CAN meddelande 
while True:
    if 3 < time.time() - t < 20:
        data_dict["UpDown"] = joystick.ly
        data_dict["Tilt_FR"] = 0
        data_dict["Parallell_FR"] = -joystick.ry
        data = msg.encode(data_dict) #Skriva om vårt CAN meddelande till 8-bits meddelande så att vi kan skicka den via CAN-buss
        #Alltså - binärt version av UpDown = -50
    else:
        data_dict["UpDown"] = 0
        data_dict["Tilt_FR"] = 0
        data_dict["Parallell_FR"] = 0
        data = msg.encode(data_dict)
        
        
    

    frame = Frame(id_=msg.frame_id, data=data) #msg.frame_id är meddelandets ID - kommer direkt från DBC definationen i rad 15
    ch.write(frame)
    #print(frame)
    time.sleep(0.05)  # 20 Hz – lagom för de flesta CAN-bussar
    
    # Exempel: använd variablerna kontinuerligt
    

    sys.stdout.write(f"\rMODE: {joystick.MODE.upper()} | "
            f"BUCKET: {joystick.bucket:+.0f}% | "
            f"LY: {joystick.ly:+.0f}% | RY: {joystick.ry:+.0f}% | "
            f"LT: {joystick.lt:>3.0f}% | RT: {joystick.rt:>3.0f}%   "  # extra blanks to overwrite leftovers
        )
    sys.stdout.flush()
    time.sleep(0.05)


# # BITRATE är 500kbit
# while True:
#     frame = ch.read(timeout=4000)
#     try:
#         msg = db.decode_message(frame.id, frame.data)
#         print(f"ID={hex(frame.id)} → {msg}")
#         # print(frame)
#         print("ID:", hex(frame.id))
#         print("DATA (binärt):", " ".join(f"{b:08b}" for b in frame.data))
#
#     except Exception:
#         print(f"Okänt meddelande: {frame}")
