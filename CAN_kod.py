import cantools
from canlib import canlib, Frame
import time

# Ladda DBC
db_sensors = cantools.database.load_file("ec20_sensors.dbc")
db_hydraulics = cantools.database.load_file("ec20_hydraulics.dbc")

# Öppna Kvaser-kanal
ch = canlib.openChannel(0, canlib.Open.EXCLUSIVE)
ch.setBusOutputControl(canlib.Driver.NORMAL)
ch.setBusParams(canlib.canBITRATE_500K)
ch.busOn()

msg = db_hydraulics.get_message_by_name("ControlMessage")

t = time.time()


# Send CAN msg
# UpDown - boom
data_dict = {"UpDown": 0, "Parallelll_RL": 0, "Parallell_FR": 0, "Tilt_FR": 0,
             "Tilt_RL": 0, "Rotate_RL": 0}
while True:
    if 3 < time.time() - t < 5:
        data_dict["UpDown"] = -50
        data = msg.encode(data_dict)
    else:
        data_dict["UpDown"] = 0
        data = msg.encode(data_dict)

    frame = Frame(id_=msg.frame_id, data=data)
    ch.write(frame)
    print(frame)
    time.sleep(0.05)  # 20 Hz – lagom för de flesta CAN-bussar


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
