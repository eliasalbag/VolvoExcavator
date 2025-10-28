import cantools
from canlib import canlib
#from canlib import can

# Ladda DBC
db = cantools.database.load_file("ec20_sensors.dbc")
# db = cantools.database.load_file("ec20_hydraulics.dbc")

# Öppna Kvaser-kanal
ch = canlib.openChannel(1, canlib.Open.EXCLUSIVE)
ch.setBusOutputControl(canlib.Driver.NORMAL)
ch.setBusParams(canlib.canBITRATE_500K)
ch.busOn()
status = ch.readStatus()
print("Bus status:", status)

# BITRATE är 500kbitt5
while True:
    try:
        frame = ch.read(timeout=4000)
    except ch.CanBusOff:
        print("BUS OFF → resetting channel…")
        # Option A: simple reset in one call
        ch.resetBus()                      # <- takes the channel off-bus then on-bus
        # Option B: manual cycle if you prefer
        # ch.busOff()
        # time.sleep(0.2)
        # ch.busOn()
    try:
        msg = db.decode_message(frame.id, frame.data)
        print(f"ID={hex(frame.id)} → {msg}")
        # print(frame)
        print("ID:", hex(frame.id))
        print("DATA (binärt):", " ".join(f"{b:08b}" for b in frame.data))

    except Exception:
        print(f"Okänt meddelande: {frame}")
