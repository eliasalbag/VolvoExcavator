import cantools
from canlib import canlib
#from canlib import can

# Ladda DBC
db = cantools.database.load_file("ec20_sensors.dbc")
# db = cantools.database.load_file("ec20_hydraulics.dbc")

# Öppna Kvaser-kanal
ch = canlib.openChannel(0, canlib.Open.EXCLUSIVE)
ch.setBusOutputControl(canlib.Driver.NORMAL)
ch.setBusParams(canlib.canBITRATE_500K)
ch.busOn()
status = ch.readStatus()
print("Bus status:", status)

"""
sensorer = []
ej_relevanta = [0xEFF0289,0xEFF0289, 0xEFF0289]
#skapar sensorobjekt
for msg in db.messages:
    if msg.id 
    sensorer.append(Sensors(msg))
"""

# BITRATE är 500kbitt5
while True:
    try:
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
            #print("ID:", hex(frame.id))
            #print("DATA (binärt):", " ".join(f"{b:08b}" for b in frame.data))

        except Exception:
            print(f"Okänt meddelande: {frame}")
        
    except KeyboardInterrupt:
        print("\nAvbryter med Ctrl+C – stänger CAN...")
        ch.busOff()
        ch.close()
        print("CAN stängd.")
