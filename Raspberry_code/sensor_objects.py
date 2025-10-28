from collections import deque


#ska ett objekt vara en sensor (krångligt), ett id (med flera singaler olika beroende på vilken sensor), eller ett objekt per signal (många objekt). Chat GPT tycker ett objekt per signal.
class Sensor:
    def __init__(self, msg, maxlen = 400):
        self.name = msg.name #kan nog inte skriva så
        self.id = msg.id
        self.buf = deque(maxlen=maxlen)  # (timestamp, value)

    def push(self, value, ts=None):
        self.buf.append((ts, self.filtered))
        #enkel filtrering behövs (low eller high pass beroende på signal)
        
	# interpolerar sensorvärde vid specifik tid
    def interpolate(self, target_ts):
        return self.buf

#håller koll på alla sensorobjekt
class SensorManager:
    def __init__(self):
        self.sensors = {} #key: sensor namn, value: sensor objekt
        
    def register_sensor(self, name):
        if name not in self.sensors:
            self.sensors[name] = Sensor(name)

    def update_sensor(self, name, value, ts=None):
        self.sensors[name].push(value, ts)

	#returnerar interpolerad sensordata vid en specifik tid
    def get_sensor_data_specified_time(self, target_ts):
        """Interpolerade värden vid gemensam tidpunkt"""
        return {name: s.interpolate(target_ts) for name, s in self.sensors.items()}
    
    def fuse_sensor_data(self, sensor_data):
        #Kalman-filter här
        return sensor_data
    
"""
I main kommer detta behöva skrivas för att skapa sensor/signal objekt:

sensor_manager = SensorManager()
for msg in dbc_sensors.messages:
    if msg.id not in [0xEFF0289,0xEFF0289, 0xEFF0289]:
        for sig in msg.signals:
            sensor_manager.register_sensor(sig.name)

och när man läser av signaler:

ts = time.monotonic()
msg = dbc_sensors.get_message_by_frame_id(frame.id)
data = msg.decode(frame.data)
for sig_name, value in data.items():
    sensor_manager.update_sensor(sig_name, value, ts)

"""