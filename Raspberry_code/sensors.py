from collections import deque

#ett objekt är en signal dvs ett unikt ID
class Encoder():
    def __init__(self, name, maxlen = 10):
        self.name = name
        self.position = deque(maxlen=maxlen)
        self.vel = deque(maxlen=maxlen)
        self.timestamps = deque(maxlen=maxlen)

    def new_data(self, data, ts=None):
        self.position.append(data["SnsrPos_20"])

    #calculate velocity for rotary encoder
    def velocity_encoder(self, time):
        if len(self.position) < 2:
            return 0
        dt = self.timestamps[-1] - self.timestamps[-2] #if dt = 0?
        dp = self.position[-1] - self.position[-2]
        return dp / dt
    
class IMU():
    def __init__(self, name, maxlen = 10):
        self.name = name
        
        for axis in ["X", "Y", "Z"]:
            self.signals[f"Angle{axis}{self.name[4:]}"] = deque(maxlen=maxlen)
            self.signals[f"AngularVelocity{axis}{self.name[4:]}"] = deque(maxlen=maxlen)
            self.signals[f"AngularAcceleration{axis}{self.name[4:]}"] = deque(maxlen=maxlen)
        self.position = None
        self.vel = None
        self.timestamps = deque(maxlen=maxlen)

    def new_data(self, data, ts=None):
        for sig_name, value in data.items():
            if sig_name in self.signals:
                self.signals[sig_name].append(value)
        self.timestamps.append(ts)
    
    #calculate velocity and position for IMU (based of X and Y values?)
    def IMU_to_joint_converter(self):
        #IMUerna sitter ju inte direkt på en led så deras position och hastighet måste konverteras till position och hastighet i led.
        #Man kan ju ra rotationerna på närliggande IMUS minus varandra, men vet inte om det bara ger approximation om det är mycket drift och brus??
        #Man jobbar ju sig utåt från hytten så känns som att det kan bli väldigt brusigt vid skopan?
        self.position = None
        self.vel = None


# Sensorfusion vid given tidpunkt
def fuse_sensors(sensor_manager, ts):
    return None
    
#ibland kommer inte sensordata som man tror, då viktigt att använda senaste värde eller interpolera.
# om tid för gammal, ta bort i fusion och flagga!