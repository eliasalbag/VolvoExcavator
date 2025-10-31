from collections import deque
import cantools

class Encoder():
    def __init__(self, name):
        self.name = name
        self.position = []
        self.vel = []
        self.timestamps = []

    def new_data(self, data, ts=None):
        self.position.append(data["SnsrPos_20"])
        self.timestamps.append(ts)
        self.velocity_encoder(ts)

    #calculate velocity for rotary encoder
    def velocity_encoder(self, time):
        if len(self.position) < 2:
            self.vel.append(0)
        else:
            dt = self.timestamps[-1] - self.timestamps[-2] #if dt = 0?
            dp = self.position[-1] - self.position[-2]
            self.vel.append(dp / dt)
        
class IMU():
    def __init__(self, name):
        self.name = name
        self.signals = {}
        
        for axis in ["X", "Y", "Z"]:
            self.signals[f"Angle{axis}{self.name[5:]}"] = []
            self.signals[f"AngularVelocity{axis}{self.name[5:]}"] = []
            self.signals[f"AngularAcceleration{axis}{self.name[5:]}"] = []
        self.position = None
        self.vel = None
        self.timestamps = []

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
    
#ibland kommer inte sensordata som man tror, då viktigt att använda senaste värde eller interpolera.
# om tid för gammal, ta bort i fusion och flagga!