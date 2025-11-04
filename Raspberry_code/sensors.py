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
        self.signals[f"Angle{self.name[4:]}"] = []
        self.signals[f"AngularVelocity{self.name[4:]}"] = []
        self.signals[f"AngularAcceleration{self.name[4:]}"] = []

        self.position = None
        self.vel = None
        self.timestamps = []

    def new_data(self, data, ts=None):
        for sig_name, value in data.items():
            if sig_name in self.signals:
                self.signals[sig_name].append(value)
        self.timestamps.append(ts)
    
#calculate velocity and position for IMU (based of X and Y values?)
def IMU_to_joint_converter(sensor_manager):
    # 128 hytt, 137 bom, 138 arm 139 skopa
    joint1_pos = sensor_manager["IMU_XAxis_137"].signals["AngleXAxis_137"][-1] - sensor_manager["IMU_XAxis_128"].signals["AngleXAxis_128"][-1]
    joint1_vel = sensor_manager["IMU_XAxis_137"].signals["AngularVelocityXAxis_137"][-1] - sensor_manager["IMU_XAxis_128"].signals["AngularVelocityXAxis_128"][-1]

    joint2_pos = sensor_manager["IMU_XAxis_138"].signals["AngleXAxis_138"][-1] - joint1_pos
    joint2_vel = sensor_manager["IMU_XAxis_138"].signals["AngularVelocityXAxis_138"][-1] - joint1_pos

    joint3_pos = sensor_manager["IMU_XAxis_139"].signals["AngleXAxis_139"][-1] - joint2_pos
    joint3_vel = sensor_manager["IMU_XAxis_139"].signals["AngularVelocityXAxis_139"][-1] - joint2_pos

    IMU_joints = [[joint1_pos, joint1_vel],[joint2_pos, joint2_vel],[joint3_pos, joint3_vel]]
    return IMU_joints

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