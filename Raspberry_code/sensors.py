from collections import deque
import cantools
import numpy as np

class Encoder():
    def __init__(self, name):
        self.name = name
        self.position = []
        self.vel = []
        self.timestamps = []
        self.offset = {"Rot_01": 30.5,"Rot_20": 30.5, "Rot_30": 30.5 }

    def new_data(self, data, ts=None):
        self.position.append(data["SnsrPos"+self.name[-3:]])
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

# Kalman filter: 
# 1. Gissa vart systemet är på väg genom att ta förra vinkeln + hastighet × tidssteg.
# 2. Korrigera gissnign med sensorerna så kolla vad encoder säger och vad IMU säger och kombinera.

class KalmanFilter:
    def __init__(self, dt): #antar att dt är konstant
        F = np.array([[1, dt], [0, 1]]) #hur man antar att ny vinkel blir utan att läsa sensorna. Detta är ny_vinkel = gammal vinkel + dt*hastighet. Och konstant hastighet.
        B = np.array([[0], [0]]) #ska vara styrsignalen om man vill göra model-based kalman men enklast att sätta 0.
        H = np.array([
                    [1, 0],  # encoder vinkel
                    [0, 1],  # encoder hastighet
                    [1, 0],  # IMU vinkel
                    [0, 1]   # IMU hastighet
                    ])
        Q = np.array([[0.01, 0],[0, 0.1]]) #Hur osäkert systemet är (modellfel, vibrationer, hydraulik). Små värden är filtret litar mer på modellen.
        
        x0 = np.array([[0], [0]]) #initial gissning på hastighet och position. Ofta börjar man på 0,0.
        P0 = np.array([[5, 0], [0, 5]]) #initial osäkerhet. Bör va hög eftersom vi inte vet startvinklarna?

        #Hur osäkra mätningarna är. Vinkel och vinkelhastighet. Första är encoder andra är IMU. Lägre värde = litar mer.
        R = np.array([[0.001, 0,    0,    0],  # encoders vinkel är väldigt pålitlig
              [0,    0.001, 0,    0],  # encoders hastighet, bra men lite brusig
              [0,    0,    0.01,  0],  # IMU vinkel, mer brusig
              [0,    0,    0,    0.01]]) # IMU hastighet

        self.F = F
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.x = x0
        self.P = P0

    def predict(self, u):
        self.x = self.F @ self.x #matrismultiplikation. gissning på ny vinkel och hastighet
        self.P = np.dot(self.F, np.dot(self.P, self.F.T)) + self.Q
        return self.x
    
    def update(self, z):
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.P.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        return self.x

 # Sensorfusion vid given tidpunkt
def fuse_sensors(sensor_manager, IMU_joints, kf, ts):
    fused = []
    IMU_joints = IMU_to_joint_converter(sensor_manager)
    encoder = ["Rot_33", "Rot_20", "Rot_01"]
    for i in range(3):
        encoder = sensor_manager["Rot_01"]

        #kontrollerar att tidsstämplar är inom rimligt tid, annars saknar vi data från någon sensor
        tolerance = 0.01
        if IMU_joints == None: #data med inte godkänd tidsstämpel från IMU
            if encoder.position[-1] > ts - tolerance:
                fused.append(encoder.position[-1], encoder.vel[-1])
                print(f"Warning. Missing data from IMUs")
                continue
        if encoder.position[-1] > ts - tolerance:
                fused.append([IMU_joints[i][0],IMU_joints[i][1]])
                print(f"Warning. Missing data from encoder: {encoder.name}")
                continue

        z = np.array([
                    [encoder.position[-1]],
                    [encoder.vel[-1]],
                    [IMU_joints[i][0]],
                    [IMU_joints[i][1]]
                    ])
        predicted_state = kf.predict(z)
        updated_state = kf.update(z)
        fused.append(updated_state)
    return fused

def IMU_to_joint_converter(sensor_manager):
    if sensor_manager["IMU_XAxis_128"].timestamps[-1] and sensor_manager["IMU_XAxis_137"].timestamps[-1] and sensor_manager["IMU_XAxis_138"].timestamps[-1] and sensor_manager["IMU_XAxis_139"].timestamps[-1]:#kontrollera att alla IMUer har godkända tidsstämplar.
        #128 hytt, 137 bom, 138 arm, 139 skopa
        joint1_pos = sensor_manager["IMU_XAxis_137"].signals["AngleXAxis_137"][-1] - sensor_manager["IMU_XAxis_128"].signals["AngleXAxis_128"][-1] - (sensor_manager["IMU_XAxis_128"].signals["AngleXAxis_128"][0]-sensor_manager["Rot_01"].position[0])
        joint1_vel = sensor_manager["IMU_XAxis_137"].signals["AngularVelocityXAxis_137"][-1] - sensor_manager["IMU_XAxis_128"].signals["AngularVelocityXAxis_128"][-1]

        joint2_pos = 360 - joint1_pos -(90 - sensor_manager["IMU_XAxis_138"].signals["AngleXAxis_138"][-1])
        joint2_vel = sensor_manager["IMU_XAxis_138"].signals["AngularVelocityXAxis_138"][-1] - joint1_vel

        joint3_pos = sensor_manager["Rot_01"].position[-1]
        joint3_vel = sensor_manager["Rot_01"].position[-1]

        IMU_joints = [[joint1_pos, joint1_vel],[joint2_pos, joint2_vel],[joint3_pos, joint3_vel]]
    else:
        IMU_joints = None
    return IMU_joints

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