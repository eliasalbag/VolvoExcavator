import matplotlib.pyplot as plt
from sensors import *

def plot(sensor_manager):
    imu = sensor_manager["IMU_XAxis_128"]
    enc = sensor_manager["Rot_01"]

    plt.figure(figsize=(12, 6))
    imu.position = imu.signals[f"AngleX{imu.name[5:]}"]
    imu.vel = imu.signals[f"AngularVelocityX{imu.name[5:]}"]

    # Position
    plt.subplot(2, 1, 1)
    plt.plot(imu.timestamps, imu.position, label="IMU Position")
    plt.plot(enc.timestamps, enc.position, label="Encoder Position")
    plt.title("Bom - Position")
    plt.xlabel("Tid [s]")
    plt.ylabel("Vinkel [rad]")
    plt.legend()
    plt.grid(True)

    # Velocity
    plt.subplot(2, 1, 2)
    plt.plot(imu.timestamps, imu.vel, label="IMU Velocity")
    plt.plot(enc.timestamps, enc.vel, label="Encoder Velocity")
    plt.title("Bom - Hastighet")
    plt.xlabel("Tid [s]")
    plt.ylabel("Hastighet [rad/s]")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()