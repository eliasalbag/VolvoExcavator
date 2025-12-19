import torch
import numpy as np
from network_classes import FeedForwardNet
from digger_kinematics import drift_compensation, inverse_differential_kinematics, forward_kinematics, forward_differential_kinematics
import time
from logitech_control import get_controller_values
import cantools
from canlib import canlib
from can_comunication import get_sensor_data, send_control_signal
import pandas as pd

# -----------------------------
# Config
# -----------------------------
# Digger parameters (length of arms)
L1 = 1.903392                                                   # Boom length
L2 = 1.351485                                                   # Stick length
L3 = 0.590267                                                   # Bucket length

# Data parameters
DTHETA_STD = np.array([0.02271073, 0.04661858, 0.0598276])      # Data for normalizing dtheta for network input
DT = 0.01                                                       # Time step between control signals calculations


# Controller paramaters
# Kanske lägga till fel i hastighet också
K_DRIFT = 3                                                         # Constant for driftcompensation: v_des = v_red + (p - p_ref) * K_DRIFT
                                                              # Higher values will result in more agressive/quicker drift compensation
K_I = 1.5
K_DRIFT = 0
K_I = 0
V_SCALE = 0.07 / 100
V_ROTATION_SCALE = 0.04 / 100

# Parameters for sensor data processing
DATA_FREQUENCY = 100
STEPS_FOR_VELOCITY_CALC = 20

SAVE_DATA = True


def get_network():
    # Import network data
    net_data = torch.load('trained_feedforward_net.pth', weights_only=False)

    # Create controller network
    net_feedforward = FeedForwardNet(input_size=6, mlp_hidden_size=net_data["MLP_HIDDEN_SIZE"], net_output_size=3)

    # Load the network with correct weights
    net_feedforward.load_state_dict(net_data["state_dict"])
    net_feedforward.eval()

    # The network parameters don't require gradients
    for param in net_feedforward.parameters():
        param.requires_grad = False

    return net_feedforward


def export_data_to_csv(data):
    df = pd.DataFrame(data)
    df.to_csv("excavator_data.csv", index=False)


def main():

    # Load feedforward network
    net_feedforward = get_network()

    # Lood DBC
    db_sensors = cantools.database.load_file("ec20_sensors.dbc")
    db_hydraulics = cantools.database.load_file("ec20_hydraulics.dbc")

    # Open both Kvaser-channels
    ch_sensors = canlib.openChannel(0, canlib.Open.EXCLUSIVE)
    ch_sensors.setBusOutputControl(canlib.Driver.NORMAL)
    ch_sensors.setBusParams(canlib.canBITRATE_500K)
    ch_sensors.busOn()
    ch_hydraulics = canlib.openChannel(1, canlib.Open.EXCLUSIVE)
    ch_hydraulics.setBusOutputControl(canlib.Driver.NORMAL)
    ch_hydraulics.setBusParams(canlib.canBITRATE_500K)
    ch_hydraulics.busOn()

    # Used for storing/calculating theta and dtheta
    cycle_time = time.time()
    # theta_time = {"theta1": time.time(), "theta2": time.time(), "theta3": time.time()}
    values = {"theta1": 0, "theta2": 0, "theta3": 0, "dtheta1": 0, "dtheta2": 0, "dtheta3": 0}
    data_history = {"theta1": [None for _ in range(STEPS_FOR_VELOCITY_CALC)],
                    "theta2": [None for _ in range(STEPS_FOR_VELOCITY_CALC)],
                    "theta3": [None for _ in range(STEPS_FOR_VELOCITY_CALC)]}
    theta_time = {"theta1": [None for _ in range(STEPS_FOR_VELOCITY_CALC)],
                  "theta2": [None for _ in range(STEPS_FOR_VELOCITY_CALC)],
                  "theta3": [None for _ in range(STEPS_FOR_VELOCITY_CALC)]}
    time_delay = round(1 / DATA_FREQUENCY, ndigits=5)

    # For sending control signals to digger
    hydraulics_msg = db_hydraulics.get_message_by_name("ControlMessage")
    hydraulics_data_dict = {"UpDown": 0, "Parallelll_RL": 0, "Parallell_FR": 0, "Tilt_FR": 0,
                            "Tilt_RL": 0, "Rotate_RL": 0}

    p_ref = None
    e_i = None
    t_last = 0
    v_ref = np.zeros(3)
    dtheta = np.zeros(3)
    theta = np.zeros(3)
    if SAVE_DATA:
        data_to_export = {"x": [], "y": []}

    try:
        while True:
            # Read user input from game controller (values are within [-100, 100])
            input_v_y, input_v_x, input_rotation = get_controller_values()

            # Scale inputs to wanted velocities (v_ref)
            v_ref[:] = [input_v_x * V_SCALE, input_v_y * V_SCALE, input_rotation * V_ROTATION_SCALE]

            # Read sensor data
            values, data_history, theta_time, cycle_time = get_sensor_data(ch_sensors, values, data_history,
                                                                           theta_time, cycle_time, db_sensors, time_delay)

            # Extract values
            dtheta[:] = [values["dtheta1"], values["dtheta2"], values["dtheta3"]]
            theta[:] = [values["theta1"], values["theta2"], values["theta3"]]

            delta_t = time.time() - t_last
            if delta_t >= DT:
                t_last = time.time()

                # Calculate current bucket position
                p = forward_kinematics(theta, L1, L2, L3)

                # Reset p_ref (position used for drift compensation) if v_ref == 0
                if np.all(v_ref == 0) or p_ref is None:
                    p_ref = p
                    e_i = 0

                # Calculate desired velocity based on drift compensation
                v_des, p_ref, e_i = drift_compensation(p, e_i, p_ref, v_ref, dt=delta_t, k=K_DRIFT, k_i=K_I)

                # Calculate desired angular velocities
                dtheta_des = inverse_differential_kinematics(theta, v_des, L1, L2, L3)

                # Normailze dtheta_des for feedforward network and convert to tensor
                n_dtheta_des = dtheta_des / DTHETA_STD
                n_dtheta_des = torch.from_numpy(n_dtheta_des).float()

                # Feedforward network predicts control signal input (hydraulics input)
                with torch.no_grad():
                    u = net_feedforward(n_dtheta_des.unsqueeze(0))
                u = u.numpy()[0, :]

                # Scale control signal u (since network output is within [-1, 1])
                u = np.round(100 * u)

                # Make sure that u is within [-100, 100]
                u = np.clip(u, a_min=-100, a_max=100)

                # Makes sure that if v_ref == 0 than u == 0
                if np.all(v_ref == 0):
                    u = u * 0

                # Make sure that u[i] = 0 if dtheta[i] = 0
                u = u * (dtheta_des != 0)

                # Save data for
                if SAVE_DATA:
                    data_to_export["x"].append(p[0])
                    data_to_export["y"].append(p[1])

                # Send control signal to hydraulics
                hydraulics_data_dict["UpDown"] = int(u[0])
                hydraulics_data_dict["Parallell_FR"] = int(u[1])
                hydraulics_data_dict["Tilt_FR"] = int(u[2])
                send_control_signal(ch_hydraulics, hydraulics_msg, hydraulics_data_dict)

    except KeyboardInterrupt:
        if SAVE_DATA:
            export_data_to_csv(data_to_export)
        return

main()
