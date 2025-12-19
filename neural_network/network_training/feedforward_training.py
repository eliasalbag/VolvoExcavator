import torch
import torch.nn as nn
import numpy as np
from tqdm import tqdm
import pandas as pd
from torch.utils.data import DataLoader, TensorDataset
from torch.optim.lr_scheduler import CyclicLR
from network_classes import FeedForwardNet
from data_handler import data_extract, load_data_csv, normalize_data


# -----------------------------
# Config
# -----------------------------
# Network parameters
MLP_HIDDEN_SIZE = 32                # The dimension of the fully connected layers in the network -
                                    # larger equals more complex network
SEQUENCE_LENGTH = 50
USE_SYNTHETIC_DATA = False

# Training parameters
LEARNING_RATE_MAX = 0.01
LEARNING_RATE_MIN = 0.00001
NUMBER_EPOCHS = 2                   # Number of epochs - how many times the network train on the entire dataset
BATCH_SIZE = 16                     # Batch size - how many data points are used for each prediction
DATA_TYPE_NP = np.float32           # Sets data type of all values used by network (either np.float32 or np.float64)
DATA_TYPE_TORCH = torch.float32     # Set to same value as DATA_TYPE_NP but as either torch.float32 or torch.float64

# Use GPU if available
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def train_network(dataloader, validation_data=None):
    # Creating the network
    # net = HybridModelNet(hydraulics_input_size=3, hydraulics_hidden_size=HYDRAULICS_GRU_HIDDEN_SIZE,
    #                      mlp_input_size=3, mlp_hidden_size=MLP_HIDDEN_SIZE, net_output_size=3).to(DEVICE)

    net = FeedForwardNet(input_size=9, mlp_hidden_size=MLP_HIDDEN_SIZE,
                         net_output_size=3)

    # Choosing optimizer for training
    optimizer = torch.optim.AdamW(net.parameters())

    # Use cyclic learning rate
    scheduler = CyclicLR(optimizer, base_lr=LEARNING_RATE_MIN, max_lr=LEARNING_RATE_MAX,
                         step_size_up=int(len(dataloader) * NUMBER_EPOCHS / 6))

    # Choosing loss function for the network training
    loss_function = nn.L1Loss()

    # Training the network
    number_training_steps = int(len(dataloader.dataset) * NUMBER_EPOCHS / BATCH_SIZE)
    with tqdm(total=number_training_steps, desc="Training progress") as pbar:

        data_learning_rate = []
        for e in range(NUMBER_EPOCHS):
            for x_theta, x_dtheta, y_u in dataloader:
                x_theta, x_dtheta, y_u = x_theta.to(DEVICE), x_dtheta.to(DEVICE), y_u.to(DEVICE)

                # Forward pass (make prediction)
                pred = net(x_dtheta, x_theta)
                loss = loss_function(pred, y_u)

                # Backward pass (update/improve network)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                # One step for the cyclic learning rate
                scheduler.step()

                # Update progress bar in terminal
                pbar.update(1)

                data_learning_rate.append(scheduler.get_last_lr())

    print("Sista LR: ", scheduler.get_last_lr())

    # Check performance on validation data
    with torch.no_grad():
        net.eval()
        accuracy = 0
        num = 0
        for x_theta, x_dtheta, y_u in validation_data:
            accuracy += torch.sum(torch.abs(net(x_dtheta, x_theta) - y_u), 0)
            num += x_dtheta.size()[0]
        accuracy = accuracy / num
        print("Accuracy: ", accuracy)

    return net


def import_data():
    # ---------------------
    # 1) LOAD DATA
    # ---------------------
    if USE_SYNTHETIC_DATA:
        # Import csv file
        df = pd.read_csv("synthetic_data.csv")

        # Creates a numpy matrix with data
        dtheta = df[["dtheta1", "dtheta2", "dtheta3"]].values.astype(np.float32)
        u = df[["boom", "stick", "bucket"]].values.astype(np.float32)

        # ---------------------
        # 2) Split data into trainging set and validation set
        # ---------------------
        dtheta_train, u_train = dtheta[:-1000, :], u[:-1000, :]
        dtheta_val, u_val = dtheta[-1000:, :], u[-1000:, :]

        # ---------------------
        # 3) Convert data to tensor
        # ---------------------
        u_train = torch.tensor(np.array(u_train), dtype=DATA_TYPE_TORCH)
        dtheta_train = torch.tensor(np.array(dtheta_train), dtype=DATA_TYPE_TORCH)
        u_val = torch.tensor(np.array(u_val), dtype=DATA_TYPE_TORCH)
        dtheta_val = torch.tensor(np.array(dtheta_val), dtype=DATA_TYPE_TORCH)

        # Create datasets
        data_train = TensorDataset(dtheta_train, u_train)
        data_val = TensorDataset(dtheta_val, u_val)
    else:
        # Import data
        theta_train, dtheta_train, u_train = load_data_csv("training_data.csv", data_type=DATA_TYPE_NP)
        theta_val, dtheta_val, u_val = load_data_csv("validation_data.csv", data_type=DATA_TYPE_NP)

        # Normalize data
        theta_train, dtheta_train, u_train, norm_data = normalize_data(theta_train, dtheta_train, u_train,
                                                                       norm_data=None, norm_choice="specific")
        theta_val, dtheta_val, u_val, _ = normalize_data(theta_val, dtheta_val, u_val,
                                                         norm_data=norm_data, norm_choice="specific")

        # Extrac data points
        data_train = data_extract(theta_train, dtheta_train, u_train, sequence_length=SEQUENCE_LENGTH, data_type=DATA_TYPE_TORCH)
        data_val = data_extract(theta_val, dtheta_val, u_val, sequence_length=SEQUENCE_LENGTH, data_type=DATA_TYPE_TORCH)

    return data_train, data_val


def main():
    # Import data
    data_train, data_val = import_data()

    # Put data in DataLoader
    data_train = DataLoader(data_train, batch_size=BATCH_SIZE, shuffle=True)
    data_val = DataLoader(data_val, batch_size=BATCH_SIZE, shuffle=False)

    # Train network
    net = train_network(data_train, validation_data=data_val)

    # Save network
    # Saving it as a state_dict (only used for backup/testing)
    torch.save({
        "state_dict": net.state_dict(),
        "MLP_HIDDEN_SIZE": MLP_HIDDEN_SIZE
    }, "trained_feedforward_net.pth")


# Run the training
main()
