import torch
import torch.nn as nn


class FeedForwardNet(nn.Module):
    def __init__(self, input_size=None, mlp_hidden_size=None,
                 net_output_size=None):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(input_size, mlp_hidden_size),
            nn.ReLU(),
            nn.Linear(mlp_hidden_size, mlp_hidden_size),
            nn.ReLU(),
            nn.Linear(mlp_hidden_size, net_output_size),
        )

    def forward(self, dtheta_desired):

        dtheta1 = dtheta_desired[:, 0]
        dtheta2 = dtheta_desired[:, 1]
        dtheta3 = dtheta_desired[:, 2]

        mask_dtheta1 = dtheta1 > 0
        mask_dtheta2 = dtheta2 > 0
        mask_dtheta3 = dtheta3 > 0

        dtheta1 = torch.stack([dtheta1 * mask_dtheta1, dtheta1 * (0 == mask_dtheta1)], dim=-1)
        dtheta2 = torch.stack([dtheta2 * mask_dtheta2, dtheta2 * (0 == mask_dtheta2)], dim=-1)
        dtheta3 = torch.stack([dtheta3 * mask_dtheta3, dtheta3 * (0 == mask_dtheta3)], dim=-1)

        x = torch.cat([dtheta1, dtheta2, dtheta3], dim=-1)

        return self.mlp(x)


class FeedForwardNetTheta(nn.Module):
    def __init__(self, input_size=None, mlp_hidden_size=None,
                 net_output_size=None):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(input_size, mlp_hidden_size),
            nn.ReLU(),
            nn.Linear(mlp_hidden_size, mlp_hidden_size),
            nn.ReLU(),
            nn.Linear(mlp_hidden_size, net_output_size),
        )

    def forward(self, dtheta_desired, theta):

        dtheta1 = dtheta_desired[:, 0]
        dtheta2 = dtheta_desired[:, 1]
        dtheta3 = dtheta_desired[:, 2]

        mask_dtheta1 = dtheta1 > 0
        mask_dtheta2 = dtheta2 > 0
        mask_dtheta3 = dtheta3 > 0

        dtheta1 = torch.stack([dtheta1 * mask_dtheta1, dtheta1 * (0 == mask_dtheta1)], dim=-1)
        dtheta2 = torch.stack([dtheta2 * mask_dtheta2, dtheta2 * (0 == mask_dtheta2)], dim=-1)
        dtheta3 = torch.stack([dtheta3 * mask_dtheta3, dtheta3 * (0 == mask_dtheta3)], dim=-1)

        x = torch.cat([dtheta1, dtheta2, dtheta3, theta], dim=-1)

        return self.mlp(x)
