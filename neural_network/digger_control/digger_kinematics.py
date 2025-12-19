import numpy as np


def calculate_jacobian(theta, l1, l2, l3, data_type=np.float32):
    """
    Calculate the jacobian for the three links (boom, stick and bucket)
    :param theta: array [theta1, theta2, theta3]
    :param l1: Boom length [m]
    :param l2: Stick length [m]
    :param l3: Bucket length [m]
    :param data_type: Set to either np.float32 or np.float64
    :return: array - 3x3 matrix
    """

    # Precompute trig values
    theta1, theta2, theta3 = theta
    s1 = np.sin(theta1)
    s12 = np.sin(theta1 + theta2)
    s123 = np.sin(theta1 + theta2 + theta3)
    c1 = np.cos(theta1)
    c12 = np.cos(theta1 + theta2)
    c123 = np.cos(theta1 + theta2 + theta3)

    # The jacobian
    J = np.array([
        [-l1 * s1 - l2 * s12 - l3 * s123, -l2 * s12 - l3 * s123, -l3 * s123],
        [l1 * c1 + l2 * c12 + l3 * c123, l2 * c12 + l3 * c123, l3 * c123],
        [1.0, 1.0, 1.0]
    ], dtype=data_type)

    return J


def inverse_differential_kinematics(theta, v_ref, l1, l2, l3, data_type=np.float32):
    """
    Calculate angular velocities given desired bucket velocity using pytorch
    :param theta: tensor [theta1, theta2, theta3]
    :param v_ref: tensor [x_dot_ref, y_dot_ref, bucket_theta_dot_ref]
    :param l1: Boom length [m]
    :param l2: Stick length [m]
    :param l3: Bucket length [m]
    :param data_type: Set to either torch.float32 or torch.float64
    :return: tensor [dtheta1, dtheta2, dtheta3]
    """

    # Calculate jacobian
    J = calculate_jacobian(theta, l1, l2, l3, data_type=data_type)

    # Pseudo-invers
    J_pinv = np.linalg.pinv(J)

    # Calculate desired angular velocities: dtheta = J_pseudoinvers * v
    dtheta_des = np.matmul(J_pinv, v_ref)

    return dtheta_des


def forward_kinematics(theta, l1, l2, l3):
    """
    Calculate bucket position and angle
    :param theta: array [theta1, theta2, theta3]
    :param l1: Boom length [m]
    :param l2: Stick length [m]
    :param l3: Bucket length [m]
    :return: array [x, y, bucket_theta]
    """

    theta1, theta2, theta3 = theta

    p_x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
    p_y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
    p_theta = theta1 + theta2 + theta3

    return np.array([p_x, p_y, p_theta])


def forward_differential_kinematics(theta, dtheta, l1, l2, l3, data_type=np.float32):
    """
    Calculate bucket velocity (horizontal, vertical and angular)
    :param theta: tensor [theta1, theta2, theta3]
    :param dtheta: tensor [dtheta1, dtheta2, dtheta3]
    :param l1: Boom length [m]
    :param l2: Stick length [m]
    :param l3: Bucket length [m]
    :param data_type: Set to either torch.float32 or torch.float64
    :return: tensor [dx, dy, bucket_dtheta]
    """

    # Calculate jacobian
    J = calculate_jacobian(theta, l1, l2, l3, data_type=data_type)

    # Calculate bucket speed: v = J * dtheta
    v = np.matmul(J, dtheta)

    return v


def drift_compensation(p, e_i, p_ref, v_ref, dt=None, k=None, k_i=None):
    """"
    Generate the positional error and multiplying it with a constant k
    :param p: array (Current bucket pose)
    :param p_ref: array (The pose bucket should have, given desired velocity)
    :param v_ref: array (Desired bucket velocity)
    :param dt: Time step between measurement/calculations
    :param k: Constant for driftcompensation
    :return: pose_error, p_ref
    """

    # Calculate the new reference position
    p_ref += v_ref * dt

    # Calculate the orthogonal position error (with respect to the velocity vector)
    e = (p_ref - p)
    if np.any(v_ref[:2] != 0):
        e_para = (np.dot(e[:2], v_ref[:2]) / np.dot(v_ref[:2], v_ref[:2])) * v_ref[:2]
        e_orth = e[:2] - e_para
        # e_orth = np.clip(e_orth, a_min=-0.005, a_max=0.005)
        e[:2] = e_orth

    # e_i
    e_i += e * dt

    # Calculate drift compensation factor (e * K)
    v_com = e * k + e_i * k_i

    # Calculate desired velocity (v_ref + e * K)
    v_des = v_ref + v_com

    return v_des, p_ref, e_i
