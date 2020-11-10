import numpy as np

# constants
g = 9.81  # gravity

# physical parameters
L = np.array([1.0, 0.8])  # link longitude
m = np.array([0.5, 0.3])  # link mass


def forward_kinematics(theta):
    """Compute the forward kinematics"""
    x = L[0] * np.cos(theta[0]) + L[1] * np.cos(np.sum(theta))
    y = L[0] * np.sin(theta[0]) + L[1] * np.sin(np.sum(theta))
    return np.array([x, y])


def inverse_kinematics(x, y):
    """Compute the inverse kinematics (from p. 218 MR)"""
    beta = np.arccos(
        (L[0] ** 2 + L[1] ** 2 - x ** 2 - y ** 2) / (2 * L[0] * L[1]))
    alpha = np.arccos(
        (x ** 2 + y ** 2 + L[0] ** 2 - L[1] ** 2) / (2 * L[0] * np.sqrt(x ** 2 + y ** 2)))
    gamma = np.arctan2(y, x)
    theta = np.array([gamma - alpha, np.pi - beta])  # rightly sol.
    # theta = np.array([gamma + alpha, beta - np.pi])  # lefty sol.
    return theta


def tip_velocity(theta, dot_theta):
    """Computes the end-effector velocity"""
    sin_12 = np.sin(np.sum(theta))
    cos_12 = np.cos(np.sum(theta))
    dot_xy = np.array([
        [-L[0] * np.sin(theta[0]) - L[1] * sin_12, - L[1] * sin_12],
        [-L[0] * np.cos(theta[0]) - L[1] * cos_12, - L[1] * cos_12],
    ]) @ dot_theta.reshape((2, 1))
    return dot_xy


def mass_matrix(theta):
    """Computes the symmetric positive-definite mass matrix"""
    m_11 = m[0] * L[0] ** 2 + m[1] * (L[0] ** 2 + 2 * L[0] * L[1] * np.cos(theta[1]) * L[1] ** 2)
    m_12 = m[1] * (L[0] * L[1] * np.cos(theta[1]) + L[1] ** 2)
    m_22 = m[1] * L[1] ** 2
    return np.array([[m_11, m_12], [m_12, m_22]])


def coriolis(theta, dot_theta):
    """Computes the coriolis vector"""
    return np.array([
        -m[1] * L[0] * L[1] * np.sin(theta[1]) * (2 * dot_theta[0] * dot_theta[1] + dot_theta[1] ** 2),
        m[1] * L[0] * L[1] * dot_theta[0] ** 2 * np.sin(theta[1])
    ])


def gravity(theta):
    """Computes the gravity vector"""
    return np.array([
        np.sum(m) * L[0] * g * np.cos(theta[0]) + m[1] * g * L[1] * np.cos(np.sum(theta)),
        m[1] * g * L[1] * np.cos(np.sum(theta))
    ])

# def joint_forces(theta, dot_theta, dot2_theta):
#     """Computes the joint forces (inverse dynamics) of 2R robot"""
#     tau_1 = (m[0] * L[0] ** 2 + m[1] * (L[0] ** 2 + 2 * L[0] * L[1] * np.cos(theta[1]) + L[1] ** 2)) * dot2_theta[0]
#     tau_1 += m[1] * (L[0] * L[1] * np.cos(theta[1]) + L[1] ** 2) * dot2_theta[1]
#     tau_1 -= m[1] * L[0] * L[1] * np.sin(theta[1]) * (2 * dot_theta[0] * dot_theta[1] + dot_theta[1] ** 2)
#     tau_1 += np.sum(m) * L[0] * g * np.cos(theta[0]) + m[1] * g * L[1] * np.cos(np.sum(theta))
#     tau_2 = m[1] * (L[0] * L[1] * np.cos(theta[1]) + L[1] ** 2) * dot2_theta[0]
#     tau_2 += m[1] * L[1] ** 2 * dot2_theta[1]
#     tau_2 += m[1] * L[0] * L[1] * dot_theta[0] ** 2 * np.sin(theta[1])
#     tau_2 += m[1] * g * L[1] * np.cos(np.sum(theta))
#     return np.array([tau_1, tau_2])


def inverse_dynamics(theta, dot_theta, dot2_theta):
    """Computes the vector of joint forces and torques given the state and desired acceleration"""
    M = mass_matrix(theta)
    h = coriolis(theta, dot_theta) + gravity(theta)
    tau = M @ dot2_theta.reshape((2, 1)) + h
    return tau


def forward_dynamics(theta, dot_theta, tau):
    """Computes the robot's acceleration given the state and joint forces and torques"""
    M = mass_matrix(theta)
    h = coriolis(theta, dot_theta) + gravity(theta)
    dot2_theta = np.linalg.inv(M) @ (tau - h).reshape((2, 1))
    return dot2_theta
