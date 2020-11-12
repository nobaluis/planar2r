import numpy as np


class Planar2R(object):

    def __init__(self, L: np.ndarray, m: np.ndarray, g=9.81):
        self.L = L
        self.m = m
        self.g = g

    def forward_kinematics(self, theta):
        """Compute the forward kinematics"""
        x = self.L[0] * np.cos(theta[0]) + self.L[1] * np.cos(np.sum(theta))
        y = self.L[0] * np.sin(theta[0]) + self.L[1] * np.sin(np.sum(theta))
        return np.array([x, y])

    def inverse_kinematics(self, x, y):
        """Compute the inverse kinematics (from p. 218 MR)"""
        beta = np.arccos(
            (self.L[0] ** 2 + self.L[1] ** 2 - x ** 2 - y ** 2) / (2 * self.L[0] * self.L[1]))
        alpha = np.arccos(
            (x ** 2 + y ** 2 + self.L[0] ** 2 - self.L[1] ** 2) / (2 * self.L[0] * np.sqrt(x ** 2 + y ** 2)))
        gamma = np.arctan2(y, x)
        theta = np.array([gamma - alpha, np.pi - beta])  # rightly sol.
        # theta = np.array([gamma + alpha, beta - np.pi])  # lefty sol.
        return theta

    def tip_velocity(self, theta, dot_theta):
        """Computes the end-effector velocity"""
        sin_12 = np.sin(np.sum(theta))
        cos_12 = np.cos(np.sum(theta))
        dot_xy = np.array([
            [-self.L[0] * np.sin(theta[0]) - self.L[1] * sin_12, - self.L[1] * sin_12],
            [-self.L[0] * np.cos(theta[0]) - self.L[1] * cos_12, - self.L[1] * cos_12],
        ]) @ dot_theta.reshape((2, 1))
        return dot_xy

    def mass_matrix(self, theta):
        """Computes the symmetric positive-definite mass matrix"""
        m_11 = self.m[0] * self.L[0] ** 2 + self.m[1] * (
                self.L[0] ** 2 + 2 * self.L[0] * self.L[1] * np.cos(theta[1]) + self.L[1] ** 2)
        m_12 = self.m[1] * (self.L[0] * self.L[1] * np.cos(theta[1]) + self.L[1] ** 2)
        m_22 = self.m[1] * self.L[1] ** 2
        return np.array([[m_11, m_12], [m_12, m_22]])

    def coriolis(self, theta, dot_theta):
        """Computes the coriolis vector"""
        return np.array([
            -self.m[1] * self.L[0] * self.L[1] * np.sin(theta[1]) * (
                    2 * dot_theta[0] * dot_theta[1] + dot_theta[1] ** 2),
            self.m[1] * self.L[0] * self.L[1] * dot_theta[0] ** 2 * np.sin(theta[1])
        ])

    def gravity(self, theta):
        """Computes the gravity vector"""
        return np.array([
            np.sum(self.m) * self.L[0] * self.g * np.cos(theta[0]) + self.m[1] * self.g * self.L[1] * np.cos(
                np.sum(theta)),
            self.m[1] * self.g * self.L[1] * np.cos(np.sum(theta))
        ])

    def inverse_dynamics(self, theta, dot_theta, dot2_theta):
        """Computes the vector of joint forces and torques given the state and desired acceleration"""
        M = self.mass_matrix(theta)
        h = self.coriolis(theta, dot_theta) + self.gravity(theta)
        tau = M @ dot2_theta.reshape((2, 1)) + h
        return tau

    def forward_dynamics(self, theta, dot_theta, tau):
        """Computes the robot's acceleration given the state and joint forces and torques"""
        M = self.mass_matrix(theta)
        h = self.coriolis(theta, dot_theta) + self.gravity(theta)
        dot2_theta = np.linalg.inv(M) @ (tau - h).reshape((2, 1))
        return dot2_theta.squeeze()
