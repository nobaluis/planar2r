import numpy as np


def parametric(u, a1=1.0, a2=1.0):
    """Computes the parametric lemniscate of Bernoulli"""
    x = (a1 * np.cos(u)) / (1.0 + np.sin(u) ** 2)
    y = (a2 * np.sin(u) * np.cos(u)) / (1.0 + np.sin(u)**2)
    return np.array([x, y])