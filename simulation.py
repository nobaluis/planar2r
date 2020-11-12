import matplotlib.pyplot as plt
import numpy as np

from planar_2r import Planar2R

if __name__ == '__main__':
    L = np.array([1.0, 0.5])  # links length
    m = np.array([0.5, 0.3])  # links mass

    theta = np.array([np.pi / 2.0, 0.0])  # initial position
    dot_theta = np.array([0.0, 0.0])  # initial vel
    tau = np.array([0.0, 0.0])  # control signal

    robot = Planar2R(L, m)  # planar 2R robot
    t = np.linspace(0, 10, 100)  # time steps
    dt = t[1] - t[0]  # step size

    state = np.empty((len(t), 2, len(theta)))

    # simulation with Euler integration
    for i, _t in enumerate(t):
        # save the last simulation state
        state[i][0, :] = theta
        state[i][1, :] = dot_theta
        # simulation step
        dot2_theta = robot.forward_dynamics(theta, dot_theta, tau)
        theta = theta + dot_theta * dt
        dot_theta = dot_theta + dot2_theta * dt

    # plot sim results (position)
    fig, ax = plt.subplots(2, 1)
    ax[0].plot(t, state[:, 0, 0])  # theta_1(t)
    ax[1].plot(t, state[:, 0, 1])  # theta_2(t)
    ax[0].set_ylabel(r'$\theta_1$')
    ax[1].set_ylabel(r'$\theta_2$')
    ax[1].set_xlabel(r'$t$')
    fig.suptitle('Position')
    fig.show()

    # plot sim results (velocity)
    fig, ax = plt.subplots(2, 1)
    ax[0].plot(t, state[:, 1, 0])  # dot_theta_1(t)
    ax[1].plot(t, state[:, 1, 1])  # dot_theta_2(t)
    ax[0].set_ylabel(r'$\dot{\theta}_1$')
    ax[1].set_ylabel(r'$\dot{\theta}_2$')
    ax[1].set_xlabel(r'$t$')
    fig.suptitle('Velocity')
    fig.show()
