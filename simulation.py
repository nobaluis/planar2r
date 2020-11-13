import matplotlib.pyplot as plt
import numpy as np

from planar_2r import Planar2R


def path_planning(u, w=1.0, h=1.0, asymmetric=True):
    """Computes the parametric lemniscate of Bernoulli"""
    x = (w * np.cos(u)) / (1.0 + np.sin(u) ** 2)
    y = (h * np.sin(u) * np.cos(u)) / (1.0 + np.sin(u) ** 2)
    if asymmetric and x < 0.0:
        x *= 0.5
    return x, y


def plot_2d(x, fx_1, fx_2, x_label, fx1_label, fx2_label):
    _fig, _ax = plt.subplots(2, 1)
    _ax[0].plot(x, fx_1)
    _ax[1].plot(x, fx_2)
    _ax[0].set_ylabel(r'${}$'.format(fx1_label))
    _ax[1].set_ylabel(r'${}$'.format(fx2_label))
    _ax[1].set_xlabel(r'${}$'.format(x_label))
    return _fig


if __name__ == '__main__':
    L = np.array([0.5, 0.5])  # links length
    m = np.array([0.5, 0.25])  # links mass

    theta = np.array([0.0, 0.0])  # initial position
    dot_theta = np.array([0.0, 0.0])  # initial vel

    robot = Planar2R(L, m, g=0.0)  # planar 2R robot
    t = np.linspace(0, 100, 1000)  # time steps
    dt = t[1] - t[0]  # step size

    k_p = 0.2 * np.eye(2)  # proportional
    k_d = 0.3 * np.eye(2)  # derivative
    theta_e = np.array([0.0, 0.0])  # pos error
    dot_theta_e = np.array([0.0, 0.0])  # vel error

    state = np.empty((len(t), 2, len(theta)))  # state values
    errors = np.empty((len(t), 2, len(theta)))  # error values
    path = np.empty((len(t), 2, 2))

    # simulation with Euler integration
    for i, _t in enumerate(t):
        # save the last simulation state
        state[i][0, :] = theta
        state[i][1, :] = dot_theta

        # desired position
        x_d, y_d = path_planning(i / t[-1], asymmetric=True)
        theta_d = robot.inverse_kinematics(x_d, y_d)

        # pd controller
        # note 1: here array.[:, None] adds one dimension (2,) -> (2, 1)
        # note 2: the array.squeeze() removes the extra dimension (2, 1) -> (2,)
        tau_c = (k_p @ theta_e[:, None] + k_d @ dot_theta_e[:, None]).squeeze()

        # simulation step
        dot2_theta = robot.forward_dynamics(theta, dot_theta, tau_c)
        theta = theta + dot_theta * dt
        dot_theta = dot_theta + dot2_theta * dt

        # compute error
        theta_e = theta_d - theta
        dot_theta_e = -dot_theta

        # save paths
        path[i][0, :] = np.array([x_d, y_d])
        path[i][1, :] = robot.forward_kinematics(theta)

        # save errors
        errors[i][0, :] = theta_e
        errors[i][0, :] = dot_theta_e

    # plot sim results (position)
    plt.style.use('bmh')
    fig_1 = plot_2d(
        t, state[:, 0, 0], state[:, 0, 1],
        't', '\\theta_1', '\\theta_2')
    fig_1.suptitle('Position')
    fig_1.show()

    # plot sim results (velocity)
    fig2 = plot_2d(
        t, state[:, 1, 0], state[:, 1, 1],
        't', '\\dot{\\theta}_1', '\\dot{\\theta}_2')
    fig2.suptitle('Velocity')
    fig2.show()

    # plot sim results (errors)
    fig3 = plot_2d(
        t, errors[:, 0, 0], errors[:, 0, 1],
        't', '\\theta_{e1}', '\\theta_{e2}')
    fig3.suptitle('Position Error')
    fig3.show()

    # plot path
    fig4, ax4 = plt.subplots()
    ax4.plot(path[:, 0, 0], path[:, 0, 1], 'C0', label='desired pos')
    ax4.plot(path[:, 1, 0], path[:, 1, 1], '*C4', label='real pos')
    ax4.axis('equal')
    ax4.legend()
    ax4.set_title('Path planning vs. real path')
    fig4.show()
