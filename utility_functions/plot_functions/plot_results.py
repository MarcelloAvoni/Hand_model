import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
from .plot_finger import plot_finger
import numpy as np

def plot_kinematics_results(finger, pulley_angles, joint_angles, tendon_tensions, motor_torque, errors, save_path=None):
    # Set global plot parameters
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['figure.autolayout'] = True  # Automatically adjust subplot parameters to give specified padding
    plt.rcParams['axes.grid'] = True  # Enable grid for all plots
    plt.rcParams['grid.alpha'] = 0.75  # Set grid transparency
    plt.rcParams['grid.linestyle'] = '-'  # Set grid line style

    font_size = 18  # Define the font size for labels

    # Create a ScalarFormatter for scientific notation
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((-1, 1))

    # Plot joint angles
    fig, ax = plt.subplots()
    for j in range(joint_angles.shape[1]):
        ax.plot(pulley_angles, joint_angles[:, j], label=f'Joint {j}')
    ax.set_xlabel('Pulley Angle [rad]', fontsize=font_size)
    ax.set_ylabel('Joint Angles [rad]', fontsize=font_size)
    ax.legend()
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/joint_angles_{finger.name}.pdf")
    plt.close(fig)

    # Plot tendon tensions
    fig, ax = plt.subplots()
    for t in range(tendon_tensions.shape[1]):
        ax.plot(pulley_angles, tendon_tensions[:, t], label=f'Tendon {t}')
    ax.set_xlabel('Pulley Angle [rad]', fontsize=font_size)
    ax.set_ylabel('Tendon Tensions [N]', fontsize=font_size)
    ax.legend()
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/tendon_tensions_{finger.name}.pdf")
    plt.close(fig)

    # Plot motor torque
    fig, ax = plt.subplots()
    ax.plot(pulley_angles, motor_torque, label='Motor Torque')
    ax.set_xlabel('Pulley Angle [rad]', fontsize=font_size)
    ax.set_ylabel('Motor Torque [Nm]', fontsize=font_size)
    ax.legend()
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/motor_torque_{finger.name}.pdf")
    plt.close(fig)

    # Plot errors
    fig, ax = plt.subplots()
    ax.plot(pulley_angles, errors, label='Error')
    ax.set_xlabel('Pulley Angle [rad]', fontsize=font_size)
    ax.set_ylabel('Error [-]', fontsize=font_size)
    ax.legend()
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/error_{finger.name}.pdf")
    plt.close(fig)

    # plot finger kinematics
    # first we extract a fraction of the positions estimated
    beta = 0.15
    num_plots = 5
    indices = np.linspace(0, len(pulley_angles) - 1, num_plots, dtype=int)
    joint_angles_plot = joint_angles[indices, :]

    # now we plot the finger kinematics
    fig, ax = plt.subplots()
    ax.grid(True)  # Enable grid for this plot
    for i in range(num_plots):
        alpha = beta + (1 - beta) * i / (num_plots - 1)
        plot_finger(ax, finger, joint_angles_plot[i, :], alpha)
    ax.set_xlabel('x [m]', fontsize=font_size)
    ax.set_ylabel('y [m]', fontsize=font_size)
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/finger_kinematics_{finger.name}.pdf")
    plt.close(fig)


def plot_statics_results(finger, force, joint_angles, tendon_tensions, motor_torque, errors, save_path=None):
    # Set global plot parameters
    plt.rcParams['font.family'] = 'Times New Roman'
    plt.rcParams['figure.autolayout'] = True  # Automatically adjust subplot parameters to give specified padding
    plt.rcParams['axes.grid'] = True  # Enable grid for all plots
    plt.rcParams['grid.alpha'] = 0.75  # Set grid transparency
    plt.rcParams['grid.linestyle'] = '-'  # Set grid line style

    font_size = 18  # Define the font size for labels

    # Create a ScalarFormatter for scientific notation
    formatter = ScalarFormatter(useMathText=True)
    formatter.set_scientific(True)
    formatter.set_powerlimits((-1, 1))

    # Plot joint angles
    fig, ax = plt.subplots()
    for j in range(joint_angles.shape[1]):
        ax.plot(force, joint_angles[:, j], label=f'Joint {j}')
    ax.set_xlabel('Force [N]', fontsize=font_size)
    ax.set_ylabel('Joint Angles [rad]', fontsize=font_size)
    ax.legend()
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/joint_angles_{finger.name}.pdf")
    plt.close(fig)

    # Plot tendon tensions
    fig, ax = plt.subplots()
    for t in range(tendon_tensions.shape[1]):
        ax.plot(force, tendon_tensions[:, t], label=f'Tendon {t}')
    ax.set_xlabel('Force [N]', fontsize=font_size)
    ax.set_ylabel('Tendon Tensions [N]', fontsize=font_size)
    ax.legend()
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/tendon_tensions_{finger.name}.pdf")
    plt.close(fig)

    # Plot motor torque
    fig, ax = plt.subplots()
    ax.plot(force, motor_torque, label='Motor Torque')
    ax.set_xlabel('Force [N]', fontsize=font_size)
    ax.set_ylabel('Motor Torque [Nm]', fontsize=font_size)
    ax.legend()
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/motor_torque_{finger.name}.pdf")
    plt.close(fig)

    # Plot errors
    fig, ax = plt.subplots()
    ax.plot(force, errors, label='Error')
    ax.set_xlabel('Force [N]', fontsize=font_size)
    ax.set_ylabel('Error [-]', fontsize=font_size)
    ax.legend()
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/error_{finger.name}.pdf")
    plt.close(fig)

    # plot finger kinematics
    # first we extract a fraction of the positions estimated
    beta = 0.15
    num_plots = 5
    indices = np.linspace(0, 1, num_plots)**2  # Squaring to bias towards 0
    indices = (indices * (len(force) - 1)).astype(int)
    joint_angles_plot = joint_angles[indices, :]

    # now we plot the finger kinematics
    fig, ax = plt.subplots()
    ax.grid(True)  # Enable grid for this plot
    for i in range(num_plots):
        alpha = beta + (1 - beta) * i / (num_plots - 1)
        plot_finger(ax, finger, joint_angles_plot[i, :], alpha)
    ax.set_xlabel('x [m]', fontsize=font_size)
    ax.set_ylabel('y [m]', fontsize=font_size)

    l_max = 0
    for i_iter in range(len(finger.L_phalanxes)):
        l_max += finger.L_phalanxes[i_iter]
    ax.set_xlim(-2*finger.r_joints[0],l_max)
    ax.set_ylim(min(-l_max,-finger.L_metacarpal-finger.r_joints[0]),0.5*l_max)
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    if save_path:
        fig.savefig(f"{save_path}/finger_kinematics_{finger.name}.pdf")
    plt.close(fig)