import matplotlib.pyplot as plt

def plot_results(pulley_angles, joint_angles, tendon_tensions, motor_torque, errors,save_path=None):
    # Plot joint angles
    fig, ax = plt.subplots()
    for j in range(joint_angles.shape[1]):
        ax.plot(pulley_angles, joint_angles[:, j], label=f'Joint {j} Angle')
    ax.set_xlabel('Pulley Angle [rad]')
    ax.set_ylabel('Joint Angles [rad]')
    ax.legend()
    ax.set_title('Joint Angles vs Pulley Angle')
    ax.grid(True)
    if save_path:
        fig.savefig(f"{save_path}/joint_angles.png")
    plt.show()
    plt.close(fig)

    # Plot tendon tensions
    fig, ax = plt.subplots()
    for t in range(tendon_tensions.shape[1]):
        ax.plot(pulley_angles, tendon_tensions[:, t], label=f'Tendon {t} Tension')
    ax.set_xlabel('Pulley Angle [rad]')
    ax.set_ylabel('Tendon Tensions [N]')
    ax.legend()
    ax.set_title('Tendon Tensions vs Pulley Angle')
    ax.grid(True)
    if save_path:
        fig.savefig(f"{save_path}/tendon_tensions.png")
    plt.show()
    plt.close(fig)

    # Plot motor torque
    fig, ax = plt.subplots()
    ax.plot(pulley_angles, motor_torque, label='Motor Torque')
    ax.set_xlabel('Pulley Angle [rad]')
    ax.set_ylabel('Motor Torque [Nm]')
    ax.legend()
    ax.set_title('Motor Torque vs Pulley Angle')
    ax.grid(True)
    if save_path:
        fig.savefig(f"{save_path}/motor_torque.png")
    plt.show()
    plt.close(fig)

    # Plot errors
    fig, ax = plt.subplots()
    ax.plot(pulley_angles, errors, label='Error')
    ax.set_xlabel('Pulley Angle [rad]')
    ax.set_ylabel('Error [-]')
    ax.legend()
    ax.set_title('Error vs Pulley Angle')
    ax.grid(True)
    if save_path:
        fig.savefig(f"{save_path}/error.png")
    plt.show()
    plt.close(fig)