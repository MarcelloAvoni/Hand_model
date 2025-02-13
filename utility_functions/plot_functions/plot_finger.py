import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from utility_functions.kinematics_functions.kinematics_calculation import *


def plot_phalanx(ax, r_1, r_2, x_1, y_1, x_2, y_2, x_1_phalanx_up, y_1_phalanx_up, x_2_phalanx_up, y_2_phalanx_up, x_1_phalanx_down, y_1_phalanx_down, x_2_phalanx_down, y_2_phalanx_down):
    # we define 2 circumferences that represent the joints
    beta = np.linspace(0, 2 * np.pi, 100)
    x_circ_1 = x_1 + r_1 * np.sin(beta)
    y_circ_1 = y_1 + r_1 * np.cos(beta)

    x_circ_2 = x_2 + r_2 * np.sin(beta)
    y_circ_2 = y_2 + r_2 * np.cos(beta)

    # we overlay lines that represent current phalanx
    ax.plot([x_1, x_2], [y_1, y_2], color='black', linewidth=1, linestyle='--')
    ax.plot([x_1_phalanx_up, x_2_phalanx_up], [y_1_phalanx_up, y_2_phalanx_up], color='black', linewidth=2, linestyle='-')
    ax.plot([x_1_phalanx_down, x_2_phalanx_down], [y_1_phalanx_down, y_2_phalanx_down], color='black', linewidth=2, linestyle='-')

    # we overlay joints
    ax.plot(x_circ_1, y_circ_1, color='black', linewidth=2, linestyle='-')
    ax.plot(x_circ_2, y_circ_2, color='black', linewidth=2, linestyle='-')



def plot_finger(ax,finger,joint_angles):

    #we first extract the relevant variables
    r_joints = finger.r_joints
    r_tip = finger.r_tip
    L_phalanxes = finger.L_phalanxes

    #we calculate the number of joints
    n_joints = len(r_joints)

    # we set the aspect of the plot to be equal
    ax.set_aspect('equal')

    # we set the limits of the plot
    l_max = 0
    for i_iter in range(len(L_phalanxes)):
        l_max += L_phalanxes[i_iter]
        
    l_max = l_max + 2*r_tip
    ax.set_xlim(-r_joints[0],l_max)
    ax.set_ylim(-l_max,l_max)

    # we set the ticks to be invisible
    ax.set_xticks([])
    ax.set_yticks([])

    # we set the labels
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')

    # we set the title
    ax.set_title('Finger Configuration')

    (x_1, y_1, x_1_phalanx_up, y_1_phalanx_up, x_2_phalanx_up, y_2_phalanx_up, x_2, y_2, x_1_phalanx_down, y_1_phalanx_down, x_2_phalanx_down, y_2_phalanx_down) = finger_kinematics(finger,joint_angles)

    #now we plot the finger
    x_0 = 0
    y_0 = 0

    angles = np.linspace(0,2*np.pi,100)
    x_circ = x_0 + r_joints[0]*np.sin(angles)
    y_circ = y_0 + r_joints[0]*np.cos(angles)

    ax.plot(x_circ,y_circ,color='black',linewidth=2,linestyle='-')

    for i_iter in range(n_joints):
        if (i_iter == n_joints - 1):
            plot_phalanx(ax, r_joints[i_iter], r_tip, x_1[i_iter], y_1[i_iter], x_2[i_iter], y_2[i_iter], x_1_phalanx_up[i_iter], y_1_phalanx_up[i_iter], x_2_phalanx_up[i_iter], y_2_phalanx_up[i_iter], x_1_phalanx_down[i_iter], y_1_phalanx_down[i_iter], x_2_phalanx_down[i_iter], y_2_phalanx_down[i_iter])
        else:
            plot_phalanx(ax, r_joints[i_iter], r_joints[i_iter+1], x_1[i_iter], y_1[i_iter], x_2[i_iter], y_2[i_iter], x_1_phalanx_up[i_iter], y_1_phalanx_up[i_iter], x_2_phalanx_up[i_iter], y_2_phalanx_up[i_iter], x_1_phalanx_down[i_iter], y_1_phalanx_down[i_iter], x_2_phalanx_down[i_iter], y_2_phalanx_down[i_iter])



    


def plot_update(frame,ax,finger,joint_angles):

    ax.clear()
    plot_finger(ax,finger,joint_angles[frame,:])


def make_animation(finger,joint_angles,save_path=None):

    # we iterate over the joints to print the finger
    fig, ax = plt.subplots()


    ani = FuncAnimation(fig,plot_update,frames=len(joint_angles),fargs=(ax,finger,joint_angles),repeat=False)
    
    if save_path:
        ani.save(save_path, writer='ffmpeg', fps=30)
    
    plt.show()