import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from utility_functions.kinematics_functions.kinematics_calculation import *
from math import sqrt


def plot_phalanx(ax, r_1, r_2, x_1, y_1, x_2, y_2, x_1_phalanx_up, y_1_phalanx_up, x_2_phalanx_up, y_2_phalanx_up, x_1_phalanx_down, y_1_phalanx_down, x_2_phalanx_down, y_2_phalanx_down,alpha=1.0):
    
    # we define 2 circumferences that represent the joints

    #we calculate the angles in wich circumference 1 touches (x_1_up,y_1_up) and (x_1_down,y_1_down)
    beta_1_up = np.arctan2(y_1_phalanx_up - y_1, x_1_phalanx_up - x_1)
    beta_1_down = np.arctan2(y_1_phalanx_down - y_1, x_1_phalanx_down - x_1)
    #we check that the angles are in the correct order
    if beta_1_up > beta_1_down:
        beta_1_up, beta_1_down = beta_1_down, beta_1_up

    beta_1 = np.linspace(beta_1_up,beta_1_down,100)

    #we plot the circumference   
    x_circ_1 = x_1 + r_1 * np.cos(beta_1+np.pi)
    y_circ_1 = y_1 + r_1 * np.sin(beta_1+np.pi)

    #we calculate the angles in wich circumference 2 touches (x_2_up,y_2_up) and (x_2_down,y_2_down)
    beta_2_up = np.arctan2(y_2_phalanx_up - y_2, x_2_phalanx_up - x_2)
    beta_2_down = np.arctan2(y_2_phalanx_down - y_2, x_2_phalanx_down - x_2)
    #we check that the angles are in the correct order
    if beta_2_up > beta_2_down:
        beta_2_up, beta_2_down = beta_2_down, beta_2_up
    

    beta_2 = np.linspace(beta_2_up,beta_2_down,100)

    #we plot the circumference
    x_circ_2 = x_2 + r_2 * np.cos(beta_2)
    y_circ_2 = y_2 + r_2 * np.sin(beta_2)

    #check onf the phalanges shape
    if sqrt((x_circ_1[49]-x_circ_2[49])**2 + (y_circ_1[49]-y_circ_2[49])**2) < sqrt((x_1-x_2)**2 + (y_1-y_2)**2):
           
        x_circ_1 = x_1 + r_1 * np.cos(beta_1)
        y_circ_1 = y_1 + r_1 * np.sin(beta_1)

        x_circ_2 = x_2 + r_2 * np.cos(beta_2+np.pi)
        y_circ_2 = y_2 + r_2 * np.sin(beta_2+np.pi)
        

    # we overlay lines that represent current phalanx
    ax.plot([x_1_phalanx_up, x_2_phalanx_up], [y_1_phalanx_up, y_2_phalanx_up], color='black', linewidth=2, linestyle='-',alpha=alpha)
    ax.plot([x_1_phalanx_down, x_2_phalanx_down], [y_1_phalanx_down, y_2_phalanx_down], color='black', linewidth=2, linestyle='-',alpha=alpha)

    # we overlay joints
    ax.plot(x_circ_1, y_circ_1, color='black', linewidth=2, linestyle='-',alpha=alpha)
    ax.plot(x_circ_2, y_circ_2, color='black', linewidth=2, linestyle='-',alpha=alpha)



def plot_finger(ax,finger,joint_angles,alpha=1.0):

    #we first extract the relevant variables
    r_joints = finger.r_joints
    r_tip = finger.r_tip
    L_phalanxes = finger.L_phalanxes
    L_metacarpal = finger.L_metacarpal

    #we calculate the number of joints
    n_joints = len(r_joints)

    # we set the aspect of the plot to be equal
    ax.set_aspect('equal')

    # we set the limits of the plot
    l_max = 0
    for i_iter in range(len(L_phalanxes)):
        l_max += L_phalanxes[i_iter]
        
    l_max = l_max + 2*r_tip
    ax.set_xlim(-2*r_joints[0],l_max)
    ax.set_ylim(min(-l_max,-L_metacarpal-r_joints[0]),l_max)

    # we set the labels
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')

    (x_1, y_1, x_1_phalanx_up, y_1_phalanx_up, x_2_phalanx_up, y_2_phalanx_up, x_2, y_2, x_1_phalanx_down, y_1_phalanx_down, x_2_phalanx_down, y_2_phalanx_down) = finger_kinematics(finger,joint_angles)

    #now we plot the finger
    x_0 = 0
    y_0 = 0

    angles = np.linspace(0,np.pi,100)
    x_circ = x_0 + r_joints[0]*np.cos(angles)
    y_circ = y_0 + r_joints[0]*np.sin(angles)

    ax.plot(x_circ,y_circ,color='black',linewidth=2,linestyle='-')
    ax.plot([x_0+r_joints[0],x_0+r_joints[0]],[y_0,y_0-L_metacarpal],color='black',linewidth=2,linestyle='-',alpha=alpha)
    ax.plot([x_0-r_joints[0],x_0-r_joints[0]],[y_0,y_0-L_metacarpal],color='black',linewidth=2,linestyle='-',alpha=alpha)
    ax.plot([x_0-r_joints[0],x_0+r_joints[0]],[y_0-L_metacarpal,y_0-L_metacarpal],color='black',linewidth=2,linestyle='-',alpha=alpha)


    for i_iter in range(n_joints):
        if (i_iter == n_joints - 1):
            plot_phalanx(ax, r_joints[i_iter], r_tip, x_1[i_iter], y_1[i_iter], x_2[i_iter], y_2[i_iter], x_1_phalanx_up[i_iter], y_1_phalanx_up[i_iter], x_2_phalanx_up[i_iter], y_2_phalanx_up[i_iter], x_1_phalanx_down[i_iter], y_1_phalanx_down[i_iter], x_2_phalanx_down[i_iter], y_2_phalanx_down[i_iter],alpha)
        else:
            plot_phalanx(ax, r_joints[i_iter], r_joints[i_iter+1], x_1[i_iter], y_1[i_iter], x_2[i_iter], y_2[i_iter], x_1_phalanx_up[i_iter], y_1_phalanx_up[i_iter], x_2_phalanx_up[i_iter], y_2_phalanx_up[i_iter], x_1_phalanx_down[i_iter], y_1_phalanx_down[i_iter], x_2_phalanx_down[i_iter], y_2_phalanx_down[i_iter],alpha)



    


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