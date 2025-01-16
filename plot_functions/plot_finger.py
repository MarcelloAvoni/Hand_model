import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

def meters_to_points(meters):

    points = meters * 1000 * 72 / 25.4

    return points

def rototranslation_given_angle(alpha,x,y,x_o,y_o):

    x_rot = x*np.cos(alpha) + y*np.sin(alpha) + x_o
    y_rot = - x*np.sin(alpha) + y*np.cos(alpha) + y_o

    return x_rot, y_rot




def plot_phalanx(ax,r_1,r_2,L_phalanx,theta,x_o,y_o,alpha):

    # we calculate the main points positions in the local reference frame
    x_1 = 2*r_1*np.sin(theta/2)
    y_1 = 2*r_1*np.cos(theta/2)

    x_1_phalanx_up = x_1 - r_1*np.cos(theta)
    y_1_phalanx_up = y_1 + r_1*np.sin(theta)

    x_2_phalanx_up = x_1_phalanx_up + (L_phalanx-r_1-r_2)*np.sin(theta)
    y_2_phalanx_up = y_1_phalanx_up + (L_phalanx-r_1-r_2)*np.cos(theta)

    x_2 = x_2_phalanx_up + r_2*np.cos(theta)
    y_2 = y_2_phalanx_up - r_2*np.sin(theta)

    x_1_phalanx_down = x_1 + r_1*np.cos(theta)
    y_1_phalanx_down = y_1 - r_1*np.sin(theta)

    x_2_phalanx_down = x_2 + r_2*np.cos(theta)
    y_2_phalanx_down = y_2 - r_2*np.sin(theta)


    # we rotate  and translate so to stay in the absolute reference frame
    x_1, y_1 = rototranslation_given_angle(alpha,x_1,y_1,x_o,y_o)
    x_1_phalanx_up, y_1_phalanx_up = rototranslation_given_angle(alpha,x_1_phalanx_up,y_1_phalanx_up,x_o,y_o)
    x_2_phalanx_up, y_2_phalanx_up = rototranslation_given_angle(alpha,x_2_phalanx_up,y_2_phalanx_up,x_o,y_o)
    x_2, y_2 = rototranslation_given_angle(alpha,x_2,y_2,x_o,y_o)
    x_1_phalanx_down, y_1_phalanx_down = rototranslation_given_angle(alpha,x_1_phalanx_down,y_1_phalanx_down,x_o,y_o)
    x_2_phalanx_down, y_2_phalanx_down = rototranslation_given_angle(alpha,x_2_phalanx_down,y_2_phalanx_down,x_o,y_o)

    # we define 2 circumferences that represent the joints
    theta = np.linspace(0,2*np.pi,100)
    x_circ_1 = x_1 + r_1*np.sin(theta)
    y_circ_1 = y_1 + r_1*np.cos(theta)

    x_circ_2 = x_2 + r_2*np.sin(theta)
    y_circ_2 = y_2 + r_2*np.cos(theta)

    #we overlay lines that represent current phalanx
    ax.plot([x_1,x_2],[y_1,y_2],color='black',linewidth=1,linestyle='--')
    ax.plot([x_1_phalanx_up,x_2_phalanx_up],[y_1_phalanx_up,y_2_phalanx_up],color='black',linewidth=2,linestyle='-')
    ax.plot([x_1_phalanx_down,x_2_phalanx_down],[y_1_phalanx_down,y_2_phalanx_down],color='black',linewidth=2,linestyle='-')

    #we overlay joints
    ax.plot(x_circ_1,y_circ_1,color='black',linewidth=2,linestyle='-')
    ax.plot(x_circ_2,y_circ_2,color='black',linewidth=2,linestyle='-')

    return x_2, y_2






def plot_finger(ax,joint_angles,r_joints,r_tip,L_phalanxes):

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
    ax.set_ylim(-r_joints[0],l_max)

    # we set the ticks to be invisible
    ax.set_xticks([])
    ax.set_yticks([])

    # we set the labels
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')

    # we set the title
    ax.set_title('Finger Configuration')

    #now we plot the finger
    x_0 = 0
    y_0 = 0

    angles = np.linspace(0,2*np.pi,100)
    x_circ = x_0 + r_joints[0]*np.sin(angles)
    y_circ = y_0 + r_joints[0]*np.cos(angles)

    ax.plot(x_circ,y_circ,color='black',linewidth=2,linestyle='-')


    alpha = 0

    for i_iter in range(n_joints):

        if (i_iter == n_joints-1):
            x_0, y_0 = plot_phalanx(ax,r_joints[i_iter],r_tip,L_phalanxes[i_iter],joint_angles[i_iter],x_0,y_0,alpha)
        else:
            x_0, y_0 = plot_phalanx(ax,r_joints[i_iter],r_joints[i_iter+1],L_phalanxes[i_iter],joint_angles[i_iter],x_0,y_0,alpha)

        alpha += joint_angles[i_iter]


def plot_update(frame,ax,joint_angles,r_joints,r_tip,L_phalanxes):

    ax.clear()
    plot_finger(ax,joint_angles[frame,:],r_joints,r_tip,L_phalanxes)


def make_animation(joint_angles,r_joints,r_tip,L_phalanxes):

    # we iterate over the joints to print the finger
    fig, ax = plt.subplots()


    ani = FuncAnimation(fig,plot_update,frames=len(joint_angles),fargs=(ax,joint_angles,r_joints,r_tip,L_phalanxes),repeat=False)
    plt.show()