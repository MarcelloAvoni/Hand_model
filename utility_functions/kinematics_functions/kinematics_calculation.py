import numpy as np

def rototranslation(alpha, x, y, x_o, y_o):

    # definition of an omothety useful for changes in system of reference
    x_rot = x * np.cos(alpha) + y * np.sin(alpha) + x_o
    y_rot = -x * np.sin(alpha) + y * np.cos(alpha) + y_o

    return x_rot, y_rot

def phalanx_kinematics(r_1, r_2, L_phalanx, theta, x_o, y_o, alpha):

    # This function calculates the main points of a phalanx in the absolute reference frame.
    # INPUTS:
    # r_1: radius of the first joint
    # r_2: radius of the second joint
    # L_phalanx: length of the phalanx
    # theta: flexion angle of the phalanx with respect to the previous one
    # x_o: x coordinate of the origin of the center of the tip of the previous phalanx
    # y_o: y coordinate of the origin of the center of the tip of the previous phalanx
    # alpha: angular position of the previous phalanx with respect to the absolute reference frame
    # OUTPUTS:
    # x_1, y_1: coordinates of the first cylinder center
    # x_1_phalanx_up, y_1_phalanx_up: coordinates of the first cylinder upper point
    # x_2_phalanx_up, y_2_phalanx_up: coordinates of the second cylinder upper point
    # x_2, y_2: coordinates of the second cylinder center
    # x_1_phalanx_down, y_1_phalanx_down: coordinates of the first cylinder lower point
    # x_2_phalanx_down, y_2_phalanx_down: coordinates of the second cylinder lower point



    # Calculate the main points positions in the local reference frame
    x_1 = 2 * r_1 * np.sin(theta / 2)
    y_1 = 2 * r_1 * np.cos(theta / 2)

    x_1_phalanx_up = x_1 - r_1 * np.cos(theta)
    y_1_phalanx_up = y_1 + r_1 * np.sin(theta)

    x_2_phalanx_up = x_1_phalanx_up + (L_phalanx - r_1 - r_2) * np.sin(theta)
    y_2_phalanx_up = y_1_phalanx_up + (L_phalanx - r_1 - r_2) * np.cos(theta)

    x_2 = x_2_phalanx_up + r_2 * np.cos(theta)
    y_2 = y_2_phalanx_up - r_2 * np.sin(theta)

    x_1_phalanx_down = x_1 + r_1 * np.cos(theta)
    y_1_phalanx_down = y_1 - r_1 * np.sin(theta)

    x_2_phalanx_down = x_2 + r_2 * np.cos(theta)
    y_2_phalanx_down = y_2 - r_2 * np.sin(theta)

    # Rotate and translate to stay in the absolute reference frame
    x_1, y_1 = rototranslation(alpha, x_1, y_1, x_o, y_o)
    x_1_phalanx_up, y_1_phalanx_up = rototranslation(alpha, x_1_phalanx_up, y_1_phalanx_up, x_o, y_o)
    x_2_phalanx_up, y_2_phalanx_up = rototranslation(alpha, x_2_phalanx_up, y_2_phalanx_up, x_o, y_o)
    x_2, y_2 = rototranslation(alpha, x_2, y_2, x_o, y_o)
    x_1_phalanx_down, y_1_phalanx_down = rototranslation(alpha, x_1_phalanx_down, y_1_phalanx_down, x_o, y_o)
    x_2_phalanx_down, y_2_phalanx_down = rototranslation(alpha, x_2_phalanx_down, y_2_phalanx_down, x_o, y_o)

    return (x_1, y_1, x_1_phalanx_up, y_1_phalanx_up, x_2_phalanx_up, y_2_phalanx_up, x_2, y_2, x_1_phalanx_down, y_1_phalanx_down, x_2_phalanx_down, y_2_phalanx_down)




def finger_kinematics(joint_angles,r_joints,r_tip,L_phalanxes):

    # This function computes the main points of a finger in the absolute reference frame.

    #we calculate the number of joints
    n_joints = len(r_joints)

    #preallocate the variables that will be used to store the coordinates of the main points
    x_1 = np.zeros(n_joints)
    y_1 = np.zeros(n_joints)
    x_1_phalanx_up = np.zeros(n_joints)
    y_1_phalanx_up = np.zeros(n_joints)
    x_2_phalanx_up = np.zeros(n_joints)
    y_2_phalanx_up = np.zeros(n_joints)
    x_2 = np.zeros(n_joints)
    y_2 = np.zeros(n_joints)
    x_1_phalanx_down = np.zeros(n_joints)
    y_1_phalanx_down = np.zeros(n_joints)
    x_2_phalanx_down = np.zeros(n_joints)
    y_2_phalanx_down = np.zeros(n_joints)

                                

    #finger's origin

    x_0 = 0
    y_0 = 0

    x_0_phalanx = 0
    y_0_phalanx = 0
    alpha = 0

    for i_iter in range(n_joints):

        if (i_iter == n_joints-1):
            (x_1[i_iter], y_1[i_iter], 
             x_1_phalanx_up[i_iter], y_1_phalanx_up[i_iter], 
             x_2_phalanx_up[i_iter], y_2_phalanx_up[i_iter], 
             x_2[i_iter], y_2[i_iter], 
             x_1_phalanx_down[i_iter], y_1_phalanx_down[i_iter], 
             x_2_phalanx_down[i_iter], y_2_phalanx_down[i_iter]) = phalanx_kinematics(
                r_joints[i_iter], r_tip, L_phalanxes[i_iter], joint_angles[i_iter], x_0_phalanx, y_0_phalanx, alpha)
        else:
            (x_1[i_iter], y_1[i_iter], 
             x_1_phalanx_up[i_iter], y_1_phalanx_up[i_iter], 
             x_2_phalanx_up[i_iter], y_2_phalanx_up[i_iter], 
             x_2[i_iter], y_2[i_iter], 
             x_1_phalanx_down[i_iter], y_1_phalanx_down[i_iter], 
             x_2_phalanx_down[i_iter], y_2_phalanx_down[i_iter]) = phalanx_kinematics(
                r_joints[i_iter],r_joints[i_iter+1],L_phalanxes[i_iter],joint_angles[i_iter],x_0_phalanx,y_0_phalanx,alpha)

        x_0_phalanx = x_2[i_iter]
        y_0_phalanx = y_2[i_iter]
        alpha += joint_angles[i_iter]

    return (x_1, y_1, x_1_phalanx_up, y_1_phalanx_up, x_2_phalanx_up, y_2_phalanx_up, x_2, y_2, x_1_phalanx_down, y_1_phalanx_down, x_2_phalanx_down, y_2_phalanx_down)
