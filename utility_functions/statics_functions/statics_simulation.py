#in this file we perfomr the statics simulation that will be used for the foot metric

# import the necessary libraries
import numpy as np
import copy


def statics_simulation(finger,Fx,Fy,M):

    n_simulations = Fx.shape[0]  # Extract the number of rows from the forces matrix

    # Preallocate arrays for results
    joint_angles = np.zeros((n_simulations, finger.n_joints))
    tendon_tensions = np.zeros((n_simulations, finger.n_tendons))
    motor_torque = np.zeros(n_simulations)
    errors = np.zeros(n_simulations)

    #clone the original finger object
    finger_clone = copy.deepcopy(finger)

    for i in range(n_simulations):

        #update the forces
        finger_clone.update_given_phalanx_wrenches(Fx[i, :], Fy[i, :], M[i, :])

        #store the joint angles and tendon tensions
        for j in range(finger_clone.n_joints):
            joint_angles[i,j] = finger_clone.joints[j].theta

        for j in range(finger_clone.n_tendons):
            tendon_tensions[i,j] = finger_clone.tendons[j].tension

        #store the motor torque and error
        motor_torque[i] = finger_clone.motor_torque
        errors[i] = finger_clone.error

    return joint_angles, tendon_tensions, motor_torque, errors
