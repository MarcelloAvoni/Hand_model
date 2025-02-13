import numpy as np
from finger_class.finger_class_definition import Finger

def clone_finger(finger):
    #Create a copy of the Finger object.

    return Finger(
        finger.name,
        finger.r_joints.copy(),
        finger.r_tip,
        finger.L_phalanxes.copy(),
        finger.l_a.copy(),
        finger.l_b.copy(),
        finger.b_a_metacarpal,
        finger.l_c.copy(),
        finger.l_d.copy(),
        finger.inf_stiff_tendons.copy(),
        finger.k_tendons.copy(),
        finger.l_springs.copy(),
        finger.l_0_springs.copy(),
        finger.k_springs.copy(),
        finger.pulley_radius_functions.copy(),
        finger.tendon_joint_interface.copy(),
        finger.tendon_spring_interface.copy(),
        finger.tendon_pulley_interface.copy()
    )

def kinematics_simulation(finger, pulley_angles):

    num_simulations = len(pulley_angles)
    
    # Preallocate arrays for results
    joint_angles = np.zeros((num_simulations, finger.n_joints))
    tendon_tensions = np.zeros((num_simulations, finger.n_tendons))
    motor_torque = np.zeros(num_simulations)
    errors = np.zeros(num_simulations)

    # Clone the original finger object
    finger_clone = clone_finger(finger)

    for i in range(num_simulations):
        # Update the pulley angle
        finger_clone.update_given_pulley_angle(pulley_angles[i])
        
        # Store the joint angles and tendon tensions
        for j in range(finger_clone.n_joints):
            joint_angles[i, j] = finger_clone.joints[j].theta
        
        for j in range(finger_clone.n_tendons):
            tendon_tensions[i, j] = finger_clone.tendons[j].tension
        
        # Store the motor torque and error
        motor_torque[i] = finger_clone.motor_torque
        errors[i] = finger_clone.error

    return joint_angles, tendon_tensions, motor_torque, errors