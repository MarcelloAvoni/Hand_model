import numpy as np

def kinematics_simulation(finger, pulley_angles):
    num_simulations = len(pulley_angles)
    
    # Preallocate arrays for results
    joint_angles = np.zeros((num_simulations, finger.n_joints))
    tendon_tensions = np.zeros((num_simulations, finger.n_tendons))
    motor_torque = np.zeros(num_simulations)
    errors = np.zeros(num_simulations)

    for i in range(num_simulations):
        # Update the pulley angle
        finger.update_given_pulley_angle(pulley_angles[i])
        
        # Store the joint angles and tendon tensions
        for j in range(finger.n_joints):
            joint_angles[i, j] = finger.joints[j].theta
        
        for j in range(finger.n_tendons):
            tendon_tensions[i, j] = finger.tendons[j].tension
        
        # Store the motor torque and error
        motor_torque[i] = finger.motor_torque
        errors[i] = finger.error

    return joint_angles, tendon_tensions, motor_torque, errors