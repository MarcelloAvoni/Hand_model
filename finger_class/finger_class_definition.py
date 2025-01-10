# FINGER CLASS DEFINITION
# in this script we define the finger class that describes all the properties of a finger

# import the necessary classes
from .joint_class.joint_class_definition import Joint
from .tendon_class.tendon_class_definition import Tendon
from .spring_class.spring_class_definition import Spring
from .pulley_class.pulley_class_definition import Pulley

# import the necessary libraries
import numpy as np
from math import atan2, sqrt
from scipy.optimize import least_squares

class Finger:

    #finger class constructor. Notice that the finger is initialized as straigth (as just assembled)
    def __init__(self,name,r_joints,r_tip,L_phalanxes,l_a,l_b,b_a_metacarpal,l_c,l_d,inf_stiff_tendons,k_tendons,l_springs,l_0_springs,k_springs,pulley_radius_functions,tendon_joint_interface,tendon_spring_interface,tendon_pulley_interface):
        
        #name of the finger (string or number)
        self.name = name


        # check on the input data consistency
        # Joints number extraction
        self.n_joints = len(r_joints)

        # check on joints number on interface matrix
        if len(tendon_joint_interface[0]) != self.n_joints:
            raise ValueError("The number of joints must match the number of columns in the tendon-joint routing matrix")

        # Pulleys number extraction
        self.n_pulleys = len(pulley_radius_functions)


        # check on pulley number on interface matrix
        if len(tendon_pulley_interface[0]) != self.n_pulleys:
            raise ValueError("The number of pulleys must match the number of columns in the tendon-pulley interface matrix")
        
        # Springs number extraction
        self.n_springs = len(l_springs)

        # check on springs number on interface matrix
        if len(tendon_spring_interface[0]) != self.n_springs:
            raise ValueError("The number of springs must match the number of columns in the tendon-spring interface matrix")
        
        # Tendons number extraction
        self.n_tendons = len(inf_stiff_tendons)

        # check on tendons number on interface matrices
        if len(tendon_joint_interface) != self.n_tendons:
            raise ValueError("The number of tendons must match the number of rows in the tendon-joint routing matrix")
        if len(tendon_spring_interface) != self.n_tendons:
            raise ValueError("The number of tendons must match the number of rows in the tendon-spring interface matrix")
        if len(tendon_pulley_interface) != self.n_tendons:
            raise ValueError("The number of tendons must match the number of rows in the tendon-pulley interface matrix")
        
        # Validate that all input lists related to joints have the same length
        lengths_inputs = [len(r_joints), len(L_phalanxes), len(l_a), len(l_b), len(l_c), len(l_d)]
        if len(set(lengths_inputs)) != 1:
            raise ValueError("All input lists related to joints must have the same number of entries")
        
        #validate that all input lists related to tendons have the same length
        lengths_inputs = [len(inf_stiff_tendons), len(k_tendons)]
        if len(set(lengths_inputs)) != 1:
            raise ValueError("All input lists related to tendons must have the same number of entries")
        
        #validate that all input lists related to springs have the same length
        lengths_inputs = [len(l_springs), len(l_0_springs), len(k_springs)]
        if len(set(lengths_inputs)) != 1:
            raise ValueError("All input lists related to springs must have the same number of entries")
        
        
        #here we save the tendon interface matrices
        self.tendon_joint_interface = tendon_joint_interface
        self.tendon_spring_interface = tendon_spring_interface
        self.tendon_pulley_interface = tendon_pulley_interface

        # Here we create vectors that map the various components of the system


        # springs and tendons
        map_tendon_to_spring = [-1] * self.n_tendons
        map_spring_to_tendon = [-1] * self.n_springs
        for i_iter in range(self.n_tendons):
            for j_iter in range(self.n_springs):
                if self.tendon_spring_interface[i_iter][j_iter] == 1:
                    map_tendon_to_spring[i_iter] = j_iter
                    map_spring_to_tendon[j_iter] = i_iter

        # pulleys and tendons
        map_tendon_to_pulley = [-1] * self.n_tendons
        map_pulley_to_tendon = [-1] * self.n_pulleys
        for i_iter in range(self.n_tendons):
            for j_iter in range(self.n_pulleys):
                if self.tendon_pulley_interface[i_iter][j_iter] == 1:
                    map_tendon_to_pulley[i_iter] = j_iter
                    map_pulley_to_tendon[j_iter] = i_iter


        #we save the mappping vectors as well
        self.map_tendon_to_spring = map_tendon_to_spring
        self.map_spring_to_tendon = map_spring_to_tendon
        self.map_tendon_to_pulley = map_tendon_to_pulley
        self.map_pulley_to_tendon = map_pulley_to_tendon


        
        # Preallocate lists for joints, tendons, springs, and pulleys
        self.pulleys = [None] * self.n_pulleys
        self.springs = [None] * self.n_springs
        self.tendons = [None] * self.n_tendons
        self.joints = [None] * self.n_joints

        # Initialize the pulley list
        for i_iter in range(self.n_pulleys):
            self.pulleys[i_iter] = Pulley(name=str(i_iter+1),
                                          radius_function=pulley_radius_functions[i_iter]
                                          )


        # Initialize the spring list
        for i_iter in range(self.n_springs):
            self.springs[i_iter] = Spring(name=str(i_iter+1),
                                          k=k_springs[i_iter],
                                          l=l_springs[i_iter],
                                          l_0=l_0_springs[i_iter]
                                          )


        #here we perform data manipulation so to correlate inputs to lower classes, specifically the joint class
        L_wrench_phalanxes = [0] * self.n_joints
        L_joint_centers = [0] * self.n_joints
        gamma_phalanxes = [0] * self.n_joints
        b_a = [0] * self.n_joints
        b_b = [0] * self.n_joints
        p_x = [0] * self.n_joints
        p_y = [0] * self.n_joints

        # Here we initialize the joint list and perform data manipulation 
        for i_iter in reversed(range(self.n_joints)):

            if(i_iter == self.n_joints-1):
                r_1 = r_joints[i_iter]
                r_2 = r_tip
            else:
                r_1 = r_joints[i_iter]
                r_2 = r_joints[i_iter+1]

            Length_x = r_1 - r_2
            Length_y = L_phalanxes[i_iter] - r_1 - r_2

            L_joint_centers[i_iter] = sqrt(Length_x**2 + Length_y**2)
            L_wrench_phalanxes[i_iter] = L_joint_centers[i_iter]/2
            gamma_phalanxes[i_iter] = atan2(Length_x, Length_y)

            p_x[i_iter] = r_2 - r_1
            p_y[i_iter] = L_phalanxes[i_iter]

            b_b[i_iter] = (2*r_2 - r_1) + 2*(L_phalanxes[i_iter] - r_1 - r_2 - l_b[i_iter])*(r_1 - r_2)/(L_phalanxes[i_iter] - r_1 - r_2)

            if(i_iter!=self.n_joints-1):
                if(i_iter == 0):
                    b_a[i_iter] = b_a_metacarpal
                else:
                    b_a[i_iter+1] = r_2 + 2*l_a[i_iter+1]/(L_phalanxes[i_iter] - r_1 - r_2)

            
            # Initialize the tension of the extensor tendons in the joint, notice that the value will be updated later
            Tension_e = 0
            Tension_t = 0
            for j_iter in range(self.n_tendons):
                if (map_tendon_to_spring[j_iter] != -1):
                    if(self.tendon_joint_interface[j_iter][i_iter] == "e"):
                        Tension_e += self.springs[map_tendon_to_spring[j_iter]].F
                    elif(self.tendon_joint_interface[j_iter][i_iter] == "t"):
                        Tension_t += self.springs[map_tendon_to_spring[j_iter]].F
                    
            # Initialize the joint
            self.joints[i_iter] = Joint(
                name=str(i_iter+1),
                r=r_joints[i_iter],
                L_phalanx=L_phalanxes[i_iter],
                l_a=l_a[i_iter],
                l_b=l_b[i_iter],
                b_a=b_a[i_iter],
                b_b=b_b[i_iter],
                l_c=l_c[i_iter],
                l_d=l_d[i_iter],
                L_wrench_phalanx=L_wrench_phalanxes[i_iter],
                gamma_phalanx=gamma_phalanxes[i_iter],
                p_x=p_x[i_iter],
                p_y=p_y[i_iter],
                T_e=Tension_e,
                T_t=Tension_t,
                T_f=0
            )

        # Initialize the tendon list
        for i_iter in range(self.n_tendons):
            Length = 0
            Tension = 0
            for j_iter in range(self.n_joints):
                if(self.tendon_joint_interface[i_iter][j_iter] == "f"):
                    Length_x, Length_y = self.joints[j_iter].l_f_components()
                    Length += sqrt(Length_x**2 + Length_y**2)
                elif(self.tendon_joint_interface[i_iter][j_iter] == "t"):
                    Length_x, Length_y = self.joints[j_iter].l_t_components()
                    Length += sqrt(Length_x**2 + Length_y**2)
                elif(self.tendon_joint_interface[i_iter][j_iter] == "e"):
                    Length += self.joints[j_iter].l_e_length()
            
            for j_iter in range(self.n_springs):
                if self.tendon_spring_interface[i_iter][j_iter] == 1:
                    Tension = self.springs[j_iter].F

            self.tendons[i_iter] = Tendon(name=str(i_iter+1),
                                          inf_stiff=inf_stiff_tendons[i_iter],
                                          elastic_const=k_tendons[i_iter],
                                          length_0=Length,length=Length,
                                          tension=Tension
                                          )
        
        # once the finger is created we can compute the initial tension of the flexor tendons
        # first we define the problem
        
        
        


    
    #method that outputs the torques of the finger in the rolling points of contact of the joints
    def output_torques(self,theta=None,T_f=None,T_t=None,T_e=None):        

        # output preallocation
        torques = [0] * self.n_joints

        for i_iter in range(self.n_joints):
            torques[i_iter] = self.joints[i_iter].joint_torque(theta[i_iter],T_f[i_iter],T_t[i_iter],T_e[i_iter])

        return torques
    


    def output_tendon_lengths(self,theta=None):

        # output preallocation
        lengths = [0] * self.n_tendons

        for i_iter in range(self.n_tendons):
            Length = 0
            for j_iter in range(self.n_joints):
                if(self.tendon_joint_interface[i_iter][j_iter] == "f"):
                    Length_x, Length_y = self.joints[j_iter].l_f_components(theta[j_iter])
                    Length += sqrt(Length_x**2 + Length_y**2)
                elif(self.tendon_joint_interface[i_iter][j_iter] == "t"):
                    Length_x, Length_y = self.joints[j_iter].l_t_components(theta[j_iter])
                    Length += sqrt(Length_x**2 + Length_y**2)
                elif(self.tendon_joint_interface[i_iter][j_iter] == "e"):
                    Length += self.joints[j_iter].l_e_length(theta[j_iter])
            lengths[i_iter] = Length
        
        return lengths
    


    #method that changes the state of the finger in order to reach equilibrium
    def finger_equations(self,variables):

        # check on dimensionality
        if (len(variables) != (self.n_joints + self.n_pulleys)):
            raise ValueError("The number of variables must match the number of joints and flexor tendons")
        
        # we extract the variables
        theta = variables[0:self.n_joints]
        flexor_tendons_tensions = variables[self.n_joints:(self.n_joints+self.n_pulleys)]

        # we calculate the state of the system according to the variables
        lengths = self.output_tendon_lengths(theta)
        delta_lengths = [0] * self.n_tendons
        for i_iter in range(self.n_tendons):
            delta_lengths[i_iter] = lengths[i_iter] - self.tendons[i_iter].length

        tensions = [0] * self.n_tendons

        # we introduce the tensions given by the springs
        for i_iter in range(self.n_springs):
            length_spring = self.springs[i_iter].l + delta_lengths[self.map_spring_to_tendon[i_iter]]
            tensions[self.map_spring_to_tendon[i_iter]] = self.springs[i_iter].output_force(length_spring)
        
        # we introduce the tensions given by the pulleys
        for i_iter in range(self.n_pulleys):
            tensions[self.map_pulley_to_tendon[i_iter]] = flexor_tendons_tensions[i_iter]

        # we compute the tensions for each joint
        T_e = [0] * self.n_joints
        T_t = [0] * self.n_joints
        T_f = [0] * self.n_joints

        for i_iter in range(self.n_joints):
            Tension_f = 0
            Tension_e = 0
            Tension_t = 0
            for j_iter in range(self.n_tendons):
                if (self.tendon_joint_interface[j_iter][i_iter] == "e"):
                    Tension_e += tensions[j_iter]
                elif (self.tendon_joint_interface[j_iter][i_iter] == "t"):
                    Tension_t += tensions[j_iter]
                elif (self.tendon_joint_interface[j_iter][i_iter] == "f"):
                    Tension_f += tensions[j_iter]
            T_e[i_iter] = Tension_e
            T_t[i_iter] = Tension_t
            T_f[i_iter] = Tension_f
        
        torques = self.output_torques(theta,T_f,T_t,T_e)

        delta_lengths_flexor = [0] * self.n_pulleys
        for i_iter in range(self.n_pulleys):
            delta_lengths_flexor[i_iter] = delta_lengths[self.map_pulley_to_tendon[i_iter]]

        residuals = torques + delta_lengths_flexor

        return np.array(residuals)