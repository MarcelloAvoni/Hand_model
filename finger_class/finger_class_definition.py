# FINGER CLASS DEFINITION
# in this script we define the finger class that describes all the properties of a finger

# import the necessary classes
from .joint_class.joint_class_definition import Joint
from .tendon_class.tendon_class_definition import Tendon
from .spring_class.spring_class_definition import Spring
from .pulley_class.pulley_class_definition import Pulley

# import the necessary libraries
import numpy as np
from math import atan2, sqrt, sin, cos
from scipy.optimize import least_squares

class Finger:

    #finger class constructor. Notice that the finger is initialized as straigth (as just assembled)
    def __init__(self,name,r_joints,r_tip,L_phalanxes,type_phalanxes,L_metacarpal,b_a_metacarpal,f_1,f_2,p_r,inf_stiff_tendons,k_tendons,l_springs,l_0_springs,k_springs,pulley_radius_functions,tendon_joint_interface,tendon_spring_interface,tendon_pulley_interface):
        
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
        lengths_inputs = [len(r_joints), len(L_phalanxes), len(type_phalanxes)]
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
        
        #first we save the finger paramters to compute kinematics
        self.r_joints = r_joints
        self.r_tip = r_tip
        self.L_phalanxes = L_phalanxes
        self.L_metacarpal = L_metacarpal
        self.f_1 = f_1
        self.f_2 = f_2
        self.p_r = p_r

        # we save the vector that contains information about the phalanxes type (1 = proximal, 2 = intermediate, 3 = distal)
        # the vector assumes that the first phalanx is always the metacarpal and is the only metacarpal
        self.type_phalanxes = type_phalanxes
        
        
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
            self.pulleys[i_iter] = Pulley(name=str(i_iter),
                                          radius_function=pulley_radius_functions[i_iter]
                                          )


        # Initialize the spring list
        for i_iter in range(self.n_springs):
            self.springs[i_iter] = Spring(name=str(i_iter),
                                          k=k_springs[i_iter],
                                          l=l_springs[i_iter],
                                          l_0=l_0_springs[i_iter]
                                          )
            

        #here we perform data manipulation so to correlate inputs to lower classes, specifically the joint class
        L_wrench_phalanxes = [0] * self.n_joints
        L_joint_centers = [0] * self.n_joints
        gamma_phalanxes = [0] * self.n_joints

        p_x = [0] * self.n_joints
        p_y = [0] * self.n_joints

        l_a = [0] * self.n_joints
        l_b = [0] * self.n_joints
        l_c = [0] * self.n_joints
        l_d = [0] * self.n_joints

        b_a = [0] * self.n_joints
        b_b = [0] * self.n_joints


        # Here we initialize the joint list and perform data manipulation 
        for i_iter in range(self.n_joints):

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

            # p initial definition
            p_x[i_iter] = r_2 - r_1
            p_y[i_iter] = L_phalanxes[i_iter]

            # auxiliary variable
            beta = atan2(2*Length_x,Length_y)

            # l_a definition
            if (i_iter == 0):
                l_a[i_iter] = 2*f_1*r_1                          #value for metacarpal
            else:
                if(self.type_phalanxes[i_iter - 1] == 1):
                    l_a[i_iter] = (2*f_2*r_1)*cos(beta)          #value for proximal and distal phalanx
                elif(self.type_phalanxes[i_iter - 1] == 2):
                    l_a[i_iter] = (0.004 + 2*f_2*r_1)*cos(beta)  #value for intermediate phalanx
                elif(self.type_phalanxes[i_iter - 1] == 3):
                    l_a[i_iter] = (2*f_2*r_1)*cos(beta)          #value for proximal and distal phalanx
                else:
                    l_a[i_iter] = (2*f_2*r_1)*cos(beta)          #generic value
                

            # l_b definition
            l_b[i_iter] = f_1*r_1*cos(beta)

            # l_c definition
            if (i_iter == 0):
                l_c[i_iter] = p_r           #value for metacarpal
            else:
                l_c[i_iter] = -p_r          #value in all other cases

            # l_d definition
            l_d[i_iter] = -p_r              # value in all cases


            # b_b definition
            b_b[i_iter] = (2*r_2 - r_1) + 2*(L_phalanxes[i_iter] - r_1 - r_2 - l_b[i_iter])*(r_1 - r_2)/(L_phalanxes[i_iter] - r_1 - r_2)

            # b_a definition
            if(i_iter == 0):
                b_a[i_iter] = b_a_metacarpal
            else:
                b_a[i_iter] = r_1 + 2*l_a[i_iter]*(r_joints[i_iter-1] - r_1)/(L_phalanxes[i_iter-1] - r_joints[i_iter-1] - r_1)

                    
            # Initialize the joint
            self.joints[i_iter] = Joint(
                name=str(i_iter),
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
                T_e=0,
                T_t=0,
                T_f=0
            )

        # save length joint centers (to be used in p_x and p_y)
        self.L_joint_centers = L_joint_centers

        # Initialize the tendon list
        lengths = self.output_tendon_lengths()

        tensions = [0] * self.n_tendons
        for i_iter in range(self.n_springs):
            tensions[self.map_spring_to_tendon[i_iter]] = self.springs[i_iter].F

        for i_iter in range(self.n_tendons):
            self.tendons[i_iter] = Tendon(name=str(i_iter),
                                          inf_stiff=inf_stiff_tendons[i_iter],
                                          elastic_const=k_tendons[i_iter],
                                          length_0=lengths[i_iter],
                                          length=lengths[i_iter],
                                          tension=tensions[i_iter]
                                          )
            
        #we initialize joints tensions which will be used for equilibrium scaling
        T_f,T_t,T_e = self.output_joint_tensions()
        for i_iter in range(self.n_joints):
            self.joints[i_iter].T_f = T_f[i_iter]
            self.joints[i_iter].T_t = T_t[i_iter]
            self.joints[i_iter].T_e = T_e[i_iter]

        #we initialize the external wrenches, given in the reference system of the corresponding phalanx
        self.Fx_phalanges = [0] * self.n_joints
        self.Fy_phalanges = [0] * self.n_joints
        self.M_phalanges = [0] * self.n_joints
        
        # once the finger is created we can compute the initial tension of the flexor tendons
        # by solving it as an equilibrium problem
        self.error = 0              # parameter that stores the error of the equilibrium
        self.motor_torque = 0       # parameter that stores the motor torque
        self.finger_equilibrium()


        
        
        


    
    #method that outputs the torques of the finger in the rolling points of contact of the joints
    def output_torques(self,theta=None,T_f=None,T_t=None,T_e=None,p_x=None,p_y=None,Fx_phalanx=None,Fy_phalanx=None,M_phalanx=None,Fx_ext=None,Fy_ext=None,M_ext=None):        

        # output preallocation
        torques = [0] * self.n_joints

        for i_iter in range(self.n_joints):
            torques[i_iter] = self.joints[i_iter].joint_torque(theta[i_iter],T_f[i_iter],T_t[i_iter],T_e[i_iter],p_x[i_iter],p_y[i_iter],Fx_phalanx[i_iter],Fy_phalanx[i_iter],M_phalanx[i_iter],Fx_ext[i_iter],Fy_ext[i_iter],M_ext[i_iter])

        return torques
    

    #method that calculates the lengths of the tendons
    def output_tendon_lengths(self,theta=None):

        # output preallocation
        lengths = [0] * self.n_tendons

        for i_iter in range(self.n_tendons):
            Length = 0
            for j_iter in range(self.n_joints):
                if(self.tendon_joint_interface[i_iter][j_iter] == "f"):
                    if theta is None:
                        Length_x, Length_y = self.joints[j_iter].l_f_components()
                    else:
                        Length_x, Length_y = self.joints[j_iter].l_f_components(theta[j_iter])
                    Length += sqrt(Length_x**2 + Length_y**2)
                elif(self.tendon_joint_interface[i_iter][j_iter] == "t"):
                    if theta is None:
                        Length_x, Length_y = self.joints[j_iter].l_t_components()
                    else:
                        Length_x, Length_y = self.joints[j_iter].l_t_components(theta[j_iter])
                    Length += sqrt(Length_x**2 + Length_y**2)
                elif(self.tendon_joint_interface[i_iter][j_iter] == "e"):
                    if theta is None:
                        Length += self.joints[j_iter].l_e_length()
                    else:
                        Length += self.joints[j_iter].l_e_length(theta[j_iter])
            lengths[i_iter] = Length
        
        return lengths
    

    #method that calculates the lengths of the springs
    def output_spring_lengths(self,theta=None):

        # output preallocation
        spring_lengths = [0] * self.n_springs

        #we calculate the change of length in the tendons
        lengths = self.output_tendon_lengths(theta)
        delta_lengths = [0] * self.n_tendons
        for i_iter in range(self.n_tendons):
            delta_lengths[i_iter] = lengths[i_iter] - self.tendons[i_iter].length

        #we calculate the tensions in the springs given the change of length in the tendons
        for i_iter in range(self.n_springs):
            spring_lengths[i_iter] = self.springs[i_iter].l + delta_lengths[self.map_spring_to_tendon[i_iter]]

        return spring_lengths


    
    #method that calculates the forces in the springs
    def output_spring_forces(self,theta=None):
        
        #output preallocation
        spring_forces = [0] * self.n_springs

        #we calculate the change of length in the springs
        spring_lengths = self.output_spring_lengths(theta)

        #we calculate the tensions in the springs given the change of length in the tendons
        for i_iter in range(self.n_springs):
            spring_forces[i_iter] = self.springs[i_iter].output_force(spring_lengths[i_iter])
        
        return spring_forces
    

    #function that outputs the joint tensions of the tendons
    def output_joint_tensions(self,theta=None,flexor_tendons_tensions=None):

        #output preallocation
        T_e = [0] * self.n_joints
        T_t = [0] * self.n_joints
        T_f = [0] * self.n_joints

        #we calculate the tensions in the springs given the change of length in the tendons
        tensions = [0] * self.n_tendons
        spring_forces = self.output_spring_forces(theta)
        for i_iter in range(self.n_springs):
            tensions[self.map_spring_to_tendon[i_iter]] = spring_forces[i_iter]
        
        #we introduce the tensions given by the pulleys
        for i_iter in range(self.n_pulleys):
            if flexor_tendons_tensions is None:
                tensions[self.map_pulley_to_tendon[i_iter]] = self.tendons[self.map_pulley_to_tendon[i_iter]].tension
            else:
                tensions[self.map_pulley_to_tendon[i_iter]] = flexor_tendons_tensions[i_iter]

        #we compute the tensions for each joint
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
        
        return T_f,T_t,T_e
    

    # function that outputs the external wrenches positions in the joints
    def output_ext_wrench_positions(self,theta=None):

        #input check
        if theta is None:
            theta = [0] * self.n_joints
            for i_iter in range(self.n_joints):
                theta[i_iter] = self.joints[i_iter].theta

        # output preallocation
        p_x = [0] * self.n_joints
        p_y = [0] * self.n_joints

        if (self.n_joints>1):
            for i_iter in range(self.n_joints-1):
                r_1 = self.joints[i_iter].r
                r_2 = self.joints[i_iter+1].r
                l_joints = self.L_joint_centers[i_iter]
                gamma = self.joints[i_iter].gamma_phalanx
                p_x[i_iter] = r_1*sin(theta[i_iter]/2) + l_joints*sin(theta[i_iter] - gamma) + r_2*sin(theta[i_iter] + theta[i_iter+1]/2)
                p_y[i_iter] = r_1*cos(theta[i_iter]/2) + l_joints*cos(theta[i_iter] - gamma) + r_2*cos(theta[i_iter] + theta[i_iter+1]/2)

        return p_x, p_y
    

    #function that outputs the external wrenches in the joints in the reference system of the previous phalanx given the wrenches on the phalanxes in the reference system of the current phalanx
    def output_ext_wrenches(self,theta=None,Fx=None,Fy=None,M=None):

        #input check
        if theta is None:
            theta = [0] * self.n_joints
            for i_iter in range(self.n_joints):
                theta[i_iter] = self.joints[i_iter].theta

        # output preallocation
        Fx_ext = [0] * self.n_joints
        Fy_ext = [0] * self.n_joints
        M_ext = [0] * self.n_joints

        # calculate the wrenches position in the joints
        p_x, p_y = self.output_ext_wrench_positions(theta)

        # calculate forces on the phalanxes in tre reference system of the previous phalanx
        Fx_phalanx, Fy_phalanx, M_phalanx = self.output_phalanx_wrenches(theta,Fx,Fy,M)

        # we calculate the external wrenches in the joints
        for i_iter in reversed(range(self.n_joints)):

            if(i_iter == self.n_joints-1):
                Fx_ext[i_iter] = 0
                Fy_ext[i_iter] = 0
                M_ext[i_iter] = 0
            else:
                #here we compute the forces in i_iter system of reference
                Fx_aux = Fx_phalanx[i_iter+1] + Fx_ext[i_iter+1]
                Fy_aux = Fy_phalanx[i_iter+1] + Fy_ext[i_iter+1]

                #the external forces need to be expressed in i_iter-1 system of reference
                Fx_ext[i_iter] = Fx_aux*cos(theta[i_iter]) + Fy_aux*sin(theta[i_iter])
                Fy_ext[i_iter] = -Fx_aux*sin(theta[i_iter]) + Fy_aux*cos(theta[i_iter])
                M_ext[i_iter] = self.joints[i_iter+1].transport_torques(theta[i_iter+1],p_x[i_iter+1],p_y[i_iter+1],Fx_phalanx[i_iter+1],Fy_phalanx[i_iter+1],M_phalanx[i_iter+1],Fx_ext[i_iter+1],Fy_ext[i_iter+1],M_ext[i_iter+1])


        return Fx_ext,Fy_ext,M_ext
    

    # we define a function that computes the phalanx wrenches in the reference system of the previous phalanx so to update the joints
    def output_phalanx_wrenches(self,theta=None,Fx=None,Fy=None,M=None):

        if theta is None:
            theta = [0] * self.n_joints
            for i_iter in range(self.n_joints):
                theta[i_iter] = self.joints[i_iter].theta
        
        if (Fx is None) & (Fy is None) & (M is None):
            Fx = [0] * self.n_joints
            Fy = [0] * self.n_joints
            M = [0] * self.n_joints
            for i_iter in range(self.n_joints):
                Fx[i_iter] = self.Fx_phalanges[i_iter]
                Fy[i_iter] = self.Fy_phalanges[i_iter]
                M[i_iter] = self.M_phalanges[i_iter]
        
        # output preallocation
        Fx_phalanx = [0] * self.n_joints
        Fy_phalanx = [0] * self.n_joints
        M_phalanx = [0] * self.n_joints

        # we rotate the wrenches in the reference system of the previous phalanx
        for i_iter in range(self.n_joints):
            Fx_phalanx[i_iter] = Fx[i_iter]*cos(theta[i_iter]) + Fy[i_iter]*sin(theta[i_iter])
            Fy_phalanx[i_iter] = -Fx[i_iter]*sin(theta[i_iter]) + Fy[i_iter]*cos(theta[i_iter])
            M_phalanx[i_iter] = M[i_iter]
        
        return Fx_phalanx,Fy_phalanx,M_phalanx



    #method that computes the equilibrium of the finger
    def finger_equations(self,variables,input_scale_factors,output_scale_factors):

        # check on dimensionality
        if (len(variables) != (self.n_joints + self.n_pulleys)):
            raise ValueError("The number of variables must match the number of joints and flexor tendons")
        
        # we extract the variables
        scaled_theta = variables[0:self.n_joints]
        scale_factor_theta = input_scale_factors[0:self.n_joints]

        scaled_flexor_tendons_tensions = variables[self.n_joints:(self.n_joints+self.n_pulleys)]
        scale_factor_flexor_tendons_tensions = input_scale_factors[self.n_joints:(self.n_joints+self.n_pulleys)]

        # we de-scale the variables
        theta = [0] * self.n_joints
        for i_iter in range(self.n_joints):
            theta[i_iter] = scaled_theta[i_iter] * scale_factor_theta[i_iter]

        
        
        flexor_tendons_tensions = [0] * self.n_pulleys
        for i_iter in range(self.n_pulleys):
            flexor_tendons_tensions[i_iter] = scaled_flexor_tendons_tensions[i_iter] * scale_factor_flexor_tendons_tensions[i_iter]

        # we calculate the spring forces according to the state variables
        spring_forces = self.output_spring_forces(theta)


        #we define the vector of tendon tensions
        tensions = [0] * self.n_tendons

        # we introduce the tensions given by the springs
        for i_iter in range(self.n_springs):
            tensions[self.map_spring_to_tendon[i_iter]] = spring_forces[i_iter]
        
        # we introduce the tensions given by the pulleys
        for i_iter in range(self.n_pulleys):
            tensions[self.map_pulley_to_tendon[i_iter]] = flexor_tendons_tensions[i_iter]

        # we compute the tensions for each joint
        T_f,T_t,T_e = self.output_joint_tensions(theta,flexor_tendons_tensions)

        # we compute torques given by the external wrenches
        p_x, p_y = self.output_ext_wrench_positions(theta)
        Fx_ext,Fy_ext,M_ext = self.output_ext_wrenches(theta)
        Fx_phalanx,Fy_phalanx,M_phalanx = self.output_phalanx_wrenches(theta)
        
        torques = self.output_torques(theta,T_f,T_t,T_e,p_x,p_y,Fx_phalanx,Fy_phalanx,M_phalanx,Fx_ext,Fy_ext,M_ext)

        #we compute the residuals on flexor tendon lengths
        lengths = self.output_tendon_lengths(theta)
        delta_lengths_flexor = [0] * self.n_pulleys
        for i_iter in range(self.n_pulleys):
            delta_lengths_flexor[i_iter] = lengths[self.map_pulley_to_tendon[i_iter]] - self.tendons[self.map_pulley_to_tendon[i_iter]].length

        #we compute the residuals
        residuals =  torques + delta_lengths_flexor

        #we scale the residuals
        for i_iter in range(self.n_joints):
            residuals[i_iter] = residuals[i_iter] / output_scale_factors[i_iter]
        

        return np.array(residuals)
    

    #we define a method to calculate the torque of the motor that actuates the pulleys
    def output_motor_torque(self):

        motor_torque = 0

        for i_iter in range(self.n_pulleys):
            motor_torque += self.tendons[self.map_pulley_to_tendon[i_iter]].tension * self.pulleys[i_iter].radius_function(self.pulleys[i_iter].angular_position)

        return motor_torque



        
    

    # we define the method that establishes the equilibrium of the finger by updating the object
    def finger_equilibrium(self,initial_guess=None):


        # we define the scaling factors for the input
        input_scale_factors_theta = (np.pi/2) * np.ones(self.n_joints)
        input_scale_factors_flexor_tendons_tensions = self.springs[0].F * np.ones(self.n_pulleys)
        input_scale_factors = np.concatenate((input_scale_factors_theta,input_scale_factors_flexor_tendons_tensions))

        #we define the scaling factors for the output
        output_scale_factors_torques = self.springs[0].F * self.joints[0].r * np.ones(self.n_joints)
        output_scale_factors_lengths = self.tendons[-1].length * np.ones(self.n_pulleys)
        output_scale_factors = np.concatenate((output_scale_factors_torques,output_scale_factors_lengths))


        #we define bounds for the variables
        lower_bounds_theta = -(1 + 1e-8) * np.ones(self.n_joints)
        upper_bounds_theta = (2 + 1e-8) * np.ones(self.n_joints)
        lower_bounds_flexor_tensions = 0 * np.ones(self.n_pulleys)
        upper_bounds_flexor_tensions = np.inf * np.ones(self.n_pulleys)
        lower_bounds = np.concatenate((lower_bounds_theta,lower_bounds_flexor_tensions))
        upper_bounds = np.concatenate((upper_bounds_theta,upper_bounds_flexor_tensions))

        # Initial guess for the variables (joint angle and flexor tendon tension)
        scaled_initial_guess = np.zeros(self.n_joints + self.n_pulleys)
        if(initial_guess is not None):
            for i_iter in range(self.n_joints + self.n_pulleys):
                scaled_initial_guess[i_iter] = initial_guess[i_iter] / input_scale_factors[i_iter]

        # Solve for equilibrium using least_squares
        result = least_squares(self.finger_equations,
                               scaled_initial_guess,
                               args=(input_scale_factors,output_scale_factors,),
                               bounds=(lower_bounds, upper_bounds),
                               loss='linear',
                               ftol=3e-16,
                               xtol=3e-16,
                               gtol=3e-16,
                               max_nfev=10000
                            )


        # we extract the equilibrium variables
        theta_eq = [0] * self.n_joints
        for i_iter in range(self.n_joints):
            theta_eq[i_iter] = result.x[i_iter] * input_scale_factors[i_iter]
        
        flexor_tendons_tensions_eq = [0] * self.n_pulleys
        for i_iter in range(self.n_pulleys):
            flexor_tendons_tensions_eq[i_iter] = result.x[self.n_joints + i_iter] * input_scale_factors[self.n_joints + i_iter]

        # now we update the object

        #we update the spring status given the lengths (the F is updated as a result)
        spring_lengths = self.output_spring_lengths(theta_eq)
        for i_iter in range(self.n_springs):
            self.springs[i_iter].update_given_length(spring_lengths[i_iter])

        # we update the tendon lengths_0 (and the tendon lengths as a result)
        tendon_lengths = self.output_tendon_lengths(theta_eq)
        for i_iter in range(self.n_tendons):
            self.tendons[i_iter].update_given_length_0(tendon_lengths[i_iter])

        #  we update the tendon tensions
        for i_iter in range(self.n_springs):
            self.tendons[self.map_spring_to_tendon[i_iter]].update_given_tension(self.springs[i_iter].F)
        
        for i_iter in range(self.n_pulleys):
            self.tendons[self.map_pulley_to_tendon[i_iter]].update_given_tension(flexor_tendons_tensions_eq[i_iter])

        # we update the joint angles
        for i_iter in range(self.n_joints):
            self.joints[i_iter].theta = theta_eq[i_iter]

        # we update the joint tensions
        T_f,T_t,T_e = self.output_joint_tensions(theta_eq,flexor_tendons_tensions_eq)
        for i_iter in range(self.n_joints):
            self.joints[i_iter].T_f = T_f[i_iter]
            self.joints[i_iter].T_t = T_t[i_iter]
            self.joints[i_iter].T_e = T_e[i_iter]

        # we update the external wrenches position
        p_x, p_y = self.output_ext_wrench_positions(theta_eq)
        for i_iter in range(self.n_joints):
            self.joints[i_iter].p_x = p_x[i_iter]
            self.joints[i_iter].p_y = p_y[i_iter]

        # we update the external wrenches
        Fx_ext,Fy_ext,M_ext = self.output_ext_wrenches(theta_eq)
        for i_iter in range(self.n_joints):
            self.joints[i_iter].Fx_ext = Fx_ext[i_iter]
            self.joints[i_iter].Fy_ext = Fy_ext[i_iter]
            self.joints[i_iter].M_ext = M_ext[i_iter]

        # we update the phalanx wrenches
        Fx_phalanx,Fy_phalanx,M_phalanx = self.output_phalanx_wrenches(theta_eq)
        for i_iter in range(self.n_joints):
            self.joints[i_iter].Fx_phalanx = Fx_phalanx[i_iter]
            self.joints[i_iter].Fy_phalanx = Fy_phalanx[i_iter]
            self.joints[i_iter].M_phalanx = M_phalanx[i_iter]

        # we update the equilibrium error
        self.error = result.cost

        # we update the motor torque
        self.motor_torque = self.output_motor_torque()
    



    # method that updates the state of the finger given the flexor tendon lengths
    def update_given_flexor_length(self,length_new,initial_guess=None):

        #we extract current state to obtain the initial guess
        if initial_guess is None:
            theta = [0] * self.n_joints
            flexor_tendons_tensions = [0] * self.n_pulleys
            for i_iter in range(self.n_joints):
                theta[i_iter] = self.joints[i_iter].theta
            for i_iter in range(self.n_pulleys):
                flexor_tendons_tensions[i_iter] = self.tendons[self.map_pulley_to_tendon[i_iter]].tension
            initial_guess = theta + flexor_tendons_tensions

        # we update the flexor lengths
        for i_iter in range(self.n_pulleys):
            self.tendons[self.map_pulley_to_tendon[i_iter]].update_given_length_0(length_new[i_iter])

        # we solve the equilibrium problem and update the state of the finger
        self.finger_equilibrium(initial_guess)



    #we define a method that updates the state of the finger given an angular position of the pulleys
    def update_given_pulley_angle(self,angular_position):
        
        # we compute the rolled lengths of the tendons
        rolled_lengths = [0] * self.n_pulleys
        for i_iter in range(self.n_pulleys):
            rolled_lengths[i_iter] = self.pulleys[i_iter].rolled_length
        
        #first we rotate the pulleys
        for i_iter in range(self.n_pulleys):
            self.pulleys[i_iter].rotate(angular_position)

        #we compute the rolled lengths increment
        rolled_lengths_increment = [0] * self.n_pulleys
        for i_iter in range(self.n_pulleys):
            rolled_lengths_increment[i_iter] = self.pulleys[i_iter].rolled_length - rolled_lengths[i_iter]

        #we update the tendon lengths
        new_lengths = [0] * self.n_pulleys
        for i_iter in range(self.n_pulleys):
            new_lengths[i_iter] = self.tendons[self.map_pulley_to_tendon[i_iter]].length - rolled_lengths_increment[i_iter]

        #we extract current state to obtain the initial guess
        theta = [0] * self.n_joints
        flexor_tendons_tensions = [0] * self.n_pulleys
        
        for i_iter in range(self.n_joints):
            theta[i_iter] = self.joints[i_iter].theta
        
        for i_iter in range(self.n_pulleys):
            flexor_tendons_tensions[i_iter] = self.tendons[self.map_pulley_to_tendon[i_iter]].tension

        initial_guess = theta + flexor_tendons_tensions

        # we update the state of the finger
        self.update_given_flexor_length(new_lengths,initial_guess)


    # we define a method that updates the state of the finger given the wrench on each phalanx
    # we assume the wrenches on the phalanxes to be expressed in the reference system of the phalanx itself
    def update_given_phalanx_wrenches(self,Fx,Fy,M):

        # we extract the current state
        theta = [0] * self.n_joints
        flexor_tendons_tensions = [0] * self.n_pulleys
        for i_iter in range(self.n_joints):
            theta[i_iter] = self.joints[i_iter].theta
        for i_iter in range(self.n_pulleys):
            flexor_tendons_tensions[i_iter] = self.tendons[self.map_pulley_to_tendon[i_iter]].tension

        # we update the wrenches on the phalanxes
        for i_iter in range(self.n_joints):
            self.Fx_phalanges[i_iter] = Fx[i_iter]
            self.Fy_phalanges[i_iter] = Fy[i_iter]
            self.M_phalanges[i_iter] = M[i_iter]


        # we define the initial guess
        initial_guess = theta + flexor_tendons_tensions

        # we update the state of the finger
        self.finger_equilibrium(initial_guess)





        

    





