# FINGER CLASS DEFINITION
# in this script we define the finger class that describes all the properties of a finger

# import the necessary classes
from joint_class.joint_class_definition import Joint
from tendon_class.tendon_class_definition import Tendon
from spring_class.spring_class_definition import Spring
from pulley_class.pulley_class_definition import Pulley

# import the necessary libraries
from math import atan2, sqrt

class Finger:

    #finger class constructor. Notice that the finger is initialized as straigth (as just assembled)
    def __init__(self,name,r_joints,r_tip,L_phalanxes,l_a,l_b,b_a,b_b,l_c,l_d,inf_stiff_tendons,k_tendons,l_springs,l_0_springs,k_springs,pulley_radius_functions,tendon_joint_interface,tendon_spring_interface,tendon_pulley_interface):
        
        self.name = name                #name of the finger (string or number)


        # check on the input data consistency
        # Joints number extraction
        n_joints = len(r_joints)

        # check on joints number on interface matrix
        if len(tendon_joint_interface[0]) != n_joints:
            raise ValueError("The number of joints must match the number of columns in the tendon-joint routing matrix")

        # Pulleys number extraction
        n_pulleys = len(pulley_radius_functions)

        # check on pulley number on interface matrix
        if len(tendon_pulley_interface[0]) != n_pulleys:
            raise ValueError("The number of pulleys must match the number of columns in the tendon-pulley interface matrix")
        
        # Springs number extraction
        n_springs = len(l_springs)

        # check on springs number on interface matrix
        if len(tendon_spring_interface[0]) != n_springs:
            raise ValueError("The number of springs must match the number of columns in the tendon-spring interface matrix")
        
        # Tendons number extraction
        n_tendons = len(inf_stiff_tendons)

        # check on tendons number on interface matrices
        if len(tendon_joint_interface) != n_tendons:
            raise ValueError("The number of tendons must match the number of rows in the tendon-joint routing matrix")
        if len(tendon_spring_interface) != n_tendons:
            raise ValueError("The number of tendons must match the number of rows in the tendon-spring interface matrix")
        if len(tendon_pulley_interface) != n_tendons:
            raise ValueError("The number of tendons must match the number of rows in the tendon-pulley interface matrix")
        
        # Validate that all input lists related to joints have the same length
        lengths_inputs = [len(r_joints), len(L_phalanxes), len(l_a), len(l_b), len(b_a), len(b_b), len(l_c), len(l_d)]
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
        
        
        
        # Preallocate lists for joints, tendons, springs, and pulleys
        self.pulleys = [None] * n_pulleys
        self.springs = [None] * n_springs
        self.tendons = [None] * n_tendons
        self.joints = [None] * n_joints

        # Initialize the pulley list
        for i_iter in range(n_pulleys):
            self.pulleys[i_iter] = Pulley(name=str(i_iter+1),
                                          radius_function=pulley_radius_functions[i_iter]
                                          )


        # Initialize the spring list
        for i_iter in range(n_springs):
            self.springs[i_iter] = Spring(name=str(i_iter+1),
                                          k=k_springs[i_iter],
                                          l=l_springs[i_iter],
                                          l_0=l_0_springs[i_iter]
                                          )


        # Initialize the tendon list
        for i_iter in range(n_tendons):
            Length = 0
            Tension = 0
            for j_iter in range(n_joints):
                if(tendon_joint_interface[i_iter][j_iter] == "f"):
                    Length += l_a[j_iter] + 2*r_joints[j_iter] + l_b[j_iter]
                elif(tendon_joint_interface[i_iter][j_iter] == "t"):
                    Length += l_c[j_iter] + 2*r_joints[j_iter] + l_d[j_iter]
                elif(tendon_joint_interface[i_iter][j_iter] == "e"):
                    Length += 2*r_joints[j_iter]
            
            for j_iter in range(n_springs):
                if tendon_spring_interface[i_iter][j_iter] == 1:
                    Tension = self.springs[j_iter].F

            self.tendons[i_iter] = Tendon(name=str(i_iter+1),
                                          inf_stiff=inf_stiff_tendons[i_iter],
                                          elastic_const=k_tendons[i_iter],
                                          length_0=Length,length=Length,
                                          tension=Tension
                                          )


        #here we perform data manipulation so to correlate inputs to lower classes, specifically the joint class
        L_wrench_phalanxes = [None] * n_joints
        gamma_phalanxes = [None] * n_joints
        p_x = [None] * n_joints
        p_y = [None] * n_joints
        Fx_phalanxes = [None] * n_joints
        Fy_phalanxes = [None] * n_joints
        M_phalanxes = [None] * n_joints
        Fx_ext = [None] * n_joints
        Fy_ext = [None] * n_joints
        M_ext = [None] * n_joints

        # Here we initialize the joint list and perform data manipulation 
        for i_iter in reversed(range(n_joints)):

            if (i_iter == n_joints-1):
                Length_x = r_joints[i_iter] - r_tip
                Length_y = L_phalanxes[i_iter] - r_joints[i_iter] - r_tip
            else:
                Length_x = r_joints[i_iter] - r_joints[i_iter+1]
                Length_y = L_phalanxes[i_iter] - r_joints[i_iter] - r_joints[i_iter+1]

            L_wrench_phalanxes[i_iter] = sqrt(Length_x**2 + Length_y**2)
            gamma_phalanxes[i_iter] = atan2(Length_x, Length_y)

            if (i_iter == n_joints - 1):
                p_x[i_iter] = r_tip - r_joints[i_iter]
            else:
                p_x[i_iter] = r_joints[i_iter + 1] - r_joints[i_iter]

            p_y[i_iter] = L_phalanxes[i_iter]

            
            Tension_e = 0
            Tension_t = 0
            Tension_f = 0
            for j_iter in range(n_tendons):
                if(tendon_joint_interface[j_iter][i_iter] == "f"):
                    Tension_f += self.tendons[j_iter].T
                elif(tendon_joint_interface[j_iter][i_iter] == "t"):
                    Tension_t += self.tendons[j_iter].T
                elif(tendon_joint_interface[j_iter][i_iter] == "e"):
                    Tension_e += self.tendons[j_iter].T

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
                Fx_phalanx=0,
                Fy_phalanx=0,
                M_phalanx=0,
                Fx_ext=0,
                Fy_ext=0,
                M_ext=0,
                T_e=Tension_e,
                T_t=Tension_t,
                T_f=Tension_f
            )
        
        # once the finger is created we can compute the initial tension of the tendons!!
        #first we define the problem
        # NEEDS TO BE FINISHED!!!!!!!




    # method to update the state of the finger by applying a wrench on each phalanx  
    def update_given_wrench(self,Fx,Fy,M):
        
        #check on dimensionality
        n_joints = len(self.joints)
        if(len(Fx) != n_joints):
            raise ValueError("The lenght of Fx must match the number of joints")
        if(len(Fy) != n_joints):
            raise ValueError("The lenght of Fy must match the number of joints")
        if(len(M) != n_joints):
            raise ValueError("The lenght of M must match the number of joints")

        

        for i_iter in reversed(range(n_joints)):
            self.joints[i_iter].Fx_phalanx = Fx[i_iter]
            self.joints[i_iter].Fy_phalanx = Fy[i_iter]
            self.joints[i_iter].M_phalanx = M[i_iter]

            if (i_iter == n_joints - 1):
                self.joints[i_iter].Fx_ext[i_iter] = 0
                self.joints[i_iter].Fy_ext[i_iter] = 0
                self.joints[i_iter].M_ext[i_iter] = 0
            else:
                self.joints[i_iter].Fx_ext[i_iter] = Fx[i_iter + 1] + self.joints[i_iter].Fx_ext[i_iter + 1]
                self.joints[i_iter].Fy_ext[i_iter] = Fy[i_iter + 1] + self.joints[i_iter].Fy_ext[i_iter + 1]
                self.joints[i_iter].M_ext[i_iter] = self.joints[i_iter + 1].transport_torques()
        

        # NEEDS TO BE FINISHED  