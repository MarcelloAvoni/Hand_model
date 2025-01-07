# DEFINITION OF THE JOINT CLASS

# Importing the necessary libraries
from math import sin, cos, atan2


class Joint:
    
    def __init__(self,name,r,L_wrench_phalanx,L_phalanx,l_a,l_b,b_a,b_b,l_c,l_d,theta,p_x,p_y,Fx_phalanx,Fy_phalanx,M_phalanx,Fx_ext,Fy_ext,M_ext,T_e,T_t,T_f):

        
        # NAME OF THE JOINT (string or number)
        self.name = name


        # GEOMETRICAL ATTRIBUTES
        self.r = r                                  # Radius of the joint [m]
        
        self.L_wrench_phalanx = L_wrench_phalanx    # Wrench position along current phalanx [m]
        self.L_phalanx = L_phalanx                  # Current phalanx length [m]
        
        self.l_a = l_a                              # flexor tendon vertical position along previous phalanx [m]
        self.b_a = b_a                              # previous phalanx width [m]
        self.l_b = l_b                              # flexor tendon vertical position along current phalanx [m]
        self.b_b = b_b                              # current phalanx width [m]

        self.l_c = l_c                              # transiting tendon vertical position along previous phalanx [m]
        self.l_d = l_d                              # transiting tendon vertical position along current phalanx [m]

        self.theta = theta                          # Joint angle [rad]

        self.p_x = p_x                              # External wrench position with respect to rolling contact along x axis [m]
        self.p_y = p_y                              # External wrench position with respect to rolling contact along y axis [m]


        # WRENCH ATTRIBUTES
        self.Fx_phalanx = Fx_phalanx                # Force on the phalanx along x axiS [N]
        self.Fy_phalanx = Fy_phalanx                # Force on the phalanx along y axis [N]
        self.M_phalanx = M_phalanx                  # Moment on the phalanx [Nm]

        self.Fx_ext = Fx_ext                        # External force on the joint along x axis [N]
        self.Fy_ext = Fy_ext                        # External force on the joint along y axis [N]
        self.M_ext = M_ext                          # External moment on the joint [Nm]

        # TENDONS ATTRIBUTES
        self.T_e = T_e                              # Extensor tendon tension [N]
        self.T_t = T_t                              # Transiting tendon tension [N]
        self.T_f = T_f                              # Flexor tendon tension [N]


    def __str__(self):
        return (f"Joint {self.name}:\n"
                f"  Geometrical Attributes:\n"
                f"    Radius: {self.r} m\n"
                f"    Wrench Position along Phalanx: {self.L_wrench_phalanx} m\n"
                f"    Phalanx Length: {self.L_phalanx} m\n"
                f"    Flexor Tendon Vertical Position (Previous Phalanx): {self.l_a} m\n"
                f"    Previous Phalanx Width: {self.b_a} m\n"
                f"    Flexor Tendon Vertical Position (Current Phalanx): {self.l_b} m\n"
                f"    Current Phalanx Width: {self.b_b} m\n"
                f"    Transiting Tendon Vertical Position (Previous Phalanx): {self.l_c} m\n"
                f"    Transiting Tendon Vertical Position (Current Phalanx): {self.l_d} m\n"
                f"    Joint Angle: {self.theta} rad\n"
                f"    External Wrench Position (x): {self.p_x} m\n"
                f"    External Wrench Position (y): {self.p_y} m\n"
                f"  Wrench Attributes:\n"
                f"    Force on Phalanx (x): {self.Fx_phalanx} N\n"
                f"    Force on Phalanx (y): {self.Fy_phalanx} N\n"
                f"    Moment on Phalanx: {self.M_phalanx} Nm\n"
                f"    External Force (x): {self.Fx_ext} N\n"
                f"    External Force (y): {self.Fy_ext} N\n"
                f"    External Moment: {self.M_ext} Nm"
                f"  Tendon Attributes:\n"
                f"    Extensor Tendon Tension: {self.T_e} N\n"
                f"    Transiting Tendon Tension: {self.T_t} N\n"
                f"    Flexor Tendon Tension: {self.T_f} N\n")


    # Method to calculate the x and y components of the flexor tendon
    def l_f_components(self):

        #x component of the flexor tendon
        l_f_x = 2*self.r*sin(self.theta/2) + self.l_b*sin(self.theta) + (self.b_b/2)*cos(self.theta) - (self.b_a/2)

        #y component of the flexor tendon
        l_f_y = self.l_a + 2*self.r*cos(self.theta/2) + self.l_b*cos(self.theta) - (self.b_b/2)*sin(self.theta)

        return l_f_x, l_f_y
    

    # Method to calculate the x and y components of the transiting tendon
    def l_t_components(self):

        #x component of the transiting tendon
        l_t_x = 2*self.r*sin(self.theta/2) + self.l_d*sin(self.theta)

        #y component of the transiting tendon
        l_t_y = self.l_c + 2*self.r*cos(self.theta/2) + self.l_d*cos(self.theta)

        return l_t_x, l_t_y
    

    # Method to calculate the length of the extensor tendon
    def l_e_length(self):

        # Length of the extensor tendon
        l_e = 2*self.r + self.r*self.theta

        return l_e
    
    
    # Method to transport the total external wrenches to the rolling point of contact of the joint and compute the torque
    def transport_wrenches(self):

        # torque due to wrench on the phalanx
        torque_phalanx = -(self.r*cos(self.theta/2) + self.L_wrench_phalanx*cos(self.theta))*self.Fx_phalanx + (self.r*sin(self.theta/2) + self.L_wrench_phalanx*sin(self.theta))*self.Fy_phalanx + self.M_phalanx

        # torquee due to external wrench
        torque_ext = -self.p_y*self.Fx_ext + self.p_x*self.Fy_ext + self.M_ext

        # total torque
        torque = torque_phalanx + torque_ext

        return torque

    

    # Method to calculate the torque around the rolling point of contact of the joint
    def joint_torque(self):
        
        # first we calculate the x and y components of the tendons
        l_f_x, l_f_y = self.l_f_components()
        l_t_x, l_t_y = self.l_t_components()
    
        # we then calculate tendon's inclination angles
        phi_f = atan2(l_f_y,l_f_x)
        phi_t = atan2(l_t_y,l_t_x)

        # here we calculate the torque due to the tendons
        # torque due to the flexor tendon
        torque_f = + (self.r*cos(self.theta/2) + self.l_b*cos(self.theta) - (self.b_b/2)*sin(self.theta))*sin(phi_f)*self.T_f - (self.r*sin(self.theta/2) + self.l_b*sin(self.theta) + (self.b_b/2)*cos(self.theta))*cos(phi_f)*self.T_f
        
        # torque due to the transiting tendon
        torque_t = + (self.r*cos(self.theta/2) + self.l_d*cos(self.theta))*sin(phi_t)*self.T_t - (self.r*sin(self.theta/2) + self.l_d*sin(self.theta))*cos(phi_t)*self.T_t

        # torque due to the extensor tendon
        torque_e = self.r*self.T_e

        # here we calculate the external torques
        torque_ext = self.transport_wrenches()

        # total torque
        torque = torque_f + torque_t + torque_e + torque_ext


        return torque