# SPRING CLASS DEFINITION
#here we define the class SPRING that is used as an interface between the finger and the output

class Spring:

    # class constructor
    def __init__(self,name,k,l,l_0):
        
        # NAME OF THE SPRING (string or number)
        self.name = name

        # SPRING ATTRIBUTES
        self.k = k                                   # Spring stiffness [N/m]
        self.l = l                                   # Spring length [m]
        self.l_0 = l_0                               # Spring rest length [m]
        self.F = self.k*(self.l - self.l_0)          # Spring force [N]

    # method to output the spring
    def __str__(self):
        return "Spring name: " + str(self.name) + "\nSpring stiffness: " + str(self.k) + " N/m\nSpring length: " + str(self.l) + " m\nSpring rest length: " + str(self.l_0) + " m\nSpring force: " + str(self.F) + " N"
    
    # method to update the spring given a new length
    def update_given_length(self,l_new):
            self.l = l_new
            self.F = self.k*(self.l - self.l_0)

    #method to output the force given a new length
    def output_force(self,l_new=None):
        if l_new is None:
            return self.F
        else:
             return self.k*(l_new - self.l_0)
         
    # method to update the spring given a new force
    def update_given_force(self,F_new):
        self.F = F_new
        self.l = self.l_0 + self.F/self.k

    # method to output the length given a new force
    def output_length(self,F_new=None):
        if F_new is None:
            return self.l
        else:
            return self.l_0 + F_new/self.k



