# TENDON CLASS DEFINITION
# here we define a tendon class to treat the tendons as separate entities from the joints, allowing for rerouting

class Tendon:

    #tendon class constructor
    def __init__(self,name,inf_stiff,elastic_const,length_0,length,tension):
        
        self.name = name                #name of the tendon (string or number)
        
        self.inf_stiff = inf_stiff      #infinite stiffness flag (boolean): 1 if the tendon is infinitely stiff, 0 otherwise

        #if the tendon is infinitely stiff, the elastic constant is set to 0
        if inf_stiff==1:
            self.elastic_const = 1E16
        else:
            self.elastic_const = elastic_const

        self.length_0 = length_0        #length of the unstretched tendon [m]
        
        if (inf_stiff==1)&(length!=length_0):
            raise ValueError(f"Error: tendon {self.name} is infinitely stiff but the length is different from the unstretched length")
        
        self.length = length            #length of the tendon under tension [m]
        
        self.tension = tension          #tension in the tendon [N]


    #method to print the tendon info
    def __str__(self):
        stiffness_status = "Yes" if self.inf_stiff else "No"
        return (f"Tendon {self.name}:\n"
                f"  Infinite Stiffness: {stiffness_status}\n"
                f"  Unstretched Length: {self.length_0} m\n"
                f"  Stretched Length: {self.length} m\n"
                f"  Tension: {self.tension} N")
    

    #method to update the tendon state given the tension
    def update_given_tension(self,new_tension):
        if new_tension>=0:
            self.tension = new_tension
        else:
            print(self.name)
            print(new_tension)
            raise ValueError(f"Error: tendon {self.name} tension set to negative value")
        
        if self.inf_stiff==0:
            self.length = self.length_0 + self.tension/self.elastic_const


    #method to update the tendon state given the rest length
    def update_given_length_0(self,new_length_0):
        if new_length_0>0:
            self.length_0 = new_length_0
        else:
            raise ValueError(f"Error: tendon {self.name} rest length set to negative value")
        
        if self.inf_stiff==0:
            self.tension = self.elastic_const*(self.length - self.length_0)
        else:
            self.length = new_length_0


    #method to update the tendon state given the stretched length
    def update_given_length(self,new_length):

        if(self.inf_stiff==1):
            raise ValueError(f"Error: tendon {self.name} is infinitely stiff and cannot be stretched, function update_given_length() not applicable")

        if (new_length-self.length_0)>0:
            self.length = new_length
        else:
            raise ValueError(f"Error: tendon {self.name} subject to buckling")
        
        if self.inf_stiff==0:
            self.tension = self.elastic_const*(self.length - self.length_0)

    # method to output the tension given a new length or length_0
    def output_tension(self,new_length=None,new_length_0=None):

        if self.inf_stiff==1:
            raise ValueError(f"Error: tendon {self.name} is infinitely stiff and cannot be stretched, function output_tension() not applicable")
        else:
            if new_length is None:
                if new_length_0 is None:
                    return self.tension
                else:
                    return self.elastic_const*(self.length - new_length_0)
            else:
                if new_length_0 is None:
                    return self.elastic_const*(new_length - self.length_0)
                else:
                    return self.elastic_const*(new_length - new_length_0)
            
    # method to output the length given a new tension or length_0
    def output_length(self,new_tension=None,new_length_0=None):

        if self.inf_stiff==1:
            raise ValueError(f"Error: tendon {self.name} is infinitely stiff and cannot be stretched, function output_length() not applicable")
        else:
            if new_tension is None:
                if new_length_0 is None:
                    return self.length
                else:
                    return new_length_0 + self.tension/self.elastic_const
            else:
                if new_length_0 is None:
                    return self.length_0 + new_tension/self.elastic_const
                else:
                    return new_length_0 + new_tension/self.elastic_const