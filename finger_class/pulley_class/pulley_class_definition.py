# PULLEY CLASS DEFINITION

# here we define a pulley class that gives us a method to correlate tendon length to motor's angular position

class Pulley:

    #class constructor
    def __init__(self,name,shape,angular_position):

        self.name = name                            # pulley name
        self.shape = shape                          # pulley shape, matrix of 2xn values: on the first row the angular position and on the second the relative radius
        self.angular_position = angular_position    # motor's angular position [rad]