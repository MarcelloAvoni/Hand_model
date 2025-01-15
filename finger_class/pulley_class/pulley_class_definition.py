# PULLEY CLASS DEFINITION

#we import the necessary libraries
from math import pi
from scipy.integrate import quad

# here we define a pulley class that gives us a method to correlate tendon length to motor's angular position

class Pulley:

    # Class constructor
    def __init__(self, name, radius_function):
        self.name = name                            # Pulley name
        self.radius_function = radius_function      # Function defining the radius as a function of angular position
        self.angular_position = 0                   # Motor's angular position (starts from 0) [rad]
        self.rolled_length = 0                      # Cable's rolled length [m]

    # Method to output the pulley state
    def __str__(self):
        return f"Pulley {self.name} is at position {self.angular_position} rad and has rolled {self.rolled_length} m of cable"

    # Method to update the pulley state given a certain angular position
    def rotate(self, new_angular_position):

        number_of_turns = (new_angular_position - self.angular_position) // (2 * pi)
        angle_incomplete_turn = (new_angular_position - self.angular_position) % (2 * pi)

        # Perform numerical integration to calculate the rolled length increment
        rolled_length_increment = number_of_turns * quad(self.radius_function, 0, 2*pi)[0]
        rolled_length_increment += quad(self.radius_function, 0, angle_incomplete_turn)[0]

        # Update the rolled length and angular position
        self.rolled_length += rolled_length_increment
        self.angular_position = new_angular_position