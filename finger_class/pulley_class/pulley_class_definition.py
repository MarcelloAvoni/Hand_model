# PULLEY CLASS DEFINITION

#we import the necessary libraries
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

    # Method to update the pulley state given a certain angular rotation
    def rotate(self, angle):

        # Calculate the current and new angular positions
        current_angle = self.angular_position
        new_angle = self.angular_position + angle

        # Perform numerical integration to calculate the rolled length increment
        rolled_length_increment, _ = quad(self.radius_function, current_angle, new_angle)

        # Update the rolled length and angular position
        self.rolled_length += rolled_length_increment
        self.angular_position = new_angle