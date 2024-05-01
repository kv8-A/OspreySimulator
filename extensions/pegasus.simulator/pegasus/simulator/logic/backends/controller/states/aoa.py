"""
This files contains the class angle of attack which will be feed into the angle of attack sensor.
"""
from pegasus.simulator.logic.state import State

class AngleOfAttack():

    def __init__(self):
        
        # Initialize with a defaullt angle of attack
        self.initial_aoa = 8.0
        self.angle_of_attack = 8.0 # angle in deg
        self.min_aoa = 0.0
        self.max_aoa = 0.0 # set max on highest Cl, to start.

    def set_angle_of_attack(self,angle):
        self.angle_of_attack = angle
        
    
    def aoa_rad2deg(self):
        """
        Returns the angle of attack in degrees
        """
        aoa_deg = np.rad2deg(self.angle_of_attack)
        return aoa_deg


    def get_aoa(self):
        return self.angle_of_attack
    
    def reset(self):
        self.angle_of_attack = self.initial_aoa 