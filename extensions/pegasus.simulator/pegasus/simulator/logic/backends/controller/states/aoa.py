"""
This files contains the class angle of attack which will be feed into the angle of attack sensor.
"""

class AngleOfAttack():

    def __init__(self):
        
        # Initialize with a defaullt angle of attack

        self.angle_of_attack = 12.2 # angle in deg
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
    