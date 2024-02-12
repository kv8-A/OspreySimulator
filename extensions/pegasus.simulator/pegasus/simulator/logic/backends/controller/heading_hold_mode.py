import numpy as np



class HeadingHoldMode():
    """
    Class that will implement the heading hold mode controller
    """

    def __init__(self):
        """
        Receives as input the reference heading and the measuered heading
        
        Args:
            sdfs
        
        """
        self.HI_e = 0.0

        self._yaw_moment = 0.0

    

    def shortest_heading_error(self, HI_ref, HI_m):
        heading_e = HI_ref - HI_m
        # Adjust for the circular nature of headings
        if heading_e > 180:
            heading_e -= 360
        elif heading_e < -180:
            heading_e += 360
        return heading_e

    def update(self, HI_ref, HI_m):
        """
        Method that will update the yaw moment if there is a heading error with the reference heading given
        """

        # heading_e = self.heading_error()
        # heading_m = self._HI_m
        # heading_r = self._HI_ref

        heading_e = self.shortest_heading_error(HI_ref, HI_m)
        # print("Heading Error: ", heading_e)

        max_yaw_moment = 0.0025

        Kp = 1 # Oscilating
        Kp = 0.0001     # Non-Unitless Kp: should be Nm/degree
        yaw_moment = -Kp * heading_e  

        # Ensure the yaw moment is within the maximum limits
        yaw_moment = min(max(yaw_moment, -max_yaw_moment), max_yaw_moment)

        # if heading_e < 30 and heading_e >0:
            
        #     # yaw_moment = Kp*heading_e

        #     # yaw_moment = max(min(yaw_moment, max_yaw_moment), -max_yaw_moment)
        #     yaw_moment = yaw_moment*-1

        

        # lines below are for a body axes with z pointing to earth.
        # yaw_moment = Kp*heading_e

        # yaw_moment = max(min(yaw_moment, max_yaw_moment), -max_yaw_moment)

        self._yaw_moment = yaw_moment
        return self._yaw_moment
    
    @property
    def yaw_moment(self):

        return self._yaw_moment
    # TODO include a way to verifiy . Thus save parameter HIref, HIm and and just simulate.


