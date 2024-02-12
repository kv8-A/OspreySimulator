"""
| File: wind.py
| Description: wind in the environment 
| License: BSD-3-Clause. Copyright (c) 2023. All rights reserved.
"""

import numpy as np


"""
Write wind class that will be respective to the total airstream velocity.
This is to make sure that at wind speeds of lets say 8m/s the drone can have an free airstream of 8m/s but a 
a linear veliocity of 0m/s
"""

"""
Some Ideas:
    -Once the cfd field is in there, calculate the cfd field and wind velocity at every position upfront
      Doing this will make sure that you only need to add the wind velocity to the drone, making this 
      computationaly efficient.

    -Take good care of direction [0,0,0] and the body position relative to the body of the drone
      look at the transformation forumalas already inside pegasus
"""

class Wind:
    def __init__(self,wind_velocity=0.0,direction="N"):

        """
        Initialize the Wind instance.
        
        The wind_velocity is a 3D vector representing the wind's velocity in the global reference frame.
        In this reference frame:
        - The x-axis typically represents the east-west direction (positive towards the east).
        - The y-axis typically represents the north-south direction (positive towards the north).
        - The z-axis typically represents the up-down direction (positive upwards).
        
        :param wind_velocity: np.array, the wind velocity vector in the global reference frame.
                            For example, a wind_velocity of np.array([-7, 0, 0]) would represent
                            a wind blowing 7m/s southward and [0,5,0] 5m.s Eastward.
        """

        self.wind_velocity = wind_velocity
        self.direction = direction

        self._wind_vector = np.array([0.0,0.0,0.0])
        self._windmag = 0.0

    def calculate_wind_vector_components(self):
        """
        Calculate the wind vector's comoponents in the following form in [u,v,w] = [x,y,z]


        The components will be in the global frame. They thus have to be transformed to the drone's frame


        """
        dir = ["N","E", "S", "W", "NE", "SE", "SW", "NW"]
        idx = dir.index(self.direction)
        # Right hand coordinates thus Against the clock angles
        dir_angles = [0,90,180,-90, 45,135,-135,-45]

        angle = dir_angles[idx]
        angle = np.deg2rad(angle)

        

        self._wind_vector = np.array([self.wind_velocity * np.cos(angle)*-1, self.wind_velocity * np.sin(angle)*-1,0.0])
        return self._wind_vector, np.rad2deg(angle)

    def calculate_wind_velocity_at_position(self, postition):
        """
        Calculate the wind velocity at given position (Probably position of drone)

        :param position: np.array, the position at which to calculate the wind velocity.
        :return: np.array, the wind velocity at the given position.
        """

        # TODO incorpareate CFD field calculations if necessary 
        # for now, we'll assume the wind velocity is a constant throughout simulation 
        return self.wind_velocity
    
    def transform_to_body_frame(self, wind_velocity, drone_orientation):
        """
        Transform the wind velocity from the global frame to the drone's body frame.
        
        :param wind_velocity: np.array, the wind velocity in the global frame.
        :param drone_orientation: np.array or suitable type, the orientation of the drone.
        :return: np.array, the wind velocity in the drone's body frame.
        """

        # TODO: Implement the actual transformation using the drone's orientation.
        # This is a placeholder transformation, and you should replace it with the actual transformation formulas.
        return wind_velocity  # Replace with actual transformation

        
# wind = Wind(2.0, "W")
# print(wind.calculate_wind_vector_components())
