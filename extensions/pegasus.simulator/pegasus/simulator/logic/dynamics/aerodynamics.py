"""
| File: aerodynamics.py
| Author: Kjell Vleeschouwer
| Description: File is based on the "drag.py" file which is the base for the linear drag force
| License: BSD-3-Clause. Copyright (c) 2023. All rights reserved.
"""
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.atmosphere.wind import Wind
import numpy as np

class Aerodynamics:
    """
    Class that serves as a template for the implementation of the Lift and Drag forces for the fixed-wing
    """

    def __init__(self):
        """
        Receives as input the drag coefficients of the vehicle as a 3x1 vector of constants
        """
        # Check how to best typcheck this # TODO 
        self.wind_direction = "N"
        self.wind_velocity = 0.0

        self._wind = Wind(wind_velocity=self.wind_velocity, direction=self.wind_direction).calculate_wind_vector_components()[0]
        self._windangle = Wind(wind_velocity=self.wind_velocity, direction=self.wind_direction).calculate_wind_vector_components()[1]
    @property
    def aero_force(self):
        """The drag force to be applied on the body frame of the vehicle

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
            frame, expressed in Newton (N) [dx, dy, dz]
        """
        return [0.0, 0.0, 0.0]
    
    def caculate_airspeed(self, groundspeed:list, euler_angle:float, state:State) -> float: 
        """
        Method that caluclates the airspeed and performs the transformation to the aircraft velocity vector
        args:
            - groundspeed
            - euler_angle: East is 90 degrees, west is -90 degrees. south is 180/-180. North is 0
        """
        
        wind = self._wind   # wind in [x,y,z] --> in the world frame
        angle_wind = self._windangle
        
        ####################
        # groundspeed = np.sqrt(groundspeed[0]**2 +groundspeed[2]**2)
        # TODO get wind angle from 
            # Wind components in the world frame
        # Vx = wind[0]
        # Vy = wind[1]
        # Vz = wind[2]

        # Vxy_vector = np.sqrt(Vx**2 + Vy**2)

        # a_delta = angle_wind - euler_angle
        # if np.abs(a_delta) < 90:  # Headwind
        #     V_xy_windspeed = Vxy_vector * np.abs(np.cos(np.deg2rad(a_delta)))  # With a headwind, wind velocity is added to ground speed
        # elif np.abs(a_delta) > 90:  # Tailwind
        #     V_xy_windspeed = - Vxy_vector * np.abs(np.cos(np.deg2rad(a_delta)))
        # else: # if equal to 90 degrees, pure sidewind so no influence on airspeed. Do check this #TODO --> Checked
        #     V_xy_windspeed = 0 
        

        # vertical_velocity = state.linear_body_velocity[2]
        # forward_velocity = state.linear_body_velocity[0]
        
        # # calculate flight path angle 
        # gamma_rad = np.arctan(vertical_velocity/forward_velocity)

        # # Now perform transformation with flight path angle
        # V_xy_windspeed = V_xy_windspeed * np.cos(gamma_rad) #
        # V_z_windspeed = Vz * np.sin(gamma_rad)

        # V_windspeed = np.sqrt(V_xy_windspeed**2 + V_z_windspeed**2)
        
        # if V_xy_windspeed > 0: 
        #     V_airspeed = groundspeed + V_windspeed # Headwind
        # else:
        #     V_airspeed = groundspeed - V_windspeed # Tailwind


        # if V_airspeed < 0:
        #     V_airspeed= 0

        # return V_airspeed
        ################
        # First perform the transformation so the direction is the same
        # if diference in anle between attitude and wind direction is bigger then 180 degrees its headwind so positive
        # if lower its tailwind. thus Tailwind... 
        a_delta = angle_wind - euler_angle
        if np.abs(a_delta) < 90:  # Headwind
            airspeed = groundspeed + self.wind_velocity * np.abs(np.cos(np.deg2rad(a_delta)))  # With a headwind, wind velocity is added to ground speed
        elif np.abs(a_delta) > 90:  # Tailwind
            airspeed = groundspeed - self.wind_velocity * np.abs(np.cos(np.deg2rad(a_delta)))
        else: # if equal to 90 degrees, pure sidewind so no influence on airspeed. Do check this #TODO --> Checked
            airspeed = groundspeed 
    #     # TODO --> input wind speed from class Wind

    #     # now that direction is the same, take value from xyplane vector to V 
    #     # 
    #     # TODO --> Calculate true airspeed 
    #     # self._lift_coefficients = self.get_cl(pitch)
    #     # TODO Validation:
    #         # Check for the wind coming out of south, for 90 deg delta etc.  --> Checked
    #         # Check for the wind coming from the east for 60 deg heading, so NE kinda. --> Checked. (Headwind)
    #         # Check for the wind coming from the east for 240 deg heading, thus -120 so kindo f SW. --> Checked
    #         # Check for the wind coming form the eas for 300 deg heading, thus -60, so kinda NW --> Checked
    #         # Check for Western wind --> Checked
    #         # Check for Southern wind --> Checked

    # return airspeed
        if airspeed < 0:
            airspeed = 0
        
            
        return airspeed

    def update(self, state: State, dt: float):
        """Method that should be implemented to update the drag force to be applied on the body frame of the vehicle

        Args:
            state (State): The current state of the vehicle.
             dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
        """
        return [0.0, 0.0, 0.0]
