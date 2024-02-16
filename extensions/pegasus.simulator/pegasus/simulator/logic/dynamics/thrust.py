"""
| File: thrust.py
| Description: Class that will create the Thrust on the body, dependente on Electric Power and throttle 
| License: BSD-3-Clause. Copytright (c) 2023. All rights reserved. 
"""


import numpy as np
from pegasus.simulator.logic.dynamics.aerodynamics import Aerodynamics
from pegasus.simulator.logic.state import State
from scipy.io import loadmat
from scipy.interpolate import interp1d
import os
from pathlib import Path


class Thrust(Aerodynamics):
    """
    Class the implements the thrust force on the rigigd body


    Model build upone the following source: https://www.grc.nasa.gov/www/k-12/airplane/propth.html
    """
    def __init__(self):



        # Initialize the Aerodynamics base class 
        super().__init__()

        # Parameters
        self.throttle = 0.6
        self._air_density = 1.225
        self._electric_power = 20 #W 
        self._propeller_efficiency = 0.9
        self._S_prop = 0.0314  # Sunyou code 
        self._prop_velocity = 20
        self._prop_rpm_max = 12000 # Source Maxon Motor 
        self._prop_k = 0.025

        

        self._thrust_force = np.array([0.0, 0.0, 0.0])


    @property
    def force(self):
        return self._thrust_force
    


    """
    Build in a rpm curve for the thurst. Motor can only give certain RPM... Throttle influences RPM
    RPM is directly in Thrust equation

    Check values with Sunyou her code
    """
    #TODO check values of thrust with code sunyou

    def propellor_velocity(self,throttle, max_rpm):
        
        omega = max_rpm * np.pi /30 * throttle
        Ve = omega * np.sqrt(self._S_prop/np.pi) 

        return Ve


    def update(self, state: State, dt: float, throttle):

        linear_vel = state.linear_velocity[0]

        body_vel = state.linear_body_velocity
        euler_angle = state.attitude_eul[0]
        euler_angle = np.rad2deg(euler_angle)*-1
        groundspeed = body_vel[0]

        Va = self.caculate_airspeed(groundspeed,euler_angle)

        # Ve = self.propellor_velocity(self.throttle, self._prop_rpm_max)
        Ve = self.propellor_velocity(throttle, self._prop_rpm_max)

        # prop_thrust = self._air_density * np.pi /4 * self._S_prop**2 *(linear_vel + 0.5 *self._prop_velocity)*self._prop_velocity
        # prop_thrust = self._air_density *  self._S_prop *(linear_vel + 0.5 *self._prop_velocity)*self._prop_velocity


        # f_thrust = self.throttle * prop_thrust

        f_thrust = 0.5 * self._air_density * self._S_prop * (Ve**2 - Va**2) * self._prop_k
        self._thrust_force = [f_thrust,0.0, 0.0]
        return self._thrust_force
    

    """
    Test thrust in certain cases --> Look of Newtons match --> Max N at around 10N. Check withouth wind applied and
    check start thrust at 100% throttle. 

    Sources:
    - https://www.flitetest.com/articles/review-of-ntm-prop-drive-26-28a-1200kv-motor
    - https://www.youtube.com/watch?v=MDDHCnIS-pc --> min 5.40

    """

