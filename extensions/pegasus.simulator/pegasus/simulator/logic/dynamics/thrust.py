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
    """
    def __init__(self, throttle=1.0):



        # Initialize the Aerodynamics base class 
        super().__init__()

        # Parameters
        self.throttle = throttle
        self._air_density = 1.225
        self._electric_power = 20 #W 
        self._propeller_efficiency = 0.9
        self._S_prop = 0.0314
        self._prop_velocity = 20

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


    def update(self, state: State, dt: float):

        linear_vel = state.linear_velocity[0]


        # prop_thrust = self._air_density * np.pi /4 * self._S_prop**2 *(linear_vel + 0.5 *self._prop_velocity)*self._prop_velocity
        prop_thrust = self._air_density *  self._S_prop *(linear_vel + 0.5 *self._prop_velocity)*self._prop_velocity

        f_thrust = self.throttle * prop_thrust
        self._thrust_force = [f_thrust,0.0, 0.0]
        return self._thrust_force
    