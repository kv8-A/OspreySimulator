"""
| File: drag.py
| Description: Drag force on body
| License: BSD-3-Clause. Copyright (c) 2023. All rights reserved.
"""
import numpy as np
from pegasus.simulator.logic.dynamics.aerodynamics import Aerodynamics
from pegasus.simulator.logic.state import State
from scipy.io import loadmat
from scipy.interpolate import interp1d
import os
from pathlib import Path


class Drag(Aerodynamics):
    """
    Class that will serve as the drag forces acting on the linear body for a fixed wing 
    """    

    def __init__(self, drag_coefficient=[0.05]):
        """
        Receives as input the drag coefficients of the vehicle 

        Args:
            drag_coefficients (list[float]): 
        """

        # Inititialize Aerodynamics base class 
        super().__init__()

        self._drag_coefficient = (drag_coefficient)
        self._air_density = 1.225
        self._S = 0.2589

        self._drag_force = np.array([0.0, 0.0, 0.0])
        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

    @property
    def force(self):
        """The drag force to be applied on the body frame of the vehicle

        Returns:
            list: A list with len==3 containing the drag force to be applied on the rigid body according to a FLU body reference
            frame, expressed in Newton (N) [lx, ly, lz]
        """
        return self._drag_force


    """
    #TODO: get CD function
    """

    def update(self, state: State, dt: float):  # TODO make dependent on aoa

        body_vel = state.linear_body_velocity
        euler_angle = state.attitude_eul[0]
        euler_angle = np.rad2deg(euler_angle)*-1
        groundspeed = body_vel[0]
        
        airspeed = self.caculate_airspeed(groundspeed,euler_angle)

        drag = 0.5*self._drag_coefficient[0] * self._air_density * self._S * (airspeed**2)

        self._drag_force = [-drag,0, 0]

        return self._drag_force
    


