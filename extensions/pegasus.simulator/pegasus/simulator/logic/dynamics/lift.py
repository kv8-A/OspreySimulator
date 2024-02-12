"""
| File: lift.py
| Description: Lift force on body
| License: BSD-3-Clause. Copyright (c) 2023. All rights reserved.
"""
import numpy as np
from pegasus.simulator.logic.dynamics.aerodynamics import Aerodynamics
from pegasus.simulator.logic.backends.controller.states import AngleOfAttack

from pegasus.simulator.logic.state import State
from scipy.io import loadmat
from scipy.interpolate import interp1d
import os
from pathlib import Path


class Lift(Aerodynamics):
    """
    Class that implements linear lift computations affecting a rigid body. It inherits the Lift base class.
    """

    def __init__(self, lift_coefficients=[1.2]):
        """
        Receives as input the lift coefficients of the vehicle as a 3x1 vector of constants

        Args:
            lift_coefficients (list[float]): The constant linear lift coefficients used to compute the total lift forces
            affecting the rigid body. The linear lift is given by diag(lx, ly, lz) * [v_x, v_y, v_z] where the velocities
            are expressed in the body frame of the rigid body (using the FRU frame convention).
        """

        # Initialize the Aerodynamics base class
        super().__init__()

        # The linear lift coefficients of the vehicle's body frame
        self._lift_coefficients = (lift_coefficients)
        self._air_density = 1.225
        # self._wind_surface = 32/4 *1.2 * 0.92
        
        # self._S = 0.211744609
        self._S = 0.2589
        # self.aoa = AngleOfAttack()

        # The lift force to apply on the vehicle's body frame
        self._lift_force = np.array([0.0, 0.0, 0.0])
        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

    @property
    def force(self):
        """The lift force to be applied on the body frame of the vehicle

        Returns:
            list: A list with len==3 containing the lift force to be applied on the rigid body according to a FLU body reference
            frame, expressed in Newton (N) [lx, ly, lz]
        """
        return self._lift_force

    def get_cl(self, alpha):
        # data = loadmat(self.curr_dir+'/C_l.mat')  # Load .mat file
        # cl_data = data['C_l'].ravel()  # Assuming 'C_l' is a 1D array in the .mat file

        # alpha_points = np.concatenate([np.arange(-8.5, 14, 0.25), [14.5, 14.75, 15]])

    
        # # Create an interpolation function based on the input data
        # f = interp1d(alpha_points, cl_data, kind='nearest', fill_value='extrapolate')

        # # Use the function to interpolate the input alpha
        # cl = f(alpha)

        # example :  cl-1.2 at 10 deg, cl = 0.14 at 0deg. alphazero = -1.5 deg --> a0 = (1.2-0.14)/(10-0) = 0.106
        # a = a0/(1+57.3*a0/(pi e AR)) = 0.088 per degree
        a = 0.088 # per deg 
        alphazero = -1.5 # deg

        cl = a * (alpha - alphazero) 


        if alpha > 12:
            cl = 1.2

        return cl
    
    def update(self, state: State, dt: float):  # add angle of attack
        """Method that updates the lift force to be applied on the body frame of the vehicle. The total lift force
        applied on the body reference frame (FLU convention) is given by diag(lx,ly,lz) * R' * v
        where v is the velocity of the vehicle expressed in the inertial frame and R' * v = velocity_body_frame

        Args:
            state (State): The current state of the vehicle.
             dt (float): The time elapsed between the previous and current function calls (s).

        Returns:
            list: A list with len==3 containing the lift force to be applied on the rigid body according to a FLU body reference
        """

        # Get the velocity of the vehicle expressed in the body frame of reference
        body_vel = state.linear_body_velocity
        euler_angle = state.attitude_eul[0]
        euler_angle = np.rad2deg(euler_angle)*-1
        groundspeed = body_vel[0]
        
        airspeed = self.caculate_airspeed(groundspeed,euler_angle)


        # # with open(self.curr_dir+'/wind_surface.txt', 'r') as f:
        # #     content = f.read()
        # # self._wind_surface = float(content)
        # # with open(self.curr_dir+'/lift_coeff.    alpha = self.aoa.get_aoa()


        # # self._lift_coefficients = float(content)

        # if airspeed < 0:
        #     airspeed = 0
        aoa = AngleOfAttack() # TODO check if most efficient 
        # alpha = self.aoa.get_aoa()
        alpha = aoa.get_aoa()
        print("alpha: ", alpha)

        cl = self.get_cl(alpha)
        print("cl: ", cl)

        # lift = 0.5*self._lift_coefficients[0] * self._air_density * self._S * (airspeed**2)

        lift = 0.5*cl * self._air_density * self._S * (airspeed**2)


        # if lift > 4.5*9.81:
        #     lift = 2.5*9.81
    
        self._lift_force = [0, 0, lift]
        return self._lift_force