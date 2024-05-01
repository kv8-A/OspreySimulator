"""
| File: flight_path_sensor.py
| Author: Kjell Vleeschouwer
| Description: File that claculates and returns the fly path , acts as fly path sensor in the sim
"""
__all__ = ["FligthPathAngle"]

import numpy as np
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.geo_mag_utils import GRAVITY_VECTOR



class FlightPathAngle(Sensor):
    """
    The class will implement the FlightPathAngle sensor for the fixed wing vehicle
    """

    def __init__(self,config={}):
        """
        Inititalize the FlightPathAngle sensor class

        Args:
            config(dict): Needs to contain the parameter for configuring the Flightpah anle sensor
            It can be empty or only contain a few parameters, no specific number needed.
        """

        # Initialize sensor class
        super().__init__(sensor_type="Flight Path Angle", update_rate=config.get("update_rate", 250.0))

        #State of the sensor
        self._state = {"flight_path_angle": 0.0}

    @property
    def state(self): 
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state:State, dt: float):
        """
        Method that will implement, update and return the flight path angle sensor
        """

        # Get the vertical and forward velocities from the state
        vertical_velocity = state.linear_body_velocity[2]
        forward_velocity = state.linear_body_velocity[0]

        flight_path_angle = np.arctan(vertical_velocity/forward_velocity)

        self._state = {"flight_path_angle": flight_path_angle}
        return self._state

