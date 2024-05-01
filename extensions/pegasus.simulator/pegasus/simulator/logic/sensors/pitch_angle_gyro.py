"""
| File: pitch_angle_gyro.py
| Author: Kjell Vleeschouwer
| Description: File that calculates and returns the pitch angle of the vehicle and acts as 
|              sensor in the sim
"""
__all__ = ["FligthPathAngle"]

import numpy as np
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.geo_mag_utils import GRAVITY_VECTOR


class PitchAngle(Sensor):
    """
    The class will implement the pitch angle sensory for the fixed wing vehicle
    """

    def __init__(self, config={}):
        """
        Initialize the PitchAngle sensor class

        Args:
            config(dict): Needs to contain the parameter for configuring the pitch anle sensor
            It can be empty or only contain a few parameters, no specific number needed.
        """

        # Initialize sensor class
        super().__init__(sensor_type="Pitch Angle", update_rate=config.get("update_rate", 250.0))

        #State of the sensor
        self._state = {"pitch_angle": 0.0}

    
    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state
    
    @Sensor.update_at_rate
    def update(self, state:State, dt: float):
        aoa = state.angle_of_attack.get_aoa()

        vertical_velocity = state.linear_body_velocity[2]
        forward_velocity = state.linear_body_velocity[0]

        flight_path_angle = np.arctan(vertical_velocity/forward_velocity)

        pitch_angle = aoa + flight_path_angle
        
        self._state = {"pitch_angle": pitch_angle}
        return self._state

        
