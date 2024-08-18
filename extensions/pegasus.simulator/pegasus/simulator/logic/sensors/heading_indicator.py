"""
| File: barometer.py
| Author: Kjell Vleeschouwer
| License: BSD-3-Clause. Copyright (c) 2024, Kjell Vleeschouwer. All rights reserved.
| Description: 

"""
__all__ = ["HeadingIndicator"]


import numpy as np
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors import Sensor
from pegasus.simulator.logic.sensors.geo_mag_utils import GRAVITY_VECTOR


class HeadingIndicator(Sensor):
    """
    The class will impolement the HeadingIndicator for aircraft and fixed wing drones
    """

    def __init__(self,config={}):
        """
        Initialize the HeadingIndicator class

        Args:
            config(dict): Needs to contain the parameters for configuring the Heading Indicator
            It can be empty or only have some of the parameters



        """

        super().__init__(sensor_type="Heading Indicator", update_rate=config.get("update_rate", 250.0))

        # state of the sensor
        self._state = {"Heading:": 0.0}
        

    @property
    def state(self):
        """
        (dict) The 'state' of the sensor, i.e. the data produced by the sensor at any given point in time
        """
        return self._state

    @Sensor.update_at_rate
    def update(self, state:State, dt: float):
        """
        Method that will implement and return the heading indicator sensor.
        """

        # Get attitude angles of the vehicle in euler angles
        attitude_angle = state.attitude_eul[0]
        attitude_angle = np.rad2deg(attitude_angle)

        # transform the isaac attitude angles to aircraft heading angles
        if attitude_angle <= 0:
            heading_angle = attitude_angle * -1
        else:
            heading_angle = 360-attitude_angle

        self._state = {"Heading:": heading_angle}

        return self._state
    

