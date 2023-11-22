"""
| File: fixed_wing.py
| Description: Definition of the FixedWing class for simulating fixed-wing aircraft.
"""

import numpy as np
from pegasus.simulator.logic.vehicles.vehicle import Vehicle
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS

class FixedWingConfig:
    """
    A data class used for configuring a Fixed Wing aircraft.
    """

    def __init__(self):
        """
        Initialization of the FixedWingConfig class.
        """

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "fixed_wing"

        # The USD file that describes the visual aspect of the vehicle
        self.usd_file = ""  # Replace with the path to your fixed-wing model's USD file

        # Dynamics specific to fixed-wing aircraft
        self.drag = LinearDrag([0.25, 0.25, 0.0])  # Example values, adjust as needed

        # The default sensors for a fixed-wing aircraft
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # The backends for control and communication, using MAVLink by default
        self.backends = [MavlinkBackend()]

class FixedWing(Vehicle):
    """
    FixedWing class - Defines a base interface for fixed-wing aircraft in the simulation.
    """

    def __init__(self, id: int, world, init_pos=[0.0, 0.0, 0.1], init_orientation=[0.0, 0.0, 0.0, 1.0], config=FixedWingConfig()):
        """
        Initialize a FixedWing instance.

        Args:
            id (int): The unique identifier for the vehicle.
            world: The simulation world context.
            init_pos (list): Initial position of the vehicle in the world.
            init_orientation (list): Initial orientation of the vehicle.
            config (FixedWingConfig): The configuration for the fixed-wing aircraft.
        """
        super().__init__(config.stage_prefix, config.usd_file, id, world, init_pos, init_orientation, config=config)

        # Additional fixed-wing specific initializations can be added here

    def update(self, delta_time):
        """
        Update logic for the fixed-wing aircraft.

        Args:
            delta_time (float): The time step for the simulation update.
        """
        # Implement the update logic specific to fixed-wing aircraft dynamics and control
        pass

    # Additional methods specific to fixed-wing aircraft can be added here