"""
| File: standard_plane.py
| Description: Definition of the StandardPlane class for simulating a fixed-wing aircraft based on the PX4 '1030_plane' model.
"""

from pegasus.simulator.logic.vehicles.fixed_wing import FixedWing, FixedWingConfig
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend

# Assuming a parameter file or module that defines the path to PX4 vehicle assets
from pegasus.simulator.params import PX4_VEHICLES

class StandardPlaneConfig(FixedWingConfig):
    def __init__(self):
        """
        Initialization of the StandardPlaneConfig class.
        """
        super().__init__()

        # Overriding the stage prefix and USD file for the standard plane
        self.stage_prefix = "fixed_wing"
        self.usd_file = PX4_VEHICLES["1030_plane"]  # Replace with the correct path to the '1030_plane' USD file

        # Maintaining the fixed-wing specific dynamics
        self.drag = LinearDrag([0.20, 0.20, 0.0])  # Example values, adjust as needed

        # Standard sensors for a fixed-wing aircraft
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # Backends for control and communication
        self.backends = [MavlinkBackend()]

class StandardPlane(FixedWing):
    def __init__(self, id: int, world, init_pos=[0.0, 0.0, 0.1], init_orientation=[0.0, 0.0, 0.0, 1.0], config=StandardPlaneConfig()):
        """
        Initialize a StandardPlane instance.

        Args:
            id (int): The unique identifier for the vehicle.
            world: The simulation world context.
            init_pos (list): Initial position of the vehicle in the world.
            init_orientation (list): Initial orientation of the vehicle.
            config (StandardPlaneConfig): The configuration for the standard plane.
        """
        super().__init__(id, world, init_pos, init_orientation, config=config)

        # Additional standard plane specific initializations can be added here

    # Here you can override or add methods specific to the standard plane's dynamics and control
    def update(self, delta_time):
        """
        Update logic for the standard plane.

        Args:
            delta_time (float): The time step for the simulation update.
        """
        # Implement the update logic specific to standard plane dynamics and control
        pass

    # Additional methods specific to the standard plane can be added here