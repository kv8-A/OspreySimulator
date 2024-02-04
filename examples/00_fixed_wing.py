#!/usr/bin/env python
"""
| File: 4_python_single_vehicle.py
| Author: Marcelo Jacinto and Joao Pinto (marcelo.jacinto@tecnico.ulisboa.pt, joao.s.pinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: This files serves as an example on how to use the control backends API to create a custom controller 
for the vehicle from scratch and use it to perform a simulation, without using PX4 nor ROS.
"""

# Imports to start Isaac Sim from this script
import carb
from omni.isaac.kit import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from omni.isaac.core.world import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.vehicles.fixedwing import Fixedwing, FixedwingConfig
# from pegasus.simulator.logic.vehicles.vehicle import Vehicle
from pegasus.simulator.logic.vehicles.vehicle_new import Vehicle

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# Import the custom python control backend
from utils.nonlinear_controller import NonlinearController

# Auxiliary scipy and numpy modules
from scipy.spatial.transform import Rotation

# Use os and pathlib for parsing the desired trajectory from a CSV file
import os
from pathlib import Path


class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        # self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Default Environment"])


        # Get the current directory used to read trajectories and save results
        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

        config_fixedwing = FixedwingConfig()
        config_fixedwing.backends = []
   
        Fixedwing(
            "/World/fixedwing",
            ROBOTS["Iris2"],
            0,
            [2.3, -1.5, 10],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_fixedwing,
        )
        # Fixedwing(
        #     "/World/quadrotor1",
        #     ROBOTS['fixed-wing_new'],
        #     0,
        #     [2.3, -1.5, 0.07],
        #     Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
        #     config=config_fixedwing,
        # )

        # add_reference_to_stage(usd_path=ROBOTS["fixed-wing"], prim_path="/World/Fancy_Robot2")
        # self.world.scene.add(Robot(prim_path="/World/Fancy_Robot2", name="fancy_robot2"))


        # Vehicle(stage_prefix="/World/fixedwing", usd_path=ROBOTS['fixed-wing'])
        

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running():

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
