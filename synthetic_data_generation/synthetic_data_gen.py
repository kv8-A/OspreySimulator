import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf

import random
from scipy.spatial.transform import Rotation as R

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, DEFAULT_WORLD_SETTINGS, CONFIG_FILE

class SyntheticDataRecorder:
    def __init__(self, usd_environment_path: str) -> None:
        self._world = World()
        stage_prefix = "/World/Layout"

        # Load the USD environment into the stage
        if self._world.stage.GetPrimAtPath(stage_prefix):
            raise Exception("A primitive already exists at the specified path")

        prim = self._world.stage.DefinePrim(stage_prefix)
        success = prim.GetReferences().AddReference(usd_environment_path)
        self._world.reset()

        # Set up the Replicator for synthetic data generation
        self.setup_replicator()

    def setup_replicator(self):
        with rep.new_layer():
            # Create a camera
            self.camera = rep.create.camera(
                position=(78.0,625,62), 
                rotation=(0.0,0.0,0.0)
            )

            # Create a render product to capture images from the camera
            self.render_product = rep.create.render_product(self.camera, (512, 512))

            # Initialize the writer to save RGB and depth images
            self.writer = rep.WriterRegistry.get("BasicWriter")
            self.writer.initialize(
                # output_dir="synthetic_data_generation/output",
                output_dir="/home/kjell/Documents/Repositories/PegasusSimulator/synthetic_data_generation/output",
                rgb=True,
                distance_to_camera=True,
            )
            self.writer.attach([self.render_product])

            # Define randomization ranges for position and rotation
            self.x_range = (-500, 500)
            self.y_range = (-500, 500)
            self.z_range = (0, 40)
            self.rx_range = (-90, 90)
            self.ry_range = (-90, 90)
            self.rz_range = (-180, 180)


            # @rep.trigger.on_frame(num_frames=5)
            # def run_camera():
            #     pose_modifier = rep.modify.pose(
            #         position=(78.0,625,62),
            #         rotation=(0.0,0.0,0.0))
            #     # Apply the modifier to the camera
            #     pose_modifier([self.camera])

            # # Randomize camera pose every frame
            # @rep.trigger.on_frame(num_frames=1)
            # def randomize_camera():
            #     position = (
            #         random.uniform(*self.x_range),
            #         random.uniform(*self.y_range),
            #         random.uniform(*self.z_range),
            #     )
            #     rotation_euler = (
            #         random.uniform(*self.rx_range),
            #         random.uniform(*self.ry_range),
            #         random.uniform(*self.rz_range),
            #     )
            #     # Convert Euler angles to quaternion
            #     rotation = R.from_euler('xyz', rotation_euler, degrees=True)
            #     rotation_quat = rotation.as_quat()  # Returns (x, y, z, w)
            #     # Replicator expects quaternions in (w, x, y, z) order
            #     rotation_quat = (rotation_quat[3], rotation_quat[0], rotation_quat[1], rotation_quat[2])

            #     # Create the modifier
            #     pose_modifier = rep.modify.pose(translation=position, rotation=rotation_quat)
            #     # Apply the modifier to the camera
            #     pose_modifier([self.camera])

    def record(self):
        # while simulation_app.is_running():
        #     self._world.step(render=True)
        #     rep.orchestrator.step()
        #     simulation_app.update()
        # carb.log_warn("Simulation stopped")
        # simulation_app.close()
        # while simulation_app.is_running():
            # self._world.step(render=True)
        rep.orchestrator.step()
            # simulation_app.update()
        carb.log_warn("Simulation stopped")
        simulation_app.close()

def main():
    # Replace with your actual USD environment path
    recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS["TU Delft"])
    recorder.record()

if __name__ == "__main__":
    main()



# import carb
# from omni.isaac.kit import SimulationApp

# # Create the simulation app with the given launch_config
# # simulation_app = SimulationApp(launch_config=config["launch_config"])
# simulation_app = SimulationApp({"headless": False})

# # Custom util functions for the example
# # import scene_based_sdg_utils

# # Late import of runtime modules (the SimulationApp needs to be created before loading the modules)
# from omni.isaac.core import World
# import omni.replicator.core as rep
# import omni.usd
# from omni.isaac.core.utils import prims
# from omni.isaac.core.utils.rotations import euler_angles_to_quat
# from omni.isaac.core.utils.stage import get_current_stage, open_stage
# # from omni.isaac.nucleus import get_assets_root_path
# from pxr import Gf

# import random 

# from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, DEFAULT_WORLD_SETTINGS, CONFIG_FILE


# class SyntheticDataRecorder:
#     def __init__(self, usd_environment_path:str) -> None:
#         self._world = World()

#         #TODO: Add the enviroment 
#         stage_prefix = "/World/Layout"

#         # self._world.scene.add_default_ground_plane()
#         # Try to check if there is already a prim with the same stage prefix in the stage
#         if self._world.stage.GetPrimAtPath(stage_prefix):
#             raise Exception("A primitive already exists at the specified path")

#         # Create the stage primitive and load the usd into it
#         prim = self._world.stage.DefinePrim(stage_prefix)
#         success = prim.GetReferences().AddReference(usd_environment_path)

#         self._world.reset()
#  # Set up the Replicator for synthetic data generation
#         self.setup_replicator()

#     def setup_replicator(self):
#         with rep.new_layer():
#             # Create a camera
#             self.camera = rep.create.camera()
            
#             # Define randomization ranges for position and rotation
#             self.x_range = (-10, 10)
#             self.y_range = (-10, 10)
#             self.z_range = (0, 5)
#             self.rx_range = (-90, 90)
#             self.ry_range = (-90, 90)
#             self.rz_range = (-180, 180)
            
#             # Randomize camera pose every frame
#             @rep.trigger.on_frame(num_frames=100)
#             def randomize_camera():
#                 position = (
#                     random.uniform(*self.x_range),
#                     random.uniform(*self.y_range),
#                     random.uniform(*self.z_range),
#                 )
#                 rotation = (
#                     random.uniform(*self.rx_range),
#                     random.uniform(*self.ry_range),
#                     random.uniform(*self.rz_range),
#                 )
#                 with self.camera:
#                     rep.modify.pose(position=position, rotation=rotation)
            
#             # Create a render product to capture images from the camera
#             self.render_product = rep.create.render_product(self.camera, (512, 512))
            
#             # Initialize the writer to save RGB and depth images
#             self.writer = rep.WriterRegistry.get("BasicWriter")
#             self.writer.initialize(
#                 output_dir="synthetic_data_generation/output",
#                 rgb=True,
#                 depth=True,
#             )
#             self.writer.attach([self.render_product])

#     def record(self):
#         while simulation_app.is_running():
#             self._world.step(render=True)
#             # Step the Replicator orchestrator to process synthetic data generation
#             rep.orchestrator.step()
#             simulation_app.update()
#         carb.log_warn("Simulation stopped")
#         simulation_app.close()

# def main():
#     # Create the SyntheticDataRecorder
#     # recorder = SyntheticDataRecorder(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
#     recorder = SyntheticDataRecorder(SIMULATION_ENVIRONMENTS["TU Delft"])
#     # recorder = SyntheticDataRecorder()

#     # Start the recording
#     recorder.record()

# if __name__ == "__main__":
#     main()