import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.utils.stage import get_current_stage
import omni.isaac.core.utils.numpy.rotations as rot_utils
from pxr import Gf

import time 
import random
import json
import os
import numpy as np
from typing import Union, List, Tuple
from scipy.spatial.transform import Rotation as R

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, DEFAULT_WORLD_SETTINGS, CONFIG_FILE

class SyntheticDataRecorder:
    def __init__(self, usd_environment_path: str) -> None:
        self.output_dir = "/home/kjell/Documents/Repositories/PegasusSimulator/synthetic_data_generation/output"
        self._world = World()
        stage_prefix = "/World/Layout"

        # Load the USD environment into the stage
        if self._world.stage.GetPrimAtPath(stage_prefix):
            raise Exception("A primitive already exists at the specified path")

        prim = self._world.stage.DefinePrim(stage_prefix)
        success = prim.GetReferences().AddReference(usd_environment_path)
        self._world.reset()

        # Set up camera variables from config # TODO
        self.focal_length = 24
        self.focus_distance = 400
        self.horizontal_aperture = 20.955
        # self.vertical_aperture = 20.955
        self.vertical_aperture = 15.2908
        self.image_width = 720
        self.image_height = 512
        # Set up the Replicator for synthetic data generation
        self.setup_replicator()

    def setup_replicator(self):
        with rep.new_layer():
            # Create a camera
            # position = (0,0,0)
            # rotation = (0,0,0)
            self.camera = rep.create.camera(
                # position= position,
                # rotation= rotation,
                focal_length=self.focal_length,
                focus_distance= self.focus_distance,
                horizontal_aperture= self.horizontal_aperture,
                # rotation= rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True)
                # vertical_aperture =5.11
            )

            # self._save_camera_pose_intrinsics(position, rotation, 0)
            # Create a render product to capture images from the camera
            self.render_product = rep.create.render_product(self.camera, (self.image_width, self.image_height))

            # Initialize the writer to save RGB and depth images
            self.writer = rep.WriterRegistry.get("BasicWriter")
            self.writer.initialize(
                # output_dir="synthetic_data_generation/output",
                output_dir=self.output_dir,
                rgb=True,
                distance_to_camera=True,
                pointcloud=True,
                pointcloud_include_unlabelled=True,
                # distance_to_image_plane=True,
                camera_params=True
            )
            self.writer.attach([self.render_product])
    
    def _get_camera_intrinsics(self) ->dict:
        # Focal lengths in pixels
        fx = (self.focal_length * self.image_width) / self.horizontal_aperture
        fy = (self.focal_length * self.image_height) / self.vertical_aperture
        # Principal point (assuming center)
        cx = self.image_width / 2
        cy = self.image_height / 2
        return {
            'fx': fx,
            'fy': fy,
            'cx': cx,
            'cy': cy
        }

    def _save_camera_pose_intrinsics(
            self,
            position: Union[Tuple[float, float, float], List[Tuple[float, float, float]]], 
            rotation: Union[Tuple[float, float, float], List[Tuple[float, float, float]]], 
            camera_idx: int) -> None:

        intrinsics = self._get_camera_intrinsics()

        if isinstance(position, list) and isinstance(rotation, list):
        # Iterate over the lists and save each pose
            for idx, (pos, rot) in enumerate(zip(position, rotation), start=camera_idx):
                camera_pose_data = {
                    'position': pos,
                    'rotation': rot,
                    'intrinsics': intrinsics
                }
                camera_pose_path = os.path.join(self.output_dir, f'camera_pose_{idx:04d}.json')
                with open(camera_pose_path, 'w') as f:
                    json.dump(camera_pose_data, f)
        else:
            # Save single position and rotation
            camera_pose_data = {
                'position': position,
                'rotation': rotation,
                'intrinsics': intrinsics
            }
            camera_pose_path = os.path.join(self.output_dir, f'camera_pose_{camera_idx:04d}.json')
            with open(camera_pose_path, 'w') as f:
                json.dump(camera_pose_data, f)


    def _generate_camera_pose(self, number_of_frames:int):
        camera_positions = [
            (random.uniform(-50, 50),random.uniform(-50, 50), random.uniform(0, 60)) for _ in range(number_of_frames)
            ]
        camera_rotations = [
            (0,0, random.uniform(0, 360)) for _ in range(number_of_frames)
            ]
        return camera_positions, camera_rotations

    def run_different_cameras(self,positions,rotations):
        with self.camera:
            pose_modifier = rep.modify.pose(
                position= rep.distribution.sequence(positions),
                rotation= rep.distribution.sequence(rotations)   
            )
            


    def record(self, number_of_frames: int):
        rep.orchestrator.step()
        rep.orchestrator.run()
    
        # Predefined positions and rotations random soar env
        # predefined_positions = [(0, 0, 30), (-20, -20, 30), (-80, -60, 30), (-50, 62, 40)]
        # predefined_rotations = [(0, -90, 0), (0, 0, 225), (0, 0, 215), (0, -90, 0)]
        # # Predefined Positions for campus env
        predefined_positions = [(0, 0, 30),(78, 625, 62)]
        predefined_rotations = [(0,-90,0), (0, 0, 50)]
        # positions, rotations = self._generate_camera_pose(number_of_frames)
        num_predefined = len(predefined_positions)
        num_generated = max(0, number_of_frames - num_predefined)
        
        generated_positions, generated_rotations = self._generate_camera_pose(num_generated)
        
        # Concatenate predefined with generated poses
        positions = predefined_positions + generated_positions
        rotations = predefined_rotations + generated_rotations

        self._save_camera_pose_intrinsics(positions, rotations, 1)
        with rep.trigger.on_frame(num_frames=number_of_frames):
            self.run_different_cameras(positions, rotations)
            while simulation_app.is_running():
                # print("Running")
                self._world.step(render=True)

        carb.log_warn("Simulation stopped")
        simulation_app.close()

def main():
    # recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS["TU_Delft"])
    recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS["Random_world_test"])
    # recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS["Random_world"])
    recorder.record(number_of_frames=4)
    # recorder.record(number_of_frames=40)
    # recorder.record(number_of_frames=2)

if __name__ == "__main__":
    main()
