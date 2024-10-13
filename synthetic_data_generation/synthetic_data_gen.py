import carb
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": True})

from omni.isaac.core import World
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.utils.stage import get_current_stage
from pxr import Gf

import random
import json
import os
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

        # Set up the Replicator for synthetic data generation
        self.setup_replicator()


    def setup_replicator(self):
        with rep.new_layer():
            # Create a camera
            # self.camera = rep.create.camera()
            position = (-80,-60,30)
            rotation = (0,0,215)
            self.camera = rep.create.camera(
                position= position,
                rotation= rotation
            )

            self._save_camera_pose(position, rotation, 0)
            # Create a render product to capture images from the camera
            self.render_product = rep.create.render_product(self.camera, (512, 512))

            # Initialize the writer to save RGB and depth images
            self.writer = rep.WriterRegistry.get("BasicWriter")
            self.writer.initialize(
                # output_dir="synthetic_data_generation/output",
                output_dir=self.output_dir,
                rgb=True,
                distance_to_camera=True,
            )
            self.writer.attach([self.render_product])

    def _save_camera_pose(
            self,
            position: Union[Tuple[float, float, float], List[Tuple[float, float, float]]], 
            rotation: Union[Tuple[float, float, float], List[Tuple[float, float, float]]], 
            camera_idx: int) -> None:

        if isinstance(position, list) and isinstance(rotation, list):
        # Iterate over the lists and save each pose
            for idx, (pos, rot) in enumerate(zip(position, rotation), start=camera_idx):
                camera_pose_data = {
                    'position': pos,
                    'rotation': rot
                }
                camera_pose_path = os.path.join(self.output_dir, f'camera_pose_{idx:04d}.json')
                with open(camera_pose_path, 'w') as f:
                    json.dump(camera_pose_data, f)
        else:
            # Save single position and rotation
            camera_pose_data = {
                'position': position,
                'rotation': rotation
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
    
        # positions = [(-20, -20, 30), (-80, -60, 30)]
        # rotations = [(0, 0, 225), (0, 0, 215)]
        positions, rotations = self._generate_camera_pose(number_of_frames)
        self._save_camera_pose(positions, rotations, 1)

        with rep.trigger.on_frame(num_frames=number_of_frames):
            self.run_different_cameras(positions, rotations)
    
        carb.log_warn("Simulation stopped")
        simulation_app.close()

def main():
    # recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS["TU Delft"])
    recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS["Random_world_test"])
    # recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS["Random_world"])
    recorder.record(number_of_frames=5)
    # recorder.record(number_of_frames=2)

if __name__ == "__main__":
    main()
