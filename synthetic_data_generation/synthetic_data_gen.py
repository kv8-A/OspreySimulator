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
            self.camera = rep.create.camera()
            # self.camera = rep.create.camera(
            #     position=(78.0,625,62), 
            #     rotation=(0.0,0.0,0.0)
            # )

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

    def run_different_cameras(self):
        self.camera = rep.create.camera()
        with self.camera:
            rep.modify.pose(position=rep.distribution.uniform((-50, -50, 0), (50, 50, 60)))

    #     return self.camera.node
        # pose_modifier = rep.modify.pose(
        #     position=(78.0,625,62),
        #     rotation=(0.0,0.0,0.0))
        # # Apply the modifier to the camera
        # pose_modifier([self.camera])

    def record(self):
        rep.orchestrator.step()
        rep.orchestrator.run()

        with rep.trigger.on_frame(num_frames=5):
            self.run_different_cameras()
        # rep.orchestrator.run()


        carb.log_warn("Simulation stopped")
        simulation_app.close()

def main():
    # Replace with your actual USD environment path
    recorder = SyntheticDataRecorder(usd_environment_path=SIMULATION_ENVIRONMENTS["TU Delft"])
    recorder.record()

if __name__ == "__main__":
    main()

