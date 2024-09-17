import carb
from omni.isaac.kit import SimulationApp

# Create the simulation app with the given launch_config
# simulation_app = SimulationApp(launch_config=config["launch_config"])
simulation_app = SimulationApp({"headless": False})

# Custom util functions for the example
# import scene_based_sdg_utils

# Late import of runtime modules (the SimulationApp needs to be created before loading the modules)
from omni.isaac.core import World
import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.utils import prims
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import get_current_stage, open_stage
# from omni.isaac.nucleus import get_assets_root_path
from pxr import Gf

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS, DEFAULT_WORLD_SETTINGS, CONFIG_FILE


class SyntheticDataRecorder:
    def __init__(self, usd_environment_path:str) -> None:
        self._world = World()

        #TODO: Add the enviroment 
        stage_prefix = "/World/Layout"

        # self._world.scene.add_default_ground_plane()
        # Try to check if there is already a prim with the same stage prefix in the stage
        if self._world.stage.GetPrimAtPath(stage_prefix):
            raise Exception("A primitive already exists at the specified path")

        # Create the stage primitive and load the usd into it
        prim = self._world.stage.DefinePrim(stage_prefix)
        success = prim.GetReferences().AddReference(usd_environment_path)

        self._world.reset()

        # self.setup_replicator()

    def record(self):
        while simulation_app.is_running():
            self._world.step(render=True)
            simulation_app.update()
        carb.log_warn("Simulation stopped")
        simulation_app.close()

        
def main():
    # Create the SyntheticDataRecorder
    # recorder = SyntheticDataRecorder(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
    recorder = SyntheticDataRecorder(SIMULATION_ENVIRONMENTS["TU Delft"])
    # recorder = SyntheticDataRecorder()

    # Start the recording
    recorder.record()

if __name__ == "__main__":
    main()

