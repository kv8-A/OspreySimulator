#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) # we can also run as headless.

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.types import ArticulationAction

import numpy as np
import carb
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot, RobotView
import carb
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.vehicles.vehicle import Vehicle

world = World()
world.scene.add_default_ground_plane()
# fancy_cube =  world.scene.add(
    # DynamicCuboid(
    #     prim_path="/World/random_cube",
    #     name="fancy_cube",
    #     position=np.array([0, 0, 1.0]),
    #     scale=np.array([0.5015, 0.5015, 0.5015]),
    #     color=np.array([0, 0, 1.0]),
    # ))

# assets_root_path = get_assets_root_path()
# if assets_root_path is None:
#     # Use carb to log warnings, errors and infos in your application (shown on terminal)
#     carb.log_error("Could not find nucleus server with /Isaac folder")
# asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
# # This will create a new XFormPrim and point it to the usd file as a reference
# # Similar to how pointers work in memory
# add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
#     # Wrap the jetbot prim root under a Robot class and add it to the Scene
# jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
#         # Note: before a reset is called, we can't access information related to an Articulation
#         # because physics handles are not initialized yet. setup_post_load is called after
#         # the first reset so we can do so there
# print("Num of degrees of freedom before first reset: " + str(jetbot_robot.num_dof)) # prints None


#### FIXED WING ADD
add_reference_to_stage(usd_path=ROBOTS["flying-wing"], prim_path="/World/Fancy_Robot2")
fixedwing_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot2", name="fancy_robot2"))

# add_reference_to_stage(usd_path=ROBOTS["fixed-wing"], prim_path="/World/Fancy_Robot3")
# fixedwing_robot2 = world.scene.add(RobotView(prim_path="/World/Fancy_Robot3", name="fancy_robot3"))


# dof_fix = fixedwing_robot.num_dof
# pos_fix = fixedwing_robot.get_joint_postitions()
# print(dof_fix)
# print(pos_fix)
# Print info


# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()

# dof = jetbot_robot.num_dof
# pos = jetbot_robot.get_joint_positions()
# print(dof)
# print(pos)

# dof_fix = fixedwing_robot.num_dof
# pos_fix = fixedwing_robot.get_joint_postitions()

# def send_robot_actions(jetbot_articulation_controller):
#     # Every articulation controller has apply_action method
#     # which takes in ArticulationAction with joint_positions, joint_efforts and joint_velocities
#     # as optional args. It accepts numpy arrays of floats OR lists of floats and None
#     # None means that nothing is applied to this dof index in this step
#     # ALTERNATIVELY, same method is called from self._jetbot.apply_action(...)
#     jetbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,
#                                                                         joint_efforts=None,
#                                                                         joint_velocities=5 * np.random.rand(2,)))
#     return

for i in range(500):
    # position, orientation = fancy_cube.get_world_pose()
    # linear_velocity = fancy_cube.get_linear_velocity()
    # # will be shown on terminal
    # print("Cube position is : " + str(position))
    # print("Cube's orientation is : " + str(orientation))
    # print("Cube's linear velocity is : " + str(linear_velocity))
    # we have control over stepping physics and rendering in this workflow
    # things run in sync

    # jetbot_articulation_controller = jetbot_robot.get_articulation_controller()
    # # Adding a physics callback to send the actions to apply actions with every
    # # physics step executed.

    # send_robot_actions= jetbot_articulation_controller.apply_action(ArticulationAction(joint_positions=None,joint_efforts=None,   joint_velocities=5 * np.random.rand(2,)))

    # # world.add_physics_callback("sending_actions", callback_fn=send_robot_actions(jetbot_articulation_controller))
    # world.add_physics_callback("sending_actions", callback_fn=send_robot_actions)
                                                                        
                                                                         
    world.step(render=True) # execute one physics step and one rendering step




# Keep the app running
while simulation_app.is_running():
    simulation_app.update()


# Shutdown and exit
# omni.timeline.get_timeline_interface().stop()

simulation_app.close() # close Isaac Sim