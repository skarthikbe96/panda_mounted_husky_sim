# visualize_husky_panda.py
# Load a USD robot (Husky+Panda), add ground, set camera, and render a bit.

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})  # show the GUI

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import open_stage, add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view

# <<< 1) Point this to YOUR USD file >>>
WAREHOUSE_USD = "/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/panda_description/panda_model/warehouse.usd"
ROBOT_USD = "/home/rebellion/mobile_robotics/gz_start/robot_models/panda_arm_model/colcon_ws/src/panda_description/panda_model/panda_mounted_husky/panda_husky_isaac.usd"

# Where to place it in the stage (you can change the name if you like)
ROBOT_PRIM = "/World/Robot"

open_stage(WAREHOUSE_USD)

# 2) Make a simple world
world = World(stage_units_in_meters=1.0)
# world.scene.add_default_ground_plane()

# 3) Reference your USD into the stage
add_reference_to_stage(usd_path=ROBOT_USD, prim_path=ROBOT_PRIM)

# stage_units = get_stage_units()
# robot_xform = XFormPrim(ROBOT_PRIM)
# robot_xform.set_world_pose(
#     position=np.array([0.0, 0.0, 0.0]) / stage_units  # raise Z if it spawns inside the floor
# )

# 4) Put the camera in a nice spot
set_camera_view(
    eye=[8.0, 0.0, 3.0],
    target=[0.0, 0.0, 1.0],
    camera_prim_path="/OmniverseKit_Persp",
)

# 5) (Optional) move the whole robot if it spawns below/inside the ground
stage_units = get_stage_units()
# world.scene.get_object(PRIM_PATH).set_world_poses(positions=np.array([[0.0, 0.0, 0.0]]) / stage_units)

# 6) Initialize and render a few seconds so you can explore in the UI
world.reset()

print("Viewer running. Close the window or press Ctrl+C in the terminal to exit.")
try:
    while simulation_app.is_running():
        world.step(render=True)
except KeyboardInterrupt:
    pass

simulation_app.close()
