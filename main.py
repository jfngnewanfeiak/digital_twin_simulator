import os
from pathlib import Path
import numpy as np
from isaacsim import SimulationApp
import time

config = {
    "launch_config": {
        "renderer": "RayTracedLighting",
        "headless": False,
    },
 
    f"env_url": f"{Path.home()}/Documents/conveyor.usdc",
    "close_app_after_run": False
}

simulation_app = SimulationApp(launch_config=config["launch_config"])

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema,Gf
import carb
import omni
# Custom util functions for the example

from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import get_current_stage, open_stage
from omni.isaac.sensor import Camera
from omni.usd import get_world_transform_matrix
from omni.isaac.core.prims import XFormPrim
from omni.timeline import get_timeline_interface

# Open the given environment in a new stage
print(f"Loading Stage {config['env_url']}")
if not open_stage(config["env_url"]):
    carb.log_error(f"Could not open stage{config['env_url']}, closing application..")
    simulation_app.close()
stage = omni.usd.get_context().get_stage()
prim = stage.GetPrimAtPath("/root/conveyor/Zone")
zone_rigid_prim = UsdPhysics.RigidBodyAPI(prim)
zone_rigid_prim.CreateKinematicEnabledAttr().Set(True)

zone_surface_prim = PhysxSchema.PhysxSurfaceVelocityAPI(prim)
zone_surface_prim.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(0,0,0.2))

work_prim = stage.GetPrimAtPath('/root/Work')

sim_dt = 1 / 120
sim_time = 60
steps = int(sim_time / sim_dt)
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),["default"])
bbox = bbox.ComputeWorldBound(prim)
zone_min = bbox.GetBox().GetMin()
zone_max = bbox.GetBox().GetMax()

timeline = get_timeline_interface()
timeline.play()
for i in range(steps):
    simulation_app.update()
    time.sleep(sim_dt)
    print(get_world_transform_matrix(work_prim))
    

timeline.stop()
print()