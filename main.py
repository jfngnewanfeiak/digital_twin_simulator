import os
from pathlib import Path
import numpy as np
import json
from isaacsim import SimulationApp
import time
from mqtt_publish import MQTT_PUB
from mqtt_subscribe import MQTT_SUB

config = {
    "launch_config": {
        "renderer": "RayTracedLighting",
        "headless": False,
    },
 
    f"env_url": f"{Path.home()}/Documents/conveyor122.usdc",
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
from omni.isaac.dynamic_control import _dynamic_control as dc
from omni.isaac.core.prims import RigidPrim
import omni.kit.actions.core


sim_start_flag = True
timeline_start = None
work_positions = []
ev3_data = []
deguti_flag = False
def on_message(client, userdata, msg):
    topic = msg.topic
    global timeline_start
    if topic == "real_feedback_data":
        data = msg.payload
        data = data.split(",")
    elif topic == "start_program":
        global sim_start_flag
        sim_start_flag = False
    elif topic == "work_position":
        global work_positions
        if timeline_start != None:
            work_positions.append(json.loads(msg.payload))
            work_positions[-1]['timestamp'] = time.time() - timeline_start
    elif topic == 'ev3/data':
        global ev3_data
        global zone_velocity
        if timeline_start != None:
            r = 0.02 # プーリ半径
            ev3_data.append(json.loads(msg.payload))
            ev3_data[-1]['timestamp'] = time.time() - timeline_start
            omega_radps = ev3_data[-1]['speed_deg'] * np.pi / 180
            zone_velocity = omega_radps * r
    elif topic == 'deguti':
        global deguti_flag
        deguti_flag = True
            
        


sub = MQTT_SUB(ip_addr='192.168.11.20', port=1883, keep_alive=60, 
               topic=[('real_feedback_data',0),('start_program',0),
                      ('work_position',0),('ev3/data',0),
                      ('deguti',0)],
               on_connect=None,on_disconnect=None, on_message=on_message)

pub = MQTT_PUB(ip_addr='192.168.11.20', port=1883, topic='ready',on_publish=None,on_connect=None,on_disconnect=None)
pub.connect()
sub.connect()
sub.subscribe()
# Open the given environment in a new stage
print(f"Loading Stage {config['env_url']}")
if not open_stage(config["env_url"]):
    carb.log_error(f"Could not open stage{config['env_url']}, closing application..")
    simulation_app.close()
stage = omni.usd.get_context().get_stage()
stage.SetEditTarget(stage.GetSessionLayer())
# PhysicsScene を取得
scene = UsdPhysics.Scene.Get(stage, "/root/PhysicsScene")
# 上書き
scene.GetGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1), Usd.TimeCode.Default())
scene.GetGravityMagnitudeAttr().Set(9.81, Usd.TimeCode.Default())
prim = stage.GetPrimAtPath("/root/Zone")
zone_rigid_prim = UsdPhysics.RigidBodyAPI(prim)
# zone_rigid_prim.CreateKinematicEnabledAttr().Set(False)

zone_velocity = 0 # global
zone_surface_prim = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
zone_rb = UsdPhysics.RigidBodyAPI(prim)
zone_rb.CreateRigidBodyEnabledAttr().Set(True)
zone_rb.CreateKinematicEnabledAttr().Set(True)
zone_surf_attr = zone_surface_prim.CreateSurfaceVelocityAttr()
zone_surf_attr.Set(Gf.Vec3f(0,zone_velocity,0))

work_prim = stage.GetPrimAtPath('/root/Work')

sim_dt = 1 / 120
sim_time = 120
steps = int(sim_time / sim_dt)
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),["default"])
bbox = bbox.ComputeWorldBound(prim)
bbox = bbox.ComputeAlignedBox()
limit = bbox.GetMin()[1]
limit = -0.5025 # 実機の方の終了と合わせた
# 照明
action_registry = omni.kit.actions.core.get_action_registry()

# switches to camera lighting
action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")

# switches to stage lighting
# action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_stage")

action.execute()

init_work_pos = get_world_transform_matrix(work_prim)
dci = dc.acquire_dynamic_control_interface()

pub.publish("ok")
print('wait for touch sensor....')
while sim_start_flag:
    time.sleep(0.001)

timeline = get_timeline_interface()
timeline.play()
timeline_start = time.time()


mperu = float(UsdGeom.GetStageMetersPerUnit(stage))
work = RigidPrim('/root/Work')
mag_attr = scene.GetGravityMagnitudeAttr()
dir_attr = scene.GetGravityDirectionAttr()

print("mag time samples:", mag_attr.GetTimeSamples())
print("dir time samples:", dir_attr.GetTimeSamples())

for i in range(steps):
    zone_surf_attr.Set(Gf.Vec3f(0,zone_velocity / mperu,0))
    simulation_app.update()
    if deguti_flag:
        pass
    
    # if limit >= get_world_transform_matrix(work_prim)[3][1]:
    #     zone_surf_attr.Set(Gf.Vec3f(0,0,0))
    #     # time.sleep(5)
    #     print(get_world_transform_matrix(work_prim))
        
    #     current_work_pos = get_world_transform_matrix(work_prim)
    #     work.set_world_pose(position=[init_work_pos[3][0],init_work_pos[3][1],init_work_pos[3][2]])
    #     print(get_world_transform_matrix(work_prim))
    #     for n in range(10):
    #         simulation_app.update()
    #         # time.sleep(1)
    #     break
    # time.sleep(sim_dt)
timeline.stop()
print("finish...")