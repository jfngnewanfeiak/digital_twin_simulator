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
work_position = None
work_yaw = None
ev3_data = []
deguti_flag = False

angles = [0,5,10,15,20,25]
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
        global work_position
        global work_yaw
        if timeline_start != None:
            work_positions.append(json.loads(msg.payload))
            work_positions[-1]['timestamp'] = time.time() - timeline_start
            work_position = work_positions[-1]['coordinate']
            work_yaw = np.rad2deg(work_positions[-1]['yaw']) - 90
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
            

def get_or_add_rotatexyz(prim_path:str,stage):
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            return op
    return xf.AddRotateXYZOp()

def inside_ratio(work_bbox):
    wx1 = work_bbox.GetMin()[0]
    wx2 = work_bbox.GetMax()[0]
    wy1 = work_bbox.GetMin()[1]
    wy2 = work_bbox.GetMax()[1]
    gx1 = -0.043985875513367224
    gx2 = -0.043985875513367224
    gy1 = -0.24668227432645462
    gy2 = -0.1766822743264546

    ix1, ix2 = max(wx1, gx1), min(wx2, gx2)
    iy1, iy2 = max(wy1, gy1), min(wy2, gy2)

    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih

    w_area = max(0.0, (wx2 - wx1)) * max(0.0, (wy2 - wy1))
    if w_area <= 0:
        return 0.0
    return inter / w_area  # 1.0ならworkが完全にゴール内



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
limit = -0.21168 # 整列用の終わりをここにする
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

# 整列用のやつ
left_op = get_or_add_rotatexyz('/root/left_m_rotation_axis',stage)
right_op = get_or_add_rotatexyz('root/right_m_rotation_axis',stage)
mperu = float(UsdGeom.GetStageMetersPerUnit(stage))
work = RigidPrim('/root/Work')
mag_attr = scene.GetGravityMagnitudeAttr()
dir_attr = scene.GetGravityDirectionAttr()

print("mag time samples:", mag_attr.GetTimeSamples())
print("dir time samples:", dir_attr.GetTimeSamples())

score_list = []

for right_angle in angles:
    for left_angle in angles:

        # データ同期 最初の一回だけ
        # シミュレーションの初期化
        # シミュレーション開始
        # zoneのvelocity
        zone_surf_attr.Set(Gf.Vec3f(0,zone_velocity/mperu,0))
        # 整列用のやつ角度変更
        right_op.Set((0,0,right_angle))
        left_op.Set((0,0,left_angle))
        for i in range(steps):
            simulation_app.update()
            
        # limitに来たら,整列の可否を計算→ break
        # ここのlimitはまた後で
        if limit >= get_world_transform_matrix(work_prim)[3][1]:
            bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),["default"])
            bbox = bbox.ComputeWorldBound(work_prim)
            bbox = bbox.ComputeAlignedBox()
            zone_surf_attr.Set(Gf.Vec3f(0,0,0))
            # time.sleep(5)
            print(get_world_transform_matrix(work_prim))
            
            current_work_pos = get_world_transform_matrix(work_prim)
            work.set_world_pose(position=[init_work_pos[3][0],init_work_pos[3][1],init_work_pos[3][2]])
            print(get_world_transform_matrix(work_prim))
            # 整列のスコアを計算
            score = inside_ratio(bbox)
            score_list.append(score)
            break

for i in range(steps):
    zone_surf_attr.Set(Gf.Vec3f(0,zone_velocity / mperu,0))
    simulation_app.update()
    if deguti_flag:
        pass
    
    if limit >= get_world_transform_matrix(work_prim)[3][1]:
        zone_surf_attr.Set(Gf.Vec3f(0,0,0))

        # time.sleep(5)
        print(get_world_transform_matrix(work_prim))
        
        current_work_pos = get_world_transform_matrix(work_prim)
        work.set_world_pose(position=[init_work_pos[3][0],init_work_pos[3][1],init_work_pos[3][2]])
        print(get_world_transform_matrix(work_prim))
        for n in range(10):
            simulation_app.update()
            # time.sleep(1)
        break
    time.sleep(sim_dt)
timeline.stop()
print("finish...")