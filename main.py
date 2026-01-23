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
from omni.isaac.core.utils.rotations import euler_angles_to_quat


MAX_SIM_TIME = 0.02
MAX_STEP = 300
elapsed = 0
sim_start_flag = True
timeline_start = None
work_positions = []
work_position = None
work_yaw = None
ev3_data = []
deguti_flag = False
stop_flag = True

angles = [0, 5, 10, 15, 20, 25]
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
        # if timeline_start != None:
        d = json.loads(msg.payload)
        # work_positions.append(json.loads(msg.payload))
        # work_positions[-1]['timestamp'] = time.time() - timeline_start
        # work_position = work_positions[-1]['coordinate']
        # work_yaw = np.rad2deg(work_positions[-1]['yaw']) - 90
        work_position = d['coordinate']
        work_yaw = np.rad2deg(d['yaw'])
    elif topic == 'ev3/data':
        global ev3_data
        global zone_velocity
        # if timeline_start != None:
        r = 0.02 # プーリ半径
        ev3_data.append(json.loads(msg.payload))
        if ev3_data[-1]['speed_deg'] != 0:
            # ev3_data[-1]['timestamp'] = time.time() - timeline_start
            omega_radps = ev3_data[-1]['speed_deg'] * np.pi / 180
            zone_velocity = omega_radps * r
    elif topic == 'deguti':
        global deguti_flag
        deguti_flag = True
    elif topic == 'work_stop':
        global stop_flag
        stop_flag = False

            

def get_or_add_rotatexyz(prim_path:str,stage):
    prim = stage.GetPrimAtPath(prim_path)
    xf = UsdGeom.Xformable(prim)
    for op in xf.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            return op
    return xf.AddRotateXYZOp()

# radから始原数に
def quat_xyzw_from_yaw_world(theta_rad):
    qd = Gf.Rotation(Gf.Vec3d(0,0,1),theta_rad).GetQuat()
    x,y,z = qd.GetImaginary()
    w = qd.GetReal()
    return (float(x),float(y),float(z),float(w))


def inside_ratio(work_bbox,goal_bbox):
    wx1 = work_bbox.GetMin()[0]
    wx2 = work_bbox.GetMax()[0]
    wy1 = work_bbox.GetMin()[1]
    wy2 = work_bbox.GetMax()[1]
    gx1 = goal_bbox.GetMin()[0]
    gx2 = goal_bbox.GetMax()[0]
    gy1 = goal_bbox.GetMin()[1]
    gy2 = goal_bbox.GetMax()[1]

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
                      ('deguti',0),('work_stop',0)],
               on_connect=None,on_disconnect=None, on_message=on_message)

pub = MQTT_PUB(ip_addr='192.168.11.20', port=1883, topic='ready',on_publish=None,on_connect=None,on_disconnect=None)
pub.connect()

best_angle_pub = MQTT_PUB(ip_addr='192.168.11.20',port=1883,topic='best_angle',on_publish=None,on_connect=None,on_disconnect=None)
best_angle_pub.connect()

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
goal_prim = stage.GetPrimAtPath('/root/goal')
goal_bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default'])
goal_bbox = goal_bbox.ComputeWorldBound(goal_prim)
goal_bbox = goal_bbox.ComputeAlignedBox()
sim_dt = 1 / 120
sim_time = 120
steps = int(sim_time / sim_dt)
bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),["default"])
bbox = bbox.ComputeWorldBound(prim)
bbox = bbox.ComputeAlignedBox()
limit = bbox.GetMin()[1]
limit = -0.16631 # 整列用の終わりをここにする
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

while stop_flag:
    time.sleep(0.001)

timeline = get_timeline_interface()
timeline.play()
t0 = timeline.get_current_time()
timeline_start = time.time()

# 整列用のやつ
left_op = get_or_add_rotatexyz('/root/left_m_rotation_axis',stage)
right_op = get_or_add_rotatexyz('/root/right_m_rotation_axis',stage)
mperu = float(UsdGeom.GetStageMetersPerUnit(stage))
work = RigidPrim('/root/Work')
# xf = UsdGeom.Xformable(work)
# rot_op = xf.AddRotateXYZOp()
mag_attr = scene.GetGravityMagnitudeAttr()
dir_attr = scene.GetGravityDirectionAttr()

print("mag time samples:", mag_attr.GetTimeSamples())
print("dir time samples:", dir_attr.GetTimeSamples())

score_list = []
yaw_list = []
use_yaw = None
limit_flag = False
pos_init,_ = work.get_world_pose()
for right_angle in angles:
    for left_angle in angles:
        # データ同期 最初の一回だけ

        # シミュレーション開始
        use_yaw = work_yaw
        
        # 整列用のやつ角度変更
        right_op.Set((-right_angle,0,0))
        left_op.Set((left_angle,0,0))
        # work.set_world_pose(position=pos_init,orientation=quat_xyzw_from_yaw_world(use_yaw))
        work.set_world_pose(position=pos_init,orientation=euler_angles_to_quat(np.array([0, 0, use_yaw]), degrees=True))
        # rot_op.Set((0, 0, use_yaw))

        print("*"*20)
        print(work.get_world_pose())
        print("*"*20)
        # zoneのvelocity
        zone_surf_attr.Set(Gf.Vec3f(0,zone_velocity/mperu,0))
        print("*"*20)
        print(f"zone velocity {zone_velocity}")
        print("*"*20)

        # while simulation_app.is_running():
        for i in range(MAX_STEP):
        # for i in range(steps):
            simulation_app.update()
            if i >= MAX_STEP:
                break
            
            # limitに来たら,整列の可否を計算→ break
            # ここのlimitはまた後で
            if limit >= get_world_transform_matrix(work_prim)[3][1]:
                # シミュレーションの初期化
                zone_surf_attr.Set(Gf.Vec3f(0,0,0))

                bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),["default"])
                bbox = bbox.ComputeWorldBound(work_prim)
                bbox = bbox.ComputeAlignedBox()

                # 整列のスコアを計算
                score = inside_ratio(bbox,goal_bbox)
                score_list.append(score)
                M = get_world_transform_matrix(work_prim)
                r00 = float(M[0][0])
                r10 = float(M[1][0])
                score_yaw = np.degrees(np.arctan2(r10,r00))
                yaw_list.append(score_yaw)
                limit_flag = True
                break
        
        # limit flag
        if limit_flag == False:
            score_list.append(0)
            yaw_list.append(None)
        else:
            limit_flag = False

# right_idx = int(score_list.index(max(score_list)) / len(angles))
# left_idx = score_list.index(max(score_list)) % len(angles)
# best_angle = {
#                         "right_angle": angles[right_idx],
#                         "left_angle": angles[left_idx]
#                     }
# best_angle_pub.publish(json.dumps(best_angle))
print(f"score_list {score_list}")
n_angle = len(angles)

sorted_indices = sorted(
    range(len(score_list)),
    key=lambda i: score_list[i],
    reverse=True
)[:3]

best_angles = []
for idx in sorted_indices:
    best_angles.append({
        "right_angle": angles[idx // n_angle],
        "left_angle":  angles[idx % n_angle],
        "score": score_list[idx]
    })
print(f"初期のyaw {use_yaw}")
print(f"score_list {score_list}")
print(f"sorted score list {sorted(range(len(score_list)),key=lambda i : score_list[i],reverse=True)}")
print(f"best_angles {best_angles}")
print(f"yaw list {yaw_list}")
best_angle_pub.publish(json.dumps(best_angles))
# for i in range(steps):
#     zone_surf_attr.Set(Gf.Vec3f(0,zone_velocity / mperu,0))
#     simulation_app.update()
#     if deguti_flag:
#         pass
    
#     if limit >= get_world_transform_matrix(work_prim)[3][1]:
#         zone_surf_attr.Set(Gf.Vec3f(0,0,0))

#         # time.sleep(5)
#         print(get_world_transform_matrix(work_prim))
        
#         current_work_pos = get_world_transform_matrix(work_prim)
#         work.set_world_pose(position=[init_work_pos[3][0],init_work_pos[3][1],init_work_pos[3][2]])
#         print(get_world_transform_matrix(work_prim))
#         for n in range(10):
#             simulation_app.update()
#             # time.sleep(1)
#         break
#     time.sleep(sim_dt)
timeline.stop()
print("finish...")