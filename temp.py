# === Isaac Sim: 外部(MQTT)から受けた位置/姿勢・速度でワークとコンベアを同期する最小サンプル ===
import json, threading, time, os, csv
import omni
from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema
import paho.mqtt.client as mqtt

STAGE = omni.usd.get_context().get_stage()

# ==== 設定 ====
WORK_PRIM = "/root/Work/"             # あなたのワークPrim
ZONE_PRIM = "/root/conveyor/zone"          # コンベア領域Prim（Colliderのみ）
BROKER    = "192.168.11.29"                    # MQTTブローカ
TOPIC_POSE= "dt/work1/pose"                # {"x":..,"y":..,"z":..,"yaw_deg":..}
TOPIC_VEL = "dt/conveyor/vel"              # {"conveyor_vel":0.10}
CSV_PATH  = os.path.expanduser("~/log/dt_sync_log.csv")

# 既定値
_pose = {"x":0.0, "y":0.0, "z":0.03, "yaw_deg":0.0}
_conveyor_vel = 0.10

# ==== MQTT ====
def on_message(client, userdata, msg):
    global _pose, _conveyor_vel
    try:
        data = json.loads(msg.payload.decode("utf-8"))
        if msg.topic == TOPIC_POSE:
            for k in ("x","y","z","yaw_deg"):
                if k in data: _pose[k] = float(data[k])
        elif msg.topic == TOPIC_VEL:
            if "conveyor_vel" in data:
                _conveyor_vel = float(data["conveyor_vel"])
    except Exception as e:
        print("MQTT parse err:", e)

def mqtt_thread():
    c = mqtt.Client()
    c.on_message = on_message
    c.connect(BROKER, 1883, 60)
    c.subscribe([(TOPIC_POSE,0),(TOPIC_VEL,0)])
    c.loop_forever()

thr = threading.Thread(target=mqtt_thread, daemon=True)
thr.start()

# ==== 便利関数 ====
def ensure_collider_only(path):
    prim = STAGE.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found: {path}")
    # Collider 付与
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateApproximationAttr("none")
    # 剛体を外す（付いていたら）
    try:
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    except Exception:
        pass

def ensure_rigidbody(path):
    prim = STAGE.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found: {path}")
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    UsdPhysics.RigidBodyAPI.Apply(prim)

def set_pose_xyzyaw(path, x,y,z, yaw_deg):
    prim = STAGE.GetPrimAtPath(path)
    xf = UsdGeom.XformCommonAPI(prim)
    xf.SetTranslate((x,y,z))
    # yawはZ軸回り
    xf.SetRotate((0.0, 0.0, yaw_deg))

def get_zone_aabb(path):
    prim = STAGE.GetPrimAtPath(path)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(prim)
    b = bbox.GetBox(); return b.GetMin(), b.GetMax()

# 初期整備
ensure_collider_only(ZONE_PRIM)
ensure_rigidbody(WORK_PRIM)

# ログ準備
if os.path.exists(CSV_PATH):
    os.remove(CSV_PATH)
with open(CSV_PATH, "w", newline="") as f:
    csv.writer(f).writerow(["t","x","y","z","yaw_deg","v_cmd","inside_zone"])

rb_iface = omni.physx.get_physx_interface()

t0 = time.time()

def on_update(e):
    # 1) 実機から来た姿勢をそのまま適用
    set_pose_xyzyaw(WORK_PRIM, _pose["x"], _pose["y"], _pose["z"], _pose["yaw_deg"])

    # 2) zone 内なら等速で押す（外部コマンド _conveyor_vel）
    mn, mx = get_zone_aabb(ZONE_PRIM)
    # ワークの現在位置（translateを再取得）
    p = UsdGeom.XformCommonAPI(STAGE.GetPrimAtPath(WORK_PRIM)).GetXformVectors()[0]  # translate only
    x,y,z = p[0][0], p[0][1], p[0][2]
    inside = (mn[0] <= x <= mx[0]) and (mn[1] <= y <= mx[1]) and (mn[2] <= z <= mx[2])

    if inside:
        # 速度上書き（X方向へ搬送）
        try:
            rb_iface.set_rigid_body_linear_velocity(WORK_PRIM, Gf.Vec3f(float(_conveyor_vel),0.0,0.0), True)
        except Exception as ex:
            pass

    # 3) ログ
    with open(CSV_PATH, "a", newline="") as f:
        csv.writer(f).writerow([f"{time.time()-t0:.3f}", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}",
                                f"{_pose['yaw_deg']:.2f}", f"{_conveyor_vel:.3f}", int(inside)])

sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
print(f"[DT] MQTT同期開始。pose:{TOPIC_POSE}, vel:{TOPIC_VEL}, log:{CSV_PATH}")
