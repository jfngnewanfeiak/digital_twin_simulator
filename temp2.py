# === Isaac Sim: MQTTなし・最小デモ ===
# - /Root/conveyor/zone (Colliderだけ) 内のワークを +X へ等速搬送
# - /Root/work/piece をRigidBody化して使う（無ければ作成）
# - x > STOP_X で停止（簡易な“判定”の代わり）
# - ログ: ~/dt_min_no_mqtt.csv

import os, time, csv
import omni
from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema

STAGE = omni.usd.get_context().get_stage()

# ===== 調整パラメータ =====
WORK_PRIM = "/root/work/piece"          # 仮想ワークのPrim
ZONE_PRIM = "/root/conveyor/zone"       # 搬送ゾーン（Colliderのみ）
GROUND    = "/root/ground"              # 地面
CONVEYOR_VEL = 0.12                     # m/s（zone内で与える速度）
STOP_X       = 0.80                     # x がこの位置を超えたら停止
CSV_PATH     = os.path.expanduser("./logs/dt_min_no_mqtt.csv")

# === 小さなユーティリティ ===
def ensure_prim(path, type_name):
    prim = STAGE.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        prim = STAGE.DefinePrim(path, type_name)
    return prim

def ensure_collider_only(path, approx="none"):
    prim = ensure_prim(path, "Xform")  # 形は後で付ける
    # メッシュがない場合はCubeを付ける
    if prim.GetTypeName() != "Cube":
        prim = UsdGeom.Cube.Define(STAGE, path).GetPrim()
    # Collisionだけ付与（Rigid Bodyは付けない）
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
    # 念のためRigidbody外す
    try:
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    except Exception:
        pass
    return prim

def ensure_rigidbody(path, size=0.03):
    prim = STAGE.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        prim = UsdGeom.Cube.Define(STAGE, path).GetPrim()
        UsdGeom.Cube(prim).CreateSizeAttr(size)
        UsdGeom.XformCommonAPI(prim).SetTranslate((0.00, 0.00, size*1.1))
    # 衝突と剛体
    UsdPhysics.CollisionAPI.Apply(prim)
    PhysxSchema.PhysxCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
    UsdPhysics.RigidBodyAPI.Apply(prim)
    return prim

def set_transform(path, translate=None, scale=None, rotateXYZ=None):
    xf = UsdGeom.XformCommonAPI(STAGE.GetPrimAtPath(path))
    if translate is not None: xf.SetTranslate(translate)
    if scale is not None:     xf.SetScale(scale)
    if rotateXYZ is not None: xf.SetRotate(rotateXYZ)

def get_zone_aabb(path):
    prim = STAGE.GetPrimAtPath(path)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"]).ComputeWorldBound(prim)
    box = bbox.GetBox()
    return box.GetMin(), box.GetMax()

def get_work_pos(path):
    xf = UsdGeom.XformCommonAPI(STAGE.GetPrimAtPath(path))
    t, _, _, _ = xf.GetXformVectors()
    return float(t[0][0]), float(t[0][1]), float(t[0][2])

# === 1) 物理シーン（なければ作る） ===
# Gravity / Solver は既定でOK（必要なら /Root/PhysicsScene を置いて調整）

# === 2) 地面（Colliderのみ） ===
if not STAGE.GetPrimAtPath(GROUND):
    ground = UsdGeom.Cube.Define(STAGE, GROUND)
    ground.CreateSizeAttr(5.0)
    set_transform(GROUND, translate=(0,0,-0.01))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(ground.GetPrim()).CreateApproximationAttr("none")

# === 3) 搬送ゾーンを用意（Colliderのみ、見やすく配置） ===
zone_prim = ensure_collider_only(ZONE_PRIM, approx="none")
# ゾーンの実寸を設定：長さ1.0m x 幅0.3m x 高さ0.05m を /Root/conveyor のあたりに
UsdGeom.Cube(zone_prim).CreateSizeAttr(1.0)
set_transform(ZONE_PRIM, translate=(0.5, 0.0, 0.025), scale=(1.0, 0.3, 0.05))
# 半透明に（視認性向上、任意）
g = UsdGeom.Gprim(zone_prim)
g.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set([Gf.Vec3f(0.2, 0.9, 0.4)])
g.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant).Set([0.35])

# === 4) ワークをRigidBody化（なければ作成） ===
work_prim = ensure_rigidbody(WORK_PRIM, size=0.03)
# 初期位置（ゾーン手前・地面より少し上）
set_transform(WORK_PRIM, translate=(0.0, 0.0, 0.03))

# === 5) ログ準備 ===
if os.path.exists(CSV_PATH):
    try: os.remove(CSV_PATH)
    except Exception: pass
with open(CSV_PATH, "w", newline="") as f:
    csv.writer(f).writerow(["t","x","y","z","v_cmd","inside_zone"])

t0 = time.time()
rb_iface = omni.physx.get_physx_interface()
stopped = False

def on_update(e):
    global stopped
    # ゾーンAABBとワーク位置
    mn, mx = get_zone_aabb(ZONE_PRIM)
    x, y, z = get_work_pos(WORK_PRIM)
    inside = (mn[0] <= x <= mx[0]) and (mn[1] <= y <= mx[1]) and (mn[2] <= z <= mx[2])

    v_cmd = 0.0
    if not stopped:
        # シンプル停止条件：x > STOP_X で停止
        if x > STOP_X:
            stopped = True
        # ゾーン内だけ速度を与える
        if inside and not stopped:
            v_cmd = CONVEYOR_VEL
            try:
                rb_iface.set_rigid_body_linear_velocity(WORK_PRIM, Gf.Vec3f(v_cmd, 0.0, 0.0), True)
            except Exception:
                pass

    # ログ
    with open(CSV_PATH, "a", newline="") as f:
        csv.writer(f).writerow([f"{time.time()-t0:.3f}", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", f"{v_cmd:.3f}", int(inside)])

sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
print(f"[NO-MQTT] 最小デモ開始：zone内で +X へ搬送 / x>{STOP_X} で停止 / log={CSV_PATH}")
