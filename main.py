import json,threading,time,os,csv
import omni
from pxr import USd,UsdGeom,Gf,UsdPhysics,PhysxSchema

import paho.mqtt.client as mqtt

stage = omni.usd.get_context().get_stage()

# settings...
WORK_PRIM = '/root/Work'               # work prim
ZONE_PRIM = '/root/conveyor/zone'      # conveyor prim
BROKER = '192.168.11.20'               # mqtt broker ip addr
TOPIC_POSE = 'dt/work1/pose'           # x,y,z
TOPIC_VEL = 'dt/conveyor/vel'          # conveyor_vel
CSV_PATH = os.path.expanduser('./log/dt_sync_log.csv')

# default value...
_pose = {'x':0.0,'y':0.0,"z":0.03,'yaw_deg':0.0}
_conveyor_vel = 0.10

