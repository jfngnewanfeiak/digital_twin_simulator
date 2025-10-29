import os

class Config:
    '''
    MQTTのブローカipやprim pathを保存
    '''
    def __init__(self):
        self.WORK_PRIM = '/root/Work'               # work prim
        self.ZONE_PRIM = '/root/conveyor/zone'      # conveyor prim
        self.BROKER = '192.168.11.20'               # mqtt broker ip addr
        self.TOPIC_POSE = 'dt/work1/pose'           # x,y,z
        self.TOPIC_VEL = 'dt/conveyor/vel'          # conveyor_vel
        self.CSV_PATH = os.path.expanduser('./log/dt_sync_log.csv')
