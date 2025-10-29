import os
import time

class Logger:
    '''
    ロギングのためのクラス、将来的にはシミュレーション上のカメラデータmo
    '''
    def __init__(self):
        self.CAV_PATH = os.path.expanduser('./log/dt_sync_log.csv')
        self.t0 = None

    
    def timer_init(self):
        self.t0 = time.time()