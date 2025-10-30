import os
import omni
class Digital_Twin_Data:
    '''
    Isaacsim内部で使う、保持するデータ
    '''
    def __init__(self):
        self.WORK_PRIM = '/root/Work'               # work prim
        self.ZONE_PRIM = '/root/conveyor/zone'      # conveyor prim
        
        self.rb_iface = omni.physx.get_physx_interface() # コンベアの速度を上書き(mqtt用のコールバックを作成する)

        self.bbox_min = None   # zone内にワークがあるか確認するための変数
        self.bbox_max = None
        self.inside = None # ワークがゾーン内か判定フラグ
        self.x,y,z = None # ワークの位置

