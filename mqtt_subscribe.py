#! /usr/bin/env python3

from paho.mqtt import client

class MQTT_SUB():
    def __init__(self, ip_addr:str, port:int, keep_alive:int ,topic:str ,
                 on_message: callable, on_connect:callable, on_disconnect:callable):
        self.client = client.Client()
        self.client.on_connect = on_connect
        self.client.on_disconnect = on_disconnect
        self.client.on_message = on_message
        self.ip_addr = ip_addr
        self.port = port
        self.keep_alive = keep_alive
        self.topic = topic

    
    def connect(self):
        self.client.connect(self.ip_addr,self.port,self.keep_alive)
    
    def subscribe(self):
        # import pdb; pdb.set_trace()
        # if not self.client.is_connected():
        #     raise Exception("\033[31m] エラー:クライアントとの接続が完了していません。connect()を実行してください")
        
        self.client.subscribe(self.topic)
        self.client.loop_start()
        


