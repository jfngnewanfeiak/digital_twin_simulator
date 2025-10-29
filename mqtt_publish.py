#! /usr/bin/env python3
from paho.mqtt import client

class MQTT_PUB():
    def __init__(self,ip_addr:str, port:int, topic:str,
                 on_publish: callable, on_connect: callable, on_disconnect: callable):
        
        self.ip_addr = ip_addr
        self.port = port
        self.topic = topic

        self.client = client.Client()
        self.client.on_connect = on_connect
        self.client.on_disconnect = on_disconnect
        self.client.on_publish = on_publish

        self.pub_msg = None
    
    def connect(self):
        self.client.connect(self.ip_addr,self.port,60)
        self.client.loop_start()
    

    def publish(self,data):
        self.pub_msg = data
        self.client.publish(self.topic,self.pub_msg)