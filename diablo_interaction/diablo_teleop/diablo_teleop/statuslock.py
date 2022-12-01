#!/usr/bin/env python3
from http.client import OK
from lib2to3.pgen2.grammar import opmap_raw
import rclpy
import sys
import time
from threading import Thread
from rclpy.node import Node
import paho.mqtt.client as mqtt
from motion_msgs.msg import MotionCtrl

same_id = 1
break_id = 0
robot_num = 3

broker_ip = '127.0.0.1'
client = mqtt.Client()
pub_topic = "status/return"
keyQueue = []
loop_mark = False
ctrlMsgs = MotionCtrl()

def on_connect(client, userdata, flag, rc):
    global loop_mark
    if rc == 0:
        loop_mark = True
        print("Connection successful")
    elif rc == 1:
        print("Protocol version error")
    elif rc == 2:
        print("Invalid client identity")
    elif rc == 3:
        print("server unavailable")
    elif rc == 4:
        print("Wrong user name or password")
    elif rc == 5:
        print("unaccredited")
    print("Connect with the result code " + str(rc))

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection %s" % rc)
    client.connect(host=broker_ip, port=1883)


def on_message(client, userdata, msg):
    global keyQueue
    rec_msgs = msg.payload.decode()
    keyQueue.append(rec_msgs)
    if rec_msgs == "999":
        keyQueue = []
    elif(len(keyQueue) == robot_num):
        if len(set(keyQueue)) == 1:
            mqtt_publish(str(keyQueue[0]),pub_topic)
        else:
            mqtt_publish(str(break_id),pub_topic)
        keyQueue = []
     
rec_id = 0
def server_loop(task_id):
    global rec_id
    if rec_id == break_id:
        return break_id
        
    while(task_id!=rec_id):
        pass
    return task_id

def mqtt_subscribe():
    global client
    client.loop_forever()

def mqtt_publish(sensor_data, topic='xxxxxxxx', qos=0):
    global client
    try:
        client.publish(topic=topic, payload=sensor_data, qos=qos)
    except KeyboardInterrupt:
        print("EXIT")
        client.disconnect()
        sys.exit(0)

def mqtt_run(subTopic):
    # client.username_pw_set(USERNAME , PASSWD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.connect(host=broker_ip, port=1883)
    client.reconnect_delay_set(min_delay=1, max_delay=2000)
    client.subscribe(subTopic, qos=0)
    subscribe_thread = Thread(target=mqtt_subscribe)
    subscribe_thread.start()

def main(args=None):
    rclpy.init(args=args) 
    node = Node("status_lock_node") 
    mqtt_run("status/lock") 
    msg = MotionCtrl()
    while loop_mark:
        if len(keyQueue) > 0:
            key = keyQueue.pop(0)
            print(key)
            if key == '`':
                break
        else:
            ctrlMsgs.value.forward = 0.0
            ctrlMsgs.value.left = 0.0
        mqtt_publish("ctrlMsgs",pub_topic)
        time.sleep(0.04)
    
    print('exit!')
    rclpy.shutdown() 


