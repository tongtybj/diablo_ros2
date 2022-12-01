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


broker_ip = '123.57.38.57'
client = mqtt.Client()
USERNAME = 'client_23aa64096ef84c0b95c9'
PASSWD = 'kEyf5b43dd3441b6a28bd7bbdfccd5e6'

keyQueue = []
ctrlMsgs = MotionCtrl()

def on_connect(client, userdata, flag, rc):
    if rc == 0:
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
    keyQueue.append(msg.payload.decode())

def mqtt_subscribe():
    global client
    client.loop_forever()

def mqtt_publish(sensor_data, topic='xxxxxxxx', qos=2):
    global client
    try:
        client.publish(topic=topic, payload=sensor_data, qos=qos)
    except KeyboardInterrupt:
        print("EXIT")
        client.disconnect()
        sys.exit(0)

def mqtt_run(subTopic):
    client.username_pw_set(USERNAME , PASSWD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    client.connect(host=broker_ip, port=1883)
    client.reconnect_delay_set(min_delay=1, max_delay=2000)
    client.subscribe(subTopic, qos=0)
    subscribe_thread = Thread(target=mqtt_subscribe)
    subscribe_thread.start()

def generMsgs(forward=None,left=None,roll=None,up=None,
                pitch=None,mode_mark=False,height_ctrl_mode = None,
                pitch_ctrl_mode = None,roll_ctrl_mode = None,stand_mode = None):
    global ctrlMsgs
    ctrlMsgs.mode_mark = mode_mark
    if forward is not None:
        ctrlMsgs.value.forward = forward
    if left is not None:
        ctrlMsgs.value.left = left
    if pitch is not None:
        ctrlMsgs.value.pitch = pitch
    if roll is not None:
        ctrlMsgs.value.roll = roll
    if up is not None:
        ctrlMsgs.value.up = up
    if height_ctrl_mode is not None:
        ctrlMsgs.mode.height_ctrl_mode = height_ctrl_mode
    if pitch_ctrl_mode is not None:
        ctrlMsgs.mode.pitch_ctrl_mode = pitch_ctrl_mode
    if roll_ctrl_mode is not None:
        ctrlMsgs.mode.roll_ctrl_mode = roll_ctrl_mode
    if stand_mode is not None:
        ctrlMsgs.mode.stand_mode = stand_mode

    
def main(args=None):
    rclpy.init(args=args) 
    node = Node("diablo_mqttop_node") 
    mqtt_run("client_23aa64096ef84c0b95c9/cmd") 
    msg = MotionCtrl()
    teleop_cmd = node.create_publisher(MotionCtrl,"diablo/MotionCmd",2)
    while True:
        if len(keyQueue) > 0:
            key = keyQueue.pop(0)
            print(key)
            if key == 'w':
                generMsgs(forward=1.0)
            elif key == 's':
                generMsgs(forward=-1.0)
            elif key == 'a':
                generMsgs(left=1.0)
            elif key == 'd':
                generMsgs(left=-1.0)
            elif key == 'e':
                generMsgs(roll=0.1)
            elif key == 'q':
                generMsgs(roll=-0.1)
            elif key == 'r':
                generMsgs(roll=0.0)

            elif key == 'h':
                generMsgs(up = -0.5)
            elif key == 'j':
                generMsgs(up = 1.0)
            elif key == 'k':
               generMsgs(up = 0.5)
            elif key == 'l':
               generMsgs(up = 0.0)
                
            elif key == 'u':
                generMsgs(pitch = 0.5)
            elif key == 'i':
                generMsgs(pitch = 0.0)
            elif key == 'o':
                generMsgs(pitch = -0.5)

            elif key == 'v':
                generMsgs(mode_mark=True,height_ctrl_mode=True)
            elif key == 'b':
                generMsgs(mode_mark=True,height_ctrl_mode=False)
            elif key == 'n':
                generMsgs(mode_mark=True,pitch_ctrl_mode=True)
            elif key == 'm':
                generMsgs(mode_mark=True,pitch_ctrl_mode=False)

            elif key == 'z':
                generMsgs(mode_mark=True,stand_mode=True)
                teleop_cmd.publish(ctrlMsgs)
                generMsgs(up=1.0)
                teleop_cmd.publish(ctrlMsgs)
            elif key == 'x':
                generMsgs(mode_mark=True,stand_mode=False)
                teleop_cmd.publish(ctrlMsgs)

            elif key == '`':
                break
        else:
            ctrlMsgs.value.forward = 0.0
            ctrlMsgs.value.left = 0.0

        teleop_cmd.publish(ctrlMsgs)
        time.sleep(0.04)
    
    print('exit!')
    rclpy.shutdown() 


