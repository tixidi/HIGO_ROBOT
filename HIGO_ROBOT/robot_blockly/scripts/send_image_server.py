#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from autobahn.asyncio.websocket import WebSocketServerProtocol
from autobahn.asyncio.websocket import WebSocketServerFactory

import json
import os
import math
import urllib  

# Python libs
import sys, time



import rosgraph.impl.graph
import rospy

import ast
import roslib
import struct

from geometry_msgs.msg import Twist
from std_msgs.msg import String



# Ros Messages
from sensor_msgs.msg import CompressedImage
from threading import Thread, Lock
import string,cgi,time

import re
import copy
mutex = Lock()

ID = '/rosnode'

topic_name = "/usb_cam/image_raw/compressed"
VERBOSE=True


class aaa():
    def  __init__(self):
        rospy.init_node('weixin_teleop')

        # Publish the Twist message to the cmd_vel topic
           
        #subscribe the voice recognitive results


        #create a Rate object to sleep the process at 5 Hz
        rate = rospy.Rate(5)
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()
        #make sure to make the robot stop by default
        self.cmd_vel.linear.x=0;
        self.cmd_vel.angular.z=0;
        
        
        # A mapping from keywords or phrases to commands
        #we consider the following simple commands, which you can extend on your own
        self.commands =            ['停止',
                                    '前进',
                                    '后退',
                                    '左转',
                                    '右转',
                                    '客厅',
                                    '厨房',
                                    '充电',
                                    '启动机械臂',
                                    '停止机械臂',
                                    '开始跟踪',
                                    '停止跟踪'
                                    ]
        rospy.loginfo("Ready to receive weixin commands")


def get_node_names(namespace=None):

    master = rosgraph.Master(ID)
    try:
        state = master.getSystemState()
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")
    nodes = []
    if namespace:
        # canonicalize namespace with leading/trailing slash
        g_ns = rosgraph.names.make_global_ns(namespace)
        for s in state:
            for t, l in s:
                nodes.extend([n for n in l if n.startswith(g_ns) or n == namespace])
    else:
        for s in state:
            for t, l in s:
                nodes.extend(l)
    return list(set(nodes))


def _sub_rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    master = rosgraph.Master(ID)
    nodes = get_node_names(namespace)
    nodes.sort()
    return nodes


def sumStartToEnd(start,end):  
    sum = start + end  
    return sum  





def rosnode_listnodes(namespace=None, list_uri=False, list_all=False):
    nodes_vector = _sub_rosnode_listnodes(namespace=namespace, list_uri=list_uri, list_all=list_all);
    return nodes_vector

class MyServerProtocol(WebSocketServerProtocol):
    def onConnect(self, request):
        print("Client connecting: {0}".format(request.peer))

    def onOpen(self):
        buf=""
        print("WebSocket connection open.")
        rospy.Subscriber('/upload_to_weixin', String, self.upload_msg_callback)  #订阅/upload_to_weixin上的消息，才会上传图片
 
        
     # 收到消息后的处理函数，其中isbinary指示是字符串形式还是二进制  
    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            print("Text message received: {0}".format(payload.decode('utf8')))
        ## echo back message verbatim
        #self.sendMessage(payload, isBinary)
        command = payload.decode('utf8')
   
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=5)
        self.cmd_con_pub = rospy.Publisher('Rog_result',                                 String, queue_size=1)



        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()
        self.send_str=""
        if   command == '前进':
                self.cmd_vel.linear.x =   0.2
                self.cmd_vel.angular.z =  0.0
        elif command == '后退':
                self.cmd_vel.linear.x =  -0.2
                self.cmd_vel.angular.z =  0.0
        elif command == '左转':
                self.cmd_vel.linear.x =   0.0
                self.cmd_vel.angular.z =  1.0
        elif command == '右转':
                self.cmd_vel.linear.x =   0.0
                self.cmd_vel.angular.z = -1.0
        elif command == '停止':
                self.cmd_vel.linear.x =   0.0
                self.cmd_vel.angular.z =  0.0
        else :
                print(command)
                
        self.cmd_vel_pub.publish(self.cmd_vel)  
        self.cmd_con_pub.publish(command)  
        self.sendMessage(payload, isBinary)



    def upload_msg_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = msg.data
        print("the site is"+command)  
        self.sendMessage(command.encode("UTF-8") )    

       # while 1:
        filepath = input('please input file path: ')
        if os.path.isfile(filepath):
           fileinfo_size = struct.calcsize('128sl')
           #fhead = struct.pack('128sl', os.path.basename(filepath),os.stat(filepath).st_size)
           #self.sendMessage(fhead)
           print('client filepath: {0}'.format(filepath))
           fp = open(filepath, 'rb')
           while 1:
               data = fp.read(1024)
               if not data:
                  print('{0} file send over...'.format(filepath))
                  break
               self.sendMessage(data)
   

    def onClose(self, wasClean, code, reason):
        print("WebSocket connection closed: {0}".format(reason))



if __name__ == '__main__':
    try:
        import asyncio
    except ImportError:
        # Trollius >= 0.3 was renamed
        import trollius as asyncio

    # factory = WebSocketServerFactory(u"ws://127.0.0.1:9000", debug=False)
    factory = WebSocketServerFactory(u"ws://0.0.0.0:9000")
    factory.protocol = MyServerProtocol
    aaa()


    loop = asyncio.get_event_loop()
    coro = loop.create_server(factory, '0.0.0.0', 9000)
    server = loop.run_until_complete(coro)

    try:
        loop.run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        loop.close()
