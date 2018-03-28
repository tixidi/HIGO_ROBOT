#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from autobahn.asyncio.websocket import WebSocketServerProtocol
from autobahn.asyncio.websocket import WebSocketServerFactory

import json
import os
import sys
import math
import urllib  

import rosgraph.impl.graph
import rospy

import ast
import roslib
import struct

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


ID = '/rosnode'

class aaa():
    def  __init__(self):
        rospy.init_node('weixin_teleop')
        # Publish the Twist message to the cmd_vel topic
     
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=5)
              
        #create a Rate object to sleep the process at 5 Hz
        rate = rospy.Rate(5)
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()
        #make sure to make the robot stop by default
        self.cmd_vel.linear.x=0;
        self.cmd_vel.angular.z=0;
        
        
        # A mapping from keywords or phrases to commands
        #we consider the following simple commands, which you can extend on your own
        self.commands =             ['停止',
                                    '前进',
                                    '后退',
                                    '左转',
                                    '右转',
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
        
     # 收到消息后的处理函数，其中isbinary指示是字符串形式还是二进制  
    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            print("Text message received: {0}".format(payload.decode('utf8')))
        ## echo back message verbatim
        #self.sendMessage(payload, isBinary)
        command = payload.decode('utf8')


   
        #if not command: return None          
        file = open("/home/wb/gohi_ws/src/HIGO_ROBOT/robot_blockly/scripts/test.txt", 'r')#以读方式打开文件 
        result=list() 
        for line in file.readlines():                              #依次读取每行 
            line = line.strip()                                    #去掉每行头尾空白 
            if not len(line) or line.startswith('#'):              #判断是否是空行或注释行 
               continue                                           #是的话，跳过不处理 
            result.append(line)                                    #保存  
            self.sendMessage(line.encode("UTF-8") ,isBinary)
        #result.sort()                                        #排序结果  
        file.close()
        with open('/home/wb/gohi_ws/src/HIGO_ROBOT/robot_blockly/scripts/new_file.txt','w') as fw:      #with方式不需要再进行close
            fw.write('%s'.join(result))

        print(command)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=5)
              
        #create a Rate object to sleep the process at 5 Hz
        rate = rospy.Rate(5)
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()
        if command == '前进':
                self.cmd_vel.linear.x = 0.2
                self.cmd_vel.angular.z = 0.0
        elif command == '后退':
                self.cmd_vel.linear.x = -0.2
                self.cmd_vel.angular.z = 0.0
        elif command == '左转':
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 1.0
        elif command == '右转':
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -1.0

        print ("linear speed : " + str(self.cmd_vel.linear.x))
        print ("angular speed: " + str(self.cmd_vel.angular.z))        


        site1_X = "10"
        site1_Y = "10"
        site2_X = "50"
        site2_Y = "50"

        self.sendMessage((site1_X+site1_Y).encode("UTF-8") ,isBinary)
        # 发送消息，binary意义同上  
        self.sendMessage(payload, isBinary)
        # 发送消息，默认字符串形式  
        self.sendMessage(payload)


       

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
