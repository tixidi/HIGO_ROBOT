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
        

        while True:
            fileinfo_size=struct.calcsize('128sl') #定义文件信息。128s表示文件名为128bytes长，l表示一个int或log文件类型，在此为文件大小
            command = payload.decode('utf8')
            if command: #如果不加这个if，第一个文件传输完成后会自动走到下一句
                filename,filesize =struct.unpack('128sl',command) #根据128sl解包文件信息，与client端的打包规则相同
                filenewname = os.path.join('/home/wb/gohi_ws/src/HIGO_ROBOT/robot_blockly/scripts',('new_'+ self.filename)) #使用strip()删除打包时附加的多余空字符
                print (filenewname,type(filenewname))
                recvd_size = 0 #定义接收了的文件大小
                file = open(self.filenewname,'wb')
                print ('stat receiving...')
                while not recvd_size == self.filesize:
                    file.write(rdata)
                file.close()
                print ('receive done')
            else:
                break;
        # 发送消息，binary意义同上  
        self.sendMessage(payload, isBinary)



       

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
