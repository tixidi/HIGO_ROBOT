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

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


import rospy
import actionlib
from actionlib_msgs.msg import *
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


ID = '/rosnode'

class aaa():
    def  __init__(self):
        rospy.init_node('weixin_teleop')
        # Publish the Twist message to the cmd_vel topic
           
        #create a Rate object to sleep the process at 5 Hz
        rate = rospy.Rate(5)
        
        # A mapping from keywords or phrases to commands
 
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
    nodes_vector = _sub_rosnode_listnodes(namespace=namespace, list_uri=list_uri, list_all=list_all)
    return nodes_vector

class MyServerProtocol(WebSocketServerProtocol):
    def onConnect(self, request):
        print("Client connecting: {0}".format(request.peer))

    def onOpen(self):

       #we consider the following simple commands, which you can extend on your own
        self.commands =            ['停止',
                                    '前进',
                                    '后退',
                                    '左转',
                                    '右转',
                                    '客厅',
                                    '厨房',
                                    '充电',
                                    '红色',
                                    '绿色',
                                    '蓝色',
                                    '启动机械臂',
                                    '停止机械臂',
                                    '开始跟踪',
                                    '停止跟踪', 
                                    '接收地图',
                                    '拍照',
                                    '保存'
                                    ]
        print("WebSocket connection open.")
        rospy.Subscriber('/upload_to_weixin', String, self.upload_msg_callback)

        rospy.Subscriber('/mobile_base/mobile_base_controller/odom', Odometry, self.upload_odom_callback)
        #rospy.Subscriber('/odom', Odometry, self.upload_odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/mobile_base_controller/cmd_vel', Twist, queue_size=5)
        self.cmd_con_pub = rospy.Publisher('Rog_result',                                 String, queue_size=1)
        robot_odom=Odometry()
        self.enable_path=0
        # range is -10m~10m
        self.odom_shift=10 
        # precision 10 is dm ,1m=10dm
        # precision 100 is cm ,1m=100cm
        self.odom_precision=100
        #max range (0~20)*100
        self.max_range=2000
        self.pix_range=350


        
     # 收到消息后的处理函数，其中isbinary指示是字符串形式还是二进制  
    def onMessage(self, payload, isBinary):
        if isBinary:
            print("Binary message received: {0} bytes".format(len(payload)))
        else:
            print("Text message received: {0}".format(payload.decode('utf8')))
        ## echo back message verbatim
        #self.sendMessage(payload, isBinary)
        command = payload.decode('utf8')
   
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()
        if (command in self.commands):
           if   command == '前进':
                self.cmd_vel.linear.x =   0.2
                self.cmd_vel.angular.z =  0.0
                self.cmd_vel_pub.publish(self.cmd_vel)  
           elif command == '后退':
                self.cmd_vel.linear.x =  -0.2
                self.cmd_vel.angular.z =  0.0
                self.cmd_vel_pub.publish(self.cmd_vel)  
           elif command == '左转':
                self.cmd_vel.linear.x =   0.0
                self.cmd_vel.angular.z =  1.0
                self.cmd_vel_pub.publish(self.cmd_vel)  
           elif command == '右转':
                self.cmd_vel.linear.x =   0.0
                self.cmd_vel.angular.z = -1.0
                self.cmd_vel_pub.publish(self.cmd_vel)  
           elif command == '停止':
                self.cmd_vel.linear.x =   0.0
                self.cmd_vel.angular.z =  0.0
                self.cmd_vel_pub.publish(self.cmd_vel)  
           elif command == '厨房':
                self.enable_path=1
           elif command == '客厅':
                self.enable_path=2
           elif command == '充电':
                self.enable_path=3
           else :
                print(command)
                self.enable_path=0
        else:
           pass             
        
        self.cmd_con_pub.publish(command)  
        self.sendMessage(payload, isBinary)



    def upload_msg_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = msg.data
        if command =='aa':
            self.enable_path=0
        self.sendMessage(command.encode("UTF-8") )       


    def upload_odom_callback(self, robot_odom):
        # Get the motion command from the recognized phrase
       
        siteX=(robot_odom.pose.pose.position.x+self.odom_shift)*self.odom_precision
        siteY=(robot_odom.pose.pose.position.y+self.odom_shift)*self.odom_precision
        siteX=int((siteX*self.pix_range)/self.max_range)
        siteY=int((siteY*self.pix_range)/self.max_range)

        robot_site_x = str(siteX)
        robot_site_y = str(siteY)
        
        print("the pose x is"+ robot_site_x)
        print("the pose y is"+ robot_site_y)
     
        if self.enable_path>0:
           self.sendMessage(('0'+','+robot_site_x+','+robot_site_y).encode("UTF-8") )
        
           


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
