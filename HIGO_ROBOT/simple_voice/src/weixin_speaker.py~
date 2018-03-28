#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot."""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class recoder_main():
 def __init__(self):
  self.define()
  rospy.Subscriber(self.topic , String , self.talker)
  rospy.spin()
   
 def define(self):
  self.say=rospy.Publisher('speak_string', String, queue_size=1)

  if not rospy.has_param('~words'):
   rospy.set_param('~words','请让一下，谢谢')
   
  if not rospy.has_param('~SpeakerSubTopic'):
   rospy.set_param('~SpeakerSubTopic', 'Rog_result')
   #rospy.set_param('~SpeakerSubTopic', 'stop_flag')
   
  self.words=rospy.get_param('~words')
  self.topic=rospy.get_param('~SpeakerSubTopic')
  self.commands =                  ['停止',
                                    '前进',
                                    '后退',
                                    '左转',
                                    '右转',
                                    '客厅',
                                    '厨房',
                                    '充电',
                                    '初始跟踪',
                                    '启动机械臂',
                                    '停止机械臂',
                                    '开始跟踪',
                                    '停止跟踪',
                                    '你好',
                                    '再见'
                                    ]
  
 def talker(self,data):
        # Get the motion command from the recognized phrase
  command = data.data
  if (command in self.commands):
   if command == '停止':
    self.say.publish(command)
   elif command == '前进':
    self.say.publish(command)
   elif command == '后退':
    self.say.publish(command)
   elif command == '左转':
    self.say.publish(command)
   elif command == '右转':
    self.say.publish(command)
   elif command == '厨房':
    self.say.publish(command)
   elif command == '客厅':
    self.say.publish(command)
   elif command == '充电':
    self.say.publish(command)
   elif command == '启动机械臂':
    self.say.publish(command)
   elif command == '停止机械臂':
    self.say.publish(command)
   elif command == '开始跟踪':
    self.say.publish(command)
   elif command == '停止跟踪':
    self.say.publish(command)
   elif command == '初始跟踪':
    self.say.publish(command)
   elif command == '你好':
    self.say.publish(command)
   elif command == '再见':
    self.say.publish(command)
  else:
   pass
   
if __name__=="__main__":
 rospy.init_node('warning2speaker')
 rospy.loginfo("initialization system")
 recoder_main()
 rospy.loginfo("process done and quit")
