#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from dynamic_reconfigure.server import Server
import dynamic_reconfigure.client
from gohi_gowhere.cfg import NavSetPointConfig



class GoToPose():
    def __init__(self):
        rospy.init_node("goto_simulator")
        self.goal_sent = False
###################################################################################################
        # Set the distance to travel

        self.Site1_X = rospy.get_param('Site1_X', 1.0) # meters
        self.Site1_Y = rospy.get_param('Site1_Y', 0.2) # meters per second
        self.Site1_Z = rospy.get_param('Site1_Z', 0.001) # meters
        
        self.Site2_X = rospy.get_param('Site2_X', 1.0) # meters
        self.Site2_Y = rospy.get_param('Site2_Y', 0.2) # meters per second
        self.Site2_Z = rospy.get_param('Site2_Z', 0.001) # meters

        #self.nav_end = rospy.get_param('nav_end', True)
###################################################################################################

	# What to do if shut down (e.g. Ctrl-C or failure)
	#rospy.on_shutdown(self.shutdown)

        #we consider the following simple commands, which you can extend on your own
        self.commands =             ['卧室',
                                    '厨房',
                                    '洗手间',
                                    '客厅',
                                    ]
        rospy.loginfo("Ready to receive voice commands")#$#######

        #subscribe the voice recognitive results
        rospy.Subscriber('/Rog_result', String, self.voice_command_callback)
         #create a Rate object to sleep the process at 5 Hz
        rate = rospy.Rate(5)


       # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

###################################################################################################          

        # Fire up the dynamic_reconfigure server
        dyn_server = Server(NavSetPointConfig, self.dynamic_reconfigure_callback)
        
        # Connect to the dynamic_reconfigure server
        dyn_client = dynamic_reconfigure.client.Client("nav_set_point", timeout=60)

###################################################################################################
    def goto(self, pos, quat):
        # Send a goal
        self.goal_sent = True
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

	# Start moving
        self.move_base.send_goal(goal)

	# Allow TurtleBot up to 60 seconds to complete task
	success = self.move_base.wait_for_result(rospy.Duration(300)) 

        state = self.move_base.get_state()
        result = False

        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stop")
        rospy.sleep(1)

    def go_to_process(self, pos, quat):
        # Customize the following values so they are appropriate for your location
        success = navigator.goto(pos, quat)
        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

###################################################################################################
        #self.start_test = False
        #params = {'start_test': False}
        #rospy.loginfo(params)
        #dyn_client.update_configuration(params)

###################################################################################################        

    def voice_command_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = msg.data
        if (command in self.commands):
            if command == '客厅':
               position = {'x': self.Site1_X, 'y' : self.Site1_Y}
               quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
   	    
            elif command == '厨房':
               position = {'x': self.Site2_X, 'y' : self.Site2_Y}
               quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
	    print ("the site is"+command)    
            navigator.go_to_process(position,quaternion)	

###################################################################################################

    def dynamic_reconfigure_callback(self, config, level):
        self.Site1_X = config['Site1_X'] 
        self.Site1_Y = config['Site1_Y']
        self.Site1_Z = config['Site1_Z'] 
        self.Site2_X = config['Site2_X'] 
        self.Site2_Y = config['Site2_Y'] 
        self.Site2_Z = config['Site2_Z'] 
     #   self.nav_end = config['nav_end']
      
        return config

###################################################################################################

if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        rospy.spin()
        

        # choice your want to go where
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

