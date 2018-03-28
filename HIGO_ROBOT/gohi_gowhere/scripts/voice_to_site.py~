#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String


class GoToPose():
    def __init__(self):

        self.goal_sent = False

	# What to do if shut down (e.g. Ctrl-C or failure)
	#rospy.on_shutdown(self.shutdown)

        #we consider the following simple commands, which you can extend on your own
        self.commands =             ['带我去柜台。',
                                    '带我去洗手间。',
                                    '带我去贵客厅。',
                                    ]
        rospy.loginfo("Ready to receive voice commands")#$#######

        #subscribe the voice recognitive results
        rospy.Subscriber('/xfspeech', String, self.voice_command_callback)
         #create a Rate object to sleep the process at 5 Hz
        rate = rospy.Rate(5)


       # Tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("Wait for the action server to come up")

	# Allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))
          

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

        

    def voice_command_callback(self, msg):
        # Get the motion command from the recognized phrase
        command = msg.data
        if (command in self.commands):
            if command == '带我去柜台。':
               position = {'x': 0.232, 'y' : -1.67}
               quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
            elif command == '带我去洗手间。':
               position = {'x': 2.15, 'y' : -0.343}
               quaternion = {'r1' : 0000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
	    print ("the site is"+command)    
            navigator.go_to_process(position,quaternion)	


if __name__ == '__main__':
    try:
        rospy.init_node('nav_test', anonymous=False)
        navigator = GoToPose()
        rospy.spin()
        

        # choice your want to go where
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")

