/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/


#include <pxpincher_lib/phantomx_interface.h>

#include <termios.h>
char getch(); // allow to capture keyboard inputs without blocking the program (we use this to disable and enable torque / relax)


// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "pxpincher_test");
  ros::NodeHandle n("~");
  
 
  pxpincher::PhantomXControl robot;
  robot.initialize();
  robot.activateInteractiveJointControl();
  //robot.setJoints({0.8, 0.6, 0.9, 1.6});
  //robot.setGripperJoint(0);
  //robot.setGripperJoint(100);
  //robot.setGripperJoint(20);
  
  ROS_INFO_STREAM(std::setprecision(2) << "Current joint configuration q=[" << robot.getJointAngles().transpose() << "]");
 
  //robot.setEndeffectorPoseInc(0, 0, -0.05, 0.1); // notice: blocking call
  
  Eigen::Affine3d tcp;
  robot.getEndeffectorState(tcp);
  ROS_INFO_STREAM(std::setprecision(2) << "TCP rotation matrix w.r.t. base:\n" << tcp.rotation());
  ROS_INFO_STREAM(std::setprecision(2) << "TCP translation vector w.r.t. base: [" << tcp.translation().transpose() << "]");
  
  //robot.setEndeffectorPoseInc(0, -0.2, 0, 0.1, false); // notice: non-blocking call
  
  ros::Rate r(10);
  while (ros::ok())
  {
      robot.publishInformationMarker();
      ros::spinOnce();
      r.sleep();
      char c=getch();
      if (c == 'y' || c == 'Y')
      {
        robot.setGripperJoint(30);
        robot.setJoints({0, 0, 0, 0});
        robot.setGripperJoint(100);
      }
      else if(c == 'n' || c == 'N')
      { 
      visualization_msgs::InteractiveMarker taskspace_marker;
      taskspace_marker.pose.orientation.w=0.7204843759536743;
      taskspace_marker.pose.orientation.x=-0.005342945922166109;
      taskspace_marker.pose.orientation.y=0.6934410929679871;
      taskspace_marker.pose.orientation.z=0.0054045445285737514;

      taskspace_marker.pose.position.x=0.0299;
      taskspace_marker.pose.position.y=0.1491;
      taskspace_marker.pose.position.z=0.0093;

      Eigen::Affine3d desired_ee;
      tf::poseMsgToEigen(taskspace_marker.pose ,desired_ee);
      robot.setEndeffectorPose(desired_ee,0.2,false,false);
      }
      else
      
       ROS_WARN_STREAM("Keep catching.");
      
  }
  

  
  
  return 0;
}

// source http://answers.ros.org/question/63491/keyboard-key-pressed/
char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 50;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if (rv != 0) // == 0 -> nothing selected
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

