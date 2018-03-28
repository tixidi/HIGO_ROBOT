#include <pxpincher_lib/phantomx_interface.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <termios.h>
char getch(); // allow to capture keyboard inputs without blocking the program (we use this to disable and enable torque / relax)

int revCommandFlag=0;
Eigen::Affine3d desired_ee;

void commandCallBack(const geometry_msgs::Twist& command)
{
      ROS_WARN_STREAM("Keep .");
      
      visualization_msgs::InteractiveMarker taskspace_marker;
      taskspace_marker.pose.orientation.w=0.8438757658004761;
      taskspace_marker.pose.orientation.x=0.0013727728510275483;
      taskspace_marker.pose.orientation.y=0.5365332961082458;
      taskspace_marker.pose.orientation.z=-0.0021386106964200735;

      taskspace_marker.pose.position.x=command.linear.x;
      taskspace_marker.pose.position.y=command.linear.y;
      taskspace_marker.pose.position.z=command.linear.z;

      
      tf::poseMsgToEigen(taskspace_marker.pose ,desired_ee);
      
      revCommandFlag=1;

}



// =============== Main function =================
int main( int argc, char** argv )
{
  ros::init(argc, argv, "pxpincher_test");
  ros::NodeHandle n("~");
  
  pxpincher::PhantomXControl robot;

  robot.initialize();
  robot.activateInteractiveJointControl();
  //--------------------------------------------------
  ros::Subscriber armCommandSubscriber = n.subscribe("/chatter", 10, commandCallBack);

  //--------------------------------------------------
  //robot.setJoints({0.8, 0.6, 0.9, 1.6});
  //robot.setGripperJoint(0);
  //robot.setGripperJoint(100);
  //robot.setGripperJoint(20);
  
  //ROS_INFO_STREAM(std::setprecision(2) << "Current joint configuration q=[" << robot.getJointAngles().transpose() << "]");
 
  //robot.setEndeffectorPoseInc(0, 0, -0.05, 0.1); // notice: blocking call
  
  //Eigen::Affine3d tcp;
  //robot.getEndeffectorState(tcp);
  //ROS_INFO_STREAM(std::setprecision(2) << "TCP rotation matrix w.r.t. base:\n" << tcp.rotation());
  //ROS_INFO_STREAM(std::setprecision(2) << "TCP translation vector w.r.t. base: [" << tcp.translation().transpose() << "]");
  
  //robot.setEndeffectorPoseInc(0, -0.2, 0, 0.1, false); // notice: non-blocking call

  ros::Rate r(10);
  while (ros::ok())
  {

      char c=getch();
      if (c == 'y' || c == 'Y')
      {
        robot.setGripperJoint(30);
        robot.setJoints({0, 0, 0, 0});
        robot.setGripperJoint(100);
      }
      else ROS_WARN_STREAM("Keep catching.");
      
      if(revCommandFlag==1)
      {
        revCommandFlag=0;
        robot.setEndeffectorPose(desired_ee,0.2,false,false);
      }
   //   robot.publishInformationMarker();
      ros::spinOnce();
      r.sleep();
      
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

