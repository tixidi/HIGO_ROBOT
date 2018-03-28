#include <pxpincher_lib/phantomx_interface.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <termios.h>
char getch(); // allow to capture keyboard inputs without blocking the program (we use this to disable and enable torque / relax)

int revCommandFlag=0;
int voice_flag=0;
Eigen::Affine3d desired_ee;

void commandCallBack(const geometry_msgs::Twist& command)
{
      ROS_WARN_STREAM("Keep .");
      
      visualization_msgs::InteractiveMarker taskspace_marker;
      //taskspace_marker.pose.orientation.w=0.8438757658004761;
      //taskspace_marker.pose.orientation.x=0.0013727728510275483;
      //taskspace_marker.pose.orientation.y=0.5365332961082458;
      //taskspace_marker.pose.orientation.z=-0.0021386106964200735;


      //taskspace_marker.pose.orientation.w=0.7204843759536743;
      //taskspace_marker.pose.orientation.x=-0.005342945922166109;
      //taskspace_marker.pose.orientation.y=0.6934410929679871;
      //taskspace_marker.pose.orientation.z=0.0054045445285737514;

      taskspace_marker.pose.position.x=command.linear.x;
      taskspace_marker.pose.position.y=command.linear.y;
      taskspace_marker.pose.position.z=command.linear.z;

      
      tf::poseMsgToEigen(taskspace_marker.pose ,desired_ee);
      
      revCommandFlag=1;

}

void voiceCommandCallBack(const std_msgs::String& command)
{

	if(command.data.compare("你好") == 0)
	{
                voice_flag=1;

	}
	if(command.data.compare("再见") == 0)
	{
                voice_flag=2;
 
	}
	if(command.data.compare("停止机械臂") == 0)
	{
                voice_flag=3;
    
	}
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
  ros::Subscriber voiceCommandArmSubscriber = n.subscribe("/Rog_result", 1000, voiceCommandCallBack);
  ros::Publisher  pub2= n.advertise<std_msgs::String>("/speak_string",1000);

  ros::Subscriber armCommandSubscriber = n.subscribe("/arm/pick", 10, commandCallBack);
  robot.setGripperJoint(80);
  robot.setJoints({-0.90, 0.90, -1.20, 0.8});
      //  robot.setGripperJoint(100);

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
        robot.setGripperJoint(15);
        robot.setJoints({-0.90, 0.90, -0.90, 0});
        robot.setGripperJoint(100);
      }
      else ROS_WARN_STREAM("Keep catching.");


      if(voice_flag==1)
      {
         voice_flag=0;
  	 robot.initialize();                                                         
         robot.setJoints({-0.90, 0.90, -0.90, 0});             //input hello while output certain actions              
	 robot.setJoints({-0.90, 0.90, -0.50, 0});
         robot.setJoints({-0.90, 0.90, -0.90, 0});
         robot.setJoints({-0.90, 0.90, -0.50, 0});
         robot.setJoints({-0.90, 0.90, -0.90, 0});
         
         printf("hello arm action end\n");
      }
      if(voice_flag==2)
      {
         voice_flag=0;     
         robot.initialize();                                                         
	 robot.setJoints({0.90, 0.90, -0.90, 0});               //input goodbye while output certain actions
  	 robot.setJoints({-0.90, 0.90, -0.90, 0});
         robot.setJoints({0.90, 0.90, -0.90, 0});
         robot.setJoints({-0.90, 0.90, -0.90, 0});
         
         printf("bye arm action end\n"); 
      }
      if(voice_flag==3)
      {
         voice_flag=0;     
         robot.setGripperJoint(15);
         robot.setJoints({-0.90, 0.90, -0.90, 0});
         robot.setGripperJoint(100);

         std_msgs::String static_msg;  
         std::stringstream ss;
         ss << "抓取完成" ;  
         static_msg.data = ss.str();  
         pub2.publish(static_msg);
      }
      
      if(revCommandFlag==1)
      {
        revCommandFlag=0;

        visualization_msgs::InteractiveMarker taskspace_marker;
        taskspace_marker.pose.orientation.w=0.7204843759536743;
        taskspace_marker.pose.orientation.x=-0.005342945922166109;
        taskspace_marker.pose.orientation.y=0.6934410929679871;
        taskspace_marker.pose.orientation.z=0.0054045445285737514;

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

