#include <gohi_hw/HIGO_ROS.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robothw");
    ros::NodeHandle nh("gohi");

    HIGO_ROS higo(nh, "serial:///dev/ttyUSB0", "/home/ros/gohi_ws/src/HIGO_ROBOT/gohi_hw/config.txt");

    higo.mainloop();
    return 0;
}
