
#!/bin/bash

roslaunch gohi_hw gohi_hw.launch  &
echo "higo base starting success!"
sleep 1


roslaunch rplidar_ros rplidar.launch  &
echo "rplidar starting success!"
sleep 1

roslaunch gohi_2dnav move_base_amcl_5cm.launch  &
echo "amcl starting success!"
sleep 1

roslaunch gohi_gowhere weixin_to_specific_point.launch &
echo "weixin nav starting success!"
sleep 1


roslaunch rbx1_vision usb_cam_left.launch  &
sleep 1
echo "left camera starting success!"

roslaunch rbx1_vision usb_cam_right.launch  &
sleep 1  
echo "right camera starting success!"

roslaunch robot_blockly weixin_con_odom_ack_server.launch  &
sleep 1.0   
echo "websocket server starting success!"


roslaunch robot_blockly image_upload_server.launch  &
sleep 1.0   
echo "http server starting success!"


roslaunch simple_voice weixin_speaker.launch &
echo "voice recognition starting success!"
sleep 1

roslaunch openni_launch openni.launch &
echo "kinect starting success!"
sleep 1

wait
exit 0
