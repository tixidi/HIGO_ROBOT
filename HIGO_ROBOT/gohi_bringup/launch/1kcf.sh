
#!/bin/bash

roslaunch tracker_kcf_ros gohi_all_kcf.launch  &
sleep 1.0   
echo "kcf starting success!"


wait
exit 0
