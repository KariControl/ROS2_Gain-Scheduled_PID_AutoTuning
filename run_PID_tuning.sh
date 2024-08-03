source /opt/ros/humble/setup.bash
. install/setup.bash
gnome-terminal -- ros2 launch time_sync time_sync.py
sleep 1s
gnome-terminal -- ros2 bag play rosbag2_2024_08_03-19_18_01/
ros2 launch pid_tuner pid_tuner.py