source /opt/ros/humble/setup.bash
. install/setup.bash
# gnome-terminal -- ros2 run plotjuggler plotjuggler
# sleep 10s
gnome-terminal -- ros2 launch gain_scheduled_control control_run.py
gnome-terminal -- ros2 launch vehicle_sim vehicle_run.py
gnome-terminal -- ros2 bag play motion_profile/my_motion_profile/
ros2 bag record /vehicle_velocity /steering /vehicle_state