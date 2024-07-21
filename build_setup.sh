source /opt/ros/humble/setup.bash
rm -r install/ build/
colcon build --symlink-install

# colcon build --symlink-install --packages-select vehicle_sim
# colcon build --symlink-install --packages-select gain_scheduled_control