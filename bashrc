echo "Sourcing ROS 2 workspace..."

source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
alias ccb='colcon build --symlink-install && source install/setup.bash'
alias sis='source install/setup.bash'

echo "ROS 2 workspace sourcing complete."