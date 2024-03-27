echo "ROS2 workspace: Sourcing bash files and setting aliases"

source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

echo "ROS2 workspace: Sourcing complete"

# Install Setup Bash
alias sis='. install/setup.bash'

# Remove Build, Install and Log
alias rbl='rm -rf build install log'

# Colcon Build All
alias cba='rbl && colcon build --symlink-install && sis'

# Colcon Build Metrojoe
alias cbm='colcon build --symlink-install --packages-select metrojoe && sis'

# Colcon Build (Metrojoe) Interfaces
alias cbi='colcon build --symlink-install --packages-select metrojoe_interfaces && sis'


echo "ROS2 workspace: Aliases set"