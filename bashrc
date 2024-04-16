set_aliases () {
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

    # Sudo Shutdown Now
    alias ssn='sudo shutdown now'

    echo "Aliases set"
}

give_permissions () {
    # Give Permission to devies
    sudo chmod +777 /dev/gpiochip*
    sudo chmod +777 /dev/i2c*
    sudo chmod +777 /dev/spidev*
    echo "Permissions given to devices"
}

# Read the value of BASH_SETUP_REQUIRED from the file
if [ -f "$HOME/.bash_setup_required" ]; then
    BASH_SETUP_REQUIRED=$(cat "$HOME/.bash_setup_required")
else
    BASH_SETUP_REQUIRED=true
fi

# Check if the setup is required
if [ "$BASH_SETUP_REQUIRED" = "true" ]; then
    give_permissions
    echo "false" > "$HOME/.bash_setup_required"
fi

set_aliases

source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
sis

echo "ROS2 workspace: Sourcing complete"