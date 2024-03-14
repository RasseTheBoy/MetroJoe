
# Table of contens

- [Building the code](#building-the-code)
- [Running the code](#running-the-code)
- [Visualize nodes](#visualize-nodes)
- [Creating a new package](#creating-a-new-package)
- [Adding new file/node to a package](#adding-a-new-filenode-to-the-package)
    - [1. Open terminal and `cd` to the package directory](#1-open-terminal-and-cd-to-the-package-directory-cd-ros2_wssrcpackage_name)
    - [2. Create the file](#2-create-the-file)
        - [Option 1 - Inside VSCode or file explorer](#option-1---inside-vscode-or-file-explorer)
        - [Option 2 - Using terminal](#option-2---using-terminal)
    - [3. Make file executable](#3-make-file-executable)
    - [4. Add text to `my_node.py` file](#4-add-text-to-my_nodepy-file)
        - [File breakdown](#file-breakdown)
            - [Interpeter and imports](#interpeter-and-imports)
            - [Node class](#node-class)
            - [Rclpy wrapper](#rclpy-wrapper)
            - [Main function](#main-function)
            - [Main function run](#main-funtion-run)
    - [5. Add the file to the `entry_point.console_srpipts` section in the setup.py](#5-add-the-file-to-the-entry_pointconsole_srpipts-section-in-the-setuppy)
        - [`console_scripts` breakdown:](#console_scripts-breakdown)
- [Launching Realsense in ROS2](#launching-realsense-in-ros2)
    - [run](#with-ros2-run)
    - [launch](#with-ros2-launch)
- [See output of a specific topic](#see-output-of-a-specific-topic)

# Shortcut for opening a new terminal/console/shell window

`CTRL + ALT + T`

# Building the code/package

After editing the files in the package, you need to build the package to see the changes.

`source ~/.bashrc` is needed to see the changes in the terminal.

```bash
cd ros2_ws/
colcon build
source ~/.bashrc
```

You could also add `symlink` to the `colcon build`

```bash
colcon build --symlink-install
```

# Running the code

```bash
cd ros2_ws/
ros2 run package_name ros2_excecutable
```

See [Console scripts breakdown](#console_scripts-breakdown) for more info about `packgage_name` and `ros2_excecutable`.

# Visualize nodes

```bash
rqt_graph
```

# Launching Realsense in ROS2

## Start the camera node

### with ROS2 `run`

```bash
ros2 run realsense2_camera realsense2_camera_node
# or, with parameters, for example - temporal and spatial filters are enabled:
ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true
```

### with ROS2 `launch`

```bash
ros2 launch realsense2_camera rs_launch.py
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=1280x720x30 pointcloud.enable:=true
```

# See output of a specific topic

```bash
ros2 topic echo /your/topic
```


# Creating a new package

```bash
cd /ros2_ws/src/
ros2 pkg create new_package_name --build-type ament_python --dependencies rclpy
```

Change "`new_package_name`" to whatever you want the package to be named. The name needs to be formatted like a Python file, ie. no spaces (use underscores instead).
If you want to create a C++ package, change `ament_python` to `ament_cmake`.

# Adding a new file/node to the package

## 1. Open terminal and `cd` to the package directory

```bash
cd /ros2_ws/src/package_name/
```

## 2. Create the file

### Option 1 - Inside VSCode or file explorer

Just to go inside you'r package directory, and create a new python file.<br>
Can be named whatever you want, but in this example we will use `my_node.py`.

### Option 2 - Using [terminal](#shortcut-for-opening-a-new-terminalconsoleshell-window)

`touch my_node.py`

## 3. Make file executable

`chmod +x my_node.py`

## 4. Add text to `my_node.py` file

```python
#!/usr/bin/env python3
import rclpy

from rclpy.node import Node


class MyNode(Node):
    def __init__(self):
        super().__init__('my_node') # type:ignore



def run_rclpy(func):
    """Automatically initialize, run and shutdown `rclpy`"""
    def wrapper(args, *args_, **kwargs):
        rclpy.init(args=args)
        try:
            func(*args_, **kwargs)
            rclpy.shutdown()

        except KeyboardInterrupt:
            print('Keyboard interrupation')
    return wrapper


@run_rclpy
def main(args=None):
    node = MyNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
```

### File breakdown

#### - Interpeter and imports -

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
```

Line:1 Interpeter line.<br>
Line:2-3 Import rclpy and it's Node module.

#### - Node class -

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node') # type:ignore
``` 

Barebones Node class.<br>
The class name, and the name in the `super().__init__()` can be different.
But for better understading, these should be somewhat same.

For some reason, VSCode intellisense doesn't like this specific `super().__init__()`, and shows it as an error. The code runs fine, so the `# type:ignore` is a TEMPORARY fix to ignore this issue.

#### - Rclpy wrapper -

```python
def run_rclpy(func):
    """Automatically initialize, run and shutdown `rclpy`"""
    def wrapper(args, *args_, **kwargs):
        rclpy.init(args=args)
        try:
            func(*args_, **kwargs)
            rclpy.shutdown()

        except KeyboardInterrupt:
            print('Keyboard interrupation')
    return wrapper
```

Wrapper for running `rclpy`.<br>
No error printed if user stops the code with a keyborad interruption (`CTRL+C`).

#### - Main function -

```python
@run_rclpy
def main(args=None):
    node = MyNode()
    rclpy.spin(node)
```

Main function with the [rclpy wrapper](#rclpy-wrapper) and a [node](#node-class).

The `rclpy.spin` function runs the given node indefinitely.<br>
Removing the `spin` function makes the node only run once.

#### - Main funtion run -

```python
if __name__ == "__main__":
    main()
```

Only run the main function if the file is run.

## 5. Add the file to the `entry_point.console_srpipts` section in the [setup.py](/src/my_robot_controller/setup.py)

Example:

```json
entry_points={
    'console_scripts': [
        "test_node = my_robot_controller.my_first_node:main"
    ]
}
```

### `console_scripts` breakdown:

- `test_node` = ROS2 Excecutable
- `my_robot_controller` = Package name
- `my_first_node` = Node file name (without file extension)
- `main` = Function to call

