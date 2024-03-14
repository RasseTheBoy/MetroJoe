# ROS2 Guide

## Table of Contents

- [ROS2 Guide](#ros2-guide)
  - [Table of Contents](#table-of-contents)
  - [Shortcut for opening a new terminal/console/shell window](#shortcut-for-opening-a-new-terminalconsoleshell-window)
  - [Building the code/package](#building-the-codepackage)
  - [Running the code](#running-the-code)

## Shortcut for opening a new terminal/console/shell window

`CTRL + ALT + T`

## Building the code/package

After editing the files in the package, you need to build the package to see the changes.

`source ~/.bashrc` is needed to see the changes in the terminal.

```bash
cd ros2_ws/
colcon build
source ~/.bashrc
```

You could also add `symlink` to the `colcon build`.\
This will create a symlink to the package in the `install` folder.\
This way, you don't need to build the package every time you make a change.

```bash
colcon build --symlink-install
```

## Running the code

Running the code is done with the `ros2 run` command.\
You need to be in the `ros2_ws` folder to run the code.

```bash
cd ros2_ws/
ros2 run <package_name> <ros2_excecutable>
```
