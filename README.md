# MetroJoe

A [Metropolia Garage](https://www.metropolia.fi/fi/tutkimus-kehitys-ja-innovaatiot/yhteistyoalustat/garage) project made by students of [Metropolia UAS](https://www.metropolia.fi/fi).

## Table of Contents

- [MetroJoe](#metrojoe)
  - [Table of Contents](#table-of-contents)
  - [Starting the `Metrojoe` image](#starting-the-metrojoe-image)
    - [Breakdown](#breakdown)
  - [ROS2 Nodes](#ros2-nodes)
    - [`main` (W.I.P.)](#main-wip)
    - [`gamepad_node` (W.I.P.)](#gamepad_node-wip)
    - [`modbus_motor_controller` (W.I.P.)](#modbus_motor_controller-wip)


## Starting the `Metrojoe` image

**It is recommender to run the image in a [devcontainer](.devcontainer/devcontainer.json) for development.**

To run the code from a terminal, use the following command:

```bash
docker run -it --rm -v ~/metrojoe/MetroJoe/ros2_ws:/ros2_ws -v /dev/input/:/dev/input/ --privileged metrojoe
```

### Breakdown

- `docker run` - Run a command in a new container
- `-it` - Keep STDIN open even if not attached, allocate a pseudo-TTY
- `--rm` - Automatically remove the container when it exits
  - Optional, but recommended when running the container multiple times (ie. for development)
- `-v ~/metrojoe/MetroJoe/ros2_pkg:/ros2_pkg` - Mount the `ros2_pkg` directory from the host to the `/ros2_pkg` directory in the container
  - The `ros2_pkg` directory should contain the ROS2 package(s) on the host machine
- `-v /dev/input/:/dev/input/` - Mount the `/dev/input/` directory from the host to the `/dev/input/` directory in the container
  - Contains the input devices on the host machine
  - Required for the joystick inputs to work
- `--device=/dev/ttyUSB0` - Give the container access to the `/dev/ttyUSB0` device
  - Gives access to the USB serial device on the host machine
  - Required for the serial communication to work (ie. with the ModBus motor controller)
- `--privileged` - Give the container full access to the host
  - Required for reading the joystick inputs from the `/dev/input/` directory
- `metrojoe` - The name of the image to run

## ROS2 Nodes

```bash
ros2 run metrojoe <node_name>
```

All of the available nodes are listed below.
(All of the nodes are still a work in progress.)

### `main` (W.I.P.)

The main node that works as the brain of the robot.

Subscribes to the `/gamepad_trigger` topic and publishes to the `/drive_speed` topic.\
In the future, more functions will be added to this node.

### `gamepad_node` (W.I.P.)

Reads the gamepad inputs and publishes them to a topic.

**Publish topics:**

- `/gamepad_trigger`
  - Type: [GamepadInput](ros2_ws/src/metrojoe_interfaces/msg/GamepadInput.msg)
    - string: name
    - int16: value
    - Test in terminal: `ros2 topic echo /gamepad_trigger`

### `modbus_motor_controller` (W.I.P.)

Controls the ModBus motor controller.

**Subscriber topics:**

- `/drive_speed`
  - Type: [DriveSpeed](ros2_ws/src/metrojoe_interfaces/msg/DriveSpeed.msg)
    - string: direction
    - int16: speed
  - Test in terminal: `ros2 topic pub --once /drive_speed metrojoe_interfaces/msg/DriveSpeed "{direction: '<forward/reverse>', speed: <0-255>}"`
