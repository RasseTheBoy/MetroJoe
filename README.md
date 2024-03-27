# MetroJoe

A [Metropolia Garage](https://www.metropolia.fi/fi/tutkimus-kehitys-ja-innovaatiot/yhteistyoalustat/garage) project made by students of [Metropolia UAS](https://www.metropolia.fi/fi).

## Starting the `Metrojoe` image

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
(All the nodes are still a work in progress.)

### `gamepad_node` (W.I.P.)

Publish topics:

- `/gamepad`
  - Type: [GamepadInput](ros2_ws/src/metrojoe_interfaces/msg/GamepadInput.msg)
    - string: name
    - int16: value
    - Test in terminal: `ros2 topic echo /gamepad`

### `modbus_motor_controller` (W.I.P.)

A publisher node that sends the motor commands to the ModBus motor controller.

Subscriber topics:

- `/drive_speed`
  - Type: [DriveSpeed](ros2_ws/src/metrojoe_interfaces/msg/DriveSpeed.msg)
    - string: direction
    - int16: speed
  - Test in terminal: `ros2 topic pub --once /drive_speed metrojoe_interfaces/msg/DriveSpeed "{direction: '<forward/reverse>', speed: <0-255>}"`
