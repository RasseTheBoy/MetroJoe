# Metrojoe interfaces

This package contains the interfaces for the metrojoe project. The interfaces are used to define the communication between the different components of the metrojoe project.

## DriveSpeed

Sent to the motor controller to set the speed of the motors.

```
string direction
int16 speed
```

- `direction`: The direction of the motors. Can be either `forward` or `backward`.
- `speed`: The speed of the motors. Can be any value between 0-255.

## GamepadInput

Sent by the gamepad node

```
string name
int16 value
```

- `name`: Name of the gamepad input
- `value`: Value of the gamepad input
    - buttons: 0/1
    - joysticks: -xxx to xxx
    - triggers: 0 - 255
    - dpad: -1/0/1