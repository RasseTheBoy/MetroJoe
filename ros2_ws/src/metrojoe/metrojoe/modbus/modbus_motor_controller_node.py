from metrojoe.node_basics import run_rclpy, spin_node, NodeBase
from .modbus_motor_controller import ModbusMotorController
from metrojoe_interfaces.msg import DriveSpeed # type: ignore
from minimalmodbus import NoResponseError

import rclpy



class ModbusMotorControllerNode(NodeBase):
    def __init__(self):
        super().__init__(node_name='modbus_motor_controller', parameter_overrides=[])

        # TODO: Add all motors to object list

        # Creates a list of ModbusMotorController objects for all of the motors
        self.motor_controller_obj_lst = [
            ModbusMotorController(slave_id, side, position)
            for (slave_id, side, position) in 
            [
                (1, 'left', 'front'),
                (2, 'right', 'front')
            ]
        ]

        self.global_motor_controller = None

        self.last_dierction = 'forward'

        # Create a subscriber for the 'motor_controller' topic
        self.subscription = self.create_subscription(
            DriveSpeed,
            'drive_speed',
            self.motor_drive_speed_callback,
            10
        )


    
    def drive_all_motors(self, speed:int):
        pass


    # a subscriber to the topic /motor_controller that will drive the motor
    def motor_drive_speed_callback(self, msg):
        _direction = msg.direction
        _speed = msg.speed

        self.log(f'Received message: {_direction}, {_speed}')
        for motor_controller_obj in self.motor_controller_obj_lst:
            motor_controller_obj.drive_direction(
                direction = _direction,
                speed_input = _speed
            )


def main():
    rclpy.init()
    while True:
        try:
            rclpy.spin_once(ModbusMotorControllerNode())

        except NoResponseError:
            continue
