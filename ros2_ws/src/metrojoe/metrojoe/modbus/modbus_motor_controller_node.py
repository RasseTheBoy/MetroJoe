from ..node_basics import run_rclpy, spin_node
from rclpy.node import Node
from .modbus_motor_controller import ModbusMotorController
from metrojoe_interfaces.msg import DriveSpeed # type: ignore



class ModbusMotorControllerNode(Node):
    def __init__(self):
        super().__init__(node_name='modbus_motor_controller', parameter_overrides=[])

        # TODO: Create a list of ModbusMotorController objects for all of the motors

        self.motor_controller_obj_lst = [
            ModbusMotorController(1, 'left')
        ]

        # Create a subscriber for the 'motor_controller' topic
        self.subscription = self.create_subscription(
            DriveSpeed,
            'drive_speed',
            self.motor_drive_speed_callback,
            3
        )

        self.log('ModbusMotorControllerNode has been created')


    def log(self, msg):
        self.get_logger().info(msg)


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







@run_rclpy
def main():
    spin_node(ModbusMotorControllerNode())