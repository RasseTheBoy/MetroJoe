from ..node_basics import run_rclpy, spin_node
from rclpy.node import Node
from .modbus_motor_controller import ModbusMotorController



class ModbusMotorControllerNode(Node):
    def __init__(self):
        super().__init__(node_name='modbus_motor_controller', parameter_overrides=[])

        # TODO: Create a list of ModbusMotorController objects for all of the motors






@run_rclpy
def main():
    spin_node(ModbusMotorControllerNode())