from typing import Any
import rclpy
from rclpy.node import Node


class NodeBase(Node):
    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self.log('Node initialized')

    
    def log(self, msg:str):
        """Print a message with the class name
        
        Parameter
        ---------
        msg (str): The message to print"""
        self.get_logger().info(msg)
        # self.get_logger().info(f'{self.__class__.__name__} {msg}')


    def declare_and_get_parameter(self, parameter_name:str, default_value) -> Any:
        """Declare and get a parameter
        
        Parameter
        ---------
        parameter_name (str): The name of the parameter
        default_value (any): The default value of the parameter
        
        Returns
        -------
        The value of the parameter"""
        self.declare_parameter(parameter_name, default_value)
        return self.get_parameter(parameter_name).value




def run_rclpy(func):
    """Automatically initialize, run and shutdown `rclpy`"""
    def wrapper(*args_, args=None, **kwargs):
        rclpy.init(args=args)
        try:
            func(*args_, **kwargs)
            rclpy.shutdown()

        except KeyboardInterrupt:
            print('\nKeyboard interrupation')
    return wrapper


def spin_node(node):
    """Spin the node"""
    rclpy.spin(node)

    node.destroy_node()
    print('Node destroyed')
    rclpy.shutdown()
    print('rclpy shutdown')


if __name__ == '__main__':
    print(rclpy.__path__)
