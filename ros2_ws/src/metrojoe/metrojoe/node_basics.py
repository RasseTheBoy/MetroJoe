import rclpy
from rclpy.node import Node


class NodeBase(Node):
    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        self.log('Node initialized')

    
    def log(self, msg:str):
        """Print a message with the class name
        
        Args:
        -----
        msg (str): The message to print"""
        self.get_logger().info(f'{self.__class__.__name__} {msg}')




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
