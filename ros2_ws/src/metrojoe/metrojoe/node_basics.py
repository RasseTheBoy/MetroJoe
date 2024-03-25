import rclpy

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

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    print('Node destroyed')
    rclpy.shutdown()
    print('rclpy shutdown')


if __name__ == '__main__':
    print(rclpy.__path__)
