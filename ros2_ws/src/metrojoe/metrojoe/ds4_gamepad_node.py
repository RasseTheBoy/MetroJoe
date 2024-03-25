from .node_basics import spin_node, run_rclpy
from .ds4_gamepad import DS4Gamepad
from rclpy.node import Node
from std_msgs.msg import String
from FastDebugger import fd


class DS4GamepadNode(Node):
    def __init__(self):
        super().__init__(node_name='ds4_gamepad_node', parameter_overrides=[])
        
        self.gamepad_obj = DS4Gamepad()

        self.publisher = self.create_publisher(
            String,
            'gamepad',
            10
        )

        self.timer = self.create_timer(0.5, self.publish_button_inputs)

    
    def publish_button_inputs(self):
        msg = String()

        for input_name, input_value in self.gamepad_obj.yield_input():
            msg.data = f'{input_name}:{input_value}'
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')



@run_rclpy
def main(args=None):
    spin_node(DS4GamepadNode())