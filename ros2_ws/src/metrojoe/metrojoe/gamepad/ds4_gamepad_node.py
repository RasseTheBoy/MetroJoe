from ..node_basics import spin_node, run_rclpy
from .ds4_gamepad import DS4Gamepad
from rclpy.node import Node
from std_msgs.msg import String
from FastDebugger import fd


class DS4GamepadNode(Node):
    def __init__(self):
        """Create a DS4GamepadNode object that publishes the gamepad inputs to the 'gamepad' topic"""
        super().__init__(node_name='ds4_gamepad_node', parameter_overrides=[])
        
        # Create a DS4Gamepad object
        self.gamepad_obj = DS4Gamepad()

        # Create a publisher for the 'gamepad' topic
        self.publisher = self.create_publisher(
            String,
            'gamepad',
            5
        )

        # Create a timer to publish the button inputs
        self.timer = self.create_timer(0.5, self.publish_button_inputs)

    
    def publish_button_inputs(self):
        """Publish the button inputs to the 'gamepad' topic"""
        # TODO: Create a custom message type for the gamepad inputs
        msg = String()

        for input_name, input_value in self.gamepad_obj.yield_input():
            msg.data = f'{input_name}:{input_value}'
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"')



@run_rclpy
def main(args=None):
    spin_node(DS4GamepadNode())