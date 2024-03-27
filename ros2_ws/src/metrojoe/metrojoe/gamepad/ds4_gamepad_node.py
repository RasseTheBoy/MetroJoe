from ..node_basics import spin_node, run_rclpy
from .ds4_gamepad import DS4Gamepad
from rclpy.node import Node
from metrojoe_interfaces.msg import GamepadInput # type: ignore
from FastDebugger import fd


class DS4GamepadNode(Node):
    def __init__(self):
        """Create a DS4GamepadNode object that publishes the gamepad inputs to the 'gamepad' topic"""
        super().__init__(node_name='ds4_gamepad_node', parameter_overrides=[])
        
        # Create a DS4Gamepad object
        self.gamepad_obj = DS4Gamepad()

        # Create a publisher for the 'gamepad' topic
        self.publisher = self.create_publisher(
            GamepadInput,
            'gamepad',
            3
        )

        # Create a timer to publish the button inputs
        self.timer = self.create_timer(0.5, self.publish_button_inputs)

    
    def publish_button_inputs(self):
        """Publish the button inputs to the 'gamepad' topic"""
        gamepad_msg = GamepadInput()

        for input_name, input_value in self.gamepad_obj.yield_input():
            gamepad_msg.name = input_name
            gamepad_msg.value = input_value
            self.publisher.publish(gamepad_msg)
            self.get_logger().info(f'Publishing: {gamepad_msg.name} = {gamepad_msg.value}')



@run_rclpy
def main():
    spin_node(DS4GamepadNode())