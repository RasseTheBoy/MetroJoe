from ..node_basics import spin_node, run_rclpy, NodeBase
from .ds4_gamepad import DS4Gamepad
from metrojoe_interfaces.msg import GamepadInput # type: ignore
from FastDebugger import fd


class DS4GamepadNode(NodeBase):
    def __init__(self):
        """Create a DS4GamepadNode object that publishes the gamepad inputs to the 'gamepad' topic"""
        super().__init__(node_name='ds4_gamepad_node')
        
        # Create a DS4Gamepad object
        self.gamepad_obj = DS4Gamepad()

        # Create a publisher for the 'gamepad_trigger' topic
        self.gamepad_trigger_publisher = self.create_publisher(
            GamepadInput,
            'gamepad_trigger',
            10
        )

        # Create a publisher for the 'gamepad_joystick' topic
        self.gamepad_joystick_publisher = self.create_publisher(
            GamepadInput,
            'gamepad_joystick',
            10
        )

        self.publish_button_inputs() # TODO: Check if this works; Remove if it doesn't

        # # Create a timer to publish the button inputs
        # self.timer = self.create_timer(0.5, self.publish_button_inputs)

    
    def publish_button_inputs(self):
        """Publish the button inputs to the 'gamepad' topic"""
        gamepad_msg = GamepadInput()

        for input_obj, input_value in self.gamepad_obj.yield_obj_and_raw_value():
            gamepad_msg.name = input_obj.name
            gamepad_msg.value = input_value

            match input_obj.class_name:
                case 'trigger':
                    self.gamepad_trigger_publisher.publish(gamepad_msg)
                    self.log(f'Publishing: {input_obj.name} = {input_value}')

                case _:
                    self.log(f'Uknown class_name: {input_obj.class_name}')



@run_rclpy
def main():
    spin_node(DS4GamepadNode())