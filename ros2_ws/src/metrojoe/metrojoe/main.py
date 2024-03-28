
from FastDebugger import fd
from metrojoe_interfaces.msg import DriveSpeed, GamepadInput # type: ignore
from metrojoe.node_basics import NodeBase, spin_node, run_rclpy


class MainBrain(NodeBase):
    def __init__(self):
        super().__init__(
            node_name='main_brain'
        )

        # Create a subscriber for the 'gamepad' topic
        self.subscription = self.create_subscription(
            GamepadInput,
            'gamepad_trigger',
            self.gamepad_input_callback,
            10
        )

    
    def gamepad_input_callback(self, msg:GamepadInput):
        """Callback function for the 'gamepad' topic
        
        Parameters:
        -----------
        msg: GamepadInput
            The message received from the 'gamepad' topic"""
        if msg.name not in ['R2', 'L2']:
            return

        # Get the speed value and format the direction
        speed = msg.value
        direction = 'forward' if msg.name == 'R2' else 'reverse'

        # Create the DriveSpeed message
        drive_speed_msg = DriveSpeed(
            direction=direction,
            speed=speed
        )

        # Publish the message
        self.get_logger().info(f'Publishing: {drive_speed_msg}')
        self.publisher = self.create_publisher(
            DriveSpeed,
            'drive_speed',
            10
        )
        self.publisher.publish(drive_speed_msg)


@run_rclpy
def main():
    spin_node(MainBrain())