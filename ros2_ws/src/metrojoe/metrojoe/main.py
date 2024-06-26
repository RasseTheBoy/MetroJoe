
from FastDebugger import fd
from metrojoe_interfaces.msg import DriveSpeed, GamepadInput, ThreeDimensional, Quaternion
from metrojoe.node_basics import NodeBase, spin_node, run_rclpy
from message_filters import TimeSynchronizer, Subscriber, ApproximateTimeSynchronizer


class MainBrain(NodeBase):
    def __init__(self):
        super().__init__(
            node_name='main_brain'
        )

        # Create a subscriber for the 'gamepad' topic
        self.gamepad_trigger_subscriber = self.create_subscription(
            GamepadInput,
            'gamepad_trigger',
            self.gamepad_trigger_callback,
            10
        )

        self.speed_publisher = self.create_publisher(
            DriveSpeed,
            'drive_speed',
            10
        )

        ApproximateTimeSynchronizer(
            [
                Subscriber(self, ThreeDimensional, 'acceleration'),
                Subscriber(self, Quaternion, 'quaternion'),
            ],
            10,
            0.1
        ).registerCallback(self.acceleration_gravity_callback)

    def acceleration_gravity_callback(self, acceleration:ThreeDimensional, quaternion:Quaternion):
        self.log(f'Acceleration: {acceleration.x}')
        self.log(f'Quaternion: {quaternion.w}')

    
    def gamepad_trigger_callback(self, msg:GamepadInput):
        """Callback function for the 'gamepad' topic
        
        Parameters:
        -----------
        msg: GamepadInput
            The message received from the 'gamepad' topic
        """
        # Create the DriveSpeed message
        drive_speed_msg = DriveSpeed(
            direction='forward' if msg.name == 'R2' else 'reverse',
            speed=msg.value
        )

        # Publish the message
        self.log(f'Publishing: {drive_speed_msg}')
        self.speed_publisher.publish(drive_speed_msg)


@run_rclpy
def main():
    spin_node(MainBrain())