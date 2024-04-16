
from metrojoe_interfaces.msg import ThreeDimensional, Temperature, Quaternion
from typing import Any, Callable
from abc import ABC, abstractmethod
from std_msgs.msg import Header



class SensorBase(ABC):
    def __init__(
        self,
        node_self,
        name: str,
        sensor_function: Callable,
        message_interface: Any
        ):
        self.node_self = node_self
        self.name = name
        self.sensor_function = sensor_function
        self.message_interface = message_interface

        # Get the parameter to see if the sensor should be disabled
        self.disabled = self.node_self.declare_and_get_parameter(f'disable_{self.name}', False)

        # Creates a publisher object with the message interface and name
        self.publisher_obj = self.node_self.create_publisher(
            self.message_interface,
            f'{self.node_self.get_name()}/{self.name}',
            10
        )
        
        # Create header message object
        self.header = Header()
        
    
    @abstractmethod
    def read_and_publish_data(self):
        pass

    def update_header_time(self):
        """Update the header time to the current time"""
        self.header.stamp = self.node_self.get_clock().now().to_msg()


class _3DSensor(SensorBase):
    def __init__(self, node_self, name: str, sensor_function: Callable[..., Any]):
        super().__init__(node_self, name, sensor_function, ThreeDimensional)

    def read_and_publish_data(self):
        self.update_header_time()
        x,y,z = [round(x, 5) for x in self.sensor_function()]

        self.publisher_obj.publish(ThreeDimensional(
            x=x,
            y=y,
            z=z,
            header=self.header
        ))


class TemperatureSensor(SensorBase):
    def __init__(self, node_self, name: str, sensor_function: Callable[..., Any]):
        super().__init__(node_self, name, sensor_function, Temperature)

    def read_and_publish_data(self):
        self.update_header_time()

        self.publisher_obj.publish(Temperature(
            value=self.sensor_function(),
            header=self.header
        ))


class QuaternionSensor(SensorBase):
    def __init__(self, node_self, name: str, sensor_function: Callable[..., Any]):
        super().__init__(node_self, name, sensor_function, Quaternion)

    def read_and_publish_data(self):
        self.update_header_time()
        x,y,z,w = [round(x, 5) for x in self.sensor_function()]

        self.publisher_obj.publish(Quaternion(
            x=x,
            y=y,
            z=z,
            w=w,
            header=self.header
        ))