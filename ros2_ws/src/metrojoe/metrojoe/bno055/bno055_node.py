from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable
from ..node_basics import spin_node, run_rclpy, NodeBase
from adafruit_bno055 import BNO055_I2C
from board import I2C
from .sensors import _3DSensor, TemperatureSensor, QuaternionSensor



class BNO055_Node(NodeBase):
    def __init__(self):
        super().__init__(node_name='bno055')
        
        self.sensor = BNO055_I2C(I2C())

        # Create a list of 3D sensor objects
        sensor_obj_lst:list[Any] = [
            _3DSensor(self, name, sensor_function) for (name, sensor_function) in 
            [
                (
                    'acceleration',
                    lambda: self.sensor.acceleration
                ),
                (
                    'magnetic',
                    lambda: self.sensor.magnetic
                ),
                (
                    'gyro',
                    lambda: self.sensor.gyro
                ),
                (
                    'euler',
                    lambda: self.sensor.euler
                ),
                (
                    'linear_acceleration',
                    lambda: self.sensor.linear_acceleration
                ),
                (
                    'gravity',
                    lambda: self.sensor.gravity
                )
            ]
        ]

        # Add temperature and quaternion sensor
        sensor_obj_lst.extend([
            TemperatureSensor(self, 'temperature', lambda: self.sensor.temperature),
            QuaternionSensor(self, 'quaternion', lambda: self.sensor.quaternion)
        ])
        
        # Separate enabled and disabled sensors
        disabled_sensor_obj_lst = []
        self.enabled_sensor_obj_lst = []
        for sensor_obj in sensor_obj_lst:
            if sensor_obj.disabled:
                disabled_sensor_obj_lst.append(sensor_obj)
            else:
                self.enabled_sensor_obj_lst.append(sensor_obj)

        # Print enabled and disabled sensors
        self.log(f'Enabled sensors: {[sensor.name for sensor in self.enabled_sensor_obj_lst]}')
        self.log(f'Disabled sensors: {[sensor.name for sensor in disabled_sensor_obj_lst]}')
        
        # Create a timer to publish sensor data
        publisher_delay_seconds = self.declare_and_get_parameter('publisher_delay', 0.2)
        self.create_timer(publisher_delay_seconds, self.publish_sensor_data)


    def publish_sensor_data(self):
        """Reads and publishes the enabled sensor data to their respective topics"""
        # Iterate over all of the enabled sensors and read and publish their data
        for sensor in self.enabled_sensor_obj_lst:
            sensor.read_and_publish_data()


@run_rclpy
def main():
    spin_node(BNO055_Node())