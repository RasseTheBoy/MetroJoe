from typing import Literal
from FastDebugger import fd

from .modbus_base import ModbusBase




class ModbusGlobalMotorController(ModbusBase):
    def __init__(self):
        super().__init__(slave_address=0)
    

    def set_default_state(self):
        """Doesn't do anything in the global motor controller"""
        pass


    def drive_speed(self, speed:int):
        """Drive all motors with the given speed"""
        self.cprint(f'Driving all motors at speed {speed}')
        self._reg_set_speed(speed)