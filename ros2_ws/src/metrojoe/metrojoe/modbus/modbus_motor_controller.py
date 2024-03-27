from time import sleep
from typing import Literal


from minimalmodbus import Instrument as MBInstrument, serial as MBSerial
from FastDebugger import fd




class ModbusMotorController:
    def __init__(
            self,
            slave_address:int,
            side:Literal['left', 'right'],
            port:str = '/dev/ttyUSB0',
            baudrate:int = 19200,
            parity:Literal['even', 'none'] = 'even',
            speed_max_input:int = 255,
            speed_max_register:int = 1000,
            speed_register_address:int = 1,
            direction_register_address:int =3
        ):
        def _get_parity() -> Literal['E', 'N']:
            """Get the parity value for the serial communication
            
            Returns:
            --------
            Literal['E', 'N']: The parity value"""
            match parity:
                case 'even':
                    return MBSerial.PARITY_EVEN
                case 'none':
                    return MBSerial.PARITY_NONE
                case _:
                    raise ValueError(f'Invalid parity: {parity!r}')
                

        # Set the class attributes
        self._slave_address = slave_address
        self._side = side
        self._speed_max_input = speed_max_input
        self._speed_max_register = speed_max_register
        self._speed_register_address = speed_register_address
        self._direction_register_address = direction_register_address

        # Create the instrument object
        self._instrument_obj = MBInstrument(port, slave_address)

        # Check if the serial port exists
        if self._instrument_obj.serial is None:
            raise ValueError(f'Serial port {port!r} not found')

        self.cprint(f'Connected to controller')

        # Update the instrument object
        self._instrument_obj.serial.baudrate = baudrate
        self._instrument_obj.serial.parity = _get_parity()

        # Define the direction map
        self.direction_map = {
            ('forward', 'left'): 1,
            ('forward', 'right'): 0,
            ('reverse', 'left'): 0,
            ('reverse', 'right'): 1,
        }


    def cprint(self, msg:str):
        """Print a message with the class name
        
        Parameters:
        -----------
        msg: str
            The message to print
        """
        print(f'[{self._slave_address}] <{self._side!r}> {msg}')


    def _change_direction(self, direction:Literal['forward', 'reverse']):
        """Change the motor direction
        
        Parameters:
        -----------
        direction: Literal['forward', 'reverse']
            The direction to change to
        """
        # Get the direction value from the direction map
        direction_value = self.direction_map.get((direction, self._side), None) # type: ignore

        # Raise error if the direction value is None
        if direction_value is None:
            raise ValueError(f'Invalid direction: {direction!r}')

        # Write the direction value to the register
        self._instrument_obj.write_register(
            self._direction_register_address,
            direction_value,
            functioncode=6
        )

        self.cprint(f'Direction changed to: {direction!r}')


    def drive_direction(self, direction:Literal['forward', 'reverse'], speed_input:int):
        """Drive the motor in a specific direction
        
        Parameters:
        -----------
        direction: Literal['forward', 'reverse']
            The direction to drive the motor
        speed_input: int
            The speed input value
        """
        # Check if the speed input is within the range
        if speed_input > self._speed_max_input:
            raise ValueError(f'Speed input {speed_input} exceeds the maximum speed input {self._speed_max_input}')

        # Change the direction
        # TODO: Have a memory of the current direction and only change if it is different
        self._change_direction(direction)

        # Calcualte the speed value for the register
        speed_register = int(speed_input * self._speed_max_register / self._speed_max_input)

        # Write the speed value to the register
        self._instrument_obj.write_register(
            self._speed_register_address,
            speed_register,
            functioncode=6
        )

        self.cprint(f'Driven {direction} at speed: {speed_input}')


    def stop_speed(self):
        """Stop the motor speed"""
        self._instrument_obj.write_register(
            self._speed_register_address,
            0,
            functioncode=6
        )

        self.cprint(f'Stopped the motor speed')



if __name__ == '__main__':
    motor_controller_1 = ModbusMotorController(1, 'left')

    motor_controller_1.drive_direction('forward', 100)
    sleep(3)
    motor_controller_1.stop_speed()

