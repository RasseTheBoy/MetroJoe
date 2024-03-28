from time import sleep
from typing import Literal


from minimalmodbus import Instrument as MBInstrument, serial as MBSerial
from FastDebugger import fd



class MotorDirectionError(Exception):
    def __init__(self, message:str='Cannot change direction while the motor is running'):
        super().__init__(message)



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
            direction_register_address:int =3,
            pwm_register_address:int = 10
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
        self._pwm_register_address = pwm_register_address

        # Set internal memory attributes
        self._last_direction = 'forward'
        self._last_speed = 0

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


    def get_pwm_reg_val(self):
        """Check the pwm register value"""
        return self._instrument_obj.read_register(self._pwm_register_address, functioncode=3)


    def _change_direction(self, direction:Literal['forward', 'reverse']):
        """Change the motor direction
        
        Parameters:
        -----------
        direction: Literal['forward', 'reverse']
            The direction to change to
        """
        # TODO: Check if the motor is running from the motor controllers register (PWM?)
        # TODO: Direction can be changed only if the motor is slow enough (doesn't need to be stopped)
        if self._last_speed != 0:
            raise MotorDirectionError('Last speed is not 0!')
        elif self.get_pwm_reg_val() != 0:
            raise MotorDirectionError('Motor PWM is not 0!')

        # Get the direction register value from the direction map
        direction_reg_value = self.direction_map.get((direction, self._side), None) # type: ignore

        # Raise error if the direction value is None
        if direction_reg_value is None:
            raise ValueError(f'Invalid direction: {direction!r}')

        # Write the direction value to the register
        self._instrument_obj.write_register(
            self._direction_register_address,
            direction_reg_value,
            functioncode=6
        )

        self._last_direction = direction

        self.cprint(f'Direction changed to: {direction!r}')

    
    def _set_speed(self, speed_input:int):
        """Set the motor speed
        
        Parameters:
        -----------
        speed_input: int
            The speed input value
        """
        # Calcualte the speed value for the register
        speed_register = int(speed_input * self._speed_max_register / self._speed_max_input)

        # Write the speed value to the register
        self._instrument_obj.write_register(
            self._speed_register_address,
            speed_register,
            functioncode=6
        )

        # Save the last speed value
        self._last_speed = speed_input

        return speed_register

        


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

        # Try chaning direction
        if direction != self._last_direction:
            try:
                self._change_direction(direction)
            except MotorDirectionError as e:
                self.cprint(f'Error: {e}')
                # TODO: Should the motor be stopped if both direction buttons are pressed?
                # self.cprint('Stopping the motor speed')
                # self.stop_speed()
                return

        _speed_register = self._set_speed(speed_input)
        
        self.cprint(f'Drive {direction!r} at speed: {speed_input} ({_speed_register})')


    def stop_speed(self):
        """Stop the motor speed"""
        self._set_speed(0)
        self.cprint(f'STOPPED THE MOTOR SPEED!')



if __name__ == '__main__':
    motor_controller_1 = ModbusMotorController(1, 'left')

    motor_controller_1.drive_direction('forward', 100)
    sleep(3)
    motor_controller_1.stop_speed()

