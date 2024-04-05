from minimalmodbus  import Instrument as MBInstrument, serial as MBSerial





from typing import Literal


class ModbusBase:
    def __init__(
            self,
            slave_address:int,
            port:str='/dev/ttyUSB0',
            baudrate:int=19200,
            parity:Literal['even', 'none'] = 'even',
            speed_max_input:int=255,
            speed_max_register:int=1000,
            MB_speed_register_address:int=1,
            MB_direction_register_address:int=3,
            MB_freq_data_address:int=4,
            MB_pwm_data_address:int=10
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
        self.slave_address = slave_address
        self._speed_max_input = speed_max_input
        self._speed_max_register = speed_max_register
        self._speed_register_address = MB_speed_register_address
        self._direction_register_address = MB_direction_register_address
        self.MB_freq_data_address = MB_freq_data_address
        self.MB_pwm_data_address = MB_pwm_data_address

        # Create the instrument object
        self._instrument_obj = MBInstrument(port, slave_address)

        # Check if the serial port exists
        if self._instrument_obj.serial is None:
            raise ValueError(f'Serial port {port!r} not found')
        
        self.cprint(f'Connected to controller')

        # Update the instrument object
        self._instrument_obj.serial.baudrate = baudrate
        self._instrument_obj.serial.parity = _get_parity()


    def cprint(self, msg:str):
        """Print a message with the class name
        
        Parameters:
        -----------
        msg: str
            The message to print
        """
        print(f'[{self.slave_address}] {msg}')


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
        self.last_speed = speed_input

        return speed_register

