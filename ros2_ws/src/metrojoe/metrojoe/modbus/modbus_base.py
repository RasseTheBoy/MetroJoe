from time import sleep
from minimalmodbus  import Instrument as MBInstrument, serial as MBSerial
from typing import Any, Literal

from serial import SerialException
from abc import ABC, abstractmethod


class ModbusBase(ABC):
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
        # Set the class attributes
        self.slave_address = slave_address
        self.port = port
        self._baudrate = baudrate
        self._parity = parity
        self._speed_max_input = speed_max_input
        self._speed_max_register = speed_max_register
        self._speed_register_address = MB_speed_register_address
        self._direction_register_address = MB_direction_register_address
        self.MB_freq_data_address = MB_freq_data_address
        self.MB_pwm_data_address = MB_pwm_data_address

        self.try_connecting_to_device()


    def try_connecting_to_device(self):
        def _get_parity() -> Literal['E', 'N']:
            """Get the parity value for the serial communication
            
            Returns
            -------
            Literal['E', 'N']: The parity value"""
            match self._parity:
                case 'even':
                    return MBSerial.PARITY_EVEN
                case 'none':
                    return MBSerial.PARITY_NONE
                case _:
                    raise ValueError(f'Invalid parity: {self._parity!r}') 
    
        retry_count = 0

        while True:
            try:
                self._instrument_obj = MBInstrument(self.port, self.slave_address)
                self.cprint(f'Connected to controller')

                if self._instrument_obj.serial is None:
                    raise ConnectionError(f'Serial port {self.port!r} not found')

                self._instrument_obj.serial.baudrate = self._baudrate
                self._instrument_obj.serial.parity = _get_parity()

                self.set_default_state()
                return

            except (ConnectionError, SerialException):
                retry_count += 1
                sleep(1)
                self.cprint(f'Failed to connect to controller. Retry: {retry_count}')


    @abstractmethod
    def set_default_state(self):
        """This method is required to be added in the child class to set the default state of the device"""
        pass


    def cprint(self, msg:str):
        """Print a message with the class name
        
        Parameters
        ----------
        msg: str
            The message to print
        """
        print(f'[{self.slave_address}] {msg}')


    def write_register(
            self,
            register_address:int,
            value:int,
            functioncode:int
        ) -> None:
        """Write a value to the register
        
        Parameters
        ---------
        register_address: int
            The register address to write to
        value: int
            The value to write to the register
        functioncode: int
            The function code to use for writing to the register
        """
        try:
            self._instrument_obj.write_register(
                register_address,
                value,
                functioncode=functioncode
            )

        except SerialException:
            self.cprint(f'Failed to write to register: {register_address}')
            self.try_connecting_to_device()


    def read_register(
            self,
            register_address:int,
            functioncode:int  
        ) -> Any:
        """Read a value from the register
        
        Parameters
        ---------
        register_address: int
            The register address to read from
        functioncode: int
            The function code to use for reading the register
        
        Returns
        -------
        Any: The value read from the register
        """
        try:
            return self._instrument_obj.read_register(
                register_address,
                functioncode=functioncode
            )
        
        except SerialException:
            self.cprint(f'Failed to read from register: {register_address}')
            self.try_connecting_to_device()



    def _set_speed(self, speed_input:int):
        """Set the motor speed
        
        Parameters
        ----------
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

