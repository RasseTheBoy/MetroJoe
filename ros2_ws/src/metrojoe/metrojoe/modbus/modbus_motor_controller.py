from metrojoe.modbus.modbus_base import ModbusBase
from FastDebugger   import fd
from typing         import Literal
from time           import sleep

import plotly.express as px



class MotorDirectionError(Exception):
    """Error raised when the motor direction cannot be changed"""
    def __init__(self, message:str='Cannot change direction while the motor is running'):
        super().__init__(message)



class ModbusMotorController(ModbusBase):
    def __init__(
            self,
            slave_address:int,
            side:Literal['left', 'right'],
            position:Literal['front', 'middle', 'back'],
        ):
        self.side = side
        self.position = position

        # Set internal memory attributes
        self.last_direction = 'forward'
        self.last_speed = 0

        # Define the direction map
        self.direction_map = {
            ('forward', 'left'): 1,
            ('forward', 'right'): 0,
            ('reverse', 'left'): 0,
            ('reverse', 'right'): 1,
        }

        super().__init__(slave_address=slave_address)
        self.set_default_state()


    def cprint(self, msg:str):
        """Print a message with the class name
        
        Parameters
        ----------
        msg: str
            The message to print
        """
        super().cprint(f'<{self.side}|{self.position}> {msg}')

    
    def set_default_state(self):
        """Set the device to its default state/position"""
        self.cprint('Setting the default state')
        self.stop_speed()
        self.change_direction('forward')
        


    def change_direction(self, direction:Literal['forward', 'reverse']):
        """Change the motor direction
        
        Parameters
        ----------
        direction: Literal['forward', 'reverse']
            The direction to change to
        """
        # TODO: Check if the motor is running from the motor controllers register (PWM/freq?)
        # TODO: Direction can be changed only if the motor is slow enough (doesn't need to be stopped)(?)

        # Check if the trigger value has been set back to 0 before changing the direction
        if self.last_speed != 0:
            raise MotorDirectionError('Last speed is not 0!')

        # Get the direction register value from the direction map
        direction_reg_value = self.direction_map.get((direction, self.side), None) # type: ignore

        # Raise error if the direction value is None
        if direction_reg_value is None:
            raise ValueError(f'Invalid direction: {direction!r}')
        
        self.write_register(
            register_address = self._direction_register_address,
            value = direction_reg_value,
            functioncode = 6
        )

        self.last_direction = direction

        self.cprint(f'Direction changed to: {direction!r}')


    def drive_direction(self, direction:Literal['forward', 'reverse'], speed_input:int):
        """Drive the motor in a specific direction
        
        Parameters
        ----------
        direction: Literal['forward', 'reverse']
            The direction to drive the motor
        speed_input: int
            The speed input value
        """
        # Check if the speed input is within the range
        if speed_input > self._speed_max_input:
            raise ValueError(f'Speed input {speed_input} exceeds the maximum speed input {self._speed_max_input}')

        # Try chaning direction
        if direction != self.last_direction:
            try:
                self.change_direction(direction)
            except MotorDirectionError as e:
                self.cprint(f'Error: {e}')
                return

        _speed_register = self._set_speed(speed_input)
        
        self.cprint(f'Drive {direction!r} at speed: {speed_input} ({_speed_register})')


    def stop_speed(self):
        """Stop the motor speed"""
        self._set_speed(0)
        self.cprint(f'STOPPED THE MOTOR SPEED! -> Set to 0')


    def get_motor_pulse_frequency(self) -> int:
        """Get the motor pulse frequency
        
        Returns
        -------
        int: The motor pulse frequency
        """
        return self.read_register(
            register_address = self.MB_freq_data_address,
            functioncode = 4
        )

    def get_motor_pwm(self) -> int:
        """Get the motor pwm value
        
        Returns
        -------
        int: The motor pwm value
        """
        return self.read_register(
            register_address = self.MB_pwm_data_address,
            functioncode = 4
        )
    

    def plot_motor_value(self, plot_value:Literal['pwm', 'freq'], loop_range:int=10, sleep_time_seconds:float=0.5, save_file:bool=False):
        """Plot the motor value
        
        Parameters
        ----------
        plot_value: Literal['pwm', 'freq']
            The value to plot
        loop_range: int
            The number of iterations to plot
        sleep_time_seconds: float
            The sleep time between each iteration
        """
        def get_motor_value() -> int:
            match plot_value:
                case 'pwm':
                    return self.get_motor_pwm()
                
                case 'freq':
                    return self.get_motor_pulse_frequency()
                
                case _:
                    raise ValueError(f'Invalid plot value: {plot_value!r}')

        motor_value_lst =  []

        # Create a list of epoch times for plotting
        epoch_time_lst = [indx*sleep_time_seconds for indx in range(loop_range)]

        for indx in range(loop_range):
            # Read the motor pulse frequency and save it to the list
            motor_value = get_motor_value()
            motor_value_lst.append(motor_value)
            self.cprint(f'<{indx}>: {motor_value}')

            # Skip sleep if last index
            if indx != loop_range - 1:
                sleep(sleep_time_seconds)

        # Create a plotly figure
        fig = px.line(
            x = epoch_time_lst,
            y = motor_value_lst,
            title = f'Motor {plot_value.capitalize()} Plot',
            labels = {'x': 'Time (s)', 'y': f'Motor {plot_value}'}
        )

        # Optionally save the plot to a file
        if save_file:
            fig.write_html(f'motor_{plot_value}_plot.html', auto_open=True)

        else:
            fig.show()






if __name__ == '__main__':
    try:
        motor_controller_test = ModbusMotorController(2, 'left', 'front')

        motor_controller_test.drive_direction('forward', 255)
        # motor_controller_test.plot_motor_value('freq', loop_range=10, sleep_time_seconds=0.5, save_file=True)
        sleep(3)
        motor_controller_test.stop_speed()

    except KeyboardInterrupt:
        print('\nKeyboardInterrupt')

