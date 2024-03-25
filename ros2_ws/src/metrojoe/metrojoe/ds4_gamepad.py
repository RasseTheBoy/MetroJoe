from typing         import Any, Generator
from evdev          import InputDevice
from FastDebugger import fd


class NoDeviceFoundError(Exception):
    """The exception for when no device is found"""
    def __init__(self, message:str='No compatible device found') -> None:
        """Initializes the NoDeviceFoundError class
        
        Parameters:
        -----------
        message: str
            The message to display
        """
        super().__init__(message)


def os_error_catch(func):
    """Catches the OSError exception"""
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except OSError:
            print('OSError: Device disconnected')
    return wrapper



class InputBase:
    """The base class for all the input objects"""
    def __init__(self, name:str, type:int, code:int) -> None:
        """Initializes the InputBase class
        
        Parameters:
        -----------
        name: str
            The name of the input object
        type: int
            The type of the input object
        code: int
            The code of the input object
        """
        self.name = name
        self.type = type
        self.code = code


    def is_type(self, event_type:int) -> bool:
        """Checks if the event type is the same as the object's type
        
        Parameters:
        -----------
        event_type: int
            The event type to check

        Returns:
        --------
        bool
            True if the event type is the same as the object's type, False otherwise
        """
        return self.type == event_type
    
    def is_code(self, event_code:int) -> bool:
        """Checks if the event code is the same as the object's code
        
        Parameters:
        -----------
        event_code: int
            The event code to check

        Returns:
        --------
        bool
            True if the event code is the same as the object's code, False otherwise
        """
        return self.code == event_code

    def is_type_and_code(self, event_type:int, event_code:int) -> bool:
        """Checks if the event type and code are the same as the object's type and code
        
        Parameters:
        -----------
        event_type: int
            The event type to check
        event_code: int
            The event code to check
        
        Returns:
        --------
        bool
            True if the event type and code are the same as the object's type and code, False otherwise
        """
        return self.is_type(event_type) and self.is_code(event_code)

    def parse_raw_value(self, event_value:int) -> int:
        """Parses the raw value of the event
        
        Parameters:
        -----------
        event_value: int
            The event value to parse
        
        Returns:
        --------
        int
            The parsed event value
        """
        return event_value

    def parse_value(self, event_value:int) -> int:
        """Parses the value of the event
        
        Parameters:
        -----------
        event_value: int
            The event value to parse
            
        Returns:
        --------
        int
            The parsed event value
        """
        return self.parse_raw_value(event_value)


class Joystick(InputBase):
    """The Joystick class for the gamepad object"""
    def __init__(self, name:str, code:int, deadzone:int=10) -> None:
        """Initializes the Joystick class
        
        Parameters:
        -----------
        name: str
            The name of the joystick object
        code: int
            The code of the joystick object
        deadzone: int
            The deadzone of the joystick object
        """
        super().__init__(name, 3, code)

        self.deadzone = deadzone
        self._joystick_center = (250 + 0) // 2


    def _check_deadzone(self, value:int) -> int|None:
        """Checks if the value is within the deadzone
        
        Parameters:
        -----------
        value: int
            The value to check
            
        Returns:
        --------
        int|None
            The value if it is outside the deadzone, None otherwise
        """
        return None if abs(value) < self.deadzone else value
    

    def parse_raw_value(self, event_value:int) -> int|None:
        """Parses the raw value of the event
        
        Parameters:
        -----------
        event_value: int
            The event value to parse
            
        Returns:
        --------
        int|None
            The parsed event value if it is outside the deadzone, None otherwise
        """
        value = super().parse_raw_value(event_value)
        return self._check_deadzone(value - self._joystick_center)


class Button(InputBase):
    """The Button class for the gamepad object"""
    def __init__(self, name:str, code:int) -> None:
        """Initializes the Button class
        
        Parameters:
        -----------
        name: str
            The name of the button object
        code: int
            The code of the button object
        """
        super().__init__(name, 1, code)

    def parse_value(self, event_value: int) -> bool:
        """Parses the value of the event
        
        Parameters:
        -----------
        event_value: int
            The event value to parse
            
        Returns:
        --------
        bool
            True if the event value is 1, False otherwise
        """
        return self.parse_raw_value(event_value) == 1


class Dpad(InputBase):
    """The Dpad class for the gamepad object"""
    def __init__(self, name:str, code:int) -> None:
        """Initializes the Dpad class
        
        Parameters:
        -----------
        name: str
            The name of the dpad object
        code: int
            The code of the dpad object
        """
        super().__init__(name, 3, code)


class Trigger(InputBase):
    """The Trigger class for the gamepad object"""
    def __init__(self, name:str, code:int) -> None:
        """Initializes the Trigger class
        
        Parameters:
        -----------
        name: str
            The name of the trigger object
        code: int
            The code of the trigger object
        """
        super().__init__(name, 3, code)



class DS4Gamepad:
    """The Gamepad class for the gamepad object"""
    def __init__(self, joystick_deadzone:float=0.05, max_event_num:int=10) -> None:
        """Initializes the Gamepad class
        
        Parameters:
        -----------
        joystick_deadzone: float
            The deadzone of the joystick, default is 0.05
        """
        def _find_compatible_devide():
            """Finds the compatible device for the gamepad object"""
            for i in range(max_event_num):
                try:
                    return InputDevice(f'/dev/input/event{i}')
                except (FileNotFoundError, PermissionError):
                    pass

            raise NoDeviceFoundError


        self.event_device = _find_compatible_devide()

        self.joystick_deadzone = joystick_deadzone

        # Setup joystick objects
        self.joystick_obj_lst = [
            Joystick(name, code_num) for (name, code_num) in
            [
                ('left_stick_x', 0),
                ('left_stick_y', 1),
                ('right_stick_x', 2),
                ('right_stick_y', 5)
            ]
        ]

        # Setup the button objects
        self.button_obj_lst = [
            Button(name, code) for name, code in
            [
                ('square', 304),
                ('cross', 305),
                ('circle', 306),
                ('triangle', 307),
                ('L1', 308),
                ('R1', 309),
                ('share', 312),
                ('options', 313),
                ('L3', 314),
                ('R3', 315),
                ('PS', 316),
                ('touchpad', 317)
            ]
        ]

        # Setup the trigger objects
        self.trigger_obj_lst = [
            Trigger(name, code_num) for (name, code_num) in
            [
                ('L2', 3),
                ('R2', 4)
            ]
        ]

        # Setup the dpad objects
        self.dpad_obj_lst = [
            Dpad(name, code_num) for (name, code_num) in
            [
                ('dpad_x', 16),
                ('dpad_y', 17)
            ]
        ]

        # Combine all the objects into one list
        self.input_obj_lst = self.joystick_obj_lst + self.button_obj_lst + self.trigger_obj_lst + self.dpad_obj_lst


    @os_error_catch
    def yield_obj(self) -> Generator[Any, None, None]:
        """Yields the object

        Warning:
        --------
        This is an infinite loop!
        
        Yields:
        -------
        obj, event_value
            The object is the input of the gamepad and the event value"""
        for event in self.event_device.read_loop():
            for input_obj in self.input_obj_lst:
                if input_obj.is_type_and_code(event.type, event.code) and input_obj.parse_raw_value(event.value) is not None:
                    yield input_obj, event.value


    def yield_input(self):
        """Yields the input of the gamepad
        
        Warning:
        --------
        This is an infinite loop!

        Yields:
        -------
        str, int
            The name of the input and the event value
        """
        for input_obj, event_value in self.yield_obj():
            yield input_obj.name, input_obj.parse_value(event_value)


    @os_error_catch
    def print_event_and_code(self):
        """Prints the event and code of the gamepad
        
        Warning:
        --------
        This is an infinite loop!
        """
        for input_obj, event_value in self.yield_obj():
            print(f'Name: {input_obj.name.ljust(15)}  -->  Value: {input_obj.parse_value(event_value)}')




def test():
    gamepad = DS4Gamepad()
    gamepad.print_event_and_code()


if __name__ == '__main__':
    test()