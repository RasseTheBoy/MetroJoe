import minimalmodbus

def main():
    # Define the Modbus slave address and port
    SLAVE_ADDRESS = 1  # Replace with your Modbus slave address
    PORT = '/dev/ttyUSB0'  # Replace with your serial port

    # Create a minimalmodbus instrument instance
    instrument = minimalmodbus.Instrument(PORT, SLAVE_ADDRESS)

    if instrument.serial is None:
        raise ValueError(f"Serial port {PORT} not found")

    # Optional: Set up the serial communication parameters
    instrument.serial.baudrate = 19200
    instrument.serial.bytesize = 8
    instrument.serial.parity = minimalmodbus.serial.PARITY_EVEN
    instrument.serial.stopbits = 1
    instrument.serial.timeout = 0.05  # Timeout in seconds

    # Function to read a holding register
    def read_register(register_address):
        try:
            value = instrument.read_register(register_address, functioncode=3)
            print(f"Value read from register {register_address}: {value}")
        except Exception as e:
            print(f"Error reading register {register_address}: {e}")

    # Function to write to a holding register
    def write_register(register_address, value):
        try:
            instrument.write_register(register_address, value, functioncode=6)
            print(f"Value {value} written to register {register_address}")
        except Exception as e:
            print(f"Error writing to register {register_address}: {e}")

    # Test read and write operations
    write_register(1, 0)  # Replace with your register address and value
    read_register(10)  # Replace with your register address


if __name__ == '__main__':
    main()
