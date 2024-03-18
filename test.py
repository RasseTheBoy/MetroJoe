#!/usr/bin/env python3

import rospy
from pymodbus.client.sync import ModbusTcpClient

def read_modbus_data():
    rospy.init_node('modbus_reader', anonymous=True)
    
    # Replace 'localhost' and '502' with the IP address and port of your Modbus simulator
    modbus_client = ModbusTcpClient('localhost', port=502)
    
    try:
        # Specify the Modbus register address and the number of registers to read
        # Replace 'address' and 'count' with appropriate values for your Modbus device
        address = 0  # Address of the register to read
        count = 1    # Number of registers to read
        
        # Read holding registers from the Modbus device
        result = modbus_client.read_holding_registers(address, count, unit=1)
        
        # Check if the reading was successful
        if result.isError():
            rospy.logerr("Failed to read Modbus data: %s", result)
        else:
            # Print the read data
            rospy.loginfo("Modbus data: %s", result.registers)
    finally:
        # Close the Modbus connection
        modbus_client.close()

if __name__ == '__main__':
    try:
        read_modbus_data()
    except rospy.ROSInterruptException:
        pass
