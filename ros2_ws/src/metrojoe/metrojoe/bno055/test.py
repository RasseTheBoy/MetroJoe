
import adafruit_bno055, board
from os import system
from FastDebugger import fd
from time import sleep


def main():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = adafruit_bno055.BNO055_I2C(i2c)


    while True:
        system('clear')
#         print(f'''
# Temperature {sensor.temperature}
# Accelerometer {sensor.acceleration} degrees C
# Magnetometer (microteslas): {sensor.magnetic}
# Gyroscope (rad/sec): {sensor.gyro}
# Euler angle: {sensor.euler}
# Quaternion: {sensor.quaternion}
# Linear acceleration (m/s^2): {sensor.linear_acceleration}
# Gravity (m/s^2): {sensor.gravity}
# ''')

        fd(sensor.quaternion)

        sleep(1)

if __name__ == '__main__':
    try:
        main()

    except KeyboardInterrupt:
        print('KeyboardInterrupt')