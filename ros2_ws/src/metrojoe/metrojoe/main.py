
from FastDebugger import fd
from metrojoe_interfaces.msg import DriveSpeed # type: ignore


def main():
    print('Hi from metrojoe.')


def test():
    # Send a message to the motor controller
    msg = DriveSpeed()
    msg.direction = 'forward'
    msg.speed = 100

    print(f'Sending message: {msg.direction}, {msg.speed}')
    


if __name__ == '__main__':
    # main()
    # test()
    fd()