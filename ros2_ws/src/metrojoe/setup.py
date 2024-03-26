from setuptools import find_packages, setup
import os

package_name = 'metrojoe'


def module_from_file(file_name:str, module_name:str='main'):
    return f'{package_name}.{file_name}:{module_name}'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'msg'), ['msg/Gamepad.msg']
        )
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='Rasmus Ohert',
    maintainer_email='ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'main_node =' + module_from_file('main'),
            f'test_node =' + module_from_file('main', 'test') ,
            f'min_talker = '+ module_from_file('minimal_test.minimal_publisher'),
            f'min_listener = '+ module_from_file('minimal_test.minimal_subscriber'),
            f'gamepad_publisher = '+ module_from_file('gamepad.ds4_gamepad_node'),
            f'modbus_motor_controller =' + module_from_file('modbus.modbus_motor_controller_node'),
        ],
        'rosidl_cli.command.generate.type_extensions': [
            'python = rosidl_generator_py.cli:GeneratePythonCommand',
        ],
    },
)
