# UBICACIÓN: ~/pzb_ws/src/pzb_utils/launch/fpga_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Argumentos de launch
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyTHS1',
        description='Puerto serie para comunicación con FPGA (GPIO 8/10 en Jetson Nano)'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='9600',
        description='Velocidad de comunicación UART'
    )
    
    # Nodo FPGA
    fpga_node = Node(
        package='pzb_utils',
        executable='fpga_node',
        name='fpga_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'timeout': 0.1
        }],
        remappings=[
            # Si necesitas remapear tópicos, agrégalos aquí
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baudrate_arg,
        fpga_node
    ])