from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('chassis_cmd_topic', default_value='/chassis_cmd_mpc'),
        DeclareLaunchArgument('chassis_yaw_offset', default_value='0.0'),
        DeclareLaunchArgument('behavior_topic', default_value='/sentry/behaivor_send'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        DeclareLaunchArgument('usb_vid', default_value='16d0'),
        DeclareLaunchArgument('usb_pid', default_value='1492'),
        DeclareLaunchArgument('control_interface', default_value='XRUSB Navigation Control'),
        Node(
        package='ros2_libxr',
        executable='ros2_libxr_node',
        name='ros2_libxr',
        output='both',
        emulate_tty=True,
        parameters=[{
            'chassis_cmd_topic': LaunchConfiguration('chassis_cmd_topic'),
            'chassis_yaw_offset': ParameterValue(
                LaunchConfiguration('chassis_yaw_offset'), value_type=float),
            'behavior_topic': LaunchConfiguration('behavior_topic'),
            'baudrate': LaunchConfiguration('baudrate'),
            'usb_vid': ParameterValue(
                LaunchConfiguration('usb_vid'), value_type=str),
            'usb_pid': ParameterValue(
                LaunchConfiguration('usb_pid'), value_type=str),
            'control_interface': LaunchConfiguration('control_interface'),
        }],
        )
    ])
