from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_topic', default_value='/cmd_vel_mpc'),
        DeclareLaunchArgument('command_timeout', default_value='0.15'),
        DeclareLaunchArgument('odom_topic', default_value='/aft_mapped_to_init'),
        DeclareLaunchArgument('odom_frame', default_value='camera_init'),
        DeclareLaunchArgument('odom_timeout', default_value='0.3'),
        DeclareLaunchArgument('rotate_latency', default_value='0.02'),
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
            'cmd_topic': LaunchConfiguration('cmd_topic'),
            'command_timeout': ParameterValue(
                LaunchConfiguration('command_timeout'), value_type=float),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'odom_timeout': ParameterValue(
                LaunchConfiguration('odom_timeout'), value_type=float),
            'rotate_latency': ParameterValue(
                LaunchConfiguration('rotate_latency'), value_type=float),
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
