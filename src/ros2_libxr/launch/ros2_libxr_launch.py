from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_topic', default_value='/cmd_vel_mpc'),
        DeclareLaunchArgument('behavior_topic', default_value='/sentry/behaivor_send'),
        DeclareLaunchArgument('baudrate', default_value='921600'),
        DeclareLaunchArgument('usb_vid', default_value='16d0'),
        DeclareLaunchArgument('usb_pid', default_value='1492'),
        DeclareLaunchArgument('control_interface', default_value='XRUSB Navigation Control'),
        # Wheel odometry from the MC02 chassis_feedback topic.
        DeclareLaunchArgument('chassis_velocity_topic', default_value='/sentry/chassis_velocity'),
        DeclareLaunchArgument('chassis_yaw_rate_source', default_value='wheel',
                              description="'wheel' or 'gyro' (chassis IMU when valid)"),
        DeclareLaunchArgument('chassis_velocity_require_all_wheels', default_value='true'),
        DeclareLaunchArgument('chassis_velocity_latency_us', default_value='1000'),
        Node(
        package='ros2_libxr',
        executable='ros2_libxr_node',
        name='ros2_libxr',
        output='both',
        emulate_tty=True,
        parameters=[{
            'cmd_topic': LaunchConfiguration('cmd_topic'),
            'behavior_topic': LaunchConfiguration('behavior_topic'),
            'baudrate': LaunchConfiguration('baudrate'),
            'usb_vid': LaunchConfiguration('usb_vid'),
            'usb_pid': LaunchConfiguration('usb_pid'),
            'control_interface': LaunchConfiguration('control_interface'),
            'chassis_velocity_topic': LaunchConfiguration('chassis_velocity_topic'),
            'chassis_yaw_rate_source': LaunchConfiguration('chassis_yaw_rate_source'),
            'chassis_velocity_require_all_wheels': ParameterValue(
                LaunchConfiguration('chassis_velocity_require_all_wheels'), value_type=bool),
            'chassis_velocity_latency_us': ParameterValue(
                LaunchConfiguration('chassis_velocity_latency_us'), value_type=int),
        }],
        )
    ])
