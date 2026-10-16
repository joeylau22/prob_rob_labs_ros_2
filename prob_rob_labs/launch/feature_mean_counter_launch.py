from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('threshold', default_value='251.0',
                              description='feature_mean at or below this is z=open'),
        DeclareLaunchArgument('window', default_value='60.0',
                              description='seconds to sample each door state'),
        DeclareLaunchArgument('countdown', default_value='5',
                              description='seconds to count down after Enter'),
        DeclareLaunchArgument('plot_path', default_value='feature_mean_sessions.png',
                              description='where to write the plot'),
        Node(
            package='prob_rob_labs',
            executable='feature_mean_counter',
            name='feature_mean_counter',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'threshold': ParameterValue(
                    LaunchConfiguration('threshold'), value_type=float),
                'window': ParameterValue(
                    LaunchConfiguration('window'), value_type=float),
                'countdown': ParameterValue(
                    LaunchConfiguration('countdown'), value_type=int),
                'plot_path': LaunchConfiguration('plot_path'),
            }]
        )
    ])
