import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('speed', default_value='0.2',
                              description='forward speed commanded to the robot'),
        DeclareLaunchArgument('open_threshold', default_value='321.0',
                              description='feature_mean at or below this is a measurement z=open'),
        DeclareLaunchArgument('p_z_open_x_open', default_value='0.9',
                              description='P(z=open | x=open)'),
        DeclareLaunchArgument('p_z_open_x_closed', default_value='0.1',
                              description='P(z=open | x=closed)'),
        DeclareLaunchArgument('p_open_command', default_value='0.2',
                              description='P(x=open | x=closed, u=door_open)'),
        DeclareLaunchArgument('move_belief', default_value='0.99',
                              description='drive when bel(x=open) reaches this'),
        Node(
            package='prob_rob_labs',
            executable='open_and_pass_door',
            name='open_and_pass_door',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'speed': ParameterValue(
                    LaunchConfiguration('speed'), value_type=float),
                'open_threshold': ParameterValue(
                    LaunchConfiguration('open_threshold'), value_type=float),
                'p_z_open_x_open': ParameterValue(
                    LaunchConfiguration('p_z_open_x_open'), value_type=float),
                'p_z_open_x_closed': ParameterValue(
                    LaunchConfiguration('p_z_open_x_closed'), value_type=float),
                'p_open_command': ParameterValue(
                    LaunchConfiguration('p_open_command'), value_type=float),
                'move_belief': ParameterValue(
                    LaunchConfiguration('move_belief'), value_type=float),
            }]
        )
    ])
