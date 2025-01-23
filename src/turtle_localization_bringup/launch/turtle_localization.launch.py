#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    frequency = 200 #Hz
    noise_mean = {'noise_mean': 0.0}
    noise_variance = {'noise_variance': 0.2}
    timing = {'timing': 1/frequency} # The time used for the publishing rate and kalman filters

      # Turtlesim Node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    )

    # Robot Position Node
    cirle_motion_node = Node(
        package='turtle_localization_pkg',
        executable='circle_motion',
        name='circle_motion',
        parameters=[
            timing
        ]
    )

    # PlotJuggler Node
    plotjuggler_node = Node(
    package='plotjuggler',
    executable='plotjuggler',
    name='plotjuggler'
)

    # Robot Position Node
    robot_position_ukf_node = Node(
        package='turtle_localization_pkg',
        executable='robot_position_ukf',
        name='robot_position_ukf',
        parameters=[
            {'pose_1_x': 0.0}, {'pose_1_y': 0.0}, {'pose_1_z': 0.0},
            {'pose_2_x': 11.0}, {'pose_2_y': 0.0}, {'pose_2_z': 0.0},
            {'pose_3_x': 11.0}, {'pose_3_y': 5.5}, {'pose_3_z': 0.0},
            {'pose_4_x': 11.0}, {'pose_4_y': 11.0}, {'pose_4_z': 0.0}, 
            {'pose_5_x': 0.0}, {'pose_5_y': 11.0}, {'pose_5_z': 0.0},
            {'pose_6_x': 0.0}, {'pose_6_y': 5.5}, {'pose_6_z': 0.0},
            noise_variance,
            timing
        ]
    )

    # Robot Position Node wit noise
    robot_position_noise_node = Node(
        package='turtle_localization_pkg',
        executable='robot_position_noise',
        name='robot_position_noise',
        parameters=[
            {'pose_1_x': 0.0}, {'pose_1_y': 0.0}, {'pose_1_z': 0.0},
            {'pose_2_x': 11.0}, {'pose_2_y': 0.0}, {'pose_2_z': 0.0},
            {'pose_3_x': 11.0}, {'pose_3_y': 5.5}, {'pose_3_z': 0.0},
            {'pose_4_x': 11.0}, {'pose_4_y': 11.0}, {'pose_4_z': 0.0}, 
            {'pose_5_x': 0.0}, {'pose_5_y': 11.0}, {'pose_5_z': 0.0},
            {'pose_6_x': 0.0}, {'pose_6_y': 5.5}, {'pose_6_z': 0.0}
        ]
    )

    # Robot Position Node wit noise
    robot_position_std_kf_node = Node(
        package='turtle_localization_pkg',
        executable='robot_position_std_kf',
        name='robot_position_std_kf',
        parameters=[
            {'pose_1_x': 0.0}, {'pose_1_y': 0.0}, {'pose_1_z': 0.0},
            {'pose_2_x': 11.0}, {'pose_2_y': 0.0}, {'pose_2_z': 0.0},
            {'pose_3_x': 11.0}, {'pose_3_y': 5.5}, {'pose_3_z': 0.0},
            {'pose_4_x': 11.0}, {'pose_4_y': 11.0}, {'pose_4_z': 0.0}, 
            {'pose_5_x': 0.0}, {'pose_5_y': 11.0}, {'pose_5_z': 0.0},
            {'pose_6_x': 0.0}, {'pose_6_y': 5.5}, {'pose_6_z': 0.0},
            noise_variance,
            timing
        ]
    )

    # Position Publisher Nodes (p1 to p6)
    d1_publisher_node = Node(
        package='turtle_localization_pkg',
        executable='d1_publisher_with_noise',
        name='d1_publisher',
        parameters=[
            {'pose_1_x': 0.0},
            {'pose_1_y': 0.0},
            {'pose_1_z': 0.0},
            noise_mean,
            noise_variance,
            timing
        ]
    )

    d2_publisher_node = Node(
        package='turtle_localization_pkg',
        executable='d2_publisher_with_noise',
        name='d2_publisher',
        parameters=[
            {'pose_2_x': 11.0},
            {'pose_2_y': 0.0},
            {'pose_2_z': 0.0},
            noise_mean,
            noise_variance,
            timing        
        ]
    )

    d3_publisher_node = Node(
        package='turtle_localization_pkg',
        executable='d3_publisher_with_noise',
        name='d3_publisher',
        parameters=[
            {'pose_3_x': 11.0},
            {'pose_3_y': 5.5},
            {'pose_3_z': 0.0},
            noise_mean,
            noise_variance,
            timing
        ]
    )

    d4_publisher_node = Node(
        package='turtle_localization_pkg',
        executable='d4_publisher_with_noise',
        name='d4_publisher',
        parameters=[
            {'pose_4_x': 11.0},
            {'pose_4_y': 11.0},
            {'pose_4_z': 0.0},
            noise_mean,
            noise_variance,
            timing       
        ]
    )

    d5_publisher_node = Node(
        package='turtle_localization_pkg',
        executable='d5_publisher_with_noise',
        name='d5_publisher',
        parameters=[
            {'pose_5_x': 0.0},
            {'pose_5_y': 11.0},
            {'pose_5_z': 0.0},
            noise_mean,
            noise_variance,
            timing
        ]
    )

    d6_publisher_node = Node(
        package='turtle_localization_pkg',
        executable='d6_publisher_with_noise',
        name='d6_publisher',
        parameters=[
            {'pose_6_x': 0.0},
            {'pose_6_y': 5.5},
            {'pose_6_z': 0.0},
            noise_mean,
            noise_variance,
            timing
        ]
    )

    # Add actions (nodes) to the launch description
    ld.add_action(robot_position_ukf_node)
    ld.add_action(robot_position_noise_node)
    ld.add_action(robot_position_std_kf_node)
    ld.add_action(d1_publisher_node)
    ld.add_action(d2_publisher_node)
    ld.add_action(d3_publisher_node)
    ld.add_action(d4_publisher_node)
    ld.add_action(d5_publisher_node)
    ld.add_action(d6_publisher_node)
    ld.add_action(cirle_motion_node)
    ld.add_action(turtlesim_node)
    ld.add_action(plotjuggler_node)

    return ld

