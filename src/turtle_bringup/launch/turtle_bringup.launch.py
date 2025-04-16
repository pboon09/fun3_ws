#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('turtle_bringup')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'fun2.rviz')

    controller_freq = 100.0
    crazy_freq = 100.0
    controller_namespace = "xxxx"
    crazy_namespace = "yyyy"
    
    kill_turtle_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/remove_turtle', 'turtlesim/srv/Kill', '{name: turtle1}'],
        output='screen'
    )
    
    spawn_xxxx_turtle_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_turtle', 'turtlesim/srv/Spawn', f'{{x: 5.44, y: 5.44, theta: 0.0, name: {controller_namespace}}}'],
        output='screen'
    )
    
    spawn_yyyy_turtle_cmd = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn_turtle', 'turtlesim/srv/Spawn', f'{{x: 5.44, y: 5.44, theta: 0.0, name: {crazy_namespace}}}'],
        output='screen'
    )
    
    turtlesim_plus_node = Node(
        package='turtlesim_plus',
        executable='turtlesim_plus_node.py',
        name='turtlesim_plus_node',
        output='screen'
    )

    crazy_pizza_node = Node(
        package='turtle_bringup',
        executable='crazy_pizza.py',
        name='crazy_pizza',
        output='screen'
    )
    
    controller_node = Node(
        package='turtle_bringup',
        executable='controller.py',
        name='controller',
        namespace=controller_namespace,
        parameters=[{'frequency': controller_freq}],
        output='screen'
    )
    
    crazy_turtle_node = Node(
        package='turtle_bringup',
        executable='controller.py',
        name='crazy_turtle',
        namespace=crazy_namespace,
        parameters=[{'frequency': crazy_freq}],
        output='screen'
    )
    
    odom_publisher_node = Node(
        package='turtle_bringup',
        executable='odom_publisher.py',
        name='odom_publisher',
        parameters=[{'turtle1_name': controller_namespace}, {'turtle2_name': crazy_namespace}],
        output='screen'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    ld = LaunchDescription()
    
    ld.add_action(turtlesim_plus_node)
    ld.add_action(kill_turtle_cmd)
    ld.add_action(spawn_xxxx_turtle_cmd)
    ld.add_action(spawn_yyyy_turtle_cmd)

    ld.add_action(odom_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(crazy_pizza_node)
    ld.add_action(crazy_turtle_node)
    ld.add_action(controller_node)

    return ld

def main(args=None):
    generate_launch_description()

if __name__ == '__main__':
    main()