from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('interactive', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='2.0'),
        DeclareLaunchArgument('goal_y', default_value='-3.0'),
        DeclareLaunchArgument('goal_z', default_value='4.5'),
        DeclareLaunchArgument('tomogram', default_value='building2_9'),
        DeclareLaunchArgument('path_topic', default_value='/pct_path_raw'),
        DeclareLaunchArgument('frame_id', default_value='building_pct'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='pct_planner', executable='pct_plan', output='screen',
             parameters=[{'use_sim_time':ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)}],
             arguments=['--scene','Go2',
                        '--tomogram',LaunchConfiguration('tomogram'),
                        '--path-topic',LaunchConfiguration('path_topic'),
                        '--frame-id',LaunchConfiguration('frame_id'),
                        '--interactive', LaunchConfiguration('interactive'),
                        '--goal', LaunchConfiguration('goal_x'),
                        LaunchConfiguration('goal_y'), LaunchConfiguration('goal_z')]),
    ])
