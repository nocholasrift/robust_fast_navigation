import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Path Setup
    pkg_share = get_package_share_directory('robust_fast_navigation')
    
    # 2. Declare Arguments (equivalent to <arg>)
    barn_arg = DeclareLaunchArgument('barn', default_value='false')
    barn_dist_arg = DeclareLaunchArgument('barn_dist', default_value='10.0')

    # 3. Define Paths to YAML files
    planner_params = os.path.join(pkg_share, 'params', 'planner_ros2.yaml')
    robo_params = os.path.join(pkg_share, 'params', 'robo_params_ros2.yaml')
    global_costmap_params = os.path.join(pkg_share, 'params', 'global_costmap_ros2.yaml')

    # Launch config variables
    ld = LaunchDescription()
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='False'))
    ld.add_action(DeclareLaunchArgument('slam_params_file', default_value=os.path.join(
        pkg_share, 'params', 'mapper_params_online_async.yaml',
    )))

    # 4. Define the Node
    planner_node = Node(
        package='robust_fast_navigation',
        executable='planner_node',
        name='robust_planner_node', # Must match the namespace in your YAML files!
        output='screen',
        # Equivalent to <param> and <rosparam>
        parameters=[
            planner_params,
            robo_params,
            global_costmap_params,
            {
                'is_barn': LaunchConfiguration('barn'),
                'barn_goal_dist': LaunchConfiguration('barn_dist'),
                'use_sim_time': use_sim_time,
            }
        ],
        # Equivalent to <remap>
        remappings=[
            ('/planner_goal', '/goal_pose'),
            ('/odometry/filtered', '/gmapping/odometry'),
        ]
    )

    ld.add_action(barn_arg)
    ld.add_action(barn_dist_arg)
    ld.add_action(planner_node)

    return ld