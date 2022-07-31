import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_gazebo = launch_ros.substitutions.FindPackageShare(package='gazebo_envs').find('gazebo_envs')
    #world_path = os.path.join(pkg_gazebo, 'world/fiducial.world')
    yaml_path = os.path.join(pkg_gazebo, 'config/landmarks_params.yaml')


    # cam_node = launch_ros.actions.Node(
    #     package='diff_rob_model',
    #     executable='land_cam',
    #     name='params_aruco',
    #     parameters=[yaml_path]
    # )
    params_node = launch_ros.actions.Node(
        package='gazebo_envs',
        executable='rviz_markers',
        name='params',
        parameters=[yaml_path]
    )
    
    
    return launch.LaunchDescription([
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path]
        #                                     , output='screen'),
        # cam_node
        params_node
    ])