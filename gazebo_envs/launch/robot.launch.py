import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    #path to description package 
    pkg_description = launch_ros.substitutions.FindPackageShare(package='diff_rob_description').find('diff_rob_description')
    #path to gazebo package
    pkg_gazebo = launch_ros.substitutions.FindPackageShare(package='gazebo_envs').find('gazebo_envs')
    #path to urdf file
    default_model_path = os.path.join(pkg_description, 'description/diff_robot.urdf.xacro')
    #path to rviz config 
    default_rviz_config_path = os.path.join(pkg_gazebo, 'rviz/diff_rob.rviz')
    #path to world file
    world_path = os.path.join(pkg_gazebo, 'world/fiducial.world')
    #path to yaml file
    yaml_path = os.path.join(pkg_gazebo, 'config', 'landmarks_params.yaml')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')]), 'use_sim_time': True}]
    )
    robot_path = launch_ros.actions.Node(
        package='diff_rob_model',
        executable='traj',
        name='traj'
    )
    cam_node = launch_ros.actions.Node(
        package='diff_rob_model',
        executable='land_cam',
        name='land_cam',
        parameters=[yaml_path]
    )
    markers_node = launch_ros.actions.Node(
        package='gazebo_envs',
        executable='rviz_markers',
        name='params',
        parameters=[yaml_path]
    )
    encoders_node = launch_ros.actions.Node(
        package='diff_rob_model',
        executable='enc_sim',
        name='encoders',
    )
    ekf_node = launch_ros.actions.Node(
        package='diff_rob_model',
        executable='ekf_loc',
        name='params',
        parameters=[yaml_path]
    )
    # joint_state_publisher_node = launch_ros.actions.Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )
    # joint_state_publisher_gui_node = launch_ros.actions.Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    # )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'diff_robot', '-topic', 'robot_description', '-x', '0.0'],
        output='screen'
    )

    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
        #               description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        # launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
        #                                     description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path]
                                            , output='screen'),
        #joint_state_publisher_node,
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_path,
        cam_node,
        markers_node,
        encoders_node,
        rviz_node,
        ekf_node
    ])