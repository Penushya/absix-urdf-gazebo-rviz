from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Launch arguments
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Include manipulator description launch
    manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('manipulator_description'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'use_rviz': use_rviz
        }.items()
    )
    
    # Forward Kinematics Node
    fk_node = Node(
        package='forward_kinematics',
        executable='fk_node',
        name='forward_kinematics_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Workspace Checker Node
    workspace_node = Node(
        package='forward_kinematics',
        executable='workspace_checker',
        name='workspace_checker_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_gazebo', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        manipulator_launch,
        fk_node,
        workspace_node,
    ])