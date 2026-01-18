from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Launch Gazebo with robot
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
    fk_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='forward_kinematics',
                executable='fk_node',
                name='forward_kinematics_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Inverse Kinematics Node
    ik_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='forward_kinematics',
                executable='ik_node',
                name='inverse_kinematics_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Workspace Checker Node
    workspace_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='forward_kinematics',
                executable='workspace_checker',
                name='workspace_checker_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Interactive IK (in new terminal)
    interactive_ik = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='forward_kinematics',
                executable='interactive_ik',
                name='interactive_ik_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                prefix='gnome-terminal --'  # Opens in new terminal
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        manipulator_launch,
        fk_node,
        ik_node,
        workspace_node,
        interactive_ik,
    ])