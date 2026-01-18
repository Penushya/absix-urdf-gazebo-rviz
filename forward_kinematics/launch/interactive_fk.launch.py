from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Interactive FK Node
    interactive_node = Node(
        package='forward_kinematics',
        executable='interactive_fk',
        name='interactive_fk_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        prefix='xterm -e'  # Opens in new terminal window
    )
    
    return LaunchDescription([
        interactive_node,
    ])