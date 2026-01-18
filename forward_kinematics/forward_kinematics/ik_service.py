#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import numpy as np
from forward_kinematics.robot_model import Robot
from ament_index_python.packages import get_package_share_directory
import os

# You'll need to create a custom service, but for now we'll use standard messages
# For a complete implementation, create: srv/ComputeIK.srv


class IKServiceNode(Node):
    def __init__(self):
        super().__init__('ik_service_node')
        
        # Load robot model
        pkg_share = get_package_share_directory('forward_kinematics')
        config_file = os.path.join(pkg_share, 'config', 'robot_parameters.json')
        
        self.robot = Robot.from_config(config_file)
        self.get_logger().info(f'IK Service ready for {self.robot.name}')
        
        # Note: For a full implementation, create a custom service
        # For now, this shows the structure
        
    def compute_ik(self, target_position, target_orientation, initial_guess=None):
        """
        Compute inverse kinematics.
        
        Args:
            target_position: [x, y, z] in meters
            target_orientation: 3x3 rotation matrix
            initial_guess: Initial joint angles
            
        Returns:
            joint_angles: Solution joint angles
            success: Whether IK succeeded
            error: Position error
        """
        try:
            joint_solution = self.robot.inverse_kinematics(
                target_position,
                target_orientation,
                initial_guess=initial_guess,
                tolerance=1e-4,
                max_iter=200
            )
            
            # Verify solution
            T_result = self.robot.forward_kinematics(joint_solution)
            achieved_position = T_result[:3, 3]
            error = np.linalg.norm(achieved_position - target_position)
            
            success = error < 0.01  # 1cm tolerance
            
            return joint_solution, success, error
            
        except Exception as e:
            self.get_logger().error(f'IK failed: {e}')
            return None, False, float('inf')


def main(args=None):
    rclpy.init(args=args)
    node = IKServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
