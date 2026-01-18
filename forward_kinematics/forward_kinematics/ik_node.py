#!/usr/bin/env python3
"""
Inverse Kinematics Node - Complete version with QoS fix
Replace: ~/absix_ws/src/forward_kinematics/forward_kinematics/ik_node.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from visualization_msgs.msg import Marker
import numpy as np
from forward_kinematics.robot_model import Robot
from ament_index_python.packages import get_package_share_directory
import os


class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        
        # Load robot model
        pkg_share = get_package_share_directory('forward_kinematics')
        config_file = os.path.join(pkg_share, 'config', 'robot_parameters.json')
        
        self.get_logger().info(f'Loading robot configuration from: {config_file}')
        self.robot = Robot.from_config(config_file)
        self.get_logger().info(f'Robot loaded: {self.robot.name} with {len(self.robot.links)} links')
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        self.target_marker_pub = self.create_publisher(Marker, '/ik_target_marker', 10)
        self.solution_pub = self.create_publisher(JointState, '/ik_solution', 10)
        
        # QoS profile for subscription - FIX FOR TOPIC PUB COMPATIBILITY
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriber for target pose with QoS
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/ik_target_pose',
            self.ik_callback,
            qos_profile
        )
        
        # Current joint state
        self.current_joint_positions = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Inverse Kinematics Node initialized!')
        self.get_logger().info('Waiting for target poses on /ik_target_pose')
        
    def joint_state_callback(self, msg):
        """Store current joint positions for IK initial guess."""
        try:
            joint_positions = []
            for link in self.robot.links:
                for i, name in enumerate(msg.name):
                    if 'robot_joint' in name or 'gripper_joint' in name:
                        if len(joint_positions) < len(self.robot.links):
                            joint_positions.append(msg.position[i])
            
            if len(joint_positions) == len(self.robot.links):
                self.current_joint_positions = joint_positions
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {e}')
    
    def ik_callback(self, msg):
        """
        Compute IK for target pose and command robot.
        """
        target_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        # Convert quaternion to rotation matrix (simplified - use identity for now)
        # In a full implementation, you'd convert the quaternion properly
        target_orientation = np.eye(3)
        
        self.get_logger().info(f'Received target: ({target_position[0]:.3f}, '
                              f'{target_position[1]:.3f}, {target_position[2]:.3f})')
        
        # Use current position as initial guess, or zeros if not available
        initial_guess = self.current_joint_positions if self.current_joint_positions else None
        
        try:
            # Compute inverse kinematics
            joint_solution = self.robot.inverse_kinematics(
                target_position,
                target_orientation,
                initial_guess=initial_guess,
                tolerance=1e-4,
                max_iter=200
            )
            
            # Verify solution with forward kinematics
            T_result = self.robot.forward_kinematics(joint_solution)
            achieved_position = T_result[:3, 3]
            error = np.linalg.norm(achieved_position - target_position)
            
            self.get_logger().info(f'IK Solution: {np.rad2deg(joint_solution)}°')
            self.get_logger().info(f'Position error: {error*1000:.2f} mm')
            
            if error < 0.01:  # 1cm tolerance
                self.get_logger().info('✓ Solution found! Sending command...')
                
                # Publish joint solution
                self.publish_joint_solution(joint_solution)
                
                # Command robot to move
                self.command_robot(joint_solution)
                
                # Publish target marker
                self.publish_target_marker(target_position, True)
            else:
                self.get_logger().warn(f'✗ Solution error too large: {error*1000:.2f} mm')
                self.publish_target_marker(target_position, False)
                
        except Exception as e:
            self.get_logger().error(f'IK computation failed: {e}')
            self.publish_target_marker(target_position, False)
    
    def publish_joint_solution(self, joint_angles):
        """Publish the IK solution as JointState message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['robot_joint1', 'robot_joint2', 'gripper_joint']
        msg.position = joint_angles.tolist()
        
        self.solution_pub.publish(msg)
    
    def command_robot(self, joint_angles, duration_sec=3.0):
        """Send joint trajectory command to robot."""
        msg = JointTrajectory()
        msg.joint_names = ['robot_joint1', 'robot_joint2', 'gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles.tolist()
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        
        msg.points.append(point)
        self.joint_cmd_pub.publish(msg)
    
    def publish_target_marker(self, position, success):
        """Publish visualization marker for target position."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'ik_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        
        if success:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.target_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()