#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from forward_kinematics.robot_model import Robot
from ament_index_python.packages import get_package_share_directory
import os


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        # Load robot model
        pkg_share = get_package_share_directory('forward_kinematics')
        config_file = os.path.join(pkg_share, 'config', 'robot_parameters.json')
        
        self.get_logger().info(f'Loading robot configuration from: {config_file}')
        self.robot = Robot.from_config(config_file)
        self.get_logger().info(f'Robot loaded: {self.robot.name} with {len(self.robot.links)} links')
        
        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/end_effector_marker', 10)
        self.workspace_marker_pub = self.create_publisher(Marker, '/workspace_boundary', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriber to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Current joint positions
        self.current_joint_positions = None
        
        # Timer for publishing
        self.timer = self.create_timer(0.1, self.publish_fk)
        
        self.get_logger().info('Forward Kinematics Node initialized!')
        
    def joint_state_callback(self, msg):
        """Callback for joint state updates."""
        try:
            # Extract positions for our controlled joints
            joint_positions = []
            for link in self.robot.links:
                # Find the corresponding joint in the message
                for i, name in enumerate(msg.name):
                    if 'revolute_joint' in name or 'robot_joint' in name or 'gripper_joint' in name:
                        if len(joint_positions) < len(self.robot.links):
                            joint_positions.append(msg.position[i])
            
            if len(joint_positions) == len(self.robot.links):
                self.current_joint_positions = joint_positions
                
        except Exception as e:
            self.get_logger().error(f'Error in joint_state_callback: {e}')
    
    def publish_fk(self):
        """Compute and publish forward kinematics."""
        if self.current_joint_positions is None:
            return
        
        try:
            # Compute forward kinematics
            T = self.robot.forward_kinematics(self.current_joint_positions)
            
            # Extract position and orientation
            position = T[:3, 3]
            rotation_matrix = T[:3, :3]
            
            # Convert rotation matrix to quaternion
            quat = self.rotation_matrix_to_quaternion(rotation_matrix)
            
            # Publish pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            pose_msg.pose.position.x = position[0]
            pose_msg.pose.position.y = position[1]
            pose_msg.pose.position.z = position[2]
            pose_msg.pose.orientation.x = quat[0]
            pose_msg.pose.orientation.y = quat[1]
            pose_msg.pose.orientation.z = quat[2]
            pose_msg.pose.orientation.w = quat[3]
            
            self.pose_pub.publish(pose_msg)
            
            # Publish TF
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'end_effector_fk'
            t.transform.translation.x = position[0]
            t.transform.translation.y = position[1]
            t.transform.translation.z = position[2]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(t)
            
            # Publish marker for visualization
            self.publish_marker(position)
            
        except Exception as e:
            self.get_logger().error(f'Error computing FK: {e}')
    
    def publish_marker(self, position):
        """Publish a marker for the end effector position."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'end_effector'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)
    
    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Convert rotation matrix to quaternion [x, y, z, w]."""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()