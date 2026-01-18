#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker
import numpy as np
from forward_kinematics.robot_model import Robot
from ament_index_python.packages import get_package_share_directory
import os


class WorkspaceCheckerNode(Node):
    def __init__(self):
        super().__init__('workspace_checker_node')
        
        # Load robot model
        pkg_share = get_package_share_directory('forward_kinematics')
        config_file = os.path.join(pkg_share, 'config', 'robot_parameters.json')
        
        self.robot = Robot.from_config(config_file)
        self.get_logger().info(f'Robot loaded: {self.robot.name}')
        
        # Sample workspace
        self.workspace_points = self.sample_workspace(samples_per_joint=15)
        self.workspace_bounds = self.get_workspace_bounds()
        
        self.get_logger().info('Workspace sampled!')
        self.get_logger().info(f"X: [{self.workspace_bounds['x']['min']:.4f}, {self.workspace_bounds['x']['max']:.4f}]")
        self.get_logger().info(f"Y: [{self.workspace_bounds['y']['min']:.4f}, {self.workspace_bounds['y']['max']:.4f}]")
        self.get_logger().info(f"Z: [{self.workspace_bounds['z']['min']:.4f}, {self.workspace_bounds['z']['max']:.4f}]")
        
        # Publishers
        self.result_pub = self.create_publisher(String, '/reachability_result', 10)
        self.marker_pub = self.create_publisher(Marker, '/target_point_marker', 10)
        
        # Subscriber for target points
        self.point_sub = self.create_subscription(
            PointStamped,
            '/target_point',
            self.check_point_callback,
            10
        )
        
        # Publish workspace boundary visualization
        self.timer = self.create_timer(1.0, self.publish_workspace_boundary)
        
    def sample_workspace(self, samples_per_joint=15):
        """Sample the robot's workspace."""
        self.get_logger().info(f'Sampling workspace with {samples_per_joint} samples per joint...')
        
        joint_samples = []
        for link in self.robot.links:
            if link.joint_limits:
                min_pos = link.joint_limits['position']['min']
                max_pos = link.joint_limits['position']['max']
            else:
                min_pos = -np.pi/2
                max_pos = np.pi/2
            
            samples = np.linspace(min_pos, max_pos, samples_per_joint)
            joint_samples.append(samples)
        
        joint_grids = np.meshgrid(*joint_samples, indexing='ij')
        all_configs = np.column_stack([grid.ravel() for grid in joint_grids])
        
        workspace = []
        for config in all_configs:
            try:
                T = self.robot.forward_kinematics(config)
                position = T[:3, 3]
                workspace.append(position)
            except:
                continue
        
        workspace_array = np.array(workspace)
        self.get_logger().info(f'Generated {len(workspace_array)} workspace points')
        return workspace_array
    
    def get_workspace_bounds(self):
        """Get workspace bounding box."""
        return {
            'x': {'min': np.min(self.workspace_points[:, 0]), 
                  'max': np.max(self.workspace_points[:, 0])},
            'y': {'min': np.min(self.workspace_points[:, 1]), 
                  'max': np.max(self.workspace_points[:, 1])},
            'z': {'min': np.min(self.workspace_points[:, 2]), 
                  'max': np.max(self.workspace_points[:, 2])}
        }
    
    def check_point_callback(self, msg):
        """Check if a point is reachable."""
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z
        
        target = np.array([x, y, z])
        distances = np.linalg.norm(self.workspace_points - target, axis=1)
        min_distance = np.min(distances)
        closest_idx = np.argmin(distances)
        closest_point = self.workspace_points[closest_idx]
        
        tolerance = 0.01  # 1cm tolerance
        is_reachable = min_distance <= tolerance
        
        # Publish result
        result_msg = String()
        result_msg.data = (
            f"Target: ({x:.4f}, {y:.4f}, {z:.4f})\\n"
            f"Reachable: {'YES ✓' if is_reachable else 'NO ✗'}\\n"
            f"Min Distance: {min_distance:.4f} m\\n"
            f"Closest Point: ({closest_point[0]:.4f}, {closest_point[1]:.4f}, {closest_point[2]:.4f})"
        )
        self.result_pub.publish(result_msg)
        
        self.get_logger().info(result_msg.data)
        
        # Publish marker
        self.publish_target_marker(target, is_reachable)
    
    def publish_target_marker(self, position, is_reachable):
        """Publish marker for target point."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_point'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        
        if is_reachable:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        marker.color.a = 0.8
        
        self.marker_pub.publish(marker)
    
    def publish_workspace_boundary(self):
        """Publish workspace boundary box."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'workspace'
        marker.id = 2
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        
        bounds = self.workspace_bounds
        
        # Define 8 corners of the bounding box
        corners = [
            [bounds['x']['min'], bounds['y']['min'], bounds['z']['min']],
            [bounds['x']['max'], bounds['y']['min'], bounds['z']['min']],
            [bounds['x']['max'], bounds['y']['max'], bounds['z']['min']],
            [bounds['x']['min'], bounds['y']['max'], bounds['z']['min']],
            [bounds['x']['min'], bounds['y']['min'], bounds['z']['max']],
            [bounds['x']['max'], bounds['y']['min'], bounds['z']['max']],
            [bounds['x']['max'], bounds['y']['max'], bounds['z']['max']],
            [bounds['x']['min'], bounds['y']['max'], bounds['z']['max']],
        ]
        
        # Define edges
        edges = [
            (0,1), (1,2), (2,3), (3,0),  # Bottom face
            (4,5), (5,6), (6,7), (7,4),  # Top face
            (0,4), (1,5), (2,6), (3,7)   # Vertical edges
        ]
        
        for edge in edges:
            p1 = Point()
            p1.x, p1.y, p1.z = corners[edge[0]]
            marker.points.append(p1)
            
            p2 = Point()
            p2.x, p2.y, p2.z = corners[edge[1]]
            marker.points.append(p2)
        
        marker.scale.x = 0.005  # Line width
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.5
        
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceCheckerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()