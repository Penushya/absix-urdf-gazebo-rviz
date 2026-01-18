#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np


class InteractiveIKNode(Node):
    def __init__(self):
        super().__init__('interactive_ik_node')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/ik_target_pose', 10)
        
        self.get_logger().info('Interactive IK Node started!')
        self.get_logger().info('Send target positions to compute inverse kinematics')
    
    def publish_target(self, x, y, z):
        """Publish target pose for IK computation."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Default orientation (identity quaternion)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        self.pose_pub.publish(msg)
        self.get_logger().info(f'Published target: ({x:.3f}, {y:.3f}, {z:.3f})')


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveIKNode()
    
    print('\n' + '='*60)
    print('🎯 INTERACTIVE INVERSE KINEMATICS')
    print('='*60)
    print('Enter target position (x y z) in meters')
    print('Based on workspace bounds:')
    print('  X: [-0.22, 0.22]')
    print('  Y: [-0.22, 0.22]')
    print('  Z: [0.00, 0.00] (for planar robot)')
    print('\nPredefined targets:')
    print('  p1    - Point 1: (0.15, 0.10, 0.0)')
    print('  p2    - Point 2: (0.10, 0.15, 0.0)')
    print('  p3    - Point 3: (-0.10, 0.10, 0.0)')
    print('  p4    - Point 4: (0.20, 0.0, 0.0)')
    print('  demo  - Run demo sequence')
    print('  q     - Quit')
    print('='*60 + '\n')
    
    # Predefined targets
    targets = {
        'p1': (0.15, 0.10, 0.0),
        'p2': (0.10, 0.15, 0.0),
        'p3': (-0.10, 0.10, 0.0),
        'p4': (0.20, 0.0, 0.0),
    }
    
    import threading
    import time
    
    def spin_thread():
        rclpy.spin(node)
    
    thread = threading.Thread(target=spin_thread, daemon=True)
    thread.start()
    
    try:
        while rclpy.ok():
            user_input = input('🎯 Target: ').strip().lower()
            
            if user_input in ['q', 'quit', 'exit']:
                break
            elif user_input == 'demo':
                print('▶️  Running IK demo sequence...')
                for name, (x, y, z) in targets.items():
                    print(f'  → {name}: ({x}, {y}, {z})')
                    node.publish_target(x, y, z)
                    time.sleep(4)  # Wait for robot to reach position
                print('✅ Demo complete!')
            elif user_input in targets:
                x, y, z = targets[user_input]
                node.publish_target(x, y, z)
            else:
                try:
                    coords = [float(c) for c in user_input.split()]
                    if len(coords) == 3:
                        node.publish_target(coords[0], coords[1], coords[2])
                    elif len(coords) == 2:
                        # Assume Z=0 for 2D input
                        node.publish_target(coords[0], coords[1], 0.0)
                    else:
                        print('❌ Enter 2 or 3 coordinates (x y [z])')
                except ValueError:
                    print('❌ Invalid input')
                    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()