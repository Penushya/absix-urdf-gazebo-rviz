#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import sys


class InteractiveFKNode(Node):
    def __init__(self):
        super().__init__('interactive_fk_node')
        
        self.point_pub = self.create_publisher(PointStamped, '/target_point', 10)
        
        self.get_logger().info('Interactive FK Node started!')
        self.get_logger().info('Enter coordinates to check reachability')
        self.get_logger().info('Format: x y z (e.g., 0.15 0.10 0.05)')
        
    def publish_point(self, x, y, z):
        """Publish a target point."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        
        self.point_pub.publish(msg)
        self.get_logger().info(f'Published target point: ({x}, {y}, {z})')


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveFKNode()
    
    print("\\n" + "="*60)
    print("Interactive Forward Kinematics Checker")
    print("="*60)
    print("Enter coordinates (x y z) or 'q' to quit")
    print("Example: 0.15 0.10 0.05")
    print("="*60 + "\\n")
    
    import threading
    
    def spin_thread():
        rclpy.spin(node)
    
    thread = threading.Thread(target=spin_thread, daemon=True)
    thread.start()
    
    try:
        while rclpy.ok():
            user_input = input("Enter coordinates: ").strip()
            
            if user_input.lower() in ['q', 'quit', 'exit']:
                break
            
            try:
                coords = [float(x) for x in user_input.split()]
                if len(coords) == 3:
                    node.publish_point(coords[0], coords[1], coords[2])
                else:
                    print("Please enter exactly 3 coordinates (x y z)")
            except ValueError:
                print("Invalid input. Please enter numeric coordinates.")
            except Exception as e:
                print(f"Error: {e}")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()