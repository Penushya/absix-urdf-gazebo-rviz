#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class JointCommander(Node):
    def __init__(self):
        super().__init__('joint_commander_node')
        
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Joint Commander initialized! Waiting for controller...')
        time.sleep(2)
        
    def send_joint_command(self, positions, duration_sec=2.0):
        msg = JointTrajectory()
        msg.joint_names = ['robot_joint1', 'robot_joint2', 'gripper_joint']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(duration_sec), 
            nanosec=int((duration_sec % 1) * 1e9)
        )
        
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f'Command sent: {positions}')


def main(args=None):
    rclpy.init(args=args)
    node = JointCommander()
    
    print('\n' + '='*60)
    print('🎮 ROBOT JOINT COMMANDER')
    print('='*60)
    print('Commands:')
    print('  home     - [0.0, 0.0, 0.0]')
    print('  up       - [0.5, 0.5, 0.0]')
    print('  down     - [-0.5, -0.5, 0.0]')
    print('  left     - [0.8, 0.0, 0.0]')
    print('  right    - [-0.8, 0.0, 0.0]')
    print('  grip     - [0.0, 0.0, 0.8]')
    print('  release  - [0.0, 0.0, -0.8]')
    print('  demo     - Run full demo')
    print('  custom   - Enter custom angles')
    print('  q        - Quit')
    print('='*60 + '\n')
    
    positions = {
        'home': [0.0, 0.0, 0.0],
        'up': [0.5, 0.5, 0.0],
        'down': [-0.5, -0.5, 0.0],
        'left': [0.8, 0.0, 0.0],
        'right': [-0.8, 0.0, 0.0],
        'grip': [0.0, 0.0, 0.8],
        'release': [0.0, 0.0, -0.8],
    }
    
    import threading
    thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    thread.start()
    
    try:
        while rclpy.ok():
            cmd = input('🤖 Command: ').strip().lower()
            
            if cmd in ['q', 'quit', 'exit']:
                break
            elif cmd == 'demo':
                print('▶️  Running demo...')
                for pos_name, pos in [('home', positions['home']),
                                      ('up', positions['up']),
                                      ('grip', positions['grip']),
                                      ('down', positions['down']),
                                      ('home', positions['home'])]:
                    print(f'  → {pos_name}')
                    node.send_joint_command(pos, 2.0)
                    time.sleep(2.5)
                print('✅ Demo complete!')
            elif cmd == 'custom':
                try:
                    angles = input('Enter 3 angles (j1 j2 gripper): ').split()
                    pos = [float(a) for a in angles]
                    if len(pos) == 3:
                        node.send_joint_command(pos, 2.0)
                    else:
                        print('❌ Need exactly 3 angles')
                except ValueError:
                    print('❌ Invalid numbers')
            elif cmd in positions:
                node.send_joint_command(positions[cmd], 2.0)
            else:
                print(f'❓ Unknown: {cmd}')
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()