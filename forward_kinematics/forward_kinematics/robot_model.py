import json
import numpy as np
from scipy.optimize import least_squares
from typing import Tuple, Optional, List, Dict, Callable
import os

class Link:
    """
    Represents a robot link with DH parameters and physical properties.
    All units are SI: meters, radians, kilograms, kg⋅m².
    """
    def __init__(self, joint_type, d, a, alpha, mass, inertia_tensor, 
                 center_of_mass, theta_offset=0, actuator_id=None, joint_limits=None):
        self.joint_type = joint_type
        self.d = float(d)
        self.a = float(a)
        self.alpha = float(alpha)
        self.theta_offset = float(theta_offset)
        self.mass = float(mass)
        self.inertia_tensor = np.array(inertia_tensor, dtype=float)
        self.center_of_mass = np.array(center_of_mass, dtype=float)
        self.actuator_id = actuator_id
        self.joint_limits = joint_limits
        
        # State variables
        self.current_position = 0.0
        self.current_velocity = 0.0
        self.current_acceleration = 0.0
        self.current_torque = 0.0


class Robot:
    """
    Represents a robot manipulator with multiple links.
    """
    def __init__(self, name):
        self.name = name
        self.links = []
        self.coulomb_coeff = []
        self.viscous_coeff = []
        self.stiction_coeff = []
        self.stiction_velocity_threshold = []
        self.gravity_vector = np.array([0, 0, -9.81])
        self.joint_limits = []

    @classmethod
    def from_config(cls, config_file):
        """
        Create a robot instance from a configuration file.
        
        :param config_file: Path to robot configuration JSON file (absolute or relative)
        :return: Robot instance
        """
        # FIX: Check if config_file is already an absolute path
        if os.path.isabs(config_file) and os.path.exists(config_file):
            config_path = config_file
        else:
            # Try relative path from package
            config_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                'config', 
                config_file
            )
        
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        robot = cls(config['robot_name'])
        
        # Load links
        for link_params in config['links']:
            robot.add_link(
                joint_type=link_params['joint_type'],
                d=link_params['d'],
                a=link_params['a'],
                alpha=link_params['alpha'],
                mass=link_params['mass'],
                inertia_tensor=link_params['inertia_tensor'],
                center_of_mass=link_params['center_of_mass'],
                theta_offset=link_params.get('theta_offset', 0),
                actuator_id=link_params.get('actuator_id', None),
                joint_limits=link_params.get('joint_limits')
            )
            
            if 'joint_limits' in link_params:
                robot.joint_limits.append(link_params['joint_limits'])
            else:
                robot.joint_limits.append({
                    'position': {'min': -np.pi/2, 'max': np.pi/2},
                    'velocity': {'min': -np.pi, 'max': np.pi},
                    'acceleration': {'min': -10.0, 'max': 10.0},
                    'torque': {'min': -100.0, 'max': 100.0}
                })
        
        # Load friction parameters
        if 'friction_parameters' in config:
            robot.set_friction_parameters(
                config['friction_parameters']['coulomb_coefficients'],
                config['friction_parameters']['viscous_coefficients'],
                config['friction_parameters']['stiction_coefficients'],
                config['friction_parameters']['stiction_velocity_thresholds']
            )
        
        # Load gravity vector
        if 'gravity_vector' in config:
            robot.set_gravity_vector(config['gravity_vector'])
        
        return robot

    def add_link(self, joint_type, d, a, alpha, mass, inertia_tensor, 
                 center_of_mass, theta_offset=0, actuator_id=None, joint_limits=None):
        """Add a link to the robot."""
        self.links.append(Link(
            joint_type, d, a, alpha, mass, inertia_tensor,
            center_of_mass, theta_offset, actuator_id, joint_limits
        ))

    def set_friction_parameters(self, coulomb, viscous, stiction, stiction_threshold):
        """Set friction parameters for all joints."""
        self.coulomb_coeff = np.array(coulomb, dtype=float)
        self.viscous_coeff = np.array(viscous, dtype=float)
        self.stiction_coeff = np.array(stiction, dtype=float)
        self.stiction_velocity_threshold = np.array(stiction_threshold, dtype=float)

    def set_gravity_vector(self, gravity_vector):
        """Set gravity vector in world frame [m/s²]."""
        self.gravity_vector = np.array(gravity_vector, dtype=float)

    def forward_kinematics(self, joint_angles):
        """
        Compute forward kinematics for the robot.
        
        :param joint_angles: List of joint variables [rad] for revolute, [m] for prismatic
        :return: 4x4 homogeneous transformation matrix of end-effector
        """
        if len(joint_angles) != len(self.links):
            raise ValueError(f"Expected {len(self.links)} joint angles, got {len(joint_angles)}")
            
        T = np.eye(4)
        for link, q in zip(self.links, joint_angles):
            if link.joint_type == "revolute":
                theta = q + link.theta_offset
                d = link.d
            else:  # prismatic
                theta = link.theta_offset
                d = q + link.d
            
            T = T @ self.dh_transform(theta, d, link.a, link.alpha)
        return T

    def inverse_kinematics(self, position, orientation, initial_guess=None, tolerance=1e-6, max_iter=100):
        """
        Compute inverse kinematics numerically using position-only objective for planar robot.
        
        :param position: Desired end-effector position [m]
        :param orientation: Desired end-effector orientation (3x3 rotation matrix) - ignored for planar
        :param initial_guess: Initial joint angles [rad]
        :param tolerance: Convergence tolerance
        :param max_iter: Maximum iterations
        :return: Joint angles [rad] that achieve desired pose
        """
        position = np.array(position, dtype=float)
        
        # FIX: For planar robot, use position-only objective (ignore Z and orientation)
        def objective(q):
            T = self.forward_kinematics(q)
            current_position = T[:3, 3]
            # Only care about X and Y for planar robot (Z is always ~0)
            position_error = position[:2] - current_position[:2]
            return position_error

        # FIX: Better initial guess using geometric approach
        if initial_guess is None:
            # Calculate approximate angles based on target position
            x, y = position[0], position[1]
            distance = np.sqrt(x**2 + y**2)
            base_angle = np.arctan2(y, x)
            
            # Get total reach
            total_reach = sum(link.a for link in self.links)
            
            if distance > total_reach * 0.95:
                # Target near workspace boundary - extend arm
                initial_guess = np.array([base_angle, 0.0, 0.0])
            elif distance < total_reach * 0.3:
                # Target close to base - fold arm
                initial_guess = np.array([base_angle, np.pi/3, -np.pi/3])
            else:
                # General case - reasonable starting configuration
                initial_guess = np.array([base_angle, np.pi/6, -np.pi/6])
        else:
            initial_guess = np.array(initial_guess)

        # Get joint limits
        lower_bounds = []
        upper_bounds = []
        for link in self.links:
            if link.joint_limits:
                lower_bounds.append(link.joint_limits['position']['min'])
                upper_bounds.append(link.joint_limits['position']['max'])
            else:
                lower_bounds.append(-np.pi)
                upper_bounds.append(np.pi)
        
        bounds = (np.array(lower_bounds), np.array(upper_bounds))

        # FIX: Use multiple random restarts if initial solve fails
        best_solution = None
        best_error = float('inf')
        
        attempts = [
            initial_guess,
            np.array([np.arctan2(position[1], position[0]), 0.5, -0.5]),
            np.array([np.arctan2(position[1], position[0]), -0.5, 0.5]),
            np.array([np.arctan2(position[1], position[0]) + 0.2, 0.3, -0.3]),
            np.array([np.arctan2(position[1], position[0]) - 0.2, 0.3, -0.3]),
        ]
        
        for attempt in attempts:
            # Clip initial guess to bounds
            attempt = np.clip(attempt, lower_bounds, upper_bounds)
            
            try:
                result = least_squares(
                    objective, 
                    attempt, 
                    bounds=bounds,
                    ftol=tolerance,
                    xtol=tolerance,
                    gtol=tolerance,
                    max_nfev=max_iter * 10,
                    method='trf',
                    diff_step=0.01  # FIX: Explicit step size for finite differences
                )
                
                # Check solution quality
                T_result = self.forward_kinematics(result.x)
                error = np.linalg.norm(T_result[:2, 3] - position[:2])
                
                if error < best_error:
                    best_error = error
                    best_solution = result.x
                    
                # If we found a good solution, stop early
                if error < 0.001:  # 1mm accuracy
                    break
                    
            except Exception as e:
                continue
        
        if best_solution is None:
            print("Warning: Inverse kinematics failed to find a solution")
            return initial_guess
            
        if best_error > 0.01:
            print(f"Warning: IK solution has {best_error*1000:.2f}mm error")
        
        return best_solution

    def jacobian(self, joint_angles):
        """
        Compute the geometric Jacobian of the robot.
        
        :param joint_angles: Current joint angles [rad]
        :return: 6xn Jacobian matrix
        """
        if len(joint_angles) != len(self.links):
            raise ValueError(f"Expected {len(self.links)} joint angles, got {len(joint_angles)}")
            
        J = np.zeros((6, len(self.links)))
        T = np.eye(4)
        
        # Get end-effector position
        T_ee = self.forward_kinematics(joint_angles)
        p_ee = T_ee[:3, 3]
        
        for i, (link, theta) in enumerate(zip(self.links, joint_angles)):
            if link.joint_type == "revolute":
                z = T[:3, 2]  # z-axis of current frame
                p = p_ee - T[:3, 3]  # vector to end-effector
                J[:3, i] = np.cross(z, p)  # linear velocity component
                J[3:, i] = z  # angular velocity component
            else:  # prismatic
                z = T[:3, 2]
                J[:3, i] = z  # linear velocity component
                J[3:, i] = 0  # no angular velocity for prismatic joint
            
            # Update transformation to next frame
            if link.joint_type == "revolute":
                Ti = self.dh_transform(theta + link.theta_offset, link.d, link.a, link.alpha)
            else:
                Ti = self.dh_transform(link.theta_offset, theta + link.d, link.a, link.alpha)
            T = T @ Ti
        
        return J

    @staticmethod
    def dh_transform(theta, d, a, alpha):
        """
        Compute DH transformation matrix.
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    @staticmethod
    def orientation_error(current, desired):
        """
        Compute orientation error between two rotation matrices.
        """
        error_matrix = desired @ current.T - np.eye(3)
        return np.array([error_matrix[2, 1], error_matrix[0, 2], error_matrix[1, 0]]) / 2

    def apply_joint_limits(self, joint_angles):
        """
        Apply joint limits to given joint angles.
        """
        return np.array([
            np.clip(angle, limit['position']['min'], limit['position']['max'])
            for angle, limit in zip(joint_angles, self.joint_limits)
        ])

    def get_actuator_ids(self):
        """Get list of actuator IDs used in the robot."""
        return [link.actuator_id for link in self.links if link.actuator_id is not None]

    def update_joint_states(self, actuator_controller):
        """Update joint states from actuator readings."""
        for link in self.links:
            if link.actuator_id is not None:
                raw_pos = actuator_controller.get_position(link.actuator_id)
                raw_vel = actuator_controller.get_velocity(link.actuator_id)
                raw_curr = actuator_controller.get_current(link.actuator_id)
                link.current_position = (
                    actuator_controller.convert_position_from_raw(link.actuator_id, raw_pos) - 
                    link.theta_offset
                )
                link.current_velocity = actuator_controller.convert_velocity_from_raw(
                    link.actuator_id, raw_vel
                )
                link.current_torque = actuator_controller.convert_current_to_torque(
                    link.actuator_id, raw_curr
                )

    def command_joint_positions(self, actuator_controller, joint_positions):
        """Command joint positions."""
        if len(joint_positions) != len(self.links):
            raise ValueError(f"Expected {len(self.links)} joint positions, got {len(joint_positions)}")
        joint_positions = self.apply_joint_limits(joint_positions)
        for link, angle in zip(self.links, joint_positions):
            if link.actuator_id is not None:
                actuator_value = actuator_controller.convert_joint_angle_to_raw(
                    link.actuator_id, angle + link.theta_offset
                )
                actuator_controller.set_position(link.actuator_id, actuator_value)

    def command_joint_velocities(self, actuator_controller, joint_velocities):
        """Command joint velocities."""
        if len(joint_velocities) != len(self.links):
            raise ValueError(f"Expected {len(self.links)} joint velocities, got {len(joint_velocities)}")
        for link, velocity in zip(self.links, joint_velocities):
            if link.actuator_id is not None:
                actuator_value = actuator_controller.convert_velocity_to_raw(
                    link.actuator_id, velocity
                )
                actuator_controller.set_velocity(link.actuator_id, actuator_value)

    def command_joint_torques(self, actuator_controller, joint_torques):
        """Command joint torques."""
        if len(joint_torques) != len(self.links):
            raise ValueError(f"Expected {len(self.links)} joint torques, got {len(joint_torques)}")
        for link, torque in zip(self.links, joint_torques):
            if link.actuator_id is not None:
                current = actuator_controller.convert_torque_to_current(
                    link.actuator_id, torque
                )
                actuator_controller.set_current(link.actuator_id, current)

    def get_joint_states(self, actuator_controller):
        """Get current joint states."""
        self.update_joint_states(actuator_controller)
        return {
            'positions': [link.current_position for link in self.links],
            'velocities': [link.current_velocity for link in self.links],
            'torques': [link.current_torque for link in self.links]
        }


# Example usage
if __name__ == "__main__":
    # Quick test of IK
    sample_config = {
        "robot_name": "Test_Planar_Robot",
        "links": [
            {"joint_type": "revolute", "d": 0, "a": 0.10, "alpha": 0, "mass": 1.0, 
             "inertia_tensor": [[1,0,0],[0,1,0],[0,0,1]], "center_of_mass": [0,0,0],
             "joint_limits": {"position": {"min": -3.14, "max": 3.14}, "velocity": {"min": -6.28, "max": 6.28}}},
            {"joint_type": "revolute", "d": 0, "a": 0.07, "alpha": 0, "mass": 1.0, 
             "inertia_tensor": [[1,0,0],[0,1,0],[0,0,1]], "center_of_mass": [0,0,0],
             "joint_limits": {"position": {"min": -1.57, "max": 1.57}, "velocity": {"min": -6.28, "max": 6.28}}},
            {"joint_type": "revolute", "d": 0, "a": 0.05, "alpha": 0, "mass": 1.0, 
             "inertia_tensor": [[1,0,0],[0,1,0],[0,0,1]], "center_of_mass": [0,0,0],
             "joint_limits": {"position": {"min": -1.57, "max": 1.57}, "velocity": {"min": -6.28, "max": 6.28}}}
        ]
    }

    with open('/tmp/test_robot.json', 'w') as f:
        json.dump(sample_config, f)

    robot = Robot.from_config('/tmp/test_robot.json')
    
    # Test IK for target (0.15, 0.10, 0.0)
    target = np.array([0.15, 0.10, 0.0])
    print(f"Target: {target}")
    
    solution = robot.inverse_kinematics(target, np.eye(3))
    print(f"IK Solution (rad): {solution}")
    print(f"IK Solution (deg): {np.rad2deg(solution)}")
    
    # Verify with FK
    T = robot.forward_kinematics(solution)
    achieved = T[:3, 3]
    print(f"Achieved position: {achieved}")
    print(f"Error: {np.linalg.norm(achieved[:2] - target[:2]) * 1000:.2f} mm")
