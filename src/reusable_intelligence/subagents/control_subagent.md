# Control Subagent

This subagent handles the control systems for Physical AI robots, managing low-level motor control, trajectory execution, and safety systems.

## Purpose
- Execute high-level commands from planning systems
- Manage low-level motor control and feedback
- Ensure safe operation within physical constraints
- Handle trajectory following and motion control

## Inputs
- High-level commands (navigation goals, manipulation tasks)
- Robot state (joint positions, velocities, effort)
- Environmental constraints and obstacles
- Safety limits and operational constraints

## Outputs
- Low-level motor commands
- Robot state feedback
- Safety status and system health
- Execution status and error reports

## Architecture

The Control Subagent follows a hierarchical control architecture:

```
┌─────────────────────────────────────────────────────────────────┐
│                    Command Interface                            │
│  (Navigation, Manipulation, Locomotion Commands)               │
└─────────────────┬───────────────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────────────┐
│                   Command Processing &                          │
│                Trajectory Generation                            │
└─────────────────┬───────────────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────────────┐
│                  Motion Control                                 │
│  (Trajectory Tracking, Feedback Control, Feedforward Control)   │
└─────────────────┬───────────────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────────────┐
│                   Safety & Limits                               │
│      (Constraints, Emergency Stop, System Health)              │
└─────────────────┬───────────────────────────────────────────────┘
                  │
┌─────────────────▼───────────────────────────────────────────────┐
│                  Hardware Interface                             │
│         (Motor Commands, Sensor Feedback, Status)              │
└─────────────────────────────────────────────────────────────────┘
```

## Implementation

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import SwitchController
import numpy as np
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
import threading
import time

@dataclass
class RobotState:
    """Data structure for robot state information"""
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    joint_efforts: np.ndarray
    timestamp: float
    control_mode: str  # 'position', 'velocity', 'effort', 'torque'

@dataclass
class ControlCommand:
    """Data structure for control commands"""
    type: str  # 'trajectory', 'velocity', 'position', 'effort'
    joints: List[str]
    values: np.ndarray
    duration: float
    timestamp: float

class ControlSubagent(Node):
    def __init__(self, node_name='control_subagent'):
        super().__init__(node_name)
        
        # Robot state
        self.current_state = RobotState(
            joint_positions=np.array([]),
            joint_velocities=np.array([]),
            joint_efforts=np.array([]),
            timestamp=0.0,
            control_mode='position'
        )
        
        # Control parameters
        self.control_gains = {
            'position': {'p': 10.0, 'i': 0.1, 'd': 0.5},
            'velocity': {'p': 5.0, 'i': 0.05, 'd': 0.1},
            'effort': {'p': 2.0, 'i': 0.1, 'd': 0.05}
        }
        
        self.control_limits = {
            'max_velocity': 2.0,  # rad/s
            'max_acceleration': 5.0,  # rad/s^2
            'max_effort': 100.0,  # Nm
            'position_tolerance': 0.01,  # rad
            'velocity_tolerance': 0.05,  # rad/s
        }
        
        # Command queue and current trajectory
        self.command_queue = []
        self.current_trajectory = None
        self.trajectory_index = 0
        self.is_executing = False
        
        # Publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/position_controller/joint_trajectory', 
            10
        )
        
        self.velocity_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controller/commands', 
            10
        )
        
        self.effort_cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/effort_controller/commands', 
            10
        )
        
        self.status_pub = self.create_publisher(
            Bool, 
            '/control_subagent/active', 
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.joint_traj_state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/position_controller/state',
            self.trajectory_state_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Services
        self.controller_switch_client = self.create_client(
            SwitchController, 
            '/controller_manager/switch_controller'
        )
        
        # Timers
        self.control_timer = self.create_timer(
            0.01,  # 100 Hz control loop
            self.control_loop_callback
        )
        
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz status update
            self.status_update_callback
        )
        
        self.get_logger().info("Control Subagent initialized")

    def joint_state_callback(self, msg):
        """Update robot state from joint feedback"""
        try:
            self.current_state.joint_positions = np.array(msg.position)
            self.current_state.joint_velocities = np.array(msg.velocity)
            self.current_state.joint_efforts = np.array(msg.effort)
            self.current_state.timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # Update control mode if possible from message
            if hasattr(msg, 'name'):
                self.get_logger().debug(f"Updated state for joints: {msg.name}")
                
        except Exception as e:
            self.get_logger().error(f"Error updating joint state: {e}")

    def trajectory_state_callback(self, msg):
        """Process trajectory controller state feedback"""
        try:
            # Extract current trajectory state
            self.get_logger().debug(f"Trajectory state: {len(msg.actual.positions)} joints")
            
        except Exception as e:
            self.get_logger().error(f"Error processing trajectory state: {e}")

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        try:
            # Convert Twist command to joint velocities for differential drive
            # This is a simplified example - real implementation would depend on robot kinematics
            linear_vel = msg.linear.x
            angular_vel = msg.angular.z
            
            # For a differential drive robot, convert to wheel velocities
            # Left wheel velocity
            left_vel = (linear_vel - angular_vel * 0.5) / 0.1  # wheel_radius = 0.1m
            # Right wheel velocity  
            right_vel = (linear_vel + angular_vel * 0.5) / 0.1
            
            # Publish velocity commands
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [left_vel, right_vel]  # Assuming joint names ['left_wheel', 'right_wheel']
            self.velocity_cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"Velocity command: linear={linear_vel}, angular={angular_vel}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing velocity command: {e}")

    def generate_trajectory(self, start_positions, end_positions, duration=5.0, steps=100):
        """Generate smooth trajectory between start and end positions"""
        # Simple linear interpolation with trapezoidal velocity profile
        num_joints = len(start_positions)
        trajectory = []
        
        # Time steps
        time_points = np.linspace(0, duration, steps)
        
        for t in time_points:
            # Trapezoidal velocity profile (simplified as sinusoidal for smoothness)
            progress = 0.5 * (1 - np.cos(np.pi * t / duration))
            positions = start_positions + progress * (end_positions - start_positions)
            
            # Calculate velocities
            if t == 0:
                velocities = np.zeros(num_joints)
            else:
                dt = duration / steps
                if len(trajectory) > 0:
                    prev_pos = trajectory[-1][0]  # Previous position
                    velocities = (positions - prev_pos) / dt
                else:
                    velocities = np.zeros(num_joints)
            
            # Calculate accelerations
            if len(trajectory) > 1:
                prev_vel = trajectory[-1][1]
                accelerations = (velocities - prev_vel) / dt
            else:
                accelerations = np.zeros(num_joints)
            
            trajectory.append((positions, velocities, accelerations))
        
        return trajectory

    def execute_joint_trajectory(self, joint_names, positions_list, time_from_start_list):
        """Execute a joint trajectory following ROS control_msgs standard"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = joint_names
        
        for positions, time_from_start in zip(positions_list, time_from_start_list):
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.time_from_start = Duration(sec=int(time_from_start), nanosec=int((time_from_start % 1) * 1e9))
            
            # Add velocity and acceleration if available
            trajectory_msg.points.append(point)
        
        self.joint_trajectory_pub.publish(trajectory_msg)
        self.is_executing = True
        
        self.get_logger().info(f"Published trajectory with {len(trajectory_msg.points)} points")

    def pid_control(self, error, integral_error, derivative_error, gains):
        """Basic PID controller"""
        control_output = (
            gains['p'] * error + 
            gains['i'] * integral_error + 
            gains['d'] * derivative_error
        )
        return control_output

    def enforce_limits(self, control_output):
        """Apply control limits to prevent exceeding physical constraints"""
        # Apply velocity limits
        if self.current_state.control_mode == 'velocity':
            control_output = np.clip(control_output, 
                                   -self.control_limits['max_velocity'], 
                                   self.control_limits['max_velocity'])
        
        # Apply effort limits
        elif self.current_state.control_mode in ['effort', 'torque']:
            control_output = np.clip(control_output, 
                                   -self.control_limits['max_effort'], 
                                   self.control_limits['max_effort'])
        
        return control_output

    def control_loop_callback(self):
        """Main control loop executing at 100Hz"""
        if not self.current_state.joint_positions.size:
            self.get_logger().debug("Waiting for joint state data...")
            return
        
        try:
            # Check if we have a trajectory to execute
            if self.is_executing and self.current_trajectory and self.trajectory_index < len(self.current_trajectory):
                # Get the current trajectory point
                target_pos, target_vel, target_acc = self.current_trajectory[self.trajectory_index]
                
                # Calculate errors
                current_pos = self.current_state.joint_positions[:len(target_pos)]  # Ensure matching dimensions
                pos_error = target_pos - current_pos
                
                # Simple PID control
                control_commands = []
                for i, error in enumerate(pos_error):
                    command = self.pid_control(
                        error, 0, 0,  # Simplified: no integral/derivative for this example
                        self.control_gains[self.current_state.control_mode]
                    )
                    control_commands.append(command)
                
                # Apply limits
                control_commands = self.enforce_limits(np.array(control_commands))
                
                # Publish commands based on control mode
                if self.current_state.control_mode == 'position':
                    # For position control, we assume trajectory controller handles it
                    pass
                elif self.current_state.control_mode == 'velocity':
                    cmd_msg = Float64MultiArray()
                    cmd_msg.data = [float(c) for c in control_commands[:len(current_pos)]]
                    self.velocity_cmd_pub.publish(cmd_msg)
                elif self.current_state.control_mode == 'effort':
                    cmd_msg = Float64MultiArray()
                    cmd_msg.data = [float(c) for c in control_commands[:len(current_pos)]]
                    self.effort_cmd_pub.publish(cmd_msg)
                
                self.trajectory_index += 1
                
                # Check if trajectory is complete
                if self.trajectory_index >= len(self.current_trajectory):
                    self.is_executing = False
                    self.trajectory_index = 0
                    self.get_logger().info("Trajectory execution completed")
            
            # Handle direct commands from queue
            if self.command_queue:
                command = self.command_queue.pop(0)
                
                if command.type == 'position':
                    # Execute position command
                    self.execute_position_command(command)
                elif command.type == 'velocity':
                    # Execute velocity command
                    self.execute_velocity_command(command)
                elif command.type == 'effort':
                    # Execute effort command
                    self.execute_effort_command(command)
                elif command.type == 'trajectory':
                    # Execute trajectory command
                    self.execute_trajectory_command(command)
        
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")

    def execute_position_command(self, command):
        """Execute position control command"""
        try:
            # Create trajectory from current position to target
            start_pos = self.current_state.joint_positions[:len(command.values)]
            trajectory = self.generate_trajectory(
                start_pos, 
                command.values, 
                duration=command.duration
            )
            
            self.current_trajectory = trajectory
            self.is_executing = True
            self.trajectory_index = 0
            
            self.get_logger().info(f"Position command executed: {command.values}")
            
        except Exception as e:
            self.get_logger().error(f"Error executing position command: {e}")

    def execute_velocity_command(self, command):
        """Execute velocity control command"""
        try:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [float(v) for v in command.values]
            self.velocity_cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"Velocity command executed: {command.values}")
            
        except Exception as e:
            self.get_logger().error(f"Error executing velocity command: {e}")

    def execute_effort_command(self, command):
        """Execute effort control command"""
        try:
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [float(v) for v in command.values]
            self.effort_cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"Effort command executed: {command.values}")
            
        except Exception as e:
            self.get_logger().error(f"Error executing effort command: {e}")

    def execute_trajectory_command(self, command):
        """Execute trajectory command"""
        try:
            joint_names = command.joints
            positions_list = [command.values]  # In real implementation, this would be a list of positions
            time_list = [command.duration]
            
            self.execute_joint_trajectory(joint_names, positions_list, time_list)
            
        except Exception as e:
            self.get_logger().error(f"Error executing trajectory command: {e}")

    def safety_check(self):
        """Perform safety checks before executing commands"""
        # Check joint limits
        # Check for collisions
        # Check system health
        # Implement emergency stop logic
        
        return True  # Simplified - real implementation would have actual checks

    def status_update_callback(self):
        """Publish control system status"""
        status_msg = Bool()
        status_msg.data = True  # Simplified - real implementation would check actual status
        self.status_pub.publish(status_msg)
        
        self.get_logger().debug(f"Control status: active={status_msg.data}, executing={self.is_executing}")

def main(args=None):
    rclpy.init(args=args)
    control_agent = ControlSubagent()
    
    try:
        rclpy.spin(control_agent)
    except KeyboardInterrupt:
        pass
    finally:
        control_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Configuration and Deployment

### Controller configuration file
```
# control_subagent_controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    position_controller:
      type: position_controllers/JointGroupPositionController

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

    effort_controller:
      type: effort_controllers/JointGroupEffortController

position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3
      # Add all joints that need position control
```

### Launch file example
```
# control_subagent.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('physical_ai_control'),
        'config',
        'control_subagent_controllers.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='physical_ai_control',
            executable='control_subagent',
            name='control_subagent',
            parameters=[
                config_file,
                {'control_gains.position.p': 10.0},
                {'control_gains.position.i': 0.1},
                {'control_gains.position.d': 0.5},
                {'control_limits.max_velocity': 2.0},
            ],
            remappings=[
                ('/joint_states', '/robot/joint_states'),
                ('/cmd_vel', '/navigation/velocity_cmd'),
            ]
        )
    ])
```

## Integration Notes
- The subagent should be integrated with the robot's hardware interface
- Controller switching should be handled carefully to avoid discontinuities
- Safety systems should be the highest priority and always active
- The subagent should provide detailed status information for system monitoring
- Trajectory generation should consider the robot's physical constraints