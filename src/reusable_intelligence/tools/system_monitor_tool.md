# Robot System Monitor Tool

This tool provides comprehensive monitoring and health checking for Physical AI robot systems, tracking performance metrics, system health, and operational status.

## Purpose
- Monitor ROS 2 system health and performance
- Track robot operational metrics
- Provide early warning of potential issues
- Generate system health reports
- Support debugging and optimization

## Inputs
- ROS 2 topic statistics
- System resource usage (CPU, memory, disk)
- Robot operational parameters
- Time-series performance data

## Outputs
- System health dashboard
- Performance metrics reports
- Anomaly detection alerts
- Resource utilization trends
- Operational status summary

## Implementation

```python
#!/usr/bin/env python3
"""
Robot System Monitor Tool

This tool monitors the health and performance of Physical AI robot systems.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_srvs.srv import Trigger
import psutil
import time
import threading
import json
import csv
from datetime import datetime
from dataclasses import dataclass
from typing import Dict, List, Optional
import numpy as np
import matplotlib.pyplot as plt
from collections import deque, defaultdict

@dataclass
class SystemMetrics:
    """Data structure for system metrics"""
    timestamp: float
    cpu_percent: float
    memory_percent: float
    disk_percent: float
    network_bytes: int
    temp_celsius: Optional[float] = None
    uptime: Optional[float] = None

@dataclass
class RobotMetrics:
    """Data structure for robot metrics"""
    timestamp: float
    joint_positions: Dict[str, float]
    joint_velocities: Dict[str, float]
    joint_efforts: Dict[str, float]
    position_errors: Dict[str, float]
    velocity_errors: Dict[str, float]
    operational_mode: str
    battery_level: Optional[float] = None
    temperature: Optional[Dict[str, float]] = None

@dataclass
class PerformanceMetrics:
    """Data structure for performance metrics"""
    timestamp: float
    control_loop_rate: float
    message_rate: float
    processing_time: float
    latency: float
    reliability: float

class RobotSystemMonitor(Node):
    def __init__(self, node_name='robot_system_monitor'):
        super().__init__(node_name)
        
        # Data storage
        self.system_metrics = deque(maxlen=1000)  # Keep last 1000 readings
        self.robot_metrics = deque(maxlen=1000)
        self.performance_metrics = deque(maxlen=1000)
        self.topic_stats = defaultdict(list)
        
        # Configuration
        self.monitoring_interval = 1.0  # seconds
        self.performance_window = 100  # number of samples for performance calculation
        
        # System thresholds for alerting
        self.alert_thresholds = {
            'cpu_percent': 80.0,
            'memory_percent': 85.0,
            'disk_percent': 90.0,
            'control_loop_rate': 90.0,  # minimum expected Hz
            'latency_ms': 50.0,
            'reliability': 0.95  # minimum message delivery rate
        }
        
        # Publishers for monitoring data
        self.system_status_pub = self.create_publisher(
            DiagnosticArray, 
            '/diagnostics', 
            QoSProfile(depth=10)
        )
        
        self.heartbeat_pub = self.create_publisher(
            Float32, 
            '/monitor/heartbeat', 
            QoSProfile(depth=10)
        )
        
        # Subscribers to monitor
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            QoSProfile(depth=10)
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            QoSProfile(depth=10)
        )
        
        # Services
        self.get_status_service = self.create_service(
            Trigger,
            'get_system_status',
            self.get_status_callback
        )
        
        self.reset_stats_service = self.create_service(
            Trigger,
            'reset_monitor_stats',
            self.reset_stats_callback
        )
        
        # Timers
        self.monitor_timer = self.create_timer(
            self.monitoring_interval,
            self.system_monitor_callback
        )
        
        self.publish_timer = self.create_timer(
            2.0,  # Publish status every 2 seconds
            self.publish_status_callback
        )
        
        # Start background monitoring
        self.is_monitoring = True
        self.monitoring_thread = threading.Thread(target=self.background_monitoring, daemon=True)
        self.monitoring_thread.start()
        
        self.get_logger().info("Robot System Monitor initialized")

    def joint_state_callback(self, msg):
        """Monitor joint state messages"""
        try:
            robot_metrics = RobotMetrics(
                timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                joint_positions=dict(zip(msg.name, msg.position)),
                joint_velocities=dict(zip(msg.name, msg.velocity)),
                joint_efforts=dict(zip(msg.name, msg.effort)),
                position_errors={},
                velocity_errors={},
                operational_mode='active'
            )
            
            # Calculate position errors if we have reference positions (simplified)
            # In real implementation, compare with desired positions
            self.robot_metrics.append(robot_metrics)
            
            # Track topic statistics
            self.topic_stats['/joint_states'].append(robot_metrics.timestamp)
            
        except Exception as e:
            self.get_logger().error(f"Error processing joint state: {e}")

    def odometry_callback(self, msg):
        """Monitor odometry messages"""
        try:
            # Process odometry data for navigation metrics
            self.get_logger().debug(f"Odometry updated: position=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})")
            
            # Track topic statistics
            self.topic_stats['/odom'].append(
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            )
            
        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {e}")

    def system_monitor_callback(self):
        """Main system monitoring callback"""
        try:
            # Collect system metrics
            cpu_percent = psutil.cpu_percent()
            memory_percent = psutil.virtual_memory().percent
            disk_percent = psutil.disk_usage('/').percent
            network_io = psutil.net_io_counters()
            
            system_metrics = SystemMetrics(
                timestamp=time.time(),
                cpu_percent=cpu_percent,
                memory_percent=memory_percent,
                disk_percent=disk_percent,
                network_bytes=network_io.bytes_sent + network_io.bytes_recv
            )
            
            self.system_metrics.append(system_metrics)
            
            # Check for alerts
            self.check_system_alerts(system_metrics)
            
        except Exception as e:
            self.get_logger().error(f"Error in system monitoring: {e}")

    def background_monitoring(self):
        """Background thread for continuous monitoring"""
        while self.is_monitoring:
            try:
                # Perform additional background monitoring tasks
                # Update performance metrics
                self.update_performance_metrics()
                
                # Sleep to avoid excessive CPU usage
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Error in background monitoring: {e}")

    def update_performance_metrics(self):
        """Update performance metrics based on collected data"""
        # Calculate control loop rate (simplified)
        current_time = time.time()
        
        # Estimate message rates
        for topic, timestamps in self.topic_stats.items():
            if len(timestamps) > 2:
                time_diffs = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
                avg_interval = np.mean(time_diffs) if time_diffs else 1.0
                rate = 1.0 / avg_interval if avg_interval > 0 else 0.0
                
                perf_metrics = PerformanceMetrics(
                    timestamp=current_time,
                    control_loop_rate=rate,
                    message_rate=rate,
                    processing_time=0.0,  # Would be measured in real implementation
                    latency=0.0,  # Would be measured in real implementation
                    reliability=0.99  # Would be calculated based on message delivery
                )
                
                self.performance_metrics.append(perf_metrics)

    def check_system_alerts(self, system_metrics: SystemMetrics):
        """Check for system metrics exceeding thresholds"""
        alerts = []
        
        if system_metrics.cpu_percent > self.alert_thresholds['cpu_percent']:
            alerts.append(f"High CPU usage: {system_metrics.cpu_percent:.1f}%")
        
        if system_metrics.memory_percent > self.alert_thresholds['memory_percent']:
            alerts.append(f"High memory usage: {system_metrics.memory_percent:.1f}%")
        
        if system_metrics.disk_percent > self.alert_thresholds['disk_percent']:
            alerts.append(f"High disk usage: {system_metrics.disk_percent:.1f}%")
        
        if alerts:
            for alert in alerts:
                self.get_logger().warn(f"SYSTEM ALERT: {alert}")

    def publish_status_callback(self):
        """Publish system status information"""
        try:
            # Create heartbeat message
            heartbeat_msg = Float32()
            heartbeat_msg.data = float(time.time())
            self.heartbeat_pub.publish(heartbeat_msg)
            
            # Create diagnostic message
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # System status
            sys_status = DiagnosticStatus()
            sys_status.name = "System Monitor"
            sys_status.level = DiagnosticStatus.OK
            sys_status.message = "Monitoring Active"
            
            # Add key metrics as key-value pairs
            if self.system_metrics:
                latest_metrics = self.system_metrics[-1]
                sys_status.values.extend([
                    {"key": "CPU %", "value": f"{latest_metrics.cpu_percent:.1f}"},
                    {"key": "Memory %", "value": f"{latest_metrics.memory_percent:.1f}"},
                    {"key": "Disk %", "value": f"{latest_metrics.disk_percent:.1f}"},
                    {"key": "Network Bytes", "value": f"{latest_metrics.network_bytes}"}
                ])
            
            diag_array.status.append(sys_status)
            
            # Robot status if available
            if self.robot_metrics:
                robot_status = DiagnosticStatus()
                robot_status.name = "Robot Status"
                robot_status.level = DiagnosticStatus.OK
                robot_status.message = "Robot Active"
                
                latest_robot = self.robot_metrics[-1]
                robot_status.values.extend([
                    {"key": "Joints", "value": f"{len(latest_robot.joint_positions)}"},
                    {"key": "Mode", "value": latest_robot.operational_mode}
                ])
                
                diag_array.status.append(robot_status)
            
            self.system_status_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def get_status_callback(self, request, response):
        """Get current system status"""
        try:
            # Prepare status information
            status_info = {
                "timestamp": time.time(),
                "system_metrics_count": len(self.system_metrics),
                "robot_metrics_count": len(self.robot_metrics),
                "performance_metrics_count": len(self.performance_metrics),
                "active_topics": list(self.topic_stats.keys()),
                "monitoring_interval": self.monitoring_interval
            }
            
            response.success = True
            response.message = json.dumps(status_info, indent=2)
            
        except Exception as e:
            response.success = False
            response.message = f"Error getting status: {str(e)}"
        
        return response

    def reset_stats_callback(self, request, response):
        """Reset monitoring statistics"""
        try:
            self.system_metrics.clear()
            self.robot_metrics.clear()
            self.performance_metrics.clear()
            self.topic_stats.clear()
            
            response.success = True
            response.message = "Statistics reset successfully"
            
        except Exception as e:
            response.success = False
            response.message = f"Error resetting stats: {str(e)}"
        
        return response

    def generate_report(self, output_format='text', output_file=None):
        """Generate comprehensive monitoring report"""
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'system_metrics_summary': self._get_system_summary(),
            'robot_metrics_summary': self._get_robot_summary(),
            'performance_summary': self._get_performance_summary(),
            'topic_statistics': self._get_topic_stats(),
            'alerts_issued': []  # Would track alerts over time
        }
        
        if output_format == 'json':
            report_str = json.dumps(report_data, indent=2)
        elif output_format == 'csv':
            report_str = self._format_csv_report(report_data)
        else:  # text format
            report_str = self._format_text_report(report_data)
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write(report_str)
            print(f"Report saved to: {output_file}")
        else:
            print(report_str)
        
        return report_data

    def _get_system_summary(self):
        """Get summary of system metrics"""
        if not self.system_metrics:
            return {}
        
        cpu_values = [m.cpu_percent for m in self.system_metrics]
        memory_values = [m.memory_percent for m in self.system_metrics]
        disk_values = [m.disk_percent for m in self.system_metrics]
        
        return {
            'cpu': {
                'min': min(cpu_values),
                'max': max(cpu_values),
                'avg': sum(cpu_values) / len(cpu_values),
                'current': cpu_values[-1] if cpu_values else 0
            },
            'memory': {
                'min': min(memory_values),
                'max': max(memory_values),
                'avg': sum(memory_values) / len(memory_values),
                'current': memory_values[-1] if memory_values else 0
            },
            'disk': {
                'min': min(disk_values),
                'max': max(disk_values),
                'avg': sum(disk_values) / len(disk_values),
                'current': disk_values[-1] if disk_values else 0
            }
        }

    def _get_robot_summary(self):
        """Get summary of robot metrics"""
        if not self.robot_metrics:
            return {}
        
        latest = self.robot_metrics[-1]
        return {
            'joint_count': len(latest.joint_positions),
            'joint_names': list(latest.joint_positions.keys()),
            'current_mode': latest.operational_mode
        }

    def _get_performance_summary(self):
        """Get summary of performance metrics"""
        if not self.performance_metrics:
            return {}
        
        rates = [m.control_loop_rate for m in self.performance_metrics]
        return {
            'control_loop_rate': {
                'min': min(rates) if rates else 0,
                'max': max(rates) if rates else 0,
                'avg': sum(rates) / len(rates) if rates else 0,
                'current': rates[-1] if rates else 0
            }
        }

    def _get_topic_stats(self):
        """Get statistics for monitored topics"""
        stats = {}
        for topic, timestamps in self.topic_stats.items():
            if len(timestamps) > 1:
                intervals = [timestamps[i+1] - timestamps[i] for i in range(len(timestamps)-1)]
                avg_rate = 1.0 / (sum(intervals) / len(intervals)) if intervals else 0
                stats[topic] = {
                    'message_count': len(timestamps),
                    'average_rate_hz': avg_rate,
                    'latest_timestamp': timestamps[-1] if timestamps else 0
                }
        return stats

    def _format_text_report(self, report_data):
        """Format report as human-readable text"""
        lines = []
        lines.append("=" * 60)
        lines.append("ROBOT SYSTEM MONITOR - COMPREHENSIVE REPORT")
        lines.append("=" * 60)
        lines.append(f"Report Generated: {report_data['timestamp']}")
        lines.append("")
        
        # System Summary
        if report_data['system_metrics_summary']:
            lines.append("SYSTEM HEALTH SUMMARY:")
            sys_sum = report_data['system_metrics_summary']
            lines.append(f"  CPU Usage: Min={sys_sum['cpu']['min']:.1f}%, Max={sys_sum['cpu']['max']:.1f}%, Avg={sys_sum['cpu']['avg']:.1f}%")
            lines.append(f"  Memory Usage: Min={sys_sum['memory']['min']:.1f}%, Max={sys_sum['memory']['max']:.1f}%, Avg={sys_sum['memory']['avg']:.1f}%")
            lines.append(f"  Disk Usage: Min={sys_sum['disk']['min']:.1f}%, Max={sys_sum['disk']['max']:.1f}%, Avg={sys_sum['disk']['avg']:.1f}%")
            lines.append("")
        
        # Robot Summary
        if report_data['robot_metrics_summary']:
            lines.append("ROBOT STATUS SUMMARY:")
            robot_sum = report_data['robot_metrics_summary']
            lines.append(f"  Active Joints: {robot_sum['joint_count']}")
            lines.append(f"  Operational Mode: {robot_sum['current_mode']}")
            lines.append(f"  Joint Names: {', '.join(robot_sum['joint_names'])}")
            lines.append("")
        
        # Performance Summary
        if report_data['performance_summary']:
            lines.append("PERFORMANCE SUMMARY:")
            perf_sum = report_data['performance_summary']
            lines.append(f"  Control Loop Rate: Min={perf_sum['control_loop_rate']['min']:.1f}, Max={perf_sum['control_loop_rate']['max']:.1f}, Avg={perf_sum['control_loop_rate']['avg']:.1f} Hz")
            lines.append("")
        
        # Topic Statistics
        if report_data['topic_statistics']:
            lines.append("TOPIC STATISTICS:")
            for topic, stats in report_data['topic_statistics'].items():
                lines.append(f"  {topic}: {stats['message_count']} messages, {stats['average_rate_hz']:.1f} Hz avg")
            lines.append("")
        
        lines.append("=" * 60)
        return "\n".join(lines)

    def _format_csv_report(self, report_data):
        """Format report as CSV"""
        # Create a simplified CSV with key metrics
        import io
        import csv
        
        output = io.StringIO()
        writer = csv.writer(output)
        
        # Write header
        writer.writerow(['metric_type', 'sub_metric', 'min_value', 'max_value', 'avg_value', 'current_value'])
        
        # Write system metrics
        if report_data['system_metrics_summary']:
            sys_sum = report_data['system_metrics_summary']
            writer.writerow(['cpu', 'usage_percent', 
                           sys_sum['cpu']['min'], sys_sum['cpu']['max'], 
                           sys_sum['cpu']['avg'], sys_sum['cpu']['current']])
            writer.writerow(['memory', 'usage_percent', 
                           sys_sum['memory']['min'], sys_sum['memory']['max'], 
                           sys_sum['memory']['avg'], sys_sum['memory']['current']])
            writer.writerow(['disk', 'usage_percent', 
                           sys_sum['disk']['min'], sys_sum['disk']['max'], 
                           sys_sum['disk']['avg'], sys_sum['disk']['current']])
        
        return output.getvalue()

    def plot_metrics(self, output_file=None):
        """Plot system metrics over time"""
        if not self.system_metrics:
            self.get_logger().warn("No system metrics available for plotting")
            return
        
        # Extract timestamps and metrics
        timestamps = [m.timestamp for m in self.system_metrics]
        cpu_values = [m.cpu_percent for m in self.system_metrics]
        memory_values = [m.memory_percent for m in self.system_metrics]
        disk_values = [m.disk_percent for m in self.system_metrics]
        
        # Normalize timestamps to start from 0
        start_time = min(timestamps)
        time_from_start = [t - start_time for t in timestamps]
        
        # Create plots
        plt.figure(figsize=(12, 8))
        
        plt.subplot(3, 1, 1)
        plt.plot(time_from_start, cpu_values, label='CPU %', color='red')
        plt.axhline(y=self.alert_thresholds['cpu_percent'], color='red', linestyle='--', label='Alert Level')
        plt.title('CPU Usage Over Time')
        plt.ylabel('Percentage (%)')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(3, 1, 2)
        plt.plot(time_from_start, memory_values, label='Memory %', color='blue')
        plt.axhline(y=self.alert_thresholds['memory_percent'], color='blue', linestyle='--', label='Alert Level')
        plt.title('Memory Usage Over Time')
        plt.ylabel('Percentage (%)')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(3, 1, 3)
        plt.plot(time_from_start, disk_values, label='Disk %', color='green')
        plt.axhline(y=self.alert_thresholds['disk_percent'], color='green', linestyle='--', label='Alert Level')
        plt.title('Disk Usage Over Time')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Percentage (%)')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        
        if output_file:
            plt.savefig(output_file)
            self.get_logger().info(f"Metrics plot saved to: {output_file}")
        else:
            plt.show()

    def shutdown_monitor(self):
        """Clean shutdown of the monitor"""
        self.is_monitoring = False
        if hasattr(self, 'monitoring_thread'):
            self.monitoring_thread.join(timeout=1.0)
        self.get_logger().info("Robot System Monitor shutdown complete")

def main(args=None):
    parser = argparse.ArgumentParser(description='Robot System Monitor')
    parser.add_argument('--format', choices=['text', 'json', 'csv'], 
                       default='text', help='Output format for reports')
    parser.add_argument('--output', type=str, help='Output file for reports')
    parser.add_argument('--plot', action='store_true', help='Generate metrics plot')
    parser.add_argument('--plot-file', type=str, help='Output file for metrics plot')
    parser.add_argument('--duration', type=float, default=60.0, help='Monitoring duration in seconds')
    
    args = parser.parse_args()
    
    rclpy.init(args=args)
    
    try:
        monitor = RobotSystemMonitor()
        
        # If run with command line args for reports, just generate the report
        if args.output or args.plot:
            # Run for specified duration to collect metrics
            start_time = time.time()
            while time.time() - start_time < args.duration:
                rclpy.spin_once(monitor, timeout_sec=0.1)
                
                if args.plot and args.plot_file:
                    monitor.plot_metrics(args.plot_file)
                elif args.plot:
                    monitor.plot_metrics()
        
        # Otherwise, run as a continuous monitor node
        else:
            rclpy.spin(monitor)
    
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if 'monitor' in locals():
            monitor.shutdown_monitor()
        rclpy.shutdown()

if __name__ == '__main__':
    import argparse
    main()
```

## Usage Examples

### Command line usage
```
# Generate a report for 30 seconds
python3 system_monitor.py --duration 30 --output report.txt

# Generate a JSON report
python3 system_monitor.py --duration 60 --format json --output report.json

# Generate a metrics plot
python3 system_monitor.py --duration 120 --plot --plot-file metrics.png

# Run as a continuous ROS node
ros2 run robot_monitor system_monitor
```

### Service usage
```
# Get current system status
ros2 service call /get_system_status std_srvs/srv/Trigger

# Reset monitoring statistics
ros2 service call /reset_monitor_stats std_srvs/srv/Trigger
```

### Python API usage
```
from system_monitor import RobotSystemMonitor

monitor = RobotSystemMonitor()
# Collect metrics for 60 seconds
time.sleep(60)
report = monitor.generate_report(format='json')
print(report)

# Create a plot of metrics
monitor.plot_metrics('metrics.png')
```

## Integration Notes
- This tool should be run alongside robot operations for continuous monitoring
- Metrics can be used for performance optimization and debugging
- Alert thresholds can be configured based on system requirements
- Integration with system dashboards for real-time visualization
- Historical data can be used for maintenance scheduling and system analysis