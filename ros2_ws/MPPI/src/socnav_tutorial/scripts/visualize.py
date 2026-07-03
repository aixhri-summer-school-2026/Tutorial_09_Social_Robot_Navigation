#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import math

class MPPItoRVizRelay(Node):
    def __init__(self):
        super().__init__('mppi_rviz_relay')
        
        # RViz Topic Advertisements
        self.ref_pub = self.create_publisher(Path, '/mppi/global_references', 10)
        self.best_pub = self.create_publisher(Path, '/mppi/best_trajectory', 10)
        self.cloud_pub = self.create_publisher(MarkerArray, '/mppi/sample_cloud', 10)
        self.robot_pub = self.create_publisher(MarkerArray, 'robot_marker_array', 10)

        
        # Listen on the exact UDP address your controller is streaming to
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 9876))
        self.sock.settimeout(0.01) # Low timeout to loop cleanly
        
        # Create a timer to spin and pull UDP data independently
        self.create_timer(0.02, self.receive_and_publish)
        self.get_logger().info("Isolated MPPI RViz Relay Node is Online!")

    def receive_and_publish(self):
        try:
            data, _ = self.sock.recvfrom(65535)
        except socket.timeout:
            return # No data sent yet, skip cycle safely

        try:
            offset = 0
            
            # 1. Unpack Global References
            num_refs = struct.unpack_from("!I", data, offset)[0]
            offset += 4
            ref_bytes = num_refs * 2 * 8 # N points * 2 coordinates * 8 bytes (float64)
            g_refs = np.frombuffer(data[offset:offset+ref_bytes], dtype=np.float64).reshape((num_refs, 2))
            offset += ref_bytes
            
            # 2. Unpack Best Trajectory
            num_best = struct.unpack_from("!I", data, offset)[0]
            offset += 4
            best_bytes = num_best * 2 * 8
            best_traj = np.frombuffer(data[offset:offset+best_bytes], dtype=np.float64).reshape((num_best, 2))
            offset += best_bytes
            
            # 3. Unpack Candidate Search Fan Cloud
            num_samples, steps = struct.unpack_from("!II", data, offset)[0:2]
            offset += 8
            sample_bytes = num_samples * steps * 2 * 8
            samples = np.frombuffer(data[offset:offset+sample_bytes], dtype=np.float64).reshape((num_samples, steps, 2))
            offset += sample_bytes

            # 4. Unpack robot pose
            pose_bytes = 3 * 8  # 3 coordinates (x, y, theta) * 8 bytes (float64) 
            robot_pose = np.frombuffer(data[offset:offset+pose_bytes], dtype=np.float64) # Creates a 1D array: [x, y]
            offset += pose_bytes
            
            # Publish to RViz!
            now = self.get_clock().now().to_msg()
            self.publish_path(self.ref_pub, g_refs, now)
            self.publish_path(self.best_pub, best_traj, now)
            self.publish_sample_cloud(samples, now)
            self.publish_robot_markers(robot_pose, now)
            
        except Exception as e:
            self.get_logger().error(f"Failed decoding payload packet: {e}")

    def publish_path(self, publisher, points, stamp):
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = "map" # Or change to "odom" matching your frame setup
        for pt in points:
            pose = PoseStamped()
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            msg.poses.append(pose)
        publisher.publish(msg)

    def publish_sample_cloud(self, samples, stamp):
        marker_array = MarkerArray()
        
        # 1. Clear out previous markers first safely
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for idx, traj in enumerate(samples):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = "map"
            
            # --- THE FIX: ISOLATE THE NAMESPACE ---
            marker.ns = "mppi_fan_lines"  # Differentiates from the clear marker
            marker.id = idx               # Unique within this namespace
            
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.015  # Thin lines
            
            # Nice translucent red visualization
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.3  
            
            for pt in traj:
                p = PoseStamped().pose.position
                p.x, p.y = pt[0], pt[1]
                marker.points.append(p)
                
            marker_array.markers.append(marker)
            
        self.cloud_pub.publish(marker_array)

    def publish_robot_markers(self, pose, now):
        marker_array = MarkerArray()
        
        # Extract pose variables
        x, y, theta = pose[0], pose[1], pose[2]

        # Standard conversion from Z-axis Euler angle to Quaternion
        cos_t = math.cos(theta * 0.5)
        sin_t = math.sin(theta * 0.5)
        
        body_orientation = Quaternion()
        body_orientation.x = 0.0
        body_orientation.y = 0.0
        body_orientation.z = sin_t
        body_orientation.w = cos_t

        # Wheel Orientation: Rotated 90 deg around X (0.7071, 0, 0, 0.7071) and rotated by yaw
        wheel_orientation = Quaternion()
        wheel_orientation.x = 0.7071 * cos_t
        wheel_orientation.y = 0.7071 * sin_t
        wheel_orientation.z = 0.7071 * sin_t
        wheel_orientation.w = 0.7071 * cos_t

        # Precompute trigonometry for offset rotations
        c = math.cos(theta)
        s = math.sin(theta)

        # ROBOT CHASSIS (Cylinder)
        body_marker = Marker()
        body_marker.header.frame_id = "map"
        body_marker.header.stamp = now
        body_marker.ns = "robot"
        body_marker.id = 0
        body_marker.type = Marker.CYLINDER
        body_marker.action = Marker.ADD
        body_marker.pose.position.x = x
        body_marker.pose.position.y = y
        body_marker.pose.position.z = 0.15  
        body_marker.pose.orientation = body_orientation
        body_marker.scale.x = 0.4   
        body_marker.scale.y = 0.4   
        body_marker.scale.z = 0.1   
        body_marker.color.r = 0.8
        body_marker.color.g = 1.0
        body_marker.color.b = 0.2
        body_marker.color.a = 1.0
        marker_array.markers.append(body_marker)

        # LEFT MAIN WHEEL 
        left_wheel = Marker()
        left_wheel.header.frame_id = "map"
        left_wheel.header.stamp = now
        left_wheel.ns = "robot"
        left_wheel.id = 1
        left_wheel.type = Marker.CYLINDER
        # Rotate local offset (x=0, y=0.22) by yaw angle
        left_wheel.pose.position.x = x + (0.0 * c - 0.22 * s)
        left_wheel.pose.position.y = y + (0.0 * s + 0.22 * c)
        left_wheel.pose.position.z = 0.1   
        left_wheel.pose.orientation = wheel_orientation
        left_wheel.scale.x = 0.2
        left_wheel.scale.y = 0.2
        left_wheel.scale.z = 0.04
        left_wheel.color.r = 0.1
        left_wheel.color.g = 0.1
        left_wheel.color.b = 0.1
        left_wheel.color.a = 1.0
        marker_array.markers.append(left_wheel)

        # RIGHT MAIN WHEEL
        right_wheel = Marker()
        right_wheel.header.frame_id = "map"
        right_wheel.header.stamp = now
        right_wheel.ns = "robot"
        right_wheel.id = 2
        right_wheel.type = Marker.CYLINDER
        # Rotate local offset (x=0, y=-0.22) by yaw angle
        right_wheel.pose.position.x = x + (0.0 * c - (-0.22) * s)
        right_wheel.pose.position.y = y + (0.0 * s + (-0.22) * c)
        right_wheel.pose.position.z = 0.1
        right_wheel.pose.orientation = wheel_orientation
        right_wheel.scale.x = 0.2
        right_wheel.scale.y = 0.2
        right_wheel.scale.z = 0.04
        right_wheel.color.r = 0.1
        right_wheel.color.g = 0.1
        right_wheel.color.b = 0.1
        right_wheel.color.a = 1.0
        marker_array.markers.append(right_wheel)

        # FRONT CASTER BALL (Sphere)
        caster_ball = Marker()
        caster_ball.header.frame_id = "map"
        caster_ball.header.stamp = now
        caster_ball.ns = "robot"
        caster_ball.id = 3
        caster_ball.type = Marker.SPHERE
        caster_ball.action = Marker.ADD
        # Rotate local offset (x=0.14, y=0) by yaw angle
        caster_ball.pose.position.x = x + (0.14 * c - 0.0 * s)
        caster_ball.pose.position.y = y + (0.14 * s + 0.0 * c)
        caster_ball.pose.position.z = 0.05  
        caster_ball.pose.orientation = body_orientation
        caster_ball.scale.x = 0.1
        caster_ball.scale.y = 0.1
        caster_ball.scale.z = 0.1
        caster_ball.color.r = 0.1
        caster_ball.color.g = 0.1
        caster_ball.color.b = 0.1  
        caster_ball.color.a = 1.0
        marker_array.markers.append(caster_ball)

        # FRONT DIRECTION MARKER (Small Box/Arrow) 
        direction_marker = Marker()
        direction_marker.header.frame_id = "map"
        direction_marker.header.stamp = now
        direction_marker.ns = "robot_arrow"
        direction_marker.id = 4
        direction_marker.type = Marker.ARROW
        direction_marker.action = Marker.ADD
        # Rotate local offset (x=0.20, y=0) by yaw angle
        direction_marker.pose.position.x = x #+ (0.10 * c - 0.0 * s)
        direction_marker.pose.position.y = y #+ (0.10 * s + 0.0 * c)
        direction_marker.pose.position.z = 0.2  
        direction_marker.pose.orientation = body_orientation
        direction_marker.scale.x = 0.2
        direction_marker.scale.y = 0.03
        direction_marker.scale.z = 0.03
        direction_marker.color.r = 1.0
        direction_marker.color.g = 0.0
        direction_marker.color.b = 0.0   
        direction_marker.color.a = 1.0
        marker_array.markers.append(direction_marker)

        # Publish the fully assembled robot model
        self.robot_pub.publish(marker_array)

def main():
    rclpy.init()
    node = MPPItoRVizRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()