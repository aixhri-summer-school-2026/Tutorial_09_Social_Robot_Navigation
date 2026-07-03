#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from socnav_msgs.msg import TrackedAgents, TrackedAgent
from nav_msgs.msg import Odometry
from message_filters import Subscriber, TimeSynchronizer
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Quaternion
import math

from geometry_msgs.msg import Quaternion

def fast_rotate_local_90(q_curr, axis='x'):
    """
    Rotates an incoming orientation quaternion by exactly 90 degrees 
    around its local X, Y, or Z axis.
    
    :param q_curr: geometry_msgs/msg/Quaternion (current heading)
    :param axis: String, either 'x', 'y', or 'z'
    :return: geometry_msgs/msg/Quaternion (rotated orientation)
    """
    # Local stack references for maximum execution speed
    cx, cy, cz, cw = q_curr.x, q_curr.y, q_curr.z, q_curr.w
    SQRT_2_OVER_2 = 0.70710678118
    
    q_out = Quaternion()
    axis_lower = axis.lower()
    
    if axis_lower == 'x':
        q_out.x = SQRT_2_OVER_2 * (cw + cx)
        q_out.y = SQRT_2_OVER_2 * (cy + cz)
        q_out.z = SQRT_2_OVER_2 * (cz - cy)
        q_out.w = SQRT_2_OVER_2 * (cw - cx)
        
    elif axis_lower == 'y':
        q_out.x = SQRT_2_OVER_2 * (cx - cz)
        q_out.y = SQRT_2_OVER_2 * (cw + cy)
        q_out.z = SQRT_2_OVER_2 * (cx + cz)
        q_out.w = SQRT_2_OVER_2 * (cw - cy)
        
    elif axis_lower == 'z':
        q_out.x = SQRT_2_OVER_2 * (cx + cy)
        q_out.y = SQRT_2_OVER_2 * (cy - cx)
        q_out.z = SQRT_2_OVER_2 * (cw + cz)
        q_out.w = SQRT_2_OVER_2 * (cw - cz)
        
    else:
        return q_curr
        
    return q_out

class AgentsNode(Node):
    def __init__(self, num_hum):
        super().__init__('tracked_agents_node')
        self.num_hum = num_hum
        self.tracked_agents_pub = []
        self.agents = TrackedAgents()        
        self.setup_subscribers_and_publisher()

    def setup_subscribers_and_publisher(self):
        agent_sub = []
        for agent_id in range(1, self.num_hum + 1):
            name = 'human' + str(agent_id)
            agent_sub.append(Subscriber(self, Odometry, "/" + name + "/base_pose_ground_truth"))

        self.tracked_agents_pub = self.create_publisher(TrackedAgents, "tracked_agents", 10)
        
        self.human_markers_pub = self.create_publisher(MarkerArray, "tracked_human_markers", 10)        
        
        # Set up message filter synchronization
        if agent_sub:
            self.pose_msg = TimeSynchronizer(agent_sub, 10)
            self.pose_msg.registerCallback(self.AgentsCB)

        self.publish_timer = self.create_timer(0.02, self.timer_callback)

    def AgentsCB(self,*msg):
        tracked_agents = TrackedAgents()
        idx = 0
        for agent_id in range(1,self.num_hum+1):
            agent = TrackedAgent()
            agent.track_id = agent_id
            agent.pose = msg[idx].pose.pose
            agent.velocity = msg[idx].twist.twist
            agent.name = "human"+str(agent_id)
            tracked_agents.agents.append(agent)
            idx += 1
            
        if(tracked_agents.agents):
            self.agents = tracked_agents

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        if self.agents.agents:
            self.agents.header.stamp = now
            self.agents.header.frame_id = "map"
            self.tracked_agents_pub.publish(self.agents)
        
        self.publish_human_markers(now)            

    def publish_human_markers(self, current_time):
        """
        This marker respresentation is inspired from the Spensor Markers used in PedSim ROS
        """

        marker_array = MarkerArray()
        BODY_HEIGHT = 1.3
        BODY_RADIUS = 0.25
        HEAD_DIAMETER = 0.3
        
        for idx, agent in enumerate(self.agents.agents):
            body_id = idx * 2
            head_id = idx * 2 + 1
            
            body_marker = Marker()
            body_marker.header.frame_id = "map"
            body_marker.header.stamp = current_time
            body_marker.ns = "human_bodies"
            body_marker.id = body_id
            body_marker.type = Marker.CYLINDER
            body_marker.action = Marker.ADD
            
            # Position the cylinder base onto the ground plane
            body_marker.pose = Pose()
            body_marker.pose.position.x = agent.pose.position.x
            body_marker.pose.position.y = agent.pose.position.y
            body_marker.pose.position.z = agent.pose.position.z + (BODY_HEIGHT / 2.0)            
            
            rot1 = fast_rotate_local_90(agent.pose.orientation, 'y')
            body_marker.pose.orientation=fast_rotate_local_90(rot1, 'x')
            
            body_marker.scale.x = BODY_HEIGHT 
            body_marker.scale.y = BODY_RADIUS  # Depth
            body_marker.scale.z = BODY_RADIUS * 2.0  # Width
            
            body_marker.color.r = 0.12
            body_marker.color.g = 0.53
            body_marker.color.b = 0.70
            body_marker.color.a = 0.65  # Alpha gives that translucent look
            
            body_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
            marker_array.markers.append(body_marker)
            
            head_marker = Marker()
            head_marker.header.frame_id = "map"
            head_marker.header.stamp = current_time
            head_marker.ns = "spencer_human_heads"
            head_marker.id = head_id
            head_marker.type = Marker.SPHERE
            head_marker.action = Marker.ADD
            
            head_marker.pose = Pose()
            head_marker.pose.position.x = agent.pose.position.x
            head_marker.pose.position.y = agent.pose.position.y
            head_marker.pose.position.z = agent.pose.position.z + BODY_HEIGHT + (HEAD_DIAMETER / 2.0)
            head_marker.pose.orientation = agent.pose.orientation
            
            head_marker.scale.x = HEAD_DIAMETER
            head_marker.scale.y = HEAD_DIAMETER
            head_marker.scale.z = HEAD_DIAMETER
            
            head_marker.color.r = 0.08
            head_marker.color.g = 0.40
            head_marker.color.b = 0.55
            head_marker.color.a = 1.0  # Solid opacity
            
            head_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
            marker_array.markers.append(head_marker)
            

            arrow_marker = Marker()
            arrow_marker.header.frame_id = "map"
            arrow_marker.header.stamp = current_time
            arrow_marker.ns = "human_arrows"
            arrow_marker.id = idx 
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # Position at waist level, using the raw agent orientation
            arrow_marker.pose = Pose()
            arrow_marker.pose.position.x = agent.pose.position.x
            arrow_marker.pose.position.y = agent.pose.position.y
            arrow_marker.pose.position.z = agent.pose.position.z + 0.8* BODY_HEIGHT  
            arrow_marker.pose.orientation = agent.pose.orientation      
            
            arrow_marker.scale.x = 0.5  
            arrow_marker.scale.y = 0.08  # Arrow width
            arrow_marker.scale.z = 0.05  # Arrow thickness
            
            arrow_marker.color.r = 1.0
            arrow_marker.color.g = 0.75
            arrow_marker.color.b = 0.0
            arrow_marker.color.a = 1.0   # Fully opaque
            
            arrow_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
            marker_array.markers.append(arrow_marker)
            
            ring_marker = Marker()
            ring_marker.header.frame_id = "map"
            ring_marker.header.stamp = current_time
            ring_marker.ns = "human_rings"
            ring_marker.id = idx  # Safe to reuse idx due to unique namespace
            ring_marker.type = Marker.LINE_STRIP
            ring_marker.action = Marker.ADD
            
            ring_marker.pose = Pose()
            ring_marker.pose.position.x = agent.pose.position.x
            ring_marker.pose.position.y = agent.pose.position.y
            ring_marker.pose.position.z = agent.pose.position.z + 0.01
            ring_marker.pose.orientation = agent.pose.orientation
            
            ring_marker.scale.x = 0.03  # 3 cm thick outline line
            
            ring_marker.color.r = 0.12
            ring_marker.color.g = 0.53
            ring_marker.color.b = 0.70
            ring_marker.color.a = 0.80  # Mostly opaque for a sharp outline
            
            RING_RADIUS = 0.6
            NUM_POINTS = 32
            import math
            from geometry_msgs.msg import Point
            
            for i in range(NUM_POINTS + 1): 
                angle = 2.0 * math.pi * i / NUM_POINTS
                p = Point()
                p.x = RING_RADIUS * math.cos(angle)
                p.y = RING_RADIUS * math.sin(angle)
                p.z = 0.0  # Flat on its local plane
                ring_marker.points.append(p)
            
            ring_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
            marker_array.markers.append(ring_marker)

        self.human_markers_pub.publish(marker_array)

def main():
    rclpy.init()
    agents = AgentsNode(num_hum=2)    
    try:
        rclpy.spin(agents)
    except KeyboardInterrupt:
        pass
    finally:
        agents.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
