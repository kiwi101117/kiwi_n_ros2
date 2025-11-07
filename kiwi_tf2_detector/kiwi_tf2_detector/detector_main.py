from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Transform, Point
from visualization_msgs.msg import Marker
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer 


import math
import rclpy
import numpy as np
import tf_transformations as tft

from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor

class DetectorNode(Node):
    def __init__(self):
        super().__init__('kiwi_detector_node')
        self.get_logger().info('kiwi_detector_node: Henlo!')

        self.tf_buffer_ = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer_, self)

        self.tf_broadcaster = StaticTransformBroadcaster(self)

        self.scan_sub = self.create_subscription(
            LaserScan, 
            'input_scan',
            self.scan_callback, 
            qos_profile_sensor_data)
        
    def scan_callback(self, msg):
        pos_to_access = len(msg.ranges)//2
        
        dist = msg.ranges[pos_to_access]

        if(not math.isinf(dist)):

            odom2laser_msg = TransformStamped()
            try:
                odom2laser_msg = self.tf_buffer_.lookup_transform(
                    target_frame='odom',
                    source_frame='base_laser_link',
                    time=Time(),
                    timeout=Duration(seconds=0.2)
                )
                self.get_logger().info(f'Get laser and TF: {dist:.2f}, laser 2 obj: {laser2obj_mat}')
            except Exception as e:
                self.get_logger().warn(f'Fail to lookup transform: {e}')
                return
        
            self.tf_broadcaster.sendTransform(odom2laser_msg)
            

class MonitorNode(Node):
    def __init__(self):
        super().__init__('kiwi_monitor_node')
        self.get_logger().info('kiwi_monitor_node: Henlo!')
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.marker_pub = self.create_publisher(Marker, 'object_marker', 1)
        self.timer = self.create_timer(0.5, self.control_cycle)

    def control_cycle(self):
        target_frame = 'odom'
        source_frame = 'base_footprint'
        odom2robot_msg = TransformStamped()
        try:
            odom2robot_msg = self.tf_buffer_.lookup_transform(
                target_frame,
                source_frame,
                time=rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'Fail to lookup transform from {target_frame} to {source_frame}: {e}')
            return
        
        x = odom2robot_msg.transform.translation.x
        y = odom2robot_msg.transform.translation.y
        z = odom2robot_msg.transform.translation.z
        theta = math.atan2(y, x)

        self.get_logger().info(f'Object at {x:.2f}, {y:.2f}, {z:.2f}, {math.degrees(theta):.1f} deg')

        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=1)

        origin = Point()
        origin.x = origin.y = origin.z = 0.0
        end = Point()
        end.x = x
        end.y = y
        end.z = z

        marker.points = [origin, end]
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = marker.color.b = 0.0

        marker.scale.x = 0.02
        marker.scale.y = marker.scale.z = 0.1
        
        self.marker_pub.publish(marker)
    
class EmptyNode(Node):
    def __init__(self):
        super().__init__('empty_node')
        self.get_logger().info('Empty node started.')


def main(args=None):
    rclpy.init(args=args)

    detector = DetectorNode()
    monitor = MonitorNode()
    demo = EmptyNode()
    
    executor = SingleThreadedExecutor()
    executor.add_node(detector)
    executor.add_node(monitor)
    executor.add_node(demo)
    executor.spin()    

    executor.shutdown()
    detector.destroy_node()
    monitor.destroy_node()
    demo.destroy_node()
    rclpy.shutdown()

if __name__=='main':
    main()