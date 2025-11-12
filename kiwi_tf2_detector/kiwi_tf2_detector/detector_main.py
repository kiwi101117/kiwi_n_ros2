from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Transform, Point, Quaternion
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
        pos_to_access = len(msg.ranges)//2 #msg.ranges.index(min(msg.ranges))
        
        dist = msg.ranges[pos_to_access]

        if(not math.isinf(dist)):
            laser2obj_mat = tft.euler_matrix(0, 0, 0) @ tft.translation_matrix((dist, 0, 0))

            target_frame = 'odom'
            source_frame = 'base_laser_link'
            odom2laser_msg = TransformStamped()
            try:
                odom2laser_msg = self.tf_buffer_.lookup_transform(
                    target_frame,
                    source_frame,
                    time=Time(),
                    timeout=Duration(seconds=0.2)
                )
                self.get_logger().info(f'Get {target_frame} to {source_frame}, laser2obj_mat: \n{laser2obj_mat}')
            except Exception as e:
                self.get_logger().warn(f'Fail to lookup transform: {e}')
                return
            
            odom2laser_mat = tft.quaternion_matrix(
                [odom2laser_msg.transform.rotation.x, 
                 odom2laser_msg.transform.rotation.y,
                 odom2laser_msg.transform.rotation.z,
                 odom2laser_msg.transform.rotation.w])
            odom2laser_mat[:3, 3] = [odom2laser_msg.transform.translation.x,
                                     odom2laser_msg.transform.translation.y,
                                     odom2laser_msg.transform.translation.z]
            odom2obj_mat = odom2laser_mat @ laser2obj_mat

            odom2obj_msg = TransformStamped()
            odom2obj_msg.header.stamp = self.get_clock().now().to_msg()
            odom2obj_msg.header.frame_id = 'odom'
            odom2obj_msg.child_frame_id = 'detected_object'

            trans = tft.translation_from_matrix(odom2obj_mat)
            odom2obj_msg.transform.translation.x = trans.item(0)
            odom2obj_msg.transform.translation.y = trans.item(1)
            odom2obj_msg.transform.translation.z = trans.item(2)

            rot = tft.quaternion_from_matrix(odom2obj_mat)
            odom2obj_msg.transform.rotation.x = rot[0]
            odom2obj_msg.transform.rotation.y = rot[1]
            odom2obj_msg.transform.rotation.z = rot[2]
            odom2obj_msg.transform.rotation.w = rot[3]
            
            self.tf_broadcaster.sendTransform(odom2obj_msg)
            

class MonitorNode(Node):
    def __init__(self):
        super().__init__('kiwi_monitor_node')
        self.get_logger().info('kiwi_monitor_node: Henlo!')
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.marker_pub = self.create_publisher(Marker, 'object_marker', 10)
        self.timer = self.create_timer(0.5, self.control_cycle)

    def control_cycle(self):
        target_frame = 'base_footprint'
        source_frame = 'detected_object'
        robot2obj_msg = TransformStamped()
        try:
            robot2obj_msg = self.tf_buffer_.lookup_transform(
                target_frame,
                source_frame,
                time=rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'Fail to lookup transform from {target_frame} to {source_frame}: {e}')
            return
        
        x = robot2obj_msg.transform.translation.x
        y = robot2obj_msg.transform.translation.y
        z = robot2obj_msg.transform.translation.z
        theta = math.atan2(y, x)

        self.get_logger().info(f'Object at {x:.2f}, {y:.2f}, {z:.2f}, {math.degrees(theta):.1f} deg')

        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()#Time.to_msg(self.get_clock().now())
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.lifetime = Duration(seconds=1).to_msg()

        origin = Point()
        origin.x = origin.y = origin.z = 0.0
        end = Point()
        end.x = x
        end.y = y
        end.z = z

        marker.points = [origin, end]
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.color.r = marker.color.b = 0.0

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