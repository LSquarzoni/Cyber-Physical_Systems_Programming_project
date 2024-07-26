import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class OdometryToTransformNode(Node):
    def __init__(self):
        super().__init__('odometry_to_transform_node')

        self.odom = None
        
        self.qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        
        # Create a subscriber to the /fmu/out/vehicle_odometry topic
        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            self.qos)
        
        # Create a TransformStamped message
        self.t = TransformStamped()
        
        # Create a TransformBroadcaster to publish transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odometry_callback(self, msg):
        # self.get_logger().info("Odometry received!")
        self.odom = msg
        # Set the header frame_id to "map"
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.header.frame_id = 'map'

        # Set the child_frame_id to "base_link"
        self.t.child_frame_id = 'base_link'

        # Set the translation from the odometry message
        self.t.transform.translation.x = float(self.odom.position[0])
        self.t.transform.translation.y = float(-self.odom.position[1])
        self.t.transform.translation.z = float(-self.odom.position[2])

        # Set the rotation from the odometry message
        self.t.transform.rotation.x = float(self.odom.q[1])
        self.t.transform.rotation.y = float(-self.odom.q[2])
        self.t.transform.rotation.z = float(-self.odom.q[3])
        self.t.transform.rotation.w = float(self.odom.q[0])
        # Publish the transform
        self.tf_broadcaster.sendTransform(self.t)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryToTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


#     import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import threading

# class MyNode(Node):
#     def __init__(self):
#         super().__init__('my_node')
        
#         # Create a TransformBroadcaster
#         self.br = TransformBroadcaster(self)
        
#         # Subscribe to the odometry topic
#         self.subscription = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10)
        
#         # Initialize a variable to store the latest odometry data
#         self.latest_odom = None

#         # Start the thread that will publish transforms
#         self.publish_thread = threading.Thread(target=self.publish_transform)
#         self.publish_thread.start()

#     def odom_callback(self, msg):
#         # Store the latest odometry data
#         self.latest_odom = msg

#     def publish_transform(self):
#         rate = self.create_rate(2)  # 2 Hz rate
#         while rclpy.ok():
#             if self.latest_odom is not None:
#                 # Create a TransformStamped message
#                 t = TransformStamped()
#                 t.header.stamp = self.get_clock().now().to_msg()
#                 t.header.frame_id = 'odom'
#                 t.child_frame_id = 'base_link'
                
#                 # Fill the TransformStamped message with the latest odometry data
#                 t.transform.translation.x = self.latest_odom.pose.pose.position.x
#                 t.transform.translation.y = self.latest_odom.pose.pose.position.y
#                 t.transform.translation.z = self.latest_odom.pose.pose.position.z
                
#                 t.transform.rotation = self.latest_odom.pose.pose.orientation
                
#                 # Broadcast the transform
#                 self.br.sendTransform(t)
            
#             rate.sleep()

# def main(args=None):
#     rclpy.init(args=args)
#     node = MyNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

