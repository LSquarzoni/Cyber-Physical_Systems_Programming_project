import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class OdometryToTransformNode(Node):
    def __init__(self):
        super().__init__('odometry_to_transform_node')
        
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
        
        # Create a TransformBroadcaster to publish transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def odometry_callback(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the header frame_id to "map"
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'

        # Set the child_frame_id to "base_link"
        t.child_frame_id = 'base_link'

        # Set the translation from the odometry message
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])

        # Set the rotation from the odometry message
        t.transform.rotation.x = float(msg.q[0])
        t.transform.rotation.y = float(msg.q[1])
        t.transform.rotation.z = float(msg.q[2])
        t.transform.rotation.w = float(msg.q[3])
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = OdometryToTransformNode()

    # Keep the node running until it's interrupted
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
