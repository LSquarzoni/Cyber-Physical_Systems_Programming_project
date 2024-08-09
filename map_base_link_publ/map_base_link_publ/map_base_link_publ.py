import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitude
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class OdometryToTransformNode(Node):
    def __init__(self):
        super().__init__('odometry_to_transform_node')

        self.loc_pos_1 = None
        self.loc_pos_2 = None
        self.att_1 = None
        self.att_2 = None
        
        self.qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        
        # Create a subscriber to the /px4_1/fmu/out/vehicle_local_position topic
        self.loc_pos_1_sub = self.create_subscription(
            VehicleLocalPosition,
            '/px4_1/fmu/out/vehicle_local_position',
            self.loc_pos_callback_1,
            self.qos)

        # Create a subscriber to the /px4_2/fmu/out/vehicle_local_position topic
        self.loc_pos_2_sub = self.create_subscription(
            VehicleLocalPosition,
            '/px4_2/fmu/out/vehicle_local_position',
            self.loc_pos_callback_2,
            self.qos)

        # Create a subscriber to the /px4_1/fmu/out/vehicle_attitude topic
        self.att_1_sub = self.create_subscription(
            VehicleAttitude,
            '/px4_1/fmu/out/vehicle_attitude',
            self.att_callback_1,
            self.qos)

        # Create a subscriber to the /px4_2/fmu/out/vehicle_local_position topic
        self.att_2_sub = self.create_subscription(
            VehicleAttitude,
            '/px4_2/fmu/out/vehicle_attitude',
            self.att_callback_2,
            self.qos)
        
        # Create TransformStamped message
        self.t1 = TransformStamped()
        self.t2 = TransformStamped()

        # Create TransformBroadcaster to publish transforms
        self.tf_broadcaster_1 = tf2_ros.TransformBroadcaster(self)
        self.tf_broadcaster_2 = tf2_ros.TransformBroadcaster(self)

    def loc_pos_callback_1(self, msg):
        self.get_logger().info('Loc pos 1 received')
        self.loc_pos_1 = msg
        self.pub_transf_1()

    def loc_pos_callback_2(self, msg):
        self.get_logger().info('Loc pos 2 received')
        self.loc_pos_2 = msg
        self.pub_transf_2()

    def att_callback_1(self, msg):
        self.get_logger().info('Att 1 received')
        self.att_1 = msg
        self.pub_transf_1()

    def att_callback_2(self, msg):
        self.get_logger().info('Att 2 received')
        self.att_2 = msg
        self.pub_transf_2()

    def pub_transf_1(self):
        if self.loc_pos_1 is not None and self.att_1 is not None:
            # Set the header frame_id to "map"
            self.t1.header.stamp = self.get_clock().now().to_msg()
            self.t1.header.frame_id = 'map'

            # Set the child_frame_id to "base_link"
            self.t1.child_frame_id = 'base_link_1'

            # Set the translation from the odometry message
            self.t1.transform.translation.x = float(self.loc_pos_1.x)
            self.t1.transform.translation.y = float(self.loc_pos_1.y)
            self.t1.transform.translation.z = float(self.loc_pos_1.z)

            # Set the rotation from the odometry message
            self.t1.transform.rotation.x = float(self.att_1.q[1])
            self.t1.transform.rotation.y = float(self.att_1.q[2])
            self.t1.transform.rotation.z = float(self.att_1.q[3])
            self.t1.transform.rotation.w = float(self.att_1.q[0])
            # Publish the transform
            self.tf_broadcaster_1.sendTransform(self.t1)
            self.get_logger().info("Map BaseLink transform 1 published")

    def pub_transf_2(self):
        if self.loc_pos_2 is not None and self.att_2 is not None:
            # Set the header frame_id to "map"
            self.t2.header.stamp = self.get_clock().now().to_msg()
            self.t2.header.frame_id = 'map'

            # Set the child_frame_id to "base_link"
            self.t2.child_frame_id = 'base_link_2'

            # Set the translation from the odometry message
            self.t2.transform.translation.x = float(self.loc_pos_2.x)
            self.t2.transform.translation.y = float(self.loc_pos_2.y)
            self.t2.transform.translation.z = float(self.loc_pos_2.z)

            # Set the rotation from the odometry message
            self.t2.transform.rotation.x = float(self.att_2.q[1])
            self.t2.transform.rotation.y = float(self.att_2.q[2])
            self.t2.transform.rotation.z = float(self.att_2.q[3])
            self.t2.transform.rotation.w = float(self.att_2.q[0])
            # Publish the transform
            self.tf_broadcaster_2.sendTransform(self.t2)
            self.get_logger().info("Map BaseLink transform 2 published")


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