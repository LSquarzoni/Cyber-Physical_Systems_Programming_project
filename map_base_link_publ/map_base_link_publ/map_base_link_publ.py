import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitude, VehicleGlobalPosition
from geometry_msgs.msg import TransformStamped
import tf2_ros
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np

class OdometryToTransformNode(Node):
    def __init__(self):
        super().__init__('odometry_to_transform_node')

        self.loc_pos_1 = None
        self.loc_pos_2 = None
        self.att_1 = None
        self.att_2 = None
        
        self.earth_radius = 6371000

        self.glob_pos_reading_1 = 0
        self.long_1 = None
        self.lat_1 = None
        self.alt_1 = None
        self.glob_x_1 = None
        self.glob_init_x_1 = None
        self.glob_y_1 = None
        self.glob_init_y_1 = None

        self.glob_pos_reading_2 = 0
        self.long_2 = None
        self.lat_2 = None
        self.alt_2 = None
        self.glob_x_2 = None
        self.glob_y_2 = None

        self.qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)
        
        # Create a subscriber to the /px4_1/fmu/out/vehicle_global_position topic
        self.loc_pos_1_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/px4_1/fmu/out/vehicle_global_position',
            self.glob_pos_callback_1,
            self.qos)
        
        # Create a subscriber to the /px4_2/fmu/out/vehicle_global_position topic
        self.loc_pos_1_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/px4_2/fmu/out/vehicle_global_position',
            self.glob_pos_callback_2,
            self.qos)
        
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

    def glob_pos_callback_1(self, msg):
        self.long_1 = msg.lon
        self.lat_1 = msg.lat
        self.alt_1 = msg.alt
        # self.get_logger().info(f'Global position of drone 1: {(self.long_1, self.lat_1, self.alt_1)}')
        self.glob_x_1 = self.earth_radius*np.cos(np.radians(self.lat_1))*np.cos(np.radians(self.long_1))
        self.glob_y_1 = self.earth_radius*np.cos(np.radians(self.lat_1))*np.sin(np.radians(self.long_1))
        if self.glob_pos_reading_1 == 0:
            self.glob_init_x_1 = self.glob_x_1
            self.glob_init_y_1 = self.glob_y_1
        self.glob_pos_reading_1 += 1
        # self.glob_z_1 = self.earth_radius*np.sin(np.radians(self.lat_1))
        # if self.glob_pos_reading_1 < 50:
        #     self.glob_x_1_list.append(self.glob_x_1)
        #     self.glob_y_1_list.append(self.glob_y_1)
        #     self.glob_z_1_list.append(self.glob_z_1)
        #     # self.get_logger().info(f'XYZ of drone 1: {(self.x_1, self.y_1, self.z_1)}')
        #     self.glob_pos_reading_1 += 1
        # elif self.glob_pos_reading_1 >= 50:
        #     self.glob_x_1_avg, self.glob_y_1_avg, self.glob_z_1_avg = np.mean(self.glob_x_1_list), np.mean(self.glob_y_1_list), np.mean(self.glob_z_1_list)
        

    def glob_pos_callback_2(self, msg):
        self.long_2 = msg.lon
        self.lat_2 = msg.lat
        self.alt_2 = msg.alt
        # self.get_logger().info(f'Global position of drone 2: {(self.long_2, self.lat_2, self.alt_2)}')
        self.glob_x_2 = self.earth_radius*np.cos(np.radians(self.lat_2))*np.cos(np.radians(self.long_2))
        self.glob_y_2 = self.earth_radius*np.cos(np.radians(self.lat_2))*np.sin(np.radians(self.long_2))
        # self.glob_z_2 = self.earth_radius*np.sin(np.radians(self.lat_2))
        # if self.glob_pos_reading_2 < 50:
        #     self.glob_x_2_list.append(self.glob_x_2)
        #     self.glob_y_2_list.append(self.glob_y_2)
        #     self.glob_z_2_list.append(self.glob_z_2)
        #     # self.get_logger().info(f'XYZ of drone 2: {(self.x_2, self.y_2, self.z_2)}')
        #     self.glob_pos_reading_2 += 1
        # elif self.glob_pos_reading_2 >= 50:
        #     self.glob_x_2_avg, self.glob_y_2_avg, self.glob_z_2_avg = np.mean(self.glob_x_2_list), np.mean(self.glob_y_2_list), np.mean(self.glob_z_2_list)


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
            self.t1.transform.translation.z = float(-self.loc_pos_1.z)

            # Set the rotation from the odometry message
            self.t1.transform.rotation.x = float(self.att_1.q[1])
            self.t1.transform.rotation.y = float(self.att_1.q[2])
            self.t1.transform.rotation.z = float(self.att_1.q[3])
            self.t1.transform.rotation.w = float(self.att_1.q[0])
            # Publish the transform
            self.tf_broadcaster_1.sendTransform(self.t1)
            self.get_logger().info("Map BaseLink transform 1 published")

    def pub_transf_2(self):
        if self.loc_pos_2 is not None and self.att_2 is not None and self.glob_x_1 is not None and self.glob_init_x_1 is not None  \
            and self.glob_y_1 is not None and self.glob_init_y_1 is not None and self.glob_x_2 is not None and self.glob_y_2 is not None:
            # Set the header frame_id to "map"
            self.t2.header.stamp = self.get_clock().now().to_msg()
            self.t2.header.frame_id = 'map'

            # Set the child_frame_id to "base_link"
            self.t2.child_frame_id = 'base_link_2'

            # Set the translation from the odometry message
            self.t2.transform.translation.x = float(self.glob_x_2 - self.glob_init_x_1) #  + self.loc_pos_2.x)
            self.t2.transform.translation.y = float(self.glob_y_2 - self.glob_init_y_1) #  + self.loc_pos_2.y)
            self.t2.transform.translation.z = float(-self.loc_pos_2.z)

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