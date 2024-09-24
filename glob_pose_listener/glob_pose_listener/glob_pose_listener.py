import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleGlobalPosition
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import numpy as np

class GlobPoseListener(Node):
    def __init__(self):
        super().__init__('glob_pose_listener')

        self.earth_radius = 6371000

        self.long_1 = None
        self.lat_1 = None
        self.alt_1 = None
        self.x_1 = None
        self.y_1 = None
        self.z_1 = None

        self.long_2 = None
        self.lat_2 = None
        self.alt_2 = None
        self.x_2 = None
        self.y_2 = None
        self.z_2 = None

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
        
    def glob_pos_callback_1(self, msg):
        self.long_1 = msg.lon
        self.lat_1 = msg.lat
        self.alt_1 = msg.alt
        self.get_logger().info(f'Global position of drone 1: {(self.long_1, self.lat_1, self.alt_1)}')
        self.x_1 = self.earth_radius*np.cos(np.radians(self.lat_1))*np.cos(np.radians(self.long_1))
        self.y_1 = self.earth_radius*np.cos(np.radians(self.lat_1))*np.sin(np.radians(self.long_1))
        self.z_1 = self.earth_radius*np.sin(np.radians(self.lat_1))
        self.get_logger().info(f'XYZ of drone 1: {(self.x_1, self.y_1, self.z_1)}')

    def glob_pos_callback_2(self, msg):
        self.long_2 = msg.lon
        self.lat_2 = msg.lat
        self.alt_2 = msg.alt
        self.get_logger().info(f'Global position of drone 2: {(self.long_2, self.lat_2, self.alt_2)}')
        self.x_2 = self.earth_radius*np.cos(np.radians(self.lat_2))*np.cos(np.radians(self.long_2))
        self.y_2 = self.earth_radius*np.cos(np.radians(self.lat_2))*np.sin(np.radians(self.long_2))
        self.z_2 = self.earth_radius*np.sin(np.radians(self.lat_2))
        self.get_logger().info(f'XYZ of drone 2: {(self.x_2, self.y_2, self.z_2)}')

def main(args=None):
    rclpy.init(args=args)
    node = GlobPoseListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()