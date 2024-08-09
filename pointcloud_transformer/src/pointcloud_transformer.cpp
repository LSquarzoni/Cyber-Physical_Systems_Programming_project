#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/qos.hpp>

class PointCloudTransformer : public rclcpp::Node
{
public:
  PointCloudTransformer()
  : Node("point_cloud_transformer")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    point_cloud_pub_1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_wrt_map_1", qos_profile);
    point_cloud_pub_2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_wrt_map_2", qos_profile);

    point_cloud_sub_1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera_1/points", qos_profile, std::bind(&PointCloudTransformer::pointCloudCallback_1, this, std::placeholders::_1));

    point_cloud_sub_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera_2/points", qos_profile, std::bind(&PointCloudTransformer::pointCloudCallback_2, this, std::placeholders::_1));
  }

private:
  // Callback function for handling point clouds from camera 1
  void pointCloudCallback_1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud from camera 1 with:");
    RCLCPP_INFO(this->get_logger(), "  Header stamp: %d", msg->header.stamp.sec);
    RCLCPP_INFO(this->get_logger(), "  Frame id: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Height: %d", msg->height);
    RCLCPP_INFO(this->get_logger(), "  Width: %d", msg->width);
    RCLCPP_INFO(this->get_logger(), "  Is_dense: %s", msg->is_dense ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Point step: %d", msg->point_step);
    RCLCPP_INFO(this->get_logger(), "  Row step: %d", msg->row_step);
    RCLCPP_INFO(this->get_logger(), "  Data size: %lu bytes", msg->data.size());

    std::string target_frame = "map";

    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
        target_frame, msg->header.frame_id, tf2::TimePointZero);

      sensor_msgs::msg::PointCloud2 transformed_msg;
      pcl_ros::transformPointCloud(target_frame, transform_stamped, *msg, transformed_msg);

      point_cloud_pub_1_->publish(transformed_msg);
      RCLCPP_INFO(this->get_logger(), "Transformed point cloud published to %s", point_cloud_pub_1_->get_topic_name());
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
  }

  // Callback function for handling point clouds from camera 2
  void pointCloudCallback_2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received point cloud from camera 2 with:");
    RCLCPP_INFO(this->get_logger(), "  Header stamp: %d", msg->header.stamp.sec);
    RCLCPP_INFO(this->get_logger(), "  Frame id: %s", msg->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  Height: %d", msg->height);
    RCLCPP_INFO(this->get_logger(), "  Width: %d", msg->width);
    RCLCPP_INFO(this->get_logger(), "  Is_dense: %s", msg->is_dense ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  Point step: %d", msg->point_step);
    RCLCPP_INFO(this->get_logger(), "  Row step: %d", msg->row_step);
    RCLCPP_INFO(this->get_logger(), "  Data size: %lu bytes", msg->data.size());

    std::string target_frame = "map";

    try
    {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(
        target_frame, msg->header.frame_id, tf2::TimePointZero);

      sensor_msgs::msg::PointCloud2 transformed_msg;
      pcl_ros::transformPointCloud(target_frame, transform_stamped, *msg, transformed_msg);

      point_cloud_pub_2_->publish(transformed_msg);
      RCLCPP_INFO(this->get_logger(), "Transformed point cloud published to %s", point_cloud_pub_2_->get_topic_name());
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
  }

  // Declare the publishers and subscribers as private member variables
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_2_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_2_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudTransformer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
