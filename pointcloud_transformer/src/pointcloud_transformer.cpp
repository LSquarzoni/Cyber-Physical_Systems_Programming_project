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

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_wrt_map", qos_profile);
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/points", qos_profile, std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    // Print basic information from the point cloud message
    RCLCPP_INFO(this->get_logger(), "Received point cloud with:");
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

      point_cloud_pub_->publish(transformed_msg);
      RCLCPP_INFO(this->get_logger(), "Transformed point cloud published to %s", point_cloud_pub_->get_topic_name());
    }
    catch (tf2::TransformException & ex)
    {
      RCLCPP_WARN(this->get_logger(), "Could not transform point cloud: %s", ex.what());
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
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
