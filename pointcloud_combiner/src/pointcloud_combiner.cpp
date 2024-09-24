#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/qos.hpp>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudCombiner : public rclcpp::Node
{
public:
  PointCloudCombiner()
  : Node("point_cloud_combiner")
  {    
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    merged_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("merged_pointcloud_wrt_map", qos_profile);

    point_cloud_sub_1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_wrt_map_1", qos_profile, std::bind(&PointCloudCombiner::pointCloudCallback_1, this, std::placeholders::_1));

    point_cloud_sub_2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_wrt_map_2", qos_profile, std::bind(&PointCloudCombiner::pointCloudCallback_2, this, std::placeholders::_1));
  }

private:

  // Callback function for handling point clouds from camera 1
  void pointCloudCallback_1(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    point_cloud_1_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received point cloud from camera 1");
    point_cloud_merging();
  }

  // Callback function for handling point clouds from camera 2
  void pointCloudCallback_2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    point_cloud_2_ = *msg;
    RCLCPP_INFO(this->get_logger(), "Received point cloud from camera 2");
    point_cloud_merging();
  }

  void point_cloud_merging()
  {
    // Check if both point clouds have been received
    if (point_cloud_1_.data.empty() || point_cloud_2_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "One or both point clouds are empty.");
        return;
    }

    // Convert sensor_msgs::msg::PointCloud2 to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(point_cloud_1_, *pcl_cloud_1);
    pcl::fromROSMsg(point_cloud_2_, *pcl_cloud_2);

    // Merge the point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_merged(new pcl::PointCloud<pcl::PointXYZ>);
    *pcl_cloud_merged = *pcl_cloud_1 + *pcl_cloud_2;

    // // Optionally apply a filter (e.g., voxel grid) to reduce point cloud density
    // pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    // voxel_grid.setInputCloud(pcl_cloud_merged);
    // voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f); // Set the voxel grid leaf size
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // voxel_grid.filter(*pcl_cloud_filtered);

    // Convert pcl::PointCloud to sensor_msgs::msg::PointCloud2
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl::toROSMsg(*pcl_cloud_merged, output_cloud);
    output_cloud.header.frame_id = "map";  // Set the appropriate frame_id
    output_cloud.header.stamp = this->get_clock()->now();

    // Publish the merged point cloud
    merged_point_cloud_pub_->publish(output_cloud);
  }

  // Declare the publishers and subscribers as private member variables
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_point_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_2_;
  sensor_msgs::msg::PointCloud2 point_cloud_1_;
  sensor_msgs::msg::PointCloud2 point_cloud_2_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudCombiner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
