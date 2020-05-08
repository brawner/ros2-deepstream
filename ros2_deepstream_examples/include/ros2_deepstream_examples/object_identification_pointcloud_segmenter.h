#ifndef __ROS2_DEEPSTREAM_EXAMPLES_OBJECT_IDENTIFICATION_POINTCLOUD_SEGMENTER__
#define _ROS2_DEEPSTREAM_EXAMPLES_OBJECT_IDENTIFICATION_POINTCLOUD_SEGMENTER_

#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ros2_deepstream_msgs/msg/nv_ds_meta_data.hpp"

class ObjectIdentificationPointCloudSegmenter : public rclcpp::Node {
public:
  ObjectIdentificationPointCloudSegmenter(const std::string& node_name = "oi_pcl_segmenter");
  ~ObjectIdentificationPointCloudSegmenter();

  bool Initialize(const std::string& point_cloud_topic, const std::string& image_metadata_topic);

  void OnPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud);
  void OnImageMetadata(const ros2_deepstream_msgs::msg::NvDsMetaData::SharedPtr image_data);

private:
  void ProcessPointCloudImageDataPair(const sensor_msgs::msg::PointCloud2& point_cloud,
                                      const ros2_deepstream_msgs::msg::NvDsMetaData& image_data);
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pcl2_subscription_;
  std::shared_ptr<rclcpp::Subscription<ros2_deepstream_msgs::msg::NvDsMetaData>> image_metadata_subscription_;

  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> point_clouds_;
  std::queue<ros2_deepstream_msgs::msg::NvDsMetaData::ConstSharedPtr> image_data_;
};

#endif  // _ROS2_DEEPSTREAM_EXAMPLES_OBJECT_IDENTIFICATION_POINTCLOUD_SEGMENTER_
