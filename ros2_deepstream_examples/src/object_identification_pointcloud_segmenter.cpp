#include "ros2_deepstream_examples/object_identification_pointcloud_segmenter.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ros2_deepstream_msgs/msg/nv_ds_meta_data.hpp"

namespace {

  /// Is this even a word?
  void SubsectPointCloud(
      const sensor_msgs::msg::PointCloud2& in_cloud, sensor_msgs::msg::PointCloud2* out_cloud,
      const sensor_msgs::msg::RegionOfInterest& region) {
    uint32_t region_size = in_cloud.point_step * region.width * region.height;
    out_cloud->data.resize(region_size);

    out_cloud->header = in_cloud.header;
    out_cloud->fields = in_cloud.fields;
    out_cloud->is_bigendian = in_cloud.is_bigendian;
    out_cloud->height = region.height;
    out_cloud->width = region.width;
    out_cloud->point_step = in_cloud.point_step;
    out_cloud->row_step = region.width * out_cloud->point_step;
    out_cloud->is_dense = in_cloud.is_dense;  // It better be!

    // There is no way this does not cause a segfault
    for (uint32_t row = 0, in_idx = 0, out_idx = 0;
         row < region.height;
         ++row, in_idx += in_cloud.row_step, out_idx += out_cloud->row_step) {
      memcpy(&out_cloud->data[out_idx], &in_cloud.data[in_idx], in_cloud.row_step);
    }
  }
}

ObjectIdentificationPointCloudSegmenter::ObjectIdentificationPointCloudSegmenter(const std::string& node_name)
: rclcpp::Node(node_name) {}

ObjectIdentificationPointCloudSegmenter::~ObjectIdentificationPointCloudSegmenter() {}

bool ObjectIdentificationPointCloudSegmenter::Initialize(
    const std::string& point_cloud_topic, const std::string& image_metadata_topic) {
  pcl2_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(point_cloud_topic, 10, std::bind(&ObjectIdentificationPointCloudSegmenter::OnPointCloud, this, std::placeholders::_1));
  image_metadata_subscription_ = create_subscription<ros2_deepstream_msgs::msg::NvDsMetaData>(image_metadata_topic, 10, std::bind(&ObjectIdentificationPointCloudSegmenter::OnImageMetadata, this, std::placeholders::_1));
  return true;
}

void ObjectIdentificationPointCloudSegmenter::OnPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud) {
  // This seems like a bad race condition...
  if (!image_data_.empty()) {
    auto image_data = image_data_.front();
    image_data_.pop();

    ProcessPointCloudImageDataPair(*point_cloud, *image_data);
  }
  while (point_clouds_.size() > 0) {
    point_clouds_.pop();
  }
  point_clouds_.push(point_cloud);
}

void ObjectIdentificationPointCloudSegmenter::OnImageMetadata(
    const ros2_deepstream_msgs::msg::NvDsMetaData::SharedPtr image_data) {
  // This seems like a bad race condition...
  if (!point_clouds_.empty()) {
    auto point_cloud = point_clouds_.front();
    point_clouds_.pop();

    ProcessPointCloudImageDataPair(*point_cloud, *image_data);
  }
  while (image_data_.size() > 0) {
    image_data_.pop();
  }

  image_data_.push(image_data);
}

void ObjectIdentificationPointCloudSegmenter::ProcessPointCloudImageDataPair(const sensor_msgs::msg::PointCloud2& point_cloud,
                                    const ros2_deepstream_msgs::msg::NvDsMetaData& image_data) {
  if (point_cloud.height == 1) {
    throw std::runtime_error("What are you trying to do, this is an unordered pointcloud!");
  }

  if (point_cloud.height != image_data.height || point_cloud.width != image_data.width) {
    throw std::runtime_error("Seriously, like OMG, the width and height of the your point cloud and image data don't even match. Deal with that first");
  }

  std::vector<sensor_msgs::msg::PointCloud2> tiny_point_clouds;
  for (const auto& region : image_data.regions) {
    tiny_point_clouds.emplace_back();
    SubsectPointCloud(point_cloud, &tiny_point_clouds.back(), region);
  }
}

int main() {
  rclcpp::init(0, nullptr);
  auto segmenter = std::make_shared<ObjectIdentificationPointCloudSegmenter>();
  segmenter->Initialize("/camera/aligned_depth_to_color/color/points", )
  rclcpp::spin(segmenter);
}
