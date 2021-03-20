//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
#ifndef LIVOX_ROS_DRIVER_LDDC_H_
#define LIVOX_ROS_DRIVER_LDDC_H_

#include <string>
#include <memory>
#include "lds.h"
#include "livox_sdk.h"

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "livox_interfaces/msg/custom_point.hpp"
#include "livox_interfaces/msg/custom_msg.hpp"

namespace livox_ros {

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

/** Send pointcloud message Data to ros subscriber or save them in rosbag file */
typedef enum {
  kOutputToRos = 0,
  kOutputToRosBagFile = 1,
} DestinationOfMessageOutput;

/** The message type of transfer */
typedef enum {
  kPointCloud2Msg = 0,
  kLivoxCustomMsg = 1,
  kPclPxyziMsg = 2,
  kLivoxImuMsg = 3,
} MessageTypeOfTransfer;

class Lddc {
 public:
  Lddc(int format, int multi_topic, int data_src, int output_type, double frq,
       std::string &frame_id);
  ~Lddc();

  int RegisterLds(Lds *lds);
  void DistributeLidarData(void);
  void CreateBagFile(const std::string &file_name);
  void PrepareExit(void);

  uint8_t GetTransferFormat(void) { return transfer_format_; }
  uint8_t IsMultiTopic(void) { return use_multi_topic_; }
  void SetRosNode(rclcpp::Node * node) { cur_node_ = node; }
  void SetPublishFrq(uint32_t frq) { publish_frq_ = frq; }

  Lds *lds_;

 private:
  int32_t GetPublishStartTime(LidarDevice *lidar, LidarDataQueue *queue,
                              uint64_t *start_time,
                              StoragePacket *storage_packet);
  uint32_t PublishPointcloud2(LidarDataQueue *queue, uint32_t packet_num,
                              uint8_t handle);
  uint32_t PublishPointcloudData(LidarDataQueue *queue, uint32_t packet_num,
                                 uint8_t handle);
  uint32_t PublishCustomPointcloud(LidarDataQueue *queue, uint32_t packet_num,
                                   uint8_t handle);
  uint32_t PublishImuData(LidarDataQueue *queue, uint32_t packet_num,
                          uint8_t handle);

  std::shared_ptr<rclcpp::PublisherBase> CreatePublisher(uint8_t msg_type,
    std::string &topic_name, uint32_t queue_size);
  std::shared_ptr<rclcpp::PublisherBase> GetCurrentPublisher(uint8_t handle);
  std::shared_ptr<rclcpp::PublisherBase> GetCurrentImuPublisher(uint8_t handle);
  void PollingLidarPointCloudData(uint8_t handle, LidarDevice *lidar);
  void PollingLidarImuData(uint8_t handle, LidarDevice *lidar);
  void InitPointcloud2MsgHeader(sensor_msgs::msg::PointCloud2& cloud);
  void FillPointsToPclMsg(PointCloud& pcl_msg,
      LivoxPointXyzrtl* src_point, uint32_t num);
  void FillPointsToCustomMsg(livox_interfaces::msg::CustomMsg& livox_msg,
      LivoxPointXyzrtl* src_point, uint32_t num, uint32_t offset_time,
      uint32_t point_interval, uint32_t echo_num);
  uint8_t transfer_format_;
  uint8_t use_multi_topic_;
  uint8_t data_src_;
  uint8_t output_type_;
  double publish_frq_;
  uint32_t publish_period_ns_;
  std::string frame_id_;

  std::shared_ptr<rclcpp::PublisherBase>private_pub_[kMaxSourceLidar];
  std::shared_ptr<rclcpp::PublisherBase>global_pub_;
  std::shared_ptr<rclcpp::PublisherBase>private_imu_pub_[kMaxSourceLidar];
  std::shared_ptr<rclcpp::PublisherBase>global_imu_pub_;
  rclcpp::Node* cur_node_;
  // rclcpp::rosbag::Bag *bag_;
};

}  // namespace livox_ros
#endif
