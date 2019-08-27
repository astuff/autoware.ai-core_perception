/*
 * Copyright 2019 AutonomouStuff, LLC
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TRAFFICLIGHT_RECOGNIZER_REGION_TLR_TENSORFLOW_REGION_TLR_TENSORFLOW_H
#define TRAFFICLIGHT_RECOGNIZER_REGION_TLR_TENSORFLOW_REGION_TLR_TENSORFLOW_H

#include <string>
#include <iostream>
#include <fstream>
#include <vector>

#include <opencv2/opencv.hpp>

#include <autoware_msgs/Signals.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>

#include "trafficlight_recognizer/context.h"

class RegionTLRTensorFlowROSNode
{
public:
  RegionTLRTensorFlowROSNode();
  ~RegionTLRTensorFlowROSNode();

  void RunRecognition();
  void ImageRawCallback(const sensor_msgs::ImageConstPtr &msg);
  void ROISignalCallback(const autoware_msgs::Signals::ConstPtr &extracted_pos);

  // The vector of data structure to save traffic light state, position, ...etc
  std::vector<Context> contexts_;

  // Service client
  ros::ServiceClient srv_client;

private:
  void GetROSParam();
  void StartSubscribersAndPublishers();
  void DetermineState(const LightState& in_current_state, Context* in_out_signal_context);
  void PublishTrafficLight(std::vector<Context> contexts);
  void PublishString(std::vector<Context> contexts);
  void PublishMarkerArray(std::vector<Context> contexts);
  void PublishImage(std::vector<Context> contexts);
  void SuperimposeCb(const std_msgs::Bool::ConstPtr &config_msg);

  // Execution parameter
  std::string image_topic_name_;
  std::string roi_topic_name_;
  double score_threshold_;
  // The threshold of state detected times to accept the state change
  int change_state_threshold_;
  bool use_converted_map_;
  bool use_peoria_hacks_;

  // Subscribers
  image_transport::Subscriber image_subscriber;
  ros::Subscriber roi_signal_subscriber;
  ros::Subscriber superimpose_sub;

  // Publishers
  ros::Publisher signal_state_publisher;
  ros::Publisher signal_state_string_publisher;
  ros::Publisher marker_publisher;
  ros::Publisher superimpose_image_publisher;
  ros::Publisher roi_image_publisher;

  // Flag to show topic will be published in latch manner
  bool kAdvertiseInLatch_;

  // A frame image acquired from topic
  cv::Mat frame_;

  // Timestamp of a frame in process
  std_msgs::Header frame_header_;

  // constant values to pass recognition states to other nodes
  const int32_t kTrafficLightRed;
  const int32_t kTrafficLightGreen;
  const int32_t kTrafficLightUnknown;
  const std::string kStringRed;
  const std::string kStringGreen;
  const std::string kStringUnknown;

  // Size of traffic light in cm
  const float LIGHT_SIZE = 0.3;
};

#endif  // TRAFFICLIGHT_RECOGNIZER_REGION_TLR_TENSORFLOW_REGION_TLR_TENSORFLOW_H
