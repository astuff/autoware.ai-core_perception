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

#include <string>
#include <algorithm>
#include <vector>

#include <autoware_msgs/RecognizeLightState.h>
#include <autoware_msgs/TrafficLight.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "trafficlight_recognizer/context.h"
#include "trafficlight_recognizer/region_tlr_tensorflow/region_tlr_tensorflow.h"

using namespace RegionTLRTensorFlow;  // NOLINT

RegionTLRTensorFlowROSNode::RegionTLRTensorFlowROSNode() :
  image_topic_name_("/image_raw"),
  roi_topic_name_("/roi_signal"),
  score_threshold_(0),
  change_state_threshold_(2),
  use_converted_map_(false),
  use_peoria_hacks_(false)
{
}

void RegionTLRTensorFlowROSNode::ImageRawCallback(const sensor_msgs::ImageConstPtr &msg)
{
  auto se_frame = cv_bridge::toCvShare(msg);

  if (msg->encoding == "bgr8")
  {
    cv::cvtColor(se_frame->image, frame_, cv::COLOR_BGR2RGB);
  }
  else if (msg->encoding == "bayer_bggr8")
  {
    cv::cvtColor(se_frame->image, frame_, cv::COLOR_BayerRG2RGB);
  }
  else if (msg->encoding == "bayer_rggb8")
  {
    cv::cvtColor(se_frame->image, frame_, cv::COLOR_BayerBG2RGB);
  }
  else
  {
    ROS_WARN("Received image of unhandled encoding type.");
  }

  frame_header_ = msg->header;
}

void RegionTLRTensorFlowROSNode::ROISignalCallback(const autoware_msgs::Signals::ConstPtr &extracted_pos)
{
  static ros::Time previous_timestamp;

  // Abort this callback if a new image is not available
  if (frame_.empty() || frame_header_.stamp == previous_timestamp)
    return;

  // The vector of data structure to save traffic light state, position, ...etc
  std::vector<Context> contexts_;

  // Acquire signal position on the image
  Context::SetContexts(&contexts_, extracted_pos, frame_.rows, frame_.cols, use_converted_map_);

  // Recognize the color of the traffic light
  for (Context &context : contexts_)
  {
    if (context.topLeft.x > context.botRight.x || context.closestLaneId == -1)
      continue;

    if (use_peoria_hacks_)
    {
      // Hack to fix ROI for a particular traffic light (right-most when crossing W. M. Kumpf Blvd.)
      // Signal IDs: 176, 185, 158, 186, 177, 157, 175, 168, 184, 167, 159, 166
      std::vector<int> signal_ids_to_shift_1 = {176, 185, 158, 186, 177, 157, 175, 168, 184, 167, 159, 166};

      // Hack to fix ROI for a particular set of traffic lights (end of loop)
      // Signal IDs: 29, 20, 81, 19, 38, 21, 30, 28, 80, 39, 37, 82 (leftmost light)
      // Signal IDs: 32, 41, 84, 23, 22, 83, 40, 42, 31, 24, 85, 33 (middle light)
      // Signal IDs: 34, 44, 43, 25, 86, 26, 35, 45, 87, 27, 36, 88 (rightmost light)
      // Signal IDs: 66, 69, 72, 75, 78 (parts of all three, cannot see on map)
      std::vector<int> signal_ids_to_shift_2 =
      {
        29, 20, 81, 19, 38, 21, 30, 28, 80, 39, 37, 82,
        32, 41, 84, 23, 22, 83, 40, 42, 31, 24, 85, 33,
        34, 44, 43, 25, 86, 26, 35, 45, 87, 27, 36, 88,
        66, 69, 72, 75, 78
      };

      if (std::find(signal_ids_to_shift_1.begin(),
                    signal_ids_to_shift_1.end(),
                    context.signalID) != signal_ids_to_shift_1.end())
      {
        int new_tl_x = context.topLeft.x + -25;
        int new_br_x = context.botRight.x + -25;
        int new_tl_y = context.topLeft.y + 10;
        int new_br_y = context.botRight.y + 10;

        if (new_tl_x < 0)
        {
          new_tl_x = 0;
        }
        else if (new_tl_x >= frame_.cols)
        {
          new_tl_x = frame_.cols - 1;
        }

        if (new_br_x < 0)
        {
          new_br_x = 0;
        }
        else if (new_br_x >= frame_.cols)
        {
          new_br_x = frame_.cols - 1;
        }

        if (new_tl_y < 0)
        {
          new_tl_y = 0;
        }
        else if (new_tl_y >= frame_.rows)
        {
          new_tl_y = frame_.rows - 1;
        }

        if (new_br_y < 0)
        {
          new_br_y = 0;
        }
        else if (new_br_y >= frame_.rows)
        {
          new_br_y = frame_.rows - 1;
        }

        context.topLeft = cv::Point(new_tl_x, new_tl_y);
        context.botRight = cv::Point(new_br_x, new_br_y);
      }
      else if (std::find(signal_ids_to_shift_2.begin(),
                         signal_ids_to_shift_2.end(),
                         context.signalID) != signal_ids_to_shift_2.end())
      {
        int new_tl_x = context.topLeft.x + -20;
        int new_br_x = context.botRight.x + -20;
        int new_tl_y = context.topLeft.y + 20;
        int new_br_y = context.botRight.y + 20;

        if (new_tl_x < 0)
        {
          new_tl_x = 0;
        }
        else if (new_tl_x >= frame_.cols)
        {
          new_tl_x = frame_.cols - 1;
        }

        if (new_br_x < 0)
        {
          new_br_x = 0;
        }
        else if (new_br_x >= frame_.cols)
        {
          new_br_x = frame_.cols - 1;
        }

        if (new_tl_y < 0)
        {
          new_tl_y = 0;
        }
        else if (new_tl_y >= frame_.rows)
        {
          new_tl_y = frame_.rows - 1;
        }

        if (new_br_y < 0)
        {
          new_br_y = 0;
        }
        else if (new_br_y >= frame_.rows)
        {
          new_br_y = frame_.rows - 1;
        }

        context.topLeft = cv::Point(new_tl_x, new_tl_y);
        context.botRight = cv::Point(new_br_x, new_br_y);
      }
    }

    // Extract ROI
    cv::Mat roi = frame_(cv::Rect(context.topLeft, context.botRight)).clone();
    cv_bridge::CvImage converter;
    converter.header = frame_header_;
    converter.encoding = sensor_msgs::image_encodings::RGB8;
    converter.image = roi;
    sensor_msgs::ImagePtr roi_img = converter.toImageMsg();

    // Publish the ROI image for top signal in vector (top signal has largest estimated
    // radius in every signals projected in a image)
    roi_image_publisher.publish(converter.toImageMsg());

    // Get current state of traffic light from current frame
    autoware_msgs::RecognizeLightState srv;
    srv.request.roi_image = *roi_img;
    LightState current_state = LightState::UNDEFINED;

    double confidence = 0.0;
    if (srv_client.call(srv))
    {
      current_state = static_cast<LightState>(srv.response.class_id);
      confidence = srv.response.confidence;
    }
    else
    {
      ROS_ERROR("Failed to call service recognize_light_state");
      return;
    }

    // The state of the traffic light WON'T be changed
    // unless the new state is found at least change_state_threshold_ times
    if (confidence >= score_threshold_)
    {
      DetermineState(current_state, &context);
    }
  }

  if (extracted_pos->Signals.size() == 0)
  {
    std::cout << "No signals in the image" << std::endl;
  }
  else
  {
    // Publish recognition result
    PublishTrafficLight(contexts_);
    PublishImage(contexts_);
    PublishMarkerArray(contexts_);
  }

  PublishString(contexts_);

  // Save timestamp of this frame so that same frame has never been process again
  previous_timestamp = frame_header_.stamp;
}

void RegionTLRTensorFlowROSNode::GetROSParam()
{
  ros::NodeHandle private_node_handle("~");

  private_node_handle.param<std::string>("image_raw_topic", image_topic_name_, "/image_raw");
  ROS_INFO("image_raw_topic: %s", image_topic_name_.c_str());

  private_node_handle.param<std::string>("roi_topic", roi_topic_name_, "/roi_signal");
  ROS_INFO("roi_topic: %s", roi_topic_name_.c_str());

  private_node_handle.param<int>("change_state_threshold", change_state_threshold_, 2);
  ROS_INFO("change_state_threshold: %d", change_state_threshold_);

  private_node_handle.param<double>("score_threshold", score_threshold_, 0.6);
  ROS_INFO("score_threshold: %f", score_threshold_);

  private_node_handle.param<bool>("use_converted_map", use_converted_map_, "false");
  ROS_INFO("use_converted_map: %s", use_converted_map_ ? "true" : "false");

  private_node_handle.param<bool>("use_peoria_hacks", use_peoria_hacks_, "false");
  ROS_INFO("use_peoria_hacks: %s", use_peoria_hacks_ ? "true" : "false");
}

void RegionTLRTensorFlowROSNode::StartSubscribersAndPublishers()
{
  ros::NodeHandle node_handle;

  // Register publishers
  signal_state_publisher = node_handle.advertise<autoware_msgs::TrafficLight>("light_color", 1);
  signal_state_string_publisher = node_handle.advertise<std_msgs::String>("sound_player", 1);
  marker_publisher = node_handle.advertise<visualization_msgs::MarkerArray>("tlr_result", 1, true);
  superimpose_image_publisher = node_handle.advertise<sensor_msgs::Image>("tlr_superimpose_image", 1);
  roi_image_publisher = node_handle.advertise<sensor_msgs::Image>("tlr_roi_image", 1);

  // Register subscribers
  image_transport::ImageTransport it(node_handle);
  image_subscriber = it.subscribe(image_topic_name_,
                                  1,
                                  &RegionTLRTensorFlowROSNode::ImageRawCallback,
                                  this);

  roi_signal_subscriber = node_handle.subscribe("/roi_signal",
                          1,
                          &RegionTLRTensorFlowROSNode::ROISignalCallback,
                          this);
}

/*!
 * DetermineState works as a latch to reduce the chance of sudden changes in the state of the traffic light, caused by
 * misclassifications in the detector. To change the traffic light state, the new candidate should be found at
 * least kChangeStateThreshold times.
 * @param current_state the current state of the traffic light as reported by the classifier.
 * @param in_out_signal_context the object containing the data of the current Traffic Light instance.
 */
void RegionTLRTensorFlowROSNode::DetermineState(const LightState& in_current_state,
                                                Context* in_out_signal_context)
{
  // if reported state by classifier is different than the previously stored
  if (in_current_state != in_out_signal_context->lightState)
  {
    // and also different from the previous difference
    if (in_current_state != in_out_signal_context->newCandidateLightState)
    {
      // set classifier result as a candidate
      in_out_signal_context->newCandidateLightState = in_current_state;
      in_out_signal_context->stateJudgeCount = 0;
    }
    else
    {
      // if classifier returned the same result previously, then increase its confidence
      in_out_signal_context->stateJudgeCount++;

      if (in_out_signal_context->stateJudgeCount > change_state_threshold_)
      {
        in_out_signal_context->stateJudgeCount = change_state_threshold_;  // prevent overflow
      }

      // if new candidate has been found enough times, change state to the new candidate
      if (in_out_signal_context->stateJudgeCount >= change_state_threshold_)
      {
        in_out_signal_context->lightState = in_current_state;
      }
    }
  }
}

void RegionTLRTensorFlowROSNode::PublishTrafficLight(std::vector<Context> contexts)
{
  autoware_msgs::TrafficLight topic;
  static int32_t previous_state = LightState::UNDEFINED;
  topic.traffic_light = LightState::UNDEFINED;

  for (const auto ctx : contexts)
  {
    switch (ctx.lightState)
    {
    case GREEN:
      topic.traffic_light = LightState::GREEN;
      break;
    case YELLOW:  // Autoware currently treats yellow as red.
    case RED:
      topic.traffic_light = LightState::RED;
      break;
    case UNDEFINED:
      topic.traffic_light = LightState::UNDEFINED;
      break;
    }

    // Publish the first state in contexts,
    // which has largest estimated radius of signal.
    // This program assume that the signal which has the largest estimated radius
    // equal the nearest one from camera.
    if (topic.traffic_light != LightState::UNDEFINED)
    {
      break;
    }
  }

  // If state changes from previous one, publish it
  static ros::Time prev_time = ros::Time::now();
  double timeout = 10.0;  // seconds
  if (topic.traffic_light != previous_state || ros::Time::now() - prev_time > ros::Duration(timeout))
  {
    prev_time = ros::Time::now();
    signal_state_publisher.publish(topic);
    previous_state = topic.traffic_light;
  }
}

void RegionTLRTensorFlowROSNode::PublishString(std::vector<Context> contexts)
{
  std_msgs::String topic;
  static std::string previous_state = UNKNOWN_STRING;
  topic.data = UNKNOWN_STRING;
  for (const auto ctx : contexts)
  {
    switch (ctx.lightState)
    {
    case GREEN:
      topic.data = GREEN_STRING;
      break;
    case YELLOW:  // Autoware currently treats yellow as red.
    case RED:
      topic.data = RED_STRING;
      break;
    case UNDEFINED:
      topic.data = UNKNOWN_STRING;
      break;
    }

    // Publish the first state in contexts,
    // which has largest estimated radius of signal.
    // This program assume that the signal which has the largest estimated radius
    // equal the nearest one from camera.
    if (topic.data != UNKNOWN_STRING)
    {
      break;
    }
  }

  // If state changes from previous one, publish it
  if (topic.data != previous_state)
  {
    signal_state_string_publisher.publish(topic);
    previous_state = topic.data;
  }
}

void RegionTLRTensorFlowROSNode::PublishMarkerArray(std::vector<Context> contexts)
{
  // Define color constants
  std_msgs::ColorRGBA color_black;
  color_black.r = 0.0f;
  color_black.g = 0.0f;
  color_black.b = 0.0f;
  color_black.a = 1.0f;

  std_msgs::ColorRGBA color_red;
  color_red.r = 1.0f;
  color_red.g = 0.0f;
  color_red.b = 0.0f;
  color_red.a = 1.0f;

  std_msgs::ColorRGBA color_yellow;
  color_yellow.r = 1.0f;
  color_yellow.g = 1.0f;
  color_yellow.b = 0.0f;
  color_yellow.a = 1.0f;

  std_msgs::ColorRGBA color_green;
  color_green.r = 0.0f;
  color_green.g = 1.0f;
  color_green.b = 0.0f;
  color_green.a = 1.0f;

  // publish all result as ROS MarkerArray
  for (const auto ctx : contexts)
  {
    visualization_msgs::MarkerArray signal_set;
    visualization_msgs::Marker red_light, yellow_light, green_light;

    // Set the frame ID
    red_light.header.frame_id = "map";
    yellow_light.header.frame_id = "map";
    green_light.header.frame_id = "map";

    // Set the namespace and ID for this markers
    red_light.ns = "tlr_result_red";
    red_light.id = ctx.signalID;

    yellow_light.ns = "tlr_result_yellow";
    yellow_light.id = ctx.signalID;

    green_light.ns = "tlr_result_green";
    green_light.id = ctx.signalID;

    // Set the markers type
    red_light.type = visualization_msgs::Marker::SPHERE;
    yellow_light.type = visualization_msgs::Marker::SPHERE;
    green_light.type = visualization_msgs::Marker::SPHERE;

    // Set the pose of the markers
    red_light.pose.position.x = ctx.redCenter3d.x;
    red_light.pose.position.y = ctx.redCenter3d.y;
    red_light.pose.position.z = ctx.redCenter3d.z;
    red_light.pose.orientation.x = 0.0;
    red_light.pose.orientation.y = 0.0;
    red_light.pose.orientation.z = 0.0;
    red_light.pose.orientation.w = 0.0;

    yellow_light.pose.position.x = ctx.yellowCenter3d.x;
    yellow_light.pose.position.y = ctx.yellowCenter3d.y;
    yellow_light.pose.position.z = ctx.yellowCenter3d.z;
    yellow_light.pose.orientation.x = 0.0;
    yellow_light.pose.orientation.y = 0.0;
    yellow_light.pose.orientation.z = 0.0;
    yellow_light.pose.orientation.w = 0.0;

    green_light.pose.position.x = ctx.greenCenter3d.x;
    green_light.pose.position.y = ctx.greenCenter3d.y;
    green_light.pose.position.z = ctx.greenCenter3d.z;
    green_light.pose.orientation.x = 0.0;
    green_light.pose.orientation.y = 0.0;
    green_light.pose.orientation.z = 0.0;
    green_light.pose.orientation.w = 0.0;

    // Set the scale of the markers. We assume lamp radius is 30cm in real world
    red_light.scale.x = RegionTLRTensorFlow::LIGHT_SIZE;
    red_light.scale.y = RegionTLRTensorFlow::LIGHT_SIZE;
    red_light.scale.z = RegionTLRTensorFlow::LIGHT_SIZE;

    yellow_light.scale.x = RegionTLRTensorFlow::LIGHT_SIZE;
    yellow_light.scale.y = RegionTLRTensorFlow::LIGHT_SIZE;
    yellow_light.scale.z = RegionTLRTensorFlow::LIGHT_SIZE;

    green_light.scale.x = RegionTLRTensorFlow::LIGHT_SIZE;
    green_light.scale.y = RegionTLRTensorFlow::LIGHT_SIZE;
    green_light.scale.z = RegionTLRTensorFlow::LIGHT_SIZE;

    // Set the color for each marker
    switch (ctx.lightState)
    {
    case GREEN:
      red_light.color = color_black;
      yellow_light.color = color_black;
      green_light.color = color_green;
      break;
    case YELLOW:
      red_light.color = color_black;
      yellow_light.color = color_yellow;
      green_light.color = color_black;
      break;
    case RED:
      red_light.color = color_red;
      yellow_light.color = color_black;
      green_light.color = color_black;
      break;
    case UNDEFINED:
      red_light.color = color_black;
      yellow_light.color = color_black;
      green_light.color = color_black;
      break;
    }

    red_light.lifetime = ros::Duration(0.1);
    yellow_light.lifetime = ros::Duration(0.1);
    green_light.lifetime = ros::Duration(0.1);

    // Pack each light marker into one
    signal_set.markers.push_back(red_light);
    signal_set.markers.push_back(yellow_light);
    signal_set.markers.push_back(green_light);

    // Publish
    marker_publisher.publish(signal_set);
  }
}

void RegionTLRTensorFlowROSNode::PublishImage(std::vector<Context> contexts)
{
  // Copy the frame image for output
  cv::Mat result_image;
  cv::cvtColor(frame_, result_image, cv::COLOR_RGB2BGR);

  // Define information for written label
  std::string label;
  int kFontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
  int kThickness = 1;
  double kFontScale = 0.8;
  int font_baseline = 0;
  CvScalar label_color;

  std::vector<int> already_drawn;
  for (const auto ctx : contexts)
  {
    if (std::find(already_drawn.begin(), already_drawn.end(), ctx.closestLaneId) != already_drawn.end()
        || ctx.closestLaneId == -1)
      continue;

    already_drawn.push_back(ctx.closestLaneId);

    // Draw recognition result on image
    switch (ctx.lightState)
    {
    case GREEN:
      label = "GREEN";
      label_color = CV_RGB(0, 255, 0);
      break;
    case YELLOW:
      label = "YELLOW";
      label_color = CV_RGB(255, 255, 0);
      break;
    case RED:
      label = "RED";
      label_color = CV_RGB(255, 0, 0);
      break;
    case UNDEFINED:
      label = "UNKNOWN";
      label_color = CV_RGB(0, 0, 0);
    }

    cv::rectangle(result_image, ctx.topLeft, ctx.botRight, label_color, 2);

    if (ctx.leftTurnSignal)
    {
      label += " LEFT";
    }
    if (ctx.rightTurnSignal)
    {
      label += " RIGHT";
    }

    // Add lane ID text
    label += " " + std::to_string(ctx.closestLaneId);
    label += " " + std::to_string(ctx.lightState);
    cv::Point label_origin = cv::Point(ctx.topLeft.x, ctx.botRight.y + font_baseline);
    cv::putText(result_image, label, label_origin, kFontFace, kFontScale, label_color, kThickness);
  }

  std_msgs::String topic;
  for (const auto ctx : contexts)
  {
    switch (ctx.lightState)
    {
    case GREEN:
      topic.data = GREEN_STRING;
      label_color = CV_RGB(0, 255, 0);
      break;
    case YELLOW:  // Autoware currently treats yellow as red.
    case RED:
      topic.data = RED_STRING;
      label_color = CV_RGB(255, 0, 0);
      break;
    case UNDEFINED:
      topic.data = UNKNOWN_STRING;
      label_color = CV_RGB(0, 0, 0);
    }

    // Publish the first state in contexts,
    // which has largest estimated radius of signal.
    // This program assume that the signal which has the largest estimated radius
    // equal the nearest one from camera.
    if (topic.data != UNKNOWN_STRING)
    {
      break;
    }
  }

  // Add text with state of closest signal
  kFontFace = cv::FONT_HERSHEY_TRIPLEX;
  kFontScale *= 3;
  kThickness *= 3;
  std::transform(topic.data.begin(), topic.data.end(), topic.data.begin(), ::toupper);
  cv::Size textSize = cv::getTextSize(topic.data, kFontFace, kFontScale, kThickness, &font_baseline);
  cv::Point label_origin((frame_.cols - textSize.width) / 2, 3 * (frame_.rows + textSize.height) / 4);
  cv::putText(result_image, topic.data, label_origin, kFontFace, kFontScale, label_color, kThickness);

  // Publish superimpose result image
  cv_bridge::CvImage converter;
  converter.header = frame_header_;
  converter.encoding = sensor_msgs::image_encodings::BGR8;
  converter.image = result_image;
  superimpose_image_publisher.publish(converter.toImageMsg());
}

int main(int argc, char *argv[])
{
  // Initialize ros node
  ros::init(argc, argv, "region_tlr_tensorflow_node");
  ros::NodeHandle n;

  // Create RegionTLRTensorFlowROSNode class object and do initialization
  RegionTLRTensorFlowROSNode region_tlr_tensorflow_ros_node;
  region_tlr_tensorflow_ros_node.srv_client =
    n.serviceClient<autoware_msgs::RecognizeLightState>("recognize_light_state", false);
  region_tlr_tensorflow_ros_node.srv_client.waitForExistence();

  // Start recognition process
  region_tlr_tensorflow_ros_node.GetROSParam();  // get execution parameters from ROS parameter server
  region_tlr_tensorflow_ros_node.StartSubscribersAndPublishers();
  ROS_INFO("Node initialized, waiting for signals from feat_proj...");
  ros::spin();

  return 0;
}
