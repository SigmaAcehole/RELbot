// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "image_tools_sdfr/cv_mat_sensor_msgs_image_type_adapter.hpp"
#include "image_tools_sdfr/visibility_control.h"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "image_tools_sdfr/policy_maps.hpp"

#include <chrono>
#include <functional>

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  image_tools_sdfr::ROSCvMatContainer,
  sensor_msgs::msg::Image);


namespace asdfr_group7{
class LightPosition : public rclcpp::Node
{
public:
  
  explicit LightPosition(const rclcpp::NodeOptions & options)
  : Node("lightposition", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    parse_parameters();
    initialize();
  }

private:
  
  void initialize()
  {
    auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(
        // The history policy determines how messages are saved until taken by
        // the reader.
        // KEEP_ALL saves all messages until they are taken.
        // KEEP_LAST enforces a limit on the number of messages that are saved,
        // specified by the "depth" parameter.
        history_policy_,
        // Depth represents how many messages to store in history when the
        // history policy is KEEP_LAST.
        depth_
    ));
    // The reliability policy can be reliable, meaning that the underlying transport layer will try
    // ensure that every message gets received in order, or best effort, meaning that the transport
    // makes no guarantees about the order or reliability of delivery.
    qos.reliability(reliability_policy_);
    pub_ = create_publisher<geometry_msgs::msg::PointStamped>(pub_topic_, 10);

    auto callback =
      [this](const image_tools_sdfr::ROSCvMatContainer & container) {
        process_image(container);
      };

    RCLCPP_INFO(this->get_logger(), "Subscribing to topic '%s'", sub_topic_.c_str());
    sub_ = create_subscription<image_tools_sdfr::ROSCvMatContainer>(sub_topic_, qos, callback);
    handle_param_update();
  }

  void handle_param_update() {
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    auto cb = [this](const rclcpp::Parameter & p) {
        brightness_threshold_ = p.as_int();
      };
    cb_handle_ = param_subscriber_->add_parameter_callback("brightness_threshold", cb);
    auto debug_cb = [this](const rclcpp::Parameter & p) {
      sc_debug_ = p.as_bool();
      RCLCPP_INFO(get_logger(), "Debug changed %d", sc_debug_);
    };
    debug_cb_handle_ = param_subscriber_->add_parameter_callback("debug", debug_cb);

    auto h_lo_cb = [this](const rclcpp::Parameter & p) {
      h_lo_ = p.as_int();
      RCLCPP_INFO(get_logger(), "h_lo_ changed %d", h_lo_);
    };
    h_lo_cb_handle_ = param_subscriber_->add_parameter_callback("h_lo", h_lo_cb);
    auto h_hi_cb = [this](const rclcpp::Parameter & p) {
      h_hi_ = p.as_int();
      RCLCPP_INFO(get_logger(), "h_hi changed %d", h_hi_);
    };
    h_hi_cb_handle_ = param_subscriber_->add_parameter_callback("h_hi", h_hi_cb);

    auto s_lo_cb = [this](const rclcpp::Parameter & p) {
      s_lo_ = p.as_int();
      RCLCPP_INFO(get_logger(), "s_lo_ changed %d", s_lo_);
    };
    s_lo_cb_handle_ = param_subscriber_->add_parameter_callback("s_lo", s_lo_cb);
    auto s_hi_cb = [this](const rclcpp::Parameter & p) {
      s_hi_ = p.as_int();
      RCLCPP_INFO(get_logger(), "s_hi changed %d", s_hi_);
    };
    s_hi_cb_handle_ = param_subscriber_->add_parameter_callback("s_hi", s_hi_cb);

    auto v_lo_cb = [this](const rclcpp::Parameter & p) {
      v_lo_ = p.as_int();
      RCLCPP_INFO(get_logger(), "v_lo_ changed %d", v_lo_);
    };
    v_lo_cb_handle_ = param_subscriber_->add_parameter_callback("v_lo", v_lo_cb);
    auto v_hi_cb = [this](const rclcpp::Parameter & p) {
      v_hi_ = p.as_int();
      RCLCPP_INFO(get_logger(), "v_hi_ changed %d", v_hi_);
    };
    v_hi_cb_handle_ = param_subscriber_->add_parameter_callback("v_hi", v_hi_cb);
  }

  
  void parse_parameters()
  {
    // Parse 'reliability' parameter
    rcl_interfaces::msg::ParameterDescriptor reliability_desc;
    reliability_desc.description = "Reliability QoS setting for the image subscription";
    reliability_desc.additional_constraints = "Must be one of: ";
    for (auto entry : image_tools_sdfr::name_to_reliability_policy_map) {
      reliability_desc.additional_constraints += entry.first + " ";
    }
    const std::string reliability_param = this->declare_parameter(
      "reliability", "reliable", reliability_desc);
    auto reliability = image_tools_sdfr::name_to_reliability_policy_map.find(reliability_param);
    if (reliability == image_tools_sdfr::name_to_reliability_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS reliability setting '" << reliability_param << "'";
      throw std::runtime_error(oss.str());
    }
    reliability_policy_ = reliability->second;

    // Parse 'history' parameter
    rcl_interfaces::msg::ParameterDescriptor history_desc;
    history_desc.description = "History QoS setting for the image subscription";
    history_desc.additional_constraints = "Must be one of: ";
    for (auto entry : image_tools_sdfr::name_to_history_policy_map) {
      history_desc.additional_constraints += entry.first + " ";
    }
    const std::string history_param = this->declare_parameter(
      "history", image_tools_sdfr::name_to_history_policy_map.begin()->first, history_desc);
    auto history = image_tools_sdfr::name_to_history_policy_map.find(history_param);
    if (history == image_tools_sdfr::name_to_history_policy_map.end()) {
      std::ostringstream oss;
      oss << "Invalid QoS history setting '" << history_param << "'";
      throw std::runtime_error(oss.str());
    }
    history_policy_ = history->second;

    // Declare and get remaining parameters
    depth_ = this->declare_parameter("depth", 10);
    window_name_ = this->declare_parameter("window_name", "");
    brightness_threshold_ = this->declare_parameter("brightness_threshold", 128);
    sub_topic_ = this->declare_parameter("image_source_arg", "image");
    RCLCPP_INFO(this->get_logger(), "Reliability: %s, History: %s, depth: %ld", reliability_param.c_str(), history_param.c_str(), depth_);
    this->declare_parameter("debug", sc_debug_);
    this->declare_parameter("h_lo", h_lo_);
    this->declare_parameter("h_hi", h_hi_);
    this->declare_parameter("s_lo", s_lo_);
    this->declare_parameter("s_hi", s_hi_);
    this->declare_parameter("v_lo", v_lo_);
    this->declare_parameter("v_hi", v_hi_);
  }

  
  void process_image(
    const image_tools_sdfr::ROSCvMatContainer & container)
  {
    //clock_gettime(CLOCK_MONOTONIC, &start_clk_);
    cv::Mat frame = container.cv_mat();
    //Convert BGR to HSV
    if (frame.type() == CV_8UC3 /* rgb8 */) {
        cv::cvtColor(frame, frame, cv::COLOR_RGB2HSV);
    } else if (frame.type() == CV_8UC2) {
      container.is_bigendian() ? 
        cv::cvtColor(frame, frame, cv::COLOR_YUV2RGB_UYVY) :
        cv::cvtColor(frame, frame, cv::COLOR_YUV2RGB_YUYV);
        
      cv::cvtColor(frame, frame, cv::COLOR_RGB2HSV);
    } else {
      return;
    }
    // cv::cvtColor(frame, frame, cv::COLOR_BGR2HSV);
    // Threshold green values
    cv::inRange(frame, cv::Scalar(h_lo_, s_lo_, v_lo_), cv::Scalar(h_hi_, s_hi_,v_hi_), frame);

    if (sc_debug_) {
      cv::imshow("Image", frame);
      cv::waitKey(1);
    }

    cv::Moments moments = cv::moments(frame);
    auto non_zero_pix = moments.m00;
    auto moment_x = moments.m10;
    //auto moment_y = moments.m01;

    double cog_x;//, cog_y = 0.0;
    double num_pix = 0.0;
    
    if (non_zero_pix != 0) {
      cog_x = moment_x / non_zero_pix;
      //cog_y = moment_y / non_zero_pix;
    } else {
      cog_x = frame.cols / 2;
      //cog_y = frame.rows / 2;
    }

    num_pix = non_zero_pix/255;
    // // Number of white pixels
    // num_pix = cv::countNonZero(frame);

    // prepare and publish cog message
    auto message = geometry_msgs::msg::PointStamped();
    // message.header.stamp = get_clock()->now();
    message.point.set__x(cog_x);
    message.point.set__y(frame.cols / 2);
    message.point.set__z(num_pix);
    pub_->publish(message);
    // clock_gettime(CLOCK_MONOTONIC, &end_clk_);
    // RCLCPP_INFO(this->get_logger(), "Time taken to calculate light position: %ld", end_clk_.tv_nsec - start_clk_.tv_nsec);
  }

  rclcpp::Subscription<image_tools_sdfr::ROSCvMatContainer>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
  size_t depth_ = rmw_qos_profile_default.depth;
  rmw_qos_reliability_policy_t reliability_policy_ = rmw_qos_profile_default.reliability;
  rmw_qos_history_policy_t history_policy_ = rmw_qos_profile_default.history;
  std::string sub_topic_ = "image";
  std::string pub_topic_ = "light_position";
  std::string window_name_;
  int32_t brightness_threshold_;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;
  struct timespec start_clk_, end_clk_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> h_lo_cb_handle_, h_hi_cb_handle_,
    s_lo_cb_handle_, s_hi_cb_handle_, v_lo_cb_handle_, v_hi_cb_handle_;
  bool sc_debug_ = false;
  int h_lo_ = 36;
  int h_hi_ = 70;
  int s_lo_ = 150;
  int s_hi_ = 255;
  int v_lo_ = 25;
  int v_hi_ = 255; 
};

}  // namespace image_tools_sdfr

RCLCPP_COMPONENTS_REGISTER_NODE(asdfr_group7::LightPosition)
