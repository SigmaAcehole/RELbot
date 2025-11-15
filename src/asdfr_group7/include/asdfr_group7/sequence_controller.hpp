#ifndef SEQUENCE_CONTROLLER_HPP_
#define SEQUENCE_CONTROLLER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "custom_msgs/msg/ros2_xeno.hpp"
#include "custom_msgs/msg/xeno2_ros.hpp"

using namespace std::chrono_literals;

constexpr auto SETPOINT_PERIOD = 300ms;

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief Sequence controller node.
 *
 * ROS2 parameters are described below:
 *
 * @param wheel_base_width [double]: Width in meters between the robot's powered wheels. Default 0.209.
 * @param wheel_radius [double]: Wheel radius in meters for the robot's powered wheels. Default 0.05.
 * @param time_step [double]: Simulator time step in seconds. Default 0.02 (corresponding to 50 Hz).
 * @param time_constant [double]: Time constant to make motor velocities behave as 1st order system. Default 0.3.
 * @param use_twist_cmd [bool]: Which command mode to use: true is Twist, false is Individual Motors. Default is false.
 * @param global_frame [string]: Name of global frame. Default is "map".
 * @param base_link_frame [string]: Name of base link frame in URDF. Default is "base_link".
 * @param right_wheel_joint [string]: Name of right wheel joint in URDF. Default is "right_wheel_joint".
 * @param left_wheel_joint [string]: Name of left wheel joint in URDF. Default is "left_wheel_joint".
 * @param caster_swivel_joint [string]: Name of caster swivel joint in URDF. Default is "caster_swivel_joint".
 * @param caster_wheel_joint [string]: Name of caster wheel joint in URDF. Default is "caster_wheel_joint".
 * @todo Potentially add topics as parameters here too.
 */

class SequenceController : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new SequenceController object
   *
   */
  SequenceController();

  const std::string input = "input";
  const std::string output = "output";

  // Default input topic names
  const std::string WEBCAM_IMAGE = "/image";
  const std::string XENO2ROS_TOPIC = "Xeno2Ros";
  const std::string LIGHT_POSITION_TOPIC =  "light_position";

  // Default output topic names
  const std::string ROS2XENO_TOPIC = "Ros2Xeno";

  // Default parameter values
  const double DEFAULT_WHEEL_BASE_WIDTH = 0.209; // [m] Width between the center of both wheels
  const double DEFAULT_WHEEL_RADIUS = 0.05;      // [m] Radius of the wheels
  const double DEFAULT_TIME_STEP = 0.02;         // [s] Time step of the simulation
  const double DEFAULT_TIME_CONSTANT = 0.3;      // [s] Time constant to make motor velocities behave as 1st order system
  const bool DEFAULT_USE_TWIST_CMD = false;      // Which command mode (true is Twist, false is Individual Motors)

  const float MAX_THETA = 60.0f * M_PI / 180;      // Max theta coordinate

private:
  // Timer to run the simulator step function peridically
  rclcpp::TimerBase::SharedPtr timer_;

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr light_position_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr webcam_input_subscription_;
  rclcpp::Subscription<custom_msgs::msg::Xeno2Ros>::SharedPtr xeno2ros_subscription_;

  // Publishers
  rclcpp::Publisher<custom_msgs::msg::Ros2Xeno>::SharedPtr ros2xeno_publisher_;

  std::vector<std::pair<float,float>> setpoint_vector_;
  float cur_x_ = 0.0f;
  float cur_y_ = 0.0f;
  float cur_theta_ = 0.0f;
  size_t count_;
  float light_x = 0.0f;
  float light_y = 0.0f;
  int image_height = 0;
  int image_width = 0;
  float des_x = 0.0f;

  float camera_x_ = 0.0f;
  float camera_y_ = 0.0f;

  int max_motor_vel = 24;

  float Kp = 1;   // Proportional gain
  float Kd = 0.1;
  float error_prev = 0;

  bool is_light_pos_rec_ = false;
  bool is_relbot_init_ = false;
  bool is_camera_dim_rec_ = false;

  enum MODE {
    STATIC,
    DYNAMIC_IMAGE
  };

  int sequence_controller_mode_ = STATIC;

  float left_motor_param_ = 0.0f;
  float right_motor_param_ = 0.0f;
  int num_green_pixels = 0;
  int min_threshold_ = 200; // # of pixels
  int max_threshold_ = 50; // pixel percentage

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> left_motor_cb_handle_, right_motor_cb_handle_, dynamic_cb_handle_,
                                                   debug_cb_handle_, min_threshold_cb_handle_, max_threshold_cb_handle_;

  bool sc_debug_ = false;

  // Debug topics

  const std::string DEBUG_XENO2ROS_TOPIC = "debug/" + XENO2ROS_TOPIC;
  const std::string DEBUG_ROS2XENO_TOPIC = "debug/" + ROS2XENO_TOPIC;

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_ros2xeno_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_xeno2ros_publisher_;

  /**
   * @brief Initialize setpoints for static controller
   *
   */
  void init_setpoints();

  /**
   * @brief setpoint retriever for static controller
   *
   */
  std::pair<float,float> get_setpoint();

  /**
   * @brief controller step
   *
   */
  std::pair<float,float> sequence_controller(float des_x, float theta);

  /**
   * @brief parameter declaration
   *
   */
  void declare_parameters();

  /**
   * @brief subscription creation
   *
   */
  void create_subscribers();

  /**
   * @brief publishers creation
   *
   */
  void create_publishers();

  /**
   * @brief image topic callback to retrieve the image dimensions used for the calculations in the controller
   *
   */
  void webcam_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img);

  /**
   * @brief light position topic callback to track the light position in the controller
   *
   */
  void light_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr point);

  /**
   * @brief Main step loop of the system.
   * Publishes motor setpoint values to the RELbot simulator to update to the next timestep
   *
   */
  void timer_callback();

  /**
  * @brief has synced to the other topic/nodes
  */
  bool canStart();

  /**
  * @brief Xeno 2 ros callback
  */
  void xeno2ros_callback(const custom_msgs::msg::Xeno2Ros::SharedPtr xeno2ros_msg);

    /**
  * @brief Setpoint values for the closed loop
  */
  std::pair<float, float> get_setpoint_for_green_object();

};
#endif // SEQUENCE_CONTROLLER_HPP_
