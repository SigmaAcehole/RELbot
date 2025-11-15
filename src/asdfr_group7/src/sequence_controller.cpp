
#include "asdfr_group7/sequence_controller.hpp"

SequenceController::SequenceController() : rclcpp::Node("sequence_controller") {
  count_ = 0;
  declare_parameters();
  create_publishers();
  create_subscribers();
  timer_ = this->create_wall_timer(SETPOINT_PERIOD, std::bind(&SequenceController::timer_callback, this));
}

void SequenceController::timer_callback()
{
  //if (!canStart()) return;

  auto setpoint = get_setpoint();

  auto des_x = setpoint.first;
  auto des_theta = std::clamp(setpoint.second, -MAX_THETA, MAX_THETA);

  auto ros2xeno_message = custom_msgs::msg::Ros2Xeno();
  std::pair<float, float> motor_setpoint_pair;
  if (sequence_controller_mode_ == DYNAMIC_IMAGE) {
    motor_setpoint_pair = sequence_controller(des_x, des_theta);
    ros2xeno_message.x = motor_setpoint_pair.first;
    ros2xeno_message.y = motor_setpoint_pair.second;
    RCLCPP_INFO(this->get_logger(), "dynamic Publishing motor: '%f' '%f'", ros2xeno_message.x, ros2xeno_message.y);
  } else {
    ros2xeno_message.x = left_motor_param_;
    ros2xeno_message.y = right_motor_param_;
    RCLCPP_INFO(this->get_logger(), "Publishing motor: '%f' '%f'", ros2xeno_message.x, ros2xeno_message.y);
  }

  ros2xeno_publisher_->publish(ros2xeno_message); 
  //RCLCPP_INFO(this->get_logger(), "Publishing motor: '%f' '%f'", ros2xeno_message.x, ros2xeno_message.y);
  if (sc_debug_) {
    auto message = geometry_msgs::msg::PointStamped();
    message.header.stamp = get_clock()->now();
    message.point.set__x(ros2xeno_message.x);
    message.point.set__y(ros2xeno_message.y);
    debug_ros2xeno_publisher_->publish(message);
  }
}

bool SequenceController::canStart() {
  return is_light_pos_rec_ && is_camera_dim_rec_;
}

std::pair<float,float> SequenceController::get_setpoint()
{
  switch (sequence_controller_mode_) {
    case STATIC:
      return std::make_pair(0, 0);

    case DYNAMIC_IMAGE:
      return get_setpoint_for_green_object();
    
    default:
      return std::make_pair(0, 0);
  }
}

std::pair<float, float> SequenceController::get_setpoint_for_green_object(){
  const double height = image_height;
  const double width = image_width;
  
  const double img_width_rad = 120 * (M_PI / 180); // change 120 to 60 if needed
  const double pixels_per_rad = (width / 2) / img_width_rad;
  
  // Update theta  
  float del_theta = ((width/2) - light_x)*(1./pixels_per_rad);
  float theta = std::clamp(del_theta, (float) -MAX_THETA, (float) MAX_THETA); 

  float num_pixel_percent = 100*(num_green_pixels)/(height*width);
  float vx = 0;

  //RCLCPP_INFO(this->get_logger(), "green: '%d', x_light '%f\n'", num_green_pixels, light_x); 

  if(num_green_pixels < min_threshold_){
    return std::make_pair(0,0);
  }
  if(num_pixel_percent >= max_threshold_){
  // Update velocity
  // if((height*width)/2 - num_green_pixels <= 0){
    vx = 0;     
    return std::make_pair(vx, theta); // putting line #95 here just to get rid of the unused warning, otherwise not necessary
  }
  else{
    vx = 0.1; // linear movement
  }

  // light_x_trans, light_x, camera_x_, theta_delta, theta);


  return std::make_pair(vx, theta);
}

std::pair<float,float> SequenceController::sequence_controller(float vx, float theta){
  float w = (theta - cur_theta_)/DEFAULT_TIME_CONSTANT;

  // float d = (des_x - cur_x_)/(std::cos(theta));
  // float vx = d/DEFAULT_TIME_CONSTANT;

  float a1 = (vx / DEFAULT_WHEEL_RADIUS);
  float a2 = (DEFAULT_WHEEL_BASE_WIDTH / 2) * (w / DEFAULT_WHEEL_RADIUS);

  float phi_left = std::clamp(a1 - a2, (float) -1.0 * max_motor_vel, (float) max_motor_vel);
  float phi_right = std::clamp(a1 + a2, (float) -1.0 * max_motor_vel, (float) max_motor_vel);
  phi_right = -phi_right;

  auto inv_cos = (std::sqrt((1/std::pow((std::cos(theta)),2) - 1)));
  float des_y = 0;
  if (theta < 0) {
    des_y = cur_y_ - (des_x - cur_x_)*inv_cos;
  } else {
    des_y = cur_y_ + (des_x - cur_x_)*inv_cos;
  }

  RCLCPP_INFO(this->get_logger(), "controller: Vx: %f, w: %f phi_l: %f, phi_r: %f\n", vx, w, phi_left, phi_right);

  // state update
  cur_x_ = des_x;
  cur_y_ = des_y;
  cur_theta_ = theta;

  return std::make_pair(phi_left, phi_right);
}

void SequenceController::declare_parameters()
{
  declare_parameter<bool>("dynamic", false);
  declare_parameter<float>("left_motor", left_motor_param_);
  declare_parameter<float>("right_motor", right_motor_param_);
  declare_parameter<bool>("debug", sc_debug_);
  declare_parameter<int>("min_threshold", min_threshold_);
  declare_parameter<int>("max_threshold", max_threshold_);

  bool dynamic = this->get_parameter("dynamic").as_bool();
  left_motor_param_ = this->get_parameter("left_motor").as_double();
  right_motor_param_ = this->get_parameter("right_motor").as_double();

  // threshold_ = this->get_parameter("threshold").as_int();
  
  if (dynamic) {
    sequence_controller_mode_ = DYNAMIC_IMAGE;
  } else {
    sequence_controller_mode_ = STATIC;
  }

  param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
  auto left_motor_cb = [this](const rclcpp::Parameter & p) {
    left_motor_param_ = p.as_double();
    RCLCPP_INFO(get_logger(), "left_motor_param_ changed %f", left_motor_param_);
  };
  auto right_motor_cb = [this](const rclcpp::Parameter & p) {
    right_motor_param_ = p.as_double();
    RCLCPP_INFO(get_logger(), "right_motor_param_ changed %f", right_motor_param_);
  };
  auto dynamic_cb = [this](const rclcpp::Parameter & p) {
    if (p.as_bool()) {
      sequence_controller_mode_ = DYNAMIC_IMAGE;
    } else {
      sequence_controller_mode_ = STATIC;
    }
    RCLCPP_INFO(get_logger(), "Dynamic changed %d", sequence_controller_mode_);
  };
  auto debug_cb = [this](const rclcpp::Parameter & p) {
    sc_debug_ = p.as_bool();
    RCLCPP_INFO(get_logger(), "Debug changed %d", sc_debug_);
  };
  auto min_threshold_cb = [this](const rclcpp::Parameter & p) {
    min_threshold_ = p.as_int();
    RCLCPP_INFO(get_logger(), "min threshold changed %d", min_threshold_);
  };
  auto max_threshold_cb = [this](const rclcpp::Parameter & p) {
    max_threshold_ = p.as_int();
    RCLCPP_INFO(get_logger(), "max threshold changed %d", max_threshold_);
  };
  
  left_motor_cb_handle_ = param_subscriber_->add_parameter_callback("left_motor", left_motor_cb);
  right_motor_cb_handle_ = param_subscriber_->add_parameter_callback("right_motor", right_motor_cb);
  dynamic_cb_handle_ = param_subscriber_->add_parameter_callback("dynamic", dynamic_cb);
  debug_cb_handle_ = param_subscriber_->add_parameter_callback("debug", debug_cb);
  min_threshold_cb_handle_ = param_subscriber_->add_parameter_callback("min_threshold", min_threshold_cb);
  max_threshold_cb_handle_ = param_subscriber_->add_parameter_callback("max_threshold", max_threshold_cb);

}

void SequenceController::create_subscribers()
{
  light_position_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
        LIGHT_POSITION_TOPIC, 10,
        std::bind(&SequenceController::light_position_callback, this, std::placeholders::_1));

  webcam_input_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    "image", 10, std::bind(&SequenceController::webcam_callback, this, std::placeholders::_1)); 

  xeno2ros_subscription_ = create_subscription<custom_msgs::msg::Xeno2Ros>(XENO2ROS_TOPIC, 10, std::bind(&SequenceController::xeno2ros_callback, this, _1));
}

void SequenceController::webcam_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img) {
  is_camera_dim_rec_ = true;
  image_height = msg_cam_img->height;
  image_width = msg_cam_img->width;
}

void SequenceController::light_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr point) 
{
  is_light_pos_rec_ = true;
  light_x = point->point.x;
  light_y = point->point.y;
  num_green_pixels = point->point.z;
}

void SequenceController::create_publishers()
{
  ros2xeno_publisher_ = create_publisher<custom_msgs::msg::Ros2Xeno>(ROS2XENO_TOPIC, 10);
  debug_ros2xeno_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(DEBUG_ROS2XENO_TOPIC, 10);
  debug_xeno2ros_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(DEBUG_XENO2ROS_TOPIC, 10);
  RCLCPP_INFO(get_logger(), "Publishing to topic %s", ROS2XENO_TOPIC.c_str());
}

void SequenceController::xeno2ros_callback(const custom_msgs::msg::Xeno2Ros::SharedPtr xeno2ros_msg) {
  RCLCPP_INFO(get_logger(), "Xeno 2 ros (x:%f, y:%f)", xeno2ros_msg->x, xeno2ros_msg->y);
  if (sc_debug_) {
    auto message = geometry_msgs::msg::PointStamped();
    message.header.stamp = get_clock()->now();
    message.point.set__x(xeno2ros_msg->x);
    message.point.set__y(xeno2ros_msg->y);
    debug_xeno2ros_publisher_->publish(message);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceController>());
  rclcpp::shutdown();
  return 0;
}
