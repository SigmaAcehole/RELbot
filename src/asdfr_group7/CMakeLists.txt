cmake_minimum_required(VERSION 3.5)

project(asdfr_group7)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgcodecs imgproc videoio)
find_package(image_tools_sdfr REQUIRED)
find_package(custom_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/light_position.cpp
)
add_executable(sequence_controller
src/sequence_controller.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp_components::component
  ${OpenCV_LIBS}
  ${image_tools_sdfr_LIBS}
  ${image_tools_sdfr_TARGETS})
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "asdfr_group7::LightPosition" EXECUTABLE lightposition)

target_include_directories(sequence_controller PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)
target_compile_features(sequence_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(${PROJECT_NAME}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

install(TARGETS sequence_controller
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS sequence_controller
  DESTINATION lib/${PROJECT_NAME})

# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_libraries(${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(rclcpp_components)

ament_target_dependencies(sequence_controller
rclcpp
std_msgs
sensor_msgs
geometry_msgs
custom_msgs)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
