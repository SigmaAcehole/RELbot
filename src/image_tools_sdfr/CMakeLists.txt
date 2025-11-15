cmake_minimum_required(VERSION 3.5)

project(image_tools_sdfr)

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

add_library(${PROJECT_NAME} SHARED
  src/cam2image.cpp
  src/cv_mat_sensor_msgs_image_type_adapter.cpp
)
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "IMAGE_TOOLS_BUILDING_DLL")
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp_components::component
  ${OpenCV_LIBS})
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "image_tools_sdfr::Cam2Image" EXECUTABLE cam2image)
rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "image_tools_sdfr::ShowImage" EXECUTABLE showimage)

install(
  TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# TODO(sloretz) stop exporting old-style CMake variables in the future
ament_export_libraries(${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(rclcpp_components)

ament_package()
