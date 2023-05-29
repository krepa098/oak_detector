cmake_minimum_required(VERSION 3.11)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/thirdparty/depthai-core EXCLUDE_FROM_ALL)

project(oak_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

# setup targets
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/pipeline.cpp
  src/camera_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  depthai_bridge
  depthai_ros_msgs
  depthai
)

target_include_directories(
  ${PROJECT_NAME}_node
  PRIVATE
  include
  ${ament_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

# add prebuild neural network
file(DOWNLOAD
  "https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/mobilenet-ssd_openvino_2021.4_6shave.blob"
  ${CMAKE_BINARY_DIR}/oak-nn.blob
  SHOW_PROGRESS
)
target_compile_definitions(${PROJECT_NAME}_node PRIVATE BLOB_PATH="${CMAKE_BINARY_DIR}/oak-nn.blob" ON)

# install
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# tests
if(AMENT_ENABLE_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(foo_gtest test/my_test.cpp)
  target_link_libraries(foo_gtest ${rclcpp_LIBRARIES} ${rmw_connext_cpp_LIBRARIES} ${std_interfaces})
endif()

# export information to downstream packages
ament_package()
