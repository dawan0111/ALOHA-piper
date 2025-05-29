cmake_minimum_required(VERSION 3.8)
project(act_episode_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

set(RECORD_SERVER_NAME act_episode_record_server)

add_library(${RECORD_SERVER_NAME}_component SHARED
  src/${RECORD_SERVER_NAME}.cpp
)
ament_target_dependencies(${RECORD_SERVER_NAME}_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  rosbag2_cpp
)

rclcpp_components_register_node(${RECORD_SERVER_NAME}_component
  PLUGIN "ACT::EpisodeRecordServer"
  EXECUTABLE ${RECORD_SERVER_NAME}
)

ament_export_targets(export_${RECORD_SERVER_NAME}_component)

install(TARGETS ${RECORD_SERVER_NAME}_component
        EXPORT export_${RECORD_SERVER_NAME}_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
