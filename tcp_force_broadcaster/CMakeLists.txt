cmake_minimum_required(VERSION 3.5)
project(tcp_force_broadcaster)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(${PROJECT_NAME} SHARED src/tcp_force_broadcaster.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  pluginlib
  geometry_msgs
  hardware_interface
)

pluginlib_export_plugin_description_file(controller_interface tcp_force_broadcaster.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES tcp_force_broadcaster.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  controller_interface pluginlib rclcpp geometry_msgs hardware_interface
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
