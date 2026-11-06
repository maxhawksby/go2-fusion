cmake_minimum_required(VERSION 3.8)
project(go2_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Panels use Qt signals/slots (Q_OBJECT), so their headers require moc.
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(dependencies
  rclcpp
  rclcpp_action
  rviz_common
  pluginlib
  nav_msgs
  geometry_msgs
  nav2_msgs
  tf2
  tf2_ros
  std_srvs
  ament_index_cpp
)

# The headers are listed so AUTOMOC finds their Q_OBJECT macros: it only scans
# headers that are sources or sit next to a .cpp, and these live in include/.
# Without moc output the panels' vtables are missing and RViz can't load them.
add_library(${PROJECT_NAME} SHARED
  src/map_viewer_panel.cpp
  src/auto_mapper_panel.cpp
  src/keyboard_teleop_panel.cpp
  include/${PROJECT_NAME}/map_viewer_panel.hpp
  include/${PROJECT_NAME}/auto_mapper_panel.hpp
  include/${PROJECT_NAME}/keyboard_teleop_panel.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# RViz loads panels through pluginlib's class_loader, which dlopen()s this
# library directly rather than linking against it, so symbols must be
# exported by default (matches rviz_common's own plugin examples).
target_compile_definitions(${PROJECT_NAME} PRIVATE "GO2_RVIZ_PLUGINS_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
