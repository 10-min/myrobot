cmake_minimum_required(VERSION 3.8)
project(myrobot_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(myrobot_interfaces REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pluginlib REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

include_directories(include)

set(library_name ${PROJECT_NAME})

set(dependencies
  behaviortree_cpp
  myrobot_interfaces
  nav2_behavior_tree
  pluginlib
)

add_library(compute_place_pose_action_bt_node SHARED src/compute_place_pose_action.cpp)

ament_target_dependencies(compute_place_pose_action_bt_node
  ${dependencies}
)

target_compile_definitions(compute_place_pose_action_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS compute_place_pose_action_bt_node
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

ament_export_include_directories(include)

ament_export_libraries(
  compute_place_pose_action_bt_node
)

ament_export_dependencies(${dependencies})

ament_package()
