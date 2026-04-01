cmake_minimum_required(VERSION 3.8)
project(user_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Quick Qml)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(myrobot_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

set(HEADERS
  include/user_gui/ViewModel.h
  include/user_gui/RosWorker.h
  include/user_gui/MapPaintItem.h
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

install(
    DIRECTORY qml map
    DESTINATION share/${PROJECT_NAME}/
)

add_executable(main 
  src/main.cpp
  src/ViewModel.cpp
  src/ManagerModel.cpp
  src/RosWorker.cpp
  src/MapPaintItem.cpp
  ${HEADERS}
)
ament_target_dependencies(main
  rclcpp
  ament_index_cpp
  nav2_msgs
  rclcpp_action
  myrobot_interfaces
)

target_link_libraries(main Qt5::Quick Qt5::Qml yaml-cpp)

install(TARGETS
  main
  DESTINATION lib/${PROJECT_NAME})

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
