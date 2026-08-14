cmake_minimum_required(VERSION 3.8)
project(unagi_game)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Obstacle.msg"
  "msg/GameState.msg"
  "action/PlayRound.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(game_world src/game_world.cpp)
target_include_directories(game_world PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(physics_node src/physics_node.cpp)
target_include_directories(physics_node PRIVATE include)
target_link_libraries(physics_node game_world "${cpp_typesupport_target}")
ament_target_dependencies(physics_node
  rclcpp rclcpp_action geometry_msgs std_msgs std_srvs
)

add_executable(renderer_node src/renderer_node.cpp)
target_include_directories(renderer_node PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(renderer_node "${cpp_typesupport_target}" ${OpenCV_LIBS})
ament_target_dependencies(renderer_node rclcpp sensor_msgs)

install(TARGETS
  game_world
  physics_node
  renderer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/keyboard_node.py
  scripts/viewer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_game_world test/test_game_world.cpp)
  if(TARGET test_game_world)
    target_link_libraries(test_game_world game_world)
    target_include_directories(test_game_world PRIVATE include)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
