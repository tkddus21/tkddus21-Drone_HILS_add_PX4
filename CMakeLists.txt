cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # <-- sensor_msgs 추가

# 기존 오프보드 컨트롤 노드
add_executable(offboard_control_node src/offboard_control_node.cpp)
ament_target_dependencies(offboard_control_node rclcpp px4_msgs)

# 오프보드 모드 전환 노드
add_executable(offboard_mode_node src/offboard_mode_node.cpp)
ament_target_dependencies(offboard_mode_node rclcpp px4_msgs)

# ARM 및 초기 목표 이동 노드
add_executable(arm_and_move_node src/arm_and_move_node.cpp)
ament_target_dependencies(arm_and_move_node rclcpp px4_msgs)

# 목표 위치 업데이트 노드 추가
add_executable(target_update_node src/target_update_node.cpp)
ament_target_dependencies(target_update_node rclcpp px4_msgs)

# 벽면과 수평 계산 노드 추가
add_executable(yaw_control_node src/yaw_control_node.cpp)
ament_target_dependencies(yaw_control_node rclcpp px4_msgs sensor_msgs)

install(TARGETS
  offboard_control_node
  offboard_mode_node
  target_update_node
  arm_and_move_node
  yaw_control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()