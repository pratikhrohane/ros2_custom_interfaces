cmake_minimum_required(VERSION 3.8)
project(training)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(training_interfaces REQUIRED)

# =======================
# Add Executables
# =======================

# Publisher node using Person.msg
add_executable(person_pub_node src/person_pub_node.cpp)
ament_target_dependencies(person_pub_node rclcpp training_interfaces)

# Subscriber node using Person.msg
add_executable(person_sub_node src/person_sub_node.cpp)
ament_target_dependencies(person_sub_node rclcpp training_interfaces)

# Service Server node using Value.srv
add_executable(add_server src/add_server.cpp)
ament_target_dependencies(add_server rclcpp training_interfaces)

# Service Client node using Value.srv
add_executable(add_client src/add_client.cpp)
ament_target_dependencies(add_client rclcpp training_interfaces)

# =======================
# Install Targets
# =======================
install(TARGETS
  person_pub_node
  person_sub_node
  add_server
  add_client
  DESTINATION lib/${PROJECT_NAME}
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
