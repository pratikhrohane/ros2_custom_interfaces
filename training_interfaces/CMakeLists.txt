cmake_minimum_required(VERSION 3.8)
project(training_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message and Service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Person.msg"
  "srv/Value.srv"
  DEPENDENCIES std_msgs
)

# Export headers to downstream packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()
