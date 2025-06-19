cmake_minimum_required(VERSION 3.22)
project(fs_backend)

# ROS 2 Buildsystem
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# === OpenCV & OpenMP ===
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

# Compiler-Optionen
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Executable (Node)
add_executable(cpu_node
  src/CPU.cc
  src/System.cc
  src/SystemNode.cc
)

target_link_libraries(cpu_node
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

# Abhängigkeiten
ament_target_dependencies(cpu_node
  rclcpp
  std_msgs
)

# Installation für ros2 run
install(TARGETS cpu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Header installieren (optional aber üblich)
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


ament_package()

