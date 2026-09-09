cmake_minimum_required(VERSION 3.15)
project(game_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2neuro_msgs REQUIRED)

# Python nodes: no setup.py in a hybrid package, so install them explicitly.
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  game_controller/two_class_threshold_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME two_class_threshold_controller
)

install(PROGRAMS
  game_controller/no_dead_zone_threshold_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME no_dead_zone_threshold_controller
)

install(PROGRAMS
  game_controller/dummy_keyboard_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dummy_keyboard_controller
)

# C++ training orchestrator (calibration/evaluation).
include_directories(include)

add_executable(training_controller
  src/training_controller.cpp
  src/trial_sequence.cpp
  src/autopilot.cpp
)
ament_target_dependencies(training_controller rclcpp std_msgs ros2neuro_msgs)

install(TARGETS training_controller DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
