cmake_minimum_required(VERSION 3.8)
project(avatar_challenge)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Install python scripts
install(PROGRAMS
  scripts/circle_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
