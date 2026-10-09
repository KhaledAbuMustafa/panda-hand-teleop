cmake_minimum_required(VERSION 3.8)
project(lab_teleop)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/hand_servo.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
