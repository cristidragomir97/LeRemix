cmake_minimum_required(VERSION 3.8)
project(leremix_control)
find_package(ament_cmake REQUIRED)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(PROGRAMS
  test_motion.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
