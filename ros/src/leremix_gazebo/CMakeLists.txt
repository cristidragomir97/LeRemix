cmake_minimum_required(VERSION 3.8)
project(leremix_gazebo)
find_package(ament_cmake REQUIRED)

# Install sim overlay and launches
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
