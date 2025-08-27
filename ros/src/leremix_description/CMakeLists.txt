cmake_minimum_required(VERSION 3.8)
project(leremix_description)
find_package(ament_cmake REQUIRED)

# Install model assets
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(DIRECTORY materials DESTINATION share/${PROJECT_NAME} OPTIONAL)

ament_package()
