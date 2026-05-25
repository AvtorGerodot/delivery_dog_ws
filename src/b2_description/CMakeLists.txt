cmake_minimum_required(VERSION 3.8)
project(b2_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Установка папок
install(DIRECTORY config meshes urdf xacro
  DESTINATION share/${PROJECT_NAME}
)

# Установка launch-файлов (Python)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()