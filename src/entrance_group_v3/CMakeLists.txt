cmake_minimum_required(VERSION 3.8)
project(entrance_group_v3)

find_package(ament_cmake REQUIRED)

# Устанавливаем все имеющиеся ресурсы пакета в share/<package>.
# ВАЖНО: перечисляем только реально существующие каталоги — иначе
# ament_cmake_symlink_install падает с ошибкой "can't find ...".
install(
  DIRECTORY urdf launch worlds materials hooks
  DESTINATION share/${PROJECT_NAME}
)

# Хук для прокидывания путей в GZ_SIM_RESOURCE_PATH при source install/setup.bash.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_resource_path.sh.in")

ament_package()
