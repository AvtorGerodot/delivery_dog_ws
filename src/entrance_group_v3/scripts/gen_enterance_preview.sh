#!/bin/bash

# Скрипт генерации preview-URDF модели входной группы (entrance_group_v3)
# из xacro-файлов. Результат сохраняется в urdf/entrance_group_preview.urdf
# с заменой model://-ссылок на относительные пути к текстурам.

# Определяем корень воркспейса — три уровня вверх от scripts/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PKG_DIR="${SCRIPT_DIR}/.."

# Проверяем наличие install/setup.bash
if [ ! -f "${WS_ROOT}/install/setup.bash" ]; then
    echo "[ERROR] install/setup.bash not found in ${WS_ROOT}"
    echo "Run 'colcon build' first."
    exit 1
fi

# Проверяем наличие xacro
if ! command -v xacro &> /dev/null; then
    echo "[ERROR] xacro not found. Please install it: sudo apt install ros-${ROS_DISTRO}-xacro"
    exit 1
fi

# Sourcing воркспейса
source "${WS_ROOT}/install/setup.bash"

# Получаем префикс установленного пакета (на случай использования $(find ...) в xacro)
PKG_PREFIX="$(ros2 pkg prefix entrance_group_v3)" || {
    echo "[ERROR] Package 'entrance_group_v3' not found. Did you build the workspace?"
    exit 1
}
PKG_SHARE="${PKG_PREFIX}/share/entrance_group_v3"
XACRO_FILE="${PKG_DIR}/urdf/entrance_group.urdf.xacro"
OUTPUT_FILE="${PKG_DIR}/urdf/entrance_group_preview.urdf"

echo "[INFO] Generating preview URDF..."
echo "[INFO] Workspace: ${WS_ROOT}"
echo "[INFO] Package share: ${PKG_SHARE}"

# Генерация URDF через xacro, затем замена путей:
# 1. Удаляем абсолютный путь к share (если он появился из-за $(find ...))
# 2. Заменяем model://entrance_group_v3/materials/textures/ на ../materials/textures/
xacro "${XACRO_FILE}" | \
    sed "s|${PKG_SHARE}/||g" | \
    sed 's|model://entrance_group_v3/materials/textures/|../materials/textures/|g' \
    > "${OUTPUT_FILE}"

if [ $? -eq 0 ]; then
    echo "[OK] Preview saved: ${OUTPUT_FILE}"
else
    echo "[ERROR] xacro failed."
    exit 1
fi