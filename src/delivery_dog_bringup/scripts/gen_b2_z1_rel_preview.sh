#!/bin/bash

# Определяем корень воркспейса (три уровня вверх от scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PKG_DIR="${SCRIPT_DIR}/.."

# Проверяем наличие install/setup.bash
if [ ! -f "${WS_ROOT}/install/setup.bash" ]; then
    echo "[ERROR] install/setup.bash not found in ${WS_ROOT}"
    echo "Run 'colcon build' first."
    exit 1
fi

source "${WS_ROOT}/install/setup.bash"

# Путь к xacro и выходной файл
XACRO_FILE="${PKG_DIR}/urdf/b2_z1.urdf.xacro"
OUTPUT_FILE="${PKG_DIR}/urdf/b2_z1_rel_preview.urdf"

echo "[INFO] Generating preview URDF with relative paths..."
echo "[INFO] Workspace: ${WS_ROOT}"

# Получаем установленные директории для зависимых пакетов
PKG_SHARE_B2="$(ros2 pkg prefix b2_description)/share/b2_description"
PKG_SHARE_Z1="$(ros2 pkg prefix z1_model)/share/z1_model"

# Запускаем xacro, затем заменяем абсолютные пути на относительные
xacro "${XACRO_FILE}" | \
    sed -e "s|${PKG_SHARE_B2}/meshes|../../b2_description/meshes|g" \
    -e "s|${PKG_SHARE_Z1}/meshes|../../z1_model/meshes|g" \
    > "${OUTPUT_FILE}"

if [ $? -eq 0 ]; then
    echo "[OK] Preview saved: ${OUTPUT_FILE}"
    echo "  - Relative paths assume preview file is placed in 'delivery_dog_bringup/urdf/'"
    echo "  - Meshes are expected in '../b2_description/meshes' and '../z1_model/meshes'"
    echo "  - To use in RViz2, run from workspace root or copy preview file accordingly."
else
    echo "[ERROR] xacro failed."
    exit 1
fi