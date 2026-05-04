#!/bin/bash

# Запуск из любой директории:
# ~/delivery_dog_ws/src/z1_model/scripts/z1_gen_preview.sh
# Или если вы уже в корне воркспейса:
# src/z1_model/scripts/z1_gen_preview.sh

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

# Sourcing воркспейса
source "${WS_ROOT}/install/setup.bash"

# Получаем префикс установленного пакета
PKG_SHARE="$(ros2 pkg prefix z1_model)/share/z1_model"
XACRO_FILE="${PKG_DIR}/urdf/z1_model.urdf.xacro"
OUTPUT_FILE="${PKG_DIR}/urdf/z1_preview.urdf"

echo "[INFO] Generating preview URDF..."
echo "[INFO] Workspace: ${WS_ROOT}"
echo "[INFO] Package share: ${PKG_SHARE}"

xacro "${XACRO_FILE}" | \
        sed "s|${PKG_SHARE}/||g" | \
        sed 's|filename="meshes/|filename="../meshes/|g' \
        > "${OUTPUT_FILE}"

if [ $? -eq 0 ]; then
    echo "[OK] Preview saved: ${OUTPUT_FILE}"
else
    echo "[ERROR] xacro failed."
    exit 1
fi