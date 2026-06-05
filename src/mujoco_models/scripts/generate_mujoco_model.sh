#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Проверка xacro
if ! command -v xacro &> /dev/null; then
    echo "❌ xacro не найден. Установите: sudo apt install ros-jazzy-xacro"
    exit 1
fi

XACRO_FILE="${PKG_DIR}/urdf/entrance_group.urdf.xacro"
TEMP_URDF="${PKG_DIR}/urdf/entrance_group_temp.urdf"
MJCF_OUTPUT="${PKG_DIR}/mjcf/entrance_group.xml"

echo "🔧 Генерация URDF из xacro..."
xacro "${XACRO_FILE}" > "${TEMP_URDF}"

# Замена model:// на относительный путь к текстурам (опционально)
sed -i 's|model://entrance_group_v3/materials/textures/|../textures/|g' "${TEMP_URDF}"

echo "🧹 Конвертация в MJCF..."
python3 "${PKG_DIR}/scripts/convert_to_mjcf.py" "${TEMP_URDF}" "${MJCF_OUTPUT}"

rm -f "${TEMP_URDF}"
echo "✅ Модель MuJoCo сохранена: ${MJCF_OUTPUT}"