#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${PKG_DIR}/../../.." && pwd)"

# Source ROS2 и workspace
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    source "${WS_ROOT}/install/setup.bash"
fi

if ! command -v xacro &> /dev/null; then
    echo "❌ xacro не найден. Установите: sudo apt install ros-jazzy-xacro"
    exit 1
fi

XACRO_FILE="${PKG_DIR}/../delivery_dog_bringup/urdf/b2_z1.urdf.xacro"
TEMP_URDF="${PKG_DIR}/urdf/b2_z1_temp.urdf"
MJCF_OUTPUT="${PKG_DIR}/mjcf/b2_z1/b2_z1.xml"

echo "🔧 Генерация URDF из xacro..."
rm -f "${TEMP_URDF}"
xacro "${XACRO_FILE}" mount_position:="0.1945 0 0.276" > "${TEMP_URDF}"
if [ $? -ne 0 ] || [ ! -s "${TEMP_URDF}" ]; then
    echo "❌ Ошибка при выполнении xacro"
    exit 1
fi

echo "🔄 Исправление путей к мешам в URDF (абсолютные пути)..."
# Заменяем относительные пути на абсолютные
sed -i "s|../../b2_description/meshes/|${PKG_DIR}/meshes/b2_z1/|g" "${TEMP_URDF}"
sed -i "s|../../z1_model/meshes/visual/|${PKG_DIR}/meshes/b2_z1/|g" "${TEMP_URDF}"
sed -i "s|../../z1_model/meshes/collision/|${PKG_DIR}/meshes/b2_z1/|g" "${TEMP_URDF}"

# В xacro мог остаться нераскрытый макрос $(find delivery_dog_bringup)
# Просто заменим его на тот же путь
sed -i "s|\$(find delivery_dog_bringup)/meshes/|${PKG_DIR}/meshes/b2_z1/|g" "${TEMP_URDF}"

# Отладка: проверим, есть ли визуальные геометрии
echo "🔍 Проверка: количество visual элементов в URDF:"
grep -c "<visual" "${TEMP_URDF}" || echo "0"

echo "🧹 Очистка и конвертация в MJCF..."
python3 "${PKG_DIR}/scripts/convert_to_mjcf.py" "${TEMP_URDF}" "${MJCF_OUTPUT}"
if [ $? -ne 0 ]; then
    echo "❌ Ошибка конвертации"
    exit 1
fi

rm -f "${TEMP_URDF}"
echo "✅ Модель MuJoCo сохранена: ${MJCF_OUTPUT}"