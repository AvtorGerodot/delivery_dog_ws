#!/usr/bin/env python3
import sys
import xml.etree.ElementTree as ET
import tempfile
from pathlib import Path
import mujoco

def fix_inertia(tree):
    """Заменяет слишком маленькие массы и инерции на минимальные разумные значения."""
    root = tree.getroot()
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            # Добавляем фиктивный инерциал для линков без него
            inertial = ET.SubElement(link, 'inertial')
            mass_elem = ET.SubElement(inertial, 'mass')
            mass_elem.set('value', '0.001')
            inertia_elem = ET.SubElement(inertial, 'inertia')
            inertia_elem.set('ixx', '1e-8')
            inertia_elem.set('ixy', '0')
            inertia_elem.set('ixz', '0')
            inertia_elem.set('iyy', '1e-8')
            inertia_elem.set('iyz', '0')
            inertia_elem.set('izz', '1e-8')
            continue

        mass_elem = inertial.find('mass')
        if mass_elem is None:
            continue
        mass = float(mass_elem.get('value', '0'))
        
        # Слишком малая масса → ставим 0.001 кг (1 г)
        if mass < 1e-5:
            mass_elem.set('value', '0.001')
            mass = 0.001

        inertia = inertial.find('inertia')
        if inertia is None:
            inertia = ET.SubElement(inertial, 'inertia')
        
        # Гарантируем наличие всех атрибутов
        needed = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']
        for attr in needed:
            if attr not in inertia.attrib:
                inertia.set(attr, '0')
        
        # Если инерции пренебрежимо малы — пересчитываем по формуле для куба 0.1м
        ixx = float(inertia.get('ixx', '0'))
        iyy = float(inertia.get('iyy', '0'))
        izz = float(inertia.get('izz', '0'))
        if ixx < 1e-10 and iyy < 1e-10 and izz < 1e-10:
            val = mass * 0.0016667  # I = m * (0.1^2)/6
            inertia.set('ixx', str(val))
            inertia.set('iyy', str(val))
            inertia.set('izz', str(val))
            inertia.set('ixy', '0')
            inertia.set('ixz', '0')
            inertia.set('iyz', '0')

def clean_urdf_for_mujoco(urdf_path, output_clean_path):
    """
    Удаляет из URDF всё, что мешает MuJoCo:
      - теги <gazebo>, <ros2_control>, <transmission>
      - все блоки <collision> (чтобы MuJoCo использовал визуальные меши)
      - PBR-подтеги внутри <material>
    Исправляет инерции.
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Удаляем Gazebo, ROS2 control, transmission
    for gazebo in root.findall('gazebo'):
        root.remove(gazebo)
    for ros2_control in root.findall('ros2_control'):
        root.remove(ros2_control)
    for transmission in root.findall('transmission'):
        root.remove(transmission)

    # Удаляем все collision-геометрии (оставляем только visual)
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            link.remove(collision)

    # Чистим материалы от PBR
    for material in root.findall(".//material"):
        for pbr in material.findall('pbr'):
            material.remove(pbr)
        if material.find('color') is None:
            color = ET.SubElement(material, 'color')
            color.set('rgba', '0.7 0.7 0.7 1')

    # Исправляем инерции
    fix_inertia(tree)

    tree.write(output_clean_path, encoding='utf-8', xml_declaration=True)

def convert_urdf_to_mjcf(urdf_path, mjcf_path):
    """Загружает очищенный URDF и сохраняет как MJCF."""
    model = mujoco.MjModel.from_xml_path(urdf_path)
    mujoco.mj_saveLastXML(mjcf_path, model)
    print(f"✅ Конвертация завершена: {mjcf_path}")

def convert_urdf_to_mjcf(urdf_path, mjcf_path):
    model = mujoco.MjModel.from_xml_path(urdf_path)
    print(f"📊 Статистика модели: {model.ngeom} геометрий, {model.nmesh} мешей, {model.nbody} тел")
    mujoco.mj_saveLastXML(mjcf_path, model)
    print(f"✅ Конвертация завершена: {mjcf_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Использование: convert_to_mjcf.py <input.urdf> <output.mjcf.xml>")
        sys.exit(1)

    input_urdf = sys.argv[1]
    output_mjcf = sys.argv[2]

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
        clean_urdf = tmp.name

    clean_urdf_for_mujoco(input_urdf, clean_urdf)
    convert_urdf_to_mjcf(clean_urdf, output_mjcf)
    Path(clean_urdf).unlink()