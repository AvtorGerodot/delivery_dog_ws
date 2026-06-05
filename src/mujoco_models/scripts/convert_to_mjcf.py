#!/usr/bin/env python3
import sys
import xml.etree.ElementTree as ET
import tempfile
from pathlib import Path
import mujoco  # импорт всего модуля

def remove_gazebo_and_clean(urdf_path, output_clean_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for gazebo in root.findall('gazebo'):
        root.remove(gazebo)
    for material in root.findall(".//material"):
        for pbr in material.findall('pbr'):
            material.remove(pbr)
        if material.find('color') is None:
            color = ET.SubElement(material, 'color')
            color.set('rgba', '0.7 0.7 0.7 1')
    tree.write(output_clean_path, encoding='utf-8', xml_declaration=True)

def convert_urdf_to_mjcf(urdf_path, mjcf_path):
    model = mujoco.MjModel.from_xml_path(urdf_path)
    mujoco.mj_saveLastXML(mjcf_path, model)   # сохраняем MJCF
    print(f"✅ Конвертация завершена: {mjcf_path}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Использование: convert_to_mjcf.py <input.urdf> <output.mjcf.xml>")
        sys.exit(1)
    input_urdf = sys.argv[1]
    output_mjcf = sys.argv[2]
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as tmp:
        clean_urdf = tmp.name
    remove_gazebo_and_clean(input_urdf, clean_urdf)
    convert_urdf_to_mjcf(clean_urdf, output_mjcf)
    Path(clean_urdf).unlink()