import importlib.util
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from launch import LaunchDescription

REPO = Path(__file__).resolve().parents[3]
INSTALLED = {
    'testbed_description': ['meshes', 'urdf', 'launch', 'rviz'],
    'testbed_gazebo': ['launch', 'worlds', 'models'],
    'testbed_bringup': ['launch', 'maps'],
    'testbed_navigation': ['launch', 'config', 'rviz'],
}
LAUNCH_FILES = sorted(REPO.glob('*/launch/*.launch.py'))


def test_map_yaml_points_at_a_real_image():
    maps = REPO / 'testbed_bringup' / 'maps'
    image = re.search(r'^image:\s*(\S+)', (maps / 'testbed_world.yaml').read_text(), re.M)[1]
    assert (maps / image).is_file(), image


@pytest.mark.parametrize('package', INSTALLED)
def test_cmake_builds_and_installs_its_assets(package):
    text = (REPO / package / 'CMakeLists.txt').read_text()
    assert 'ament_package()' in text
    installed = ' '.join(re.findall(r'install\s*\((.*?)\)', text, re.S))
    for directory in INSTALLED[package]:
        assert (REPO / package / directory).is_dir(), directory
        assert re.search(rf'\b{directory}\b', installed), f'{directory} is not installed'


@pytest.mark.parametrize('package', INSTALLED)
def test_launched_packages_are_declared(package):
    manifest = ET.parse(REPO / package / 'package.xml').getroot()
    declared = {e.text for e in manifest if e.tag in ('depend', 'exec_depend')} | {package}
    used = set()
    for path in (REPO / package / 'launch').glob('*.launch.py'):
        text = path.read_text()
        used |= set(re.findall(r"package\s*=\s*['\"](\w+)", text))
        used |= set(re.findall(
            r"(?:get_package_share_directory|FindPackageShare)\(\s*['\"](\w+)", text))
    assert used <= declared, used - declared


def test_the_three_stages_exist():
    names = {p.name for p in LAUNCH_FILES}
    assert {'map_loader.launch.py', 'localization.launch.py', 'navigation.launch.py'} <= names


@pytest.mark.parametrize('path', LAUNCH_FILES, ids=lambda p: f'{p.parts[-3]}/{p.name}')
def test_launch_file_is_valid(path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert isinstance(module.generate_launch_description(), LaunchDescription)
    text = path.read_text()
    if 'robot_state_publisher' in text or 'nav2_' in text or 'rviz2' in text:
        assert 'use_sim_time' in text
