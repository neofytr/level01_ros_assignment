import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from geometry import in_base_link, joint_origin, part_geometry

REPO = Path(__file__).resolve().parents[3]
DESCRIPTION = REPO / 'testbed_description'
CONFIG = REPO / 'testbed_navigation' / 'config'
GAZEBO = ET.parse(DESCRIPTION / 'urdf' / 'testbed.gazebo').getroot()
DRIVE = GAZEBO.find(".//plugin[@filename='libgazebo_ros_diff_drive.so']")
LIDAR = GAZEBO.find(".//sensor[@type='ray']")


def test_wheel_separation_matches_the_wheels():
    wheels = in_base_link(DESCRIPTION)
    measured = wheels['left_wheel_1'][1] - wheels['right_wheel_1'][1]
    assert abs(measured - float(DRIVE.findtext('wheel_separation'))) < 0.001, measured


def test_base_footprint_sits_on_the_drive_axis():
    wheels = in_base_link(DESCRIPTION)
    offset = joint_origin(DESCRIPTION, 'fixed_base_joint')
    for axis in (0, 1):
        midpoint = (wheels['left_wheel_1'][axis] + wheels['right_wheel_1'][axis]) / 2
        assert abs(midpoint + offset[axis]) < 0.005, (axis, midpoint, offset)


def test_sensor_frames_sit_on_their_meshes():
    parts = part_geometry(DESCRIPTION)
    for link in ('imu_link_1', 'dummy_link_1', 'lidar_link_1'):
        centre, half = parts[link]
        assert all(abs(c) <= h + 0.005 for c, h in zip(centre, half)), (link, centre)


def test_no_ros1_leftovers():
    for path in (DESCRIPTION / 'urdf').iterdir():
        text = path.read_text()
        assert 'libgazebo_ros_control.so' not in text and '<transmission' not in text, path.name


def test_lidar_sees_across_the_arena():
    assert float(LIDAR.findtext('ray/range/max')) >= 10.0


def test_navigation_config_matches_the_robot():
    nav = yaml.safe_load((CONFIG / 'nav2_params.yaml').read_text())
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    frame = DRIVE.findtext('robot_base_frame')
    assert frame == 'base_footprint'
    assert frame == local['robot_base_frame'] == nav['bt_navigator']['ros__parameters']['robot_base_frame']
    assert local['global_frame'] == DRIVE.findtext('odometry_frame')
    assert LIDAR.findtext('plugin/ros/remapping') == '~/out:=scan'
    assert local['obstacle_layer']['scan']['topic'] == '/scan'
