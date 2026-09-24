import math
import os
import signal
import subprocess
import time

import pytest
import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.msg import Costmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

PROBE = ("<sdf version='1.6'><model name='probe'><static>true</static><link name='link'>"
         "<collision name='c'><geometry><box><size>0.4 0.4 0.5</size></box></geometry></collision>"
         "<visual name='v'><geometry><box><size>0.4 0.4 0.5</size></box></geometry></visual>"
         "</link></model></sdf>")

MANAGED = ['map_server', 'amcl', 'controller_server', 'planner_server',
           'behavior_server', 'bt_navigator', 'velocity_smoother']


def active(node):
    try:
        out = subprocess.run(['ros2', 'lifecycle', 'get', f'/{node}'],
                             capture_output=True, text=True, timeout=15).stdout
    except subprocess.TimeoutExpired:
        return False
    return out.startswith('active')


def truth():
    out = subprocess.run(['gz', 'model', '-m', 'testbed', '-p'],
                         capture_output=True, text=True, timeout=15).stdout.split()
    return (float(out[0]), float(out[1])) if len(out) == 6 else None


def run(node, action, name, goal, timeout):
    client = ActionClient(node, action, name)
    assert client.wait_for_server(timeout_sec=30), name
    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=30)
    result = sent.result().get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=timeout)
    assert result.result().status == 4, name


def navigate(node, x, y):
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = 'map'
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    goal.pose.pose.orientation.w = 1.0
    run(node, NavigateToPose, 'navigate_to_pose', goal, 180)
    reached = truth()
    assert math.hypot(reached[0] - x, reached[1] - y) < 0.4, reached


@pytest.fixture(scope='module')
def node():
    stack = subprocess.Popen(
        ['ros2', 'launch', 'testbed_navigation', 'bringup.launch.py', 'rviz:=false', 'gui:=false'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        deadline = time.time() + 240
        while not (truth() and all(active(n) for n in MANAGED)):
            assert time.time() < deadline, 'navigation stack never came up'
            time.sleep(5)
        rclpy.init()
        node = Node('navigation_test')
        yield node
        node.destroy_node()
        rclpy.shutdown()
    finally:
        os.killpg(stack.pid, signal.SIGKILL)


def test_reaches_a_goal_in_the_open(node):
    navigate(node, 3.0, 5.0)


def test_finds_the_doorway_into_the_next_room(node):
    navigate(node, 0.0, -3.0)


def test_amcl_recovers_from_a_wrong_initial_pose(node):
    poses = []
    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', poses.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def error_from(x, y):
        rclpy.spin_once(node, timeout_sec=0.1)
        estimate = poses[-1].pose.pose.position if poses else None
        return math.hypot(estimate.x - x, estimate.y - y) if estimate else math.inf

    x, y = truth()
    guess = PoseWithCovarianceStamped()
    guess.header.frame_id = 'map'
    guess.pose.pose.position.x = x + 0.4
    guess.pose.pose.position.y = y - 0.3
    guess.pose.pose.orientation.w = 1.0
    guess.pose.covariance[0] = guess.pose.covariance[7] = 0.25
    guess.pose.covariance[35] = 0.1

    publisher = node.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
    deadline = time.time() + 15
    while error_from(x + 0.4, y - 0.3) > 0.05:
        assert time.time() < deadline, 'AMCL never took the displaced initial pose'
        publisher.publish(guess)

    spin = Spin.Goal()
    spin.target_yaw = 2 * math.pi
    run(node, Spin, 'spin', spin, 120)

    x, y = truth()
    assert error_from(x, y) < 0.15, 'AMCL did not pull a 0.5 m error back onto the robot'


def test_sees_an_obstacle_that_is_not_on_the_map(node):
    costmaps = []
    node.create_subscription(Costmap, 'local_costmap/costmap_raw', costmaps.append, 1)
    x, y = truth()
    target = (x + 1.0, y)

    def marked():
        rclpy.spin_once(node, timeout_sec=0.2)
        if not costmaps:
            return None
        meta = costmaps[-1].metadata
        return any(
            value == 254 and math.hypot(
                meta.origin.position.x + (index % meta.size_x + 0.5) * meta.resolution - target[0],
                meta.origin.position.y + (index // meta.size_x + 0.5) * meta.resolution - target[1],
            ) < 0.3
            for index, value in enumerate(costmaps[-1].data))

    deadline = time.time() + 15
    while marked() is None:
        assert time.time() < deadline, 'no local costmap published'
    assert not marked(), 'the spot is already occupied before anything is placed there'

    spawn = node.create_client(SpawnEntity, 'spawn_entity')
    assert spawn.wait_for_service(timeout_sec=30)
    request = SpawnEntity.Request(name='probe', xml=PROBE)
    request.initial_pose.position.x, request.initial_pose.position.y = target
    request.initial_pose.position.z = 0.25
    rclpy.spin_until_future_complete(node, spawn.call_async(request), timeout_sec=30)

    deadline = time.time() + 15
    while not marked():
        assert time.time() < deadline, 'the lidar never put the new box into the local costmap'
