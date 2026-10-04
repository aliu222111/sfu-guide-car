"""A cold start resets the EKF only when two RTAB-Map matches agree."""

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped


@pytest.fixture
def manager(tmp_path):
    from guide_mission.localization_manager_node import LocalizationManager

    rclpy.init()
    node = LocalizationManager()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def pose(x, y):
    m = PoseWithCovarianceStamped()
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    return m


def test_needs_two_agreeing_matches(manager):
    clock = iter([0.0, 1.0, 2.0, 3.0, 20.0])
    manager.now = lambda: next(clock)
    assert not manager.confirmed(pose(10.0, 5.0))  # one match: not yet
    assert not manager.confirmed(pose(12.3, 5.0))  # disagrees by 2.3 m
    assert manager.confirmed(pose(12.5, 5.1))  # agrees with the one before
    assert manager.confirmed(pose(12.6, 5.1))
    assert not manager.confirmed(pose(12.6, 5.1))  # 17 s later: too old to count
