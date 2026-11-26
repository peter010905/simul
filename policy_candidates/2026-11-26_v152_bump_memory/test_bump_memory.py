"""Checks for the .58 bump-memory candidate (no ROS node)."""
import osp_mission.mission as mission


def test_bump_contacts_feed_the_known_obstacle_topic():
    src = open(mission.__file__).read()
    assert "def remember_bump" in src and "self.remember_bump(" in src
    assert "if self.bump_points:" in src
    assert "create_publisher(PointCloud2,'mission/known_obstacles',qos_profile_sensor_data)\n" in src
