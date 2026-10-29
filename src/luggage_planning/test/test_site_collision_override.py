"""The site override allows current and subsequently added MoveIt objects."""
import pytest

pytest.importorskip('moveit_msgs')

from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene
from luggage_planning.planning_scene_client import PlanningSceneClient, robot_link_names, acm_pair_allowed


def test_allow_all_collisions_sets_pairs_and_defaults():
    scene = PlanningScene()
    scene.allowed_collision_matrix.entry_names = ['elfin_link5', 'site_door_clearance_top']
    scene.allowed_collision_matrix.entry_values = [
        AllowedCollisionEntry(enabled=[False, False]),
        AllowedCollisionEntry(enabled=[False, False]),
    ]
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client._override_robot_links = robot_link_names(
        '<robot name="test"><link name="camera_link"/><link name="eef_mount_adapter"/></robot>')
    client.get_scene = lambda components, timeout: scene
    applied = []
    client._apply = lambda diff, timeout: (applied.append(diff) or (True, 'ok'))

    ok, _ = client.allow_all_collisions()

    assert ok and len(applied) == 1
    matrix = applied[0].allowed_collision_matrix
    assert all(all(entry.enabled) for entry in matrix.entry_values)
    defaults = dict(zip(matrix.default_entry_names, matrix.default_entry_values))
    assert defaults['elfin_link5'] and defaults['pickup_box']
    assert defaults['site_door_clearance_top']
    assert acm_pair_allowed(matrix, 'camera_link', 'eef_mount_adapter')
    assert acm_pair_allowed(matrix, 'camera_link', 'pickup_box')


def test_override_fails_when_robot_links_are_unknown():
    client = PlanningSceneClient.__new__(PlanningSceneClient)
    client.get_scene = lambda components, timeout: PlanningScene()
    client._apply = lambda *args, **kwargs: pytest.fail('incomplete matrix applied')
    ok, message = client.allow_all_collisions()
    assert not ok and 'robot link names unavailable' in message
