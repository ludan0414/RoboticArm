"""The site override allows current and subsequently added MoveIt objects."""
import pytest

pytest.importorskip('moveit_msgs')

from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene
from luggage_planning.planning_scene_client import PlanningSceneClient


def test_allow_all_collisions_sets_pairs_and_defaults():
    scene = PlanningScene()
    scene.allowed_collision_matrix.entry_names = ['elfin_link5', 'site_door_clearance_top']
    scene.allowed_collision_matrix.entry_values = [
        AllowedCollisionEntry(enabled=[False, False]),
        AllowedCollisionEntry(enabled=[False, False]),
    ]
    client = PlanningSceneClient.__new__(PlanningSceneClient)
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
