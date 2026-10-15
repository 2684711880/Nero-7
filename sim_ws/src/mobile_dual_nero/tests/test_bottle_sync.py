"""Attached bottles must not receive world MOVE updates from Gazebo sync."""
import runpy
from pathlib import Path
from types import SimpleNamespace
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject

Sync = runpy.run_path(str(Path(__file__).parents[1] / 'scripts/sync_bottle'))['BottleSync']

def body(operation):
    a = AttachedCollisionObject()
    a.object.id = 'water_bottle'
    a.object.operation = operation
    return a

def test_attach_and_detach_diffs():
    node = SimpleNamespace(attached=False)
    s = PlanningScene()
    s.is_diff = s.robot_state.is_diff = True
    s.robot_state.attached_collision_objects = [body(CollisionObject.ADD)]
    Sync.scene_update(node, s)
    assert node.attached
    s.robot_state.attached_collision_objects = []
    Sync.scene_update(node, s)
    assert node.attached  # unrelated incremental scene must retain attachment
    s.robot_state.attached_collision_objects = [body(CollisionObject.REMOVE)]
    Sync.scene_update(node, s)
    assert not node.attached

def test_full_snapshot_clears_removed_attachment():
    node = SimpleNamespace(attached=True)
    Sync.scene_update(node, PlanningScene())
    assert not node.attached

def test_attached_feedback_does_not_publish_world_move():
    node = SimpleNamespace(attached=True, last=-1.,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10**9)))
    # No publisher exists: any attempted publish would fail this check.
    Sync.update(node, SimpleNamespace(name=['water_bottle']))
