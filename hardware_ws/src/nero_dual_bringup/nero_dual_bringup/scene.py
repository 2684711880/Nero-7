"""Fixed world geometry for real-arm planning; never publishes motion commands."""
import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

DEFAULTS = dict(support_height=.28, cabinet_height=.60, cabinet_width=1.20,
                cabinet_depth=.40, cabinet_front=.50)


def scene_objects(support_height=.28, cabinet_height=.60, cabinet_width=1.20,
                  cabinet_depth=.40, cabinet_front=.50):
    values = (support_height, cabinet_height, cabinet_width, cabinet_depth, cabinet_front)
    if (not all(math.isfinite(v) for v in values)
            or min(support_height, cabinet_height) < 0
            or min(cabinet_width, cabinet_depth, cabinet_front) <= 0):
        raise ValueError('Scene dimensions must be finite and positive; heights may be zero.')
    objects = [('real_ground', (20., 20., .10), (0., 0., -.05))]
    if support_height > 0:
        objects.append(('support_pedestal', (.40, .35, support_height),
                        (0., 0., support_height/2)))
    if cabinet_height > 0:
        objects.append(('front_cabinet', (cabinet_depth, cabinet_width, cabinet_height),
                        (cabinet_front+cabinet_depth/2, 0., cabinet_height/2)))
    return objects


def call(node, service_type, name, request):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=30):
        raise RuntimeError(f'{name} unavailable')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done() or future.result() is None:
        raise RuntimeError(f'{name} did not complete')
    return future.result()


def sync_viewer(node, wait_seconds=20.0):
    """Send a full snapshot after a late-starting RViz subscribes, not only the ADD diff."""
    publisher = node.create_publisher(
        PlanningScene, '/monitored_planning_scene',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + wait_seconds
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    if publisher.get_subscription_count() == 0:
        node.get_logger().info('No scene viewer yet; geometry remains registered in MoveIt.')
        return
    query = GetPlanningScene.Request()
    query.components.components = (
        PlanningSceneComponents.SCENE_SETTINGS | PlanningSceneComponents.ROBOT_STATE |
        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.OCTOMAP |
        PlanningSceneComponents.TRANSFORMS | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
        PlanningSceneComponents.LINK_PADDING_AND_SCALING | PlanningSceneComponents.OBJECT_COLORS)
    snapshot = call(node, GetPlanningScene, 'get_planning_scene', query).scene
    snapshot.is_diff = False
    publisher.publish(snapshot)
    # Keep the publisher alive briefly so the reliable sample reaches the viewer.
    deadline = time.monotonic() + .5
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    node.get_logger().info('Full planning scene synchronized to viewer.')


def main():
    rclpy.init()
    node = Node('real_arm_cabinet_scene')
    code = 0
    try:
        geometry = {key: node.declare_parameter(key, value).value for key, value in DEFAULTS.items()}
        objects = scene_objects(**geometry)
        query = GetPlanningScene.Request()
        query.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX | PlanningSceneComponents.WORLD_OBJECT_NAMES
        current = call(node, GetPlanningScene, 'get_planning_scene', query).scene
        scene = PlanningScene(is_diff=True)
        scene.allowed_collision_matrix = current.allowed_collision_matrix
        acm = scene.allowed_collision_matrix
        # The fixed torso sits on its pedestal. Keep both arms checked against all obstacles.
        for name in ('real_ground', 'support_pedestal', 'base_link'):
            if name not in acm.entry_names:
                acm.entry_names.append(name)
                for row in acm.entry_values:
                    row.enabled.append(False)
                acm.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(acm.entry_names)))
        for support in ('real_ground', 'support_pedestal'):
            i, j = acm.entry_names.index(support), acm.entry_names.index('base_link')
            acm.entry_values[i].enabled[j] = acm.entry_values[j].enabled[i] = True
        for name, dimensions, xyz in objects:
            obj = CollisionObject(id=name)
            obj.header.frame_id = 'world'
            obj.operation = CollisionObject.ADD
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dimensions))]
            pose = Pose()
            pose.orientation.w = 1.
            pose.position.x, pose.position.y, pose.position.z = xyz
            obj.primitive_poses = [pose]
            scene.world.collision_objects.append(obj)
        # Remove the cabinet/pedestal left by a prior configuration when disabled.
        for name in ({'front_cabinet', 'support_pedestal'} & {obj.id for obj in current.world.collision_objects}) - {obj[0] for obj in objects}:
            scene.world.collision_objects.append(CollisionObject(id=name, operation=CollisionObject.REMOVE))
        result = call(node, ApplyPlanningScene, 'apply_planning_scene',
                      ApplyPlanningScene.Request(scene=scene))
        if not result.success:
            raise RuntimeError('MoveIt rejected real-arm scene')
        node.get_logger().info(f'Real-arm world scene applied: {objects}')
        sync_viewer(node)
    except Exception as error:
        node.get_logger().error(str(error))
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)
