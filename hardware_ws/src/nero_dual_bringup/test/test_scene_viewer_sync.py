"""No CAN: reproduce a viewer subscribing after the original scene ADD was sent."""
import os
import threading
import time
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from nero_dual_bringup.scene import sync_viewer

assert os.environ.get('ROS_DOMAIN_ID') == '94', 'Use isolated test domain 94'
rclpy.init()
server = rclpy.create_node('scene_snapshot_test_server')
loader = rclpy.create_node('scene_snapshot_test_loader')
snapshot = PlanningScene()
obj = CollisionObject(id='front_cabinet', operation=CollisionObject.ADD)
obj.header.frame_id = 'world'
obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[.4, 1.2, .6])]
pose = Pose()
pose.orientation.w = 1.
pose.position.x, pose.position.z = .85, .3
obj.primitive_poses = [pose]
snapshot.world.collision_objects = [obj]
service = server.create_service(GetPlanningScene, '/get_planning_scene',
                               lambda req, res: GetPlanningScene.Response(scene=snapshot))
executor = rclpy.executors.SingleThreadedExecutor()
executor.add_node(server)
worker = threading.Thread(target=executor.spin)
worker.start()
received = []
subscription = []


def late_viewer():
    # A volatile viewer cannot retrieve an old one-shot diff after its publisher exited.
    subscription.append(server.create_subscription(PlanningScene, '/monitored_planning_scene',
                                                  received.append, 10))


timer = threading.Timer(.5, late_viewer)
timer.start()
try:
    sync_viewer(loader, wait_seconds=4.)
    assert received, 'Late viewer missed scene'
    assert not received[-1].is_diff, 'Viewer needs a full snapshot'
    cabinet = received[-1].world.collision_objects[0]
    assert cabinet.id == 'front_cabinet'
    assert list(cabinet.primitives[0].dimensions) == [.4, 1.2, .6]
    print('PASS: delayed volatile viewer received complete cabinet scene; no CAN or motion nodes.')
finally:
    timer.join()
    executor.shutdown()
    worker.join()
    loader.destroy_node()
    server.destroy_node()
    rclpy.shutdown()
