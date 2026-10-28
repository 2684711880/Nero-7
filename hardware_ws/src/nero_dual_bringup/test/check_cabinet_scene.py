"""Read-only scene/height/park checks against the isolated driver-free mock launch."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import rclpy
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters

rclpy.init()
node = rclpy.create_node('check_real_cabinet_scene_mock')


def call(typ, name, request):
    client = node.create_client(typ, name)
    assert client.wait_for_service(timeout_sec=20), name
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    assert future.done() and future.result(), name
    return future.result()


request = GetPlanningScene.Request()
request.components.components = 1023
scene = call(GetPlanningScene, '/get_planning_scene', request).scene
objects = {o.id: o for o in scene.world.collision_objects}
report = {'hardware_connected': False, 'scene': {}}
for name in ('real_ground', 'front_cabinet', 'support_pedestal'):
    obj = objects[name]
    p, q = obj.pose.position, obj.primitive_poses[0].position
    report['scene'][name] = {'size': list(obj.primitives[0].dimensions),
                            'xyz': [p.x+q.x, p.y+q.y, p.z+q.z]}
cabinet = report['scene']['front_cabinet']
assert abs(cabinet['xyz'][2]+cabinet['size'][2]/2-.6) < 1e-9
assert abs(cabinet['xyz'][0]-cabinet['size'][0]/2-.65) < 1e-9
for key in ('robot_description', 'robot_description_semantic'):
    request = GetParameters.Request(names=[key])
    value = call(GetParameters, '/move_group/get_parameters', request).values[0].string_value
    if key == 'robot_description':
        robot = ET.fromstring(value)
        xyz = list(map(float, robot.find("joint[@name='world_joint']/origin").get('xyz').split()))
        assert xyz == [0, 0, .28]
        report['support_height'] = xyz[2]
    else:
        semantic = ET.fromstring(value)
        park = semantic.find("group_state[@name='park'][@group='both_arms']")
        assert len(park.findall('joint')) == 14
        request = GetStateValidity.Request()
        request.robot_state.joint_state.name = [j.get('name') for j in park.findall('joint')]
        request.robot_state.joint_state.position = [float(j.get('value')) for j in park.findall('joint')]
        request.robot_state.joint_state.name += ['left_gripper', 'right_gripper']
        request.robot_state.joint_state.position.extend([.04, .04])
        validity = call(GetStateValidity, '/check_state_validity', request)
        report['park_valid'] = validity.valid
        report['contacts'] = [[c.contact_body_1, c.contact_body_2] for c in validity.contacts]
        assert validity.valid, report['contacts']
acm = scene.allowed_collision_matrix
cabinet_index = acm.entry_names.index('front_cabinet') if 'front_cabinet' in acm.entry_names else None
if cabinet_index is not None:
    assert not any(acm.entry_values[cabinet_index].enabled)
report['cabinet_collision_checking_enabled'] = True
Path('/home/a/Nero-7/hardware_ws/validation/dual_moveit/cabinet_scene_report.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
node.destroy_node()
rclpy.shutdown()
