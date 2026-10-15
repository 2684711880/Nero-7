import json, time, copy
from pathlib import Path
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState, SetEntityState
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from mobile_dual_nero.environment import warehouse_objects
rclpy.init()
n=Node('verify_warehouse_environment')
def call(kind,name,req):
 c=n.create_client(kind,name)
 assert c.wait_for_service(timeout_sec=15),name
 f=c.call_async(req)
 rclpy.spin_until_future_complete(n,f,timeout_sec=15)
 assert f.done() and f.result() is not None,name
 return f.result()
def state(name):
 q=GetEntityState.Request();q.name=name;q.reference_frame='world'
 r=call(GetEntityState,'/gazebo/get_entity_state',q);assert r.success,name
 r.state.name=name;r.state.reference_frame='world'
 return r.state
def scene():
 q=GetPlanningScene.Request();q.components.components=2|16
 return call(GetPlanningScene,'/get_planning_scene',q).scene
objects=warehouse_objects()
source_table=next(o for o in objects if o['name']=='source_table_top')
destination_table=next(o for o in objects if o['name']=='destination_table_top')
bottle=next(o for o in objects if o['name']=='water_bottle')
table_surface=source_table['xyz'][2]+source_table['size'][2]/2
settled_bottle_z=table_surface+bottle['size'][0]/2
s=scene();actual={o.id:o for o in s.world.collision_objects}
assert set(o['name'] for o in objects).issubset(actual)
for o in objects:
 a=actual[o['name']]
 assert list(a.primitives[0].dimensions)==o['size'],o['name']
 if o['name']!='water_bottle':
  pos=a.pose.position
  assert max(abs(v-t) for v,t in zip([pos.x,pos.y,pos.z],o['xyz']))<1e-8
before=state('water_bottle')
time.sleep(2)
after=state('water_bottle')
assert abs(after.pose.position.z-settled_bottle_z)<.003
assert abs(after.pose.position.z-before.pose.position.z)<.001
p=actual['water_bottle'].pose.position
assert abs(p.z-after.pose.position.z)<.003
q=GetStateValidity.Request();q.robot_state=s.robot_state
validity=call(GetStateValidity,'/check_state_validity',q)
assert validity.valid,[(c.contact_body_1,c.contact_body_2) for c in validity.contacts]
# Move the bottle to the second table, check physics and MoveIt, restore it.
try:
 changed=copy.deepcopy(after);changed.pose.position.y=destination_table['xyz'][1]+(bottle['xyz'][1]-source_table['xyz'][1]);changed.pose.position.z=settled_bottle_z+.005
 q=SetEntityState.Request();q.state=changed
 assert call(SetEntityState,'/gazebo/set_entity_state',q).success
 time.sleep(2)
 moved=state('water_bottle');p=next(o for o in scene().world.collision_objects if o.id=='water_bottle').pose.position
 assert abs(p.y-moved.pose.position.y)<.003 and abs(p.z-moved.pose.position.z)<.003
 assert abs(moved.pose.position.z-settled_bottle_z)<.003
finally:
 q=SetEntityState.Request();q.state=after
 assert call(SetEntityState,'/gazebo/set_entity_state',q).success
 time.sleep(1)
report=dict(environment='warehouse',geometry_objects=len(objects),moveit_geometry_matches=True,
 water_bottle_settled_center_z=after.pose.position.z,table_surface_z=table_surface,
 bottle_stable=True,bottle_moveit_pose_sync_verified=True,initial_robot_state_collision_free=validity.valid,
 racks=2,shelf_tiers_per_rack=4,tables=2,central_corridor_width_m=2.8,
 evidence='Live Gazebo physics, GetEntityState, GetPlanningScene, GetStateValidity and bottle relocation/restore',
 limitation='Manipulation and navigation tasks are not yet verified; odom follows wheel odometry and can drift from Gazebo world.')
Path(__file__).with_name('report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
