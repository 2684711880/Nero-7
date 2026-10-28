import math
import xml.etree.ElementTree as ET
from ranger_air_gazebo.model import simulation_description, contact_spawn_pose


def test_initial_stance_matches_original_front_rear_axle_offset():
    description=simulation_description()
    z,roll,pitch=contact_spawn_pose(description)
    assert abs(roll)<1e-10
    assert abs(pitch-math.atan(-.001/.388))<1e-10
    # Compute the four actual cylinder contact heights after the rotation.
    # At zero roll/steering the tire axes remain horizontal and radius unchanged.
    front=-math.sin(pitch)*.194+math.cos(pitch)*.0605-.0605+z
    rear=-math.sin(pitch)*(-.194)+math.cos(pitch)*.0615-.0605+z
    assert abs(front-.003)<1e-10 and abs(rear-.003)<1e-10
    # Axle kinematics were not flattened or silently rewritten.
    root=ET.fromstring(description)
    assert root.find("joint[@name='rl_wheel_joint']") is not None
