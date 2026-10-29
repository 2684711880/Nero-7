import sys,unittest
from pathlib import Path
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from real_hole_two_stage import scale_trajectory_time

class FeedTimingTest(unittest.TestCase):
    def test_preserves_path_and_scales_time_and_derivatives(self):
        t=RobotTrajectory();a=JointTrajectoryPoint(positions=[1.],velocities=[.4],accelerations=[.8]);b=JointTrajectoryPoint(positions=[2.],velocities=[.2],accelerations=[-.4]);b.time_from_start.sec=1;t.joint_trajectory.points=[a,b]
        scale_trajectory_time(t,.05)
        self.assertEqual(t.joint_trajectory.points[1].time_from_start.sec,20)
        self.assertEqual(list(t.joint_trajectory.points[1].positions),[2.])
        self.assertAlmostEqual(t.joint_trajectory.points[0].velocities[0],.02)
        self.assertAlmostEqual(t.joint_trajectory.points[0].accelerations[0],.002)
    def test_rejects_nonincreasing_time(self):
        t=RobotTrajectory();t.joint_trajectory.points=[JointTrajectoryPoint(),JointTrajectoryPoint()]
        with self.assertRaises(ValueError):scale_trajectory_time(t,.05)
