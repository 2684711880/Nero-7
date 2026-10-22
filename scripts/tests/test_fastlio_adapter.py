"""Output-frame alignment must support upright, inverted and tilted IMUs."""
import importlib.machinery
import importlib.util
from pathlib import Path
import unittest
import numpy as np

path=Path(__file__).resolve().parents[2]/'sim_ws/src/nero_mid360_sim/scripts/fastlio_output_adapter'
loader=importlib.machinery.SourceFileLoader('fastlio_adapter',str(path))
spec=importlib.util.spec_from_loader(loader.name,loader)
adapter=importlib.util.module_from_spec(spec);loader.exec_module(adapter)


class AlignmentTest(unittest.TestCase):
    def test_upright_stays_identity(self):
        rotation,_=adapter.gravity_alignment([0,0,9.81])
        np.testing.assert_allclose(rotation,np.eye(3),atol=1e-12)

    def test_inverted_mount_becomes_upright_in_viewer(self):
        rotation,_=adapter.gravity_alignment([0,0,-9.81])
        np.testing.assert_allclose(rotation,np.diag([1,-1,-1]),atol=1e-12)
        np.testing.assert_allclose(rotation@np.array([0,0,-9.81]),[0,0,9.81],atol=1e-12)

    def test_tilt_alignment_is_proper_rotation_and_preserves_length(self):
        acceleration=np.array([.25,-.4,-9.8])
        original=acceleration.copy()
        rotation,_=adapter.gravity_alignment(acceleration)
        np.testing.assert_array_equal(acceleration,original)
        np.testing.assert_allclose(rotation@acceleration,[0,0,np.linalg.norm(acceleration)],atol=1e-10)
        np.testing.assert_allclose(rotation.T@rotation,np.eye(3),atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(rotation),1.)
        self.assertGreater(rotation[0,0],0.)
        self.assertAlmostEqual(rotation[1,0],0.)


if __name__=='__main__':unittest.main()
