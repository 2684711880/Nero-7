#!/usr/bin/env python3
"""Offline validation only: never opens CAN or calls an arm driver."""
import json
import argparse
from pathlib import Path
import sys
import time

import numpy as np
import pinocchio as pin

root = Path(__file__).resolve().parents[1]
vendor = root / 'third_party/agilex-arm-gravity-compensation'
sys.path.insert(0, str(vendor))
from core.agx_pinocchio import AgxPinocchio

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--urdf', default=str(vendor / 'nero/nero/urdf/nero_description.urdf'))
args = parser.parse_args()
model = AgxPinocchio(args.urdf)
assert model.nq == model.nv == 7, (model.nq, model.nv)
zeros = np.zeros(7)
poses = [zeros, np.array([0.1, 0.4, -0.3, 0.2, -0.1, 0.2, 0.1])]
samples = []
for q in poses:
    torque = model.inverse_dynamics(q, zeros, zeros, np.eye(3))
    reference = pin.computeGeneralizedGravity(model.robot.model, model.robot.data, q).copy()
    assert np.isfinite(torque).all()
    assert np.allclose(torque, reference, atol=1e-9)
    samples.append({'q_rad': q.tolist(), 'tau_Nm': torque.tolist()})
start = time.perf_counter()
for _ in range(1000):
    model.inverse_dynamics(poses[1], zeros, zeros, np.eye(3))
duration = time.perf_counter() - start
report = {
    'validation': 'offline model and dynamics only; no CAN communication',
    'pinocchio_version': pin.__version__, 'nq': model.nq, 'nv': model.nv,
    'urdf': args.urdf,
    'samples': samples, 'mean_compute_ms': duration,
    'hardware_validation': False,
    'unresolved': ['standard gripper model versus installed gripper',
                   'startup transition after unexpected motion', 'official exit move_j behavior'],
}
out = root / 'log/gravity_environment'
out.mkdir(parents=True, exist_ok=True)
(out / 'offline_report.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
