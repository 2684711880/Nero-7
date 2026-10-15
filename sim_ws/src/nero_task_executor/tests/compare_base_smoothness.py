#!/usr/bin/env python3
"""Compare measured same-task baseline/candidate, including bottle acceptance."""
import argparse
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--baseline', required=True)
parser.add_argument('--candidate', required=True)
parser.add_argument('--bottle', required=True)
parser.add_argument('--output', required=True)
args = parser.parse_args()
baseline = json.loads(Path(args.baseline).read_text())
candidate = json.loads(Path(args.candidate).read_text())
bottle = json.loads(Path(args.bottle).read_text())
checks = {'baseline_completed': baseline['task_status']['status'] == 'SUCCEEDED',
          'candidate_completed': candidate['task_status']['status'] == 'SUCCEEDED',
          'bottle_placement_and_return': bottle['passed']}
metrics = {}
for phase in ('b2', 'b3'):
    old, new = baseline['metrics'][phase], candidate['metrics'][phase]
    checks[phase+'_blocked_fraction'] = (new['cruise_blocked_fraction'] < .05 and
        new['cruise_blocked_fraction'] < old['cruise_blocked_fraction']*.25)
    checks[phase+'_actual_stops'] = new['cruise_near_zero_actual_fraction'] < .02
    checks[phase+'_no_slowdown'] = new['wall_duration_s'] < old['wall_duration_s']*1.2
    metrics[phase] = {'baseline': old, 'candidate': new}
report = {'passed': all(checks.values()), 'checks': checks, 'metrics': metrics,
          'bottle_metrics': bottle['metrics'], 'hardware_connected': False,
          'measurement_window': 'middle 60 percent of each base action; wheel blocking counted when both translation and yaw are requested'}
Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
