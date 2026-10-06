#!/usr/bin/env python3
"""Read-only closed-run evaluation; explicitly retain an interrupted missing Gate."""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import precision_eval as ev

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--run', type=Path, required=True)
p.add_argument('--scene', type=Path, required=True)
p.add_argument('--exact-offset-data', type=Path, required=True)
p.add_argument('--stop-record', type=Path)
a = p.parse_args()
run = a.run.resolve()
gate = run / 'gate_status.json'
if not gate.exists():
    if not a.stop_record:
        p.error('Missing final Gate requires the preserved explicit stop record')
    stop = json.loads(a.stop_record.read_text())
    if stop.get('status') != 'INTERRUPTED_DIAGNOSTIC' or Path(stop['run']).resolve() != run:
        p.error('Stop record does not identify this interrupted diagnostic run')
    original = ev.read_json
    ev.read_json = lambda path: dict(status=None, reason=None, failed_checks=None) if Path(path) == gate else original(path)
result = ev.evaluate(run, a.scene.resolve())
ev.attach_exact_offsets(result, a.exact_offset_data.resolve())
if not gate.exists():
    result.update(diagnostic_status='INTERRUPTED_DIAGNOSTIC_FAIL', gate_status_file_present=False,
                  diagnostic_reason='third_slot_commitment_position_drift',
                  note_missing_gate='Explicit operator stop; original final Gate absent, not fabricated.')
print(json.dumps(result, indent=2))
