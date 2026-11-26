"""Compare the real navigation-result callback on saved stationary cancellations."""
import argparse
from concurrent.futures import Future
import json
from pathlib import Path
import re
from types import SimpleNamespace as NS
from unittest.mock import Mock

from osp_mission.mission import Mission
from osp_mission.navigation_flow import NavigationFlow


def replay(row, root):
    console=(root/row['run']/'console.log').read_text(errors='replace')
    goals=[(float(stamp),(float(x),float(y))) for stamp,x,y in re.findall(
        r'\[([\d.]+)\] \[mission\]: Navigation goal \(([-\d.]+), ([-\d.]+)\)',console)]
    outcomes=[]
    for stall in row['stalls']:
        stamp=stall['stamp']
        goal=next((xy for t,xy in reversed(goals) if t<=stamp),None)
        if goal is None:
            raise RuntimeError('Recorded navigation goal unavailable')
        flow=NavigationFlow();flow.selected(dict(zip(('x','y'),goal)),None)
        node=NS(state='running',kind='nav',goal_handle=object(),cancel_reason='navigation_stalled',
                now=lambda:stamp,destination=goal,excluded=[],navigation_flow=flow,
                recovery_pending=False,resume_action=None,scan_after=False,
                get_logger=lambda:Mock())
        result=Future();result.set_result(NS(status=5))
        Mission.completed(node,result)
        scan=stall['sensor_scan']
        outcomes.append(dict(stamp=stamp,goal=goal,
            recovery_pending=node.recovery_pending,
            recorded_rear_clear=scan.get('rear_clear') if scan else None,
            retreat_scan_gate_passed=bool(node.recovery_pending and scan and scan.get('rear_clear'))))
    return dict(run=row['run'],snapshots=outcomes)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('analysis',type=Path)
    parser.add_argument('runs',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    result=[replay(row,args.runs) for row in json.loads(args.analysis.read_text())]
    args.output.write_text(json.dumps(result,indent=2))
    print([(row['run'],sum(s['recovery_pending'] for s in row['snapshots']),
            sum(s['retreat_scan_gate_passed'] for s in row['snapshots'])) for row in result])
