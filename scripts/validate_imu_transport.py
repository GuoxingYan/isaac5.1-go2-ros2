"""Check received ROS IMU timestamps, independently of simulation-side sampling."""
import json
from pathlib import Path
import numpy as np


def validate(evidence):
    times=np.asarray(evidence.get('imu_times',[]),dtype=float)
    # A probe may span multiple simulator runs. Check only the latest epoch.
    resets=np.flatnonzero(np.diff(times)<0)
    if len(resets): times=times[resets[-1]+1:]
    dt=np.diff(times)
    hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.
    gap=float(dt.max()) if len(dt) else None
    return {'name':'ROS2 MID360 IMU transport continuous 200Hz',
            'pass':bool(len(times)>100 and np.isfinite(times).all() and np.all(dt>0) and abs(hz-200)<1 and gap<.0051),
            'samples':len(times),'received_simulated_hz':float(hz),'maximum_gap_s':gap,
            'evidence_source':'ros_topics.json: actual received sensor_msgs/Imu header timestamps, latest simulation epoch'}

if __name__=='__main__':
    import sys
    directory=Path(sys.argv[1])
    result=validate(json.loads((directory/'ros_topics.json').read_text()))
    (directory/'imu_transport_evidence.json').write_text(json.dumps(result,indent=2))
    report=json.loads((directory/'results.json').read_text())
    report['results']=[x for x in report['results'] if x['name']!=result['name']]+[result]
    (directory/'results.json').write_text(json.dumps(report,indent=2))
    print(result)
