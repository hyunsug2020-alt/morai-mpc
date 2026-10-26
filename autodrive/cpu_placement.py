"""Portable opt-in CPU placement, configured per destination PC."""
import json,os,sys
from pathlib import Path
PROFILE=Path(__file__).resolve().parent/'performance_cpu.json'
KEYS=('STUDENT_IO_CPUS','STUDENT_COMPUTE_CPUS','STUDENT_SIM_WORKER_CPUS','AUTODRIVE_LIDAR_CPUS')
try:
    settings=json.loads(PROFILE.read_text())
    for key in KEYS:os.environ.setdefault(key,str(settings.get(key,'')))
except (OSError,ValueError):pass

def pin(role,key):
    spec=os.environ.get(key,'')
    if not spec or not hasattr(os,'sched_setaffinity'):return
    try:
        cpus=set()
        for part in spec.split(','):
            ends=part.split('-');cpus.update(range(int(ends[0]),int(ends[-1])+1))
        os.sched_setaffinity(0,cpus)
        print('[cpu] '+role+': '+spec,flush=True)
    except (OSError,ValueError) as exc:print('[cpu] keeping default for '+role+': '+str(exc),file=sys.stderr)

def pin_io_thread(role):pin(role,'STUDENT_IO_CPUS')
def pin_compute_thread(role):pin(role,'STUDENT_COMPUTE_CPUS')
