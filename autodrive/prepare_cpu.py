#!/usr/bin/env python3
"""Apply the native-thread placement measured on this i7-13700K workstation."""
import json,os,time
from pathlib import Path


def main():
    if '13th Gen Intel(R) Core(TM) i7-13700K' not in Path('/proc/cpuinfo').read_text():
        print('[cpu] Other CPU model: retaining simulator placement');return
    from cpu_placement import pin_io_thread  # loads this PC's opt-in profile
    spec=os.environ.get('STUDENT_SIM_WORKER_CPUS','')
    if not spec:return
    cpus=set()
    for part in spec.split(','):
        ends=part.split('-');cpus.update(range(int(ends[0]),int(ends[-1])+1))
    simulators=[]
    for d in Path('/proc').iterdir():
        if not d.name.isdigit():continue
        try:cmd=(d/'cmdline').read_bytes().split(b'\0')[0].decode()
        except OSError:continue
        if Path(cmd).name=='Simulator.x86_64':simulators.append(int(d.name))
    if len(simulators)!=1:
        print('[cpu] Simulator instance ambiguous/missing: retaining placement');return
    pid=simulators[0];hz=os.sysconf('SC_CLK_TCK')
    def snapshot():
        result={}
        for d in Path(f'/proc/{pid}/task').iterdir():
            try:
                if int(d.name)==pid or (d/'comm').read_text().strip()!='Simulator.x86_6':continue
                f=(d/'stat').read_text().rsplit(')',1)[1].split()
                result[int(d.name)]=(int(f[11])+int(f[12]))/hz
            except OSError:pass
        return result
    before=snapshot();started=time.monotonic();time.sleep(.5);after=snapshot();elapsed=time.monotonic()-started
    hot=[tid for tid in after if tid in before and (after[tid]-before[tid])/elapsed>.85]
    # Only the exact four-thread pattern tested with the four-camera preset.
    # Never move the main/render threads or unknown numbers of native workers.
    if len(hot)!=4:
        print('[cpu] Native worker pattern differs: retaining placement');return
    applied=[]
    for tid in hot:
        try:os.sched_setaffinity(tid,cpus);applied.append(tid)
        except OSError as exc:print('[cpu] Native worker unchanged:',tid,exc)
    print('[cpu] Measured native busy workers -> '+spec+': '+str(applied),flush=True)


if __name__=='__main__':main()
