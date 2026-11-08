"""Honest replay labels derived from saved phases and selected-cycle evidence."""
import json
from pathlib import Path
import sys

import numpy as np

REPO=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(REPO/'nvidia-sim/harvest_s300'))
from selected_cycle_evidence import MANIPULATION_PHASES


def segment_label(row,proof):
    phase=row.get('phase','actual');campaign=row.get('campaign','patrol');index=int(row['index'])
    label=f"카메라 순회 {campaign}-{index:03d} · "+{'scan':'카메라 관측','move':'조금씩 전진',
        'harvest':'수확 시도','lift':'리프트 조절'}.get(phase,phase)
    detail=None
    if phase=='harvest' and proof.get('record_kind')!='partial_diagnostic':
        trial=Path(proof['trial']);summary=json.loads((trial/'summary.json').read_text())
        cycles=summary.get('selected_cycle_evidence',{}).get('cycles',[])
        all_rejected=bool(cycles) and all(cycle.get('status') in ('planning_rejected','planning_failed') and
            cycle.get('recorded_physical_cycle') is False and cycle.get('planned_manipulation_started') is False
            for cycle in cycles)
        if all_rejected:
            with np.load(trial/'states.npz',allow_pickle=False) as state:
                phases=set(state['phase'].tolist());times=state['time']
            if not phases & MANIPULATION_PHASES and len(times)>=2:
                duration=float(times[-1]-times[0])
                detail=dict(kind='camera_observation_with_unexecuted_planning_rejections',
                    observed_physics_duration_s=duration,actual_manipulation_cycles=0,
                    unexecuted_planning_rejections=len(cycles),physical_harvest_success_claimed=False,
                    summary_sha256=proof.get('summary_sha256'),states_sha256=proof['states_sha256'])
                label=f"카메라 순회 {campaign}-{index:03d} · 접근계획 차단·관측{duration:g}초, 실제수확0회"
    if proof.get('physical_stop_reason'):label+=' · 중단 '+str(proof['physical_stop_reason'])
    description=None
    if detail:
        description=(f"실제로 저장된 {detail['observed_physics_duration_s']:g}초 카메라 관측입니다. "
            f"접근 계획 {detail['unexecuted_planning_rejections']}개가 실행 전에 거절되어 실제 수확 동작은 0회입니다. "
            "명목 계획을 실제 진입 영상으로 재생하지 않습니다.")
    return label,description,detail
