"""Offline seating evidence. Neither geometry nor this timeline is hook_success."""
import numpy as np


def contact_identity(contact, targets, rear_wires, phase):
    names=set(contact['geoms'])
    return (phase in ('seat','hold','verify') and bool(names & set(targets))
            and bool(names & set(rear_wires)))


def timeline_evidence(rows, *, initially_seated, dt, hold_seconds=1., confirm_seconds=.2):
    """Require elapsed time and continuous samples; count is not a duration."""
    if dt<=0 or not rows:raise ValueError('Need timestamped samples and positive dt')
    times=np.array([r['time_s'] for r in rows])
    if not np.isfinite(times).all() or np.any(np.diff(times)<=0):raise ValueError('Times must strictly increase')
    def interval(phase,minimum):
        part=[r for r in rows if r['phase']==phase]
        ts=np.array([r['time_s'] for r in part]);duration=float(ts[-1]-ts[0]) if len(ts)>1 else 0.
        gap=float(np.max(np.diff(ts))) if len(ts)>1 else None
        complete=len(ts)>1 and duration>=minimum-1e-8 and gap<=dt*1.5
        seated=complete and all(r['seated'] and r['target_contact'] for r in part)
        return dict(samples=len(part),elapsed_s=duration,max_sample_gap_s=gap,continuous=bool(complete),seated_with_force=bool(seated))
    hold=interval('hold',hold_seconds);verify=interval('verify',confirm_seconds)
    before=[r for r in rows if r['phase'] not in ('hold','verify')]
    entered=not initially_seated and any(r['seated'] and r['target_contact'] for r in before)
    non_target=max((r['non_target_force_N'] for r in rows),default=0.)
    valid=all(r['physics_valid'] for r in rows)
    return dict(initially_seated=bool(initially_seated),entered_after_start=bool(entered),hold=hold,verify=verify,
                contact_retention_evidence=bool(entered and hold['seated_with_force'] and verify['seated_with_force'] and valid and non_target<.01),
                max_non_target_force_N=non_target,physics_valid=bool(valid),hook_success=None,training_eligible=False,
                scope='Continuous measured seating/contact evidence; not detachment or mechanical escape proof; .01N reporting floor')
