"""Read-only per-step capsule overlap check. No poses or impulses are modified."""
import numpy as np
from scipy.spatial.transform import Rotation
from connection_audit import segment_distance


class ToolContactAudit:
    def __init__(self, model, tolerance=.0005):
        self.shapes=[s for s in model['shapes'] if s.get('enabled') is not False];self.tolerance=tolerance
        self.tool=[s for s in self.shapes if '/RingCollision/' in s['path'] or '/RailCollision/' in s['path']]
        self.plant=[s for s in self.shapes if '/ElasticPlant/' in s['path'] or '/HarvestableStem/Harvestables/' in s['path']]
        self.selected=self.tool+self.plant
        self.ids=np.array([s['body'] for s in self.selected]);self.ends=np.array([s['local_endpoints'] for s in self.selected]);self.r=np.array([s['radius_m'] for s in self.selected])
        self.rows=[];self.worst=None

    def reset(self):
        self.rows=[];self.worst=None

    def sample(self, poses, time_s):
        p=poses[self.ids]
        if not np.isfinite(p).all():raise ValueError('Nonfinite native pose during tool collision audit')
        r=Rotation.from_quat(p[:,[4,5,6,3]]).as_matrix();w=np.einsum('nij,nkj->nki',r,self.ends)+p[:,:3,None].transpose(0,2,1)
        n=len(self.tool);a,b=w[:n],w[n:];ra,rb=self.r[:n],self.r[n:]
        # Conservative AABB shortlist, inflated 1 mm to log close approaches.
        lo=w.min(axis=1)-self.r[:,None]-.0005;hi=w.max(axis=1)+self.r[:,None]+.0005
        mask=np.all((lo[:n,None]<=hi[None,n:])&(lo[None,n:]<=hi[:n,None]),axis=2)
        ia,ib=np.where(mask)
        gap=segment_distance(a[ia,0],a[ia,1],b[ib,0],b[ib,1])-ra[ia]-rb[ib] if len(ia) else np.array([])
        if len(gap):
            k=gap.argmin();row=dict(time_s=time_s,gap_m=float(gap[k]),tool=self.tool[ia[k]]['path'],plant=self.plant[ib[k]]['path'])
            if self.worst is None or row['gap_m']<self.worst['gap_m']:self.worst=row
        else:row=dict(time_s=time_s,gap_m=None,tool=None,plant=None)
        row['invalid_penetration']=row['gap_m'] is not None and row['gap_m'] < -self.tolerance
        self.rows.append(row);return row

    def report(self):
        return dict(passed=not any(r['invalid_penetration'] for r in self.rows),sample_count=len(self.rows),tolerance_m=self.tolerance,
            worst=self.worst,violating_steps=sum(r['invalid_penetration'] for r in self.rows),scope='Every sampled physics step; tool arc and rails vs plant capsule/sphere shapes; not mesh/proximal housing collision certification')


def initialize_world_audits(world, directory, tolerance=.0005):
    """Export once, reuse geometry for independent clones; no scene reset."""
    import copy,json
    from connection_audit import export_connections
    export_connections(world.slots[0],directory,reset=False)
    model=json.loads((directory/'live_connections.json').read_text())
    for slot in world.slots:
        local=copy.deepcopy(model)
        for shape in local['shapes']:shape['path']=shape['path'].replace('/env_0/',f'/env_{slot.index}/')
        slot.tool_audit=ToolContactAudit(local,tolerance)
