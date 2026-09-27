import copy
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from retime_seating_trace import retime


def test_retiming_preserves_original_vertices_and_piecewise_linear_path():
    rows=[dict(command=[i,i*i],phase=p) for i,p in enumerate(['insert','seat','seat','hold','verify'])]
    before=copy.deepcopy(rows);new,indices=retime(rows,3)
    assert rows==before
    assert [new[i] for i in indices]==rows
    assert indices[-1]-indices[-2]==1
    for a,b,lo,hi in zip(rows,rows[1:],indices,indices[1:]):
        for j in range(lo,hi+1):
            f=(j-lo)/(hi-lo)
            np.testing.assert_allclose(new[j]['command'],np.array(a['command'])*(1-f)+np.array(b['command'])*f)


def test_factor_one_preserves_every_command_and_phase():
    rows=[dict(command=[0.,1.],phase='seat'),dict(command=[2.,3.],phase='hold')]
    assert retime(rows,1)[0]==rows
