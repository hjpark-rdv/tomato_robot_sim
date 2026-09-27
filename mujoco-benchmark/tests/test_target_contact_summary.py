import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from summarize_target_contact_trial import force_interval


def test_force_duration_does_not_fill_missing_steps_or_zero_contact():
    rows=[dict(time_s=t,contact_categories_live={'fruit':dict(force_sum_N=f,max_normal_N=f/2)})
          for t,f in [(0.1,1.),(0.2,2.),(0.5,3.),(0.6,0.),(0.7,1.)]]
    result=force_interval(rows,'contact_categories_live','fruit',.1)
    assert result['observed_positive_elapsed_s']==pytest.approx(.1)
    assert result['max_time_s']==pytest.approx(.4)
    assert result['max_force_sum_N']==3.
    assert result['max_single_normal_N']==1.5


def test_single_positive_sample_is_not_a_hold_interval():
    row=dict(time_s=2.,contact_categories_private={'fruit':dict(force_sum_N=.2,max_normal_N=.1)})
    r=force_interval([row],'contact_categories_private','fruit',1/240)
    assert r['observed_positive_elapsed_s']==0.
    assert r['max_time_s']==2.
