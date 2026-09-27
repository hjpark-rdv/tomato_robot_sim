import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from search_legacy_baseline import evaluation_trace


def test_saved_legacy_commands_preserved_and_original_phases_untouched():
    plan=dict(parameters={'trajectory_mode':'staged6d'})
    rows=[dict(command=[i]*7,phase=p) for i,p in enumerate(['ready','entry','insert','rise','hold'])]
    new,meta=evaluation_trace(plan,rows,True)
    assert [r['command'] for r in new]==[r['command'] for r in rows]
    assert [r['phase'] for r in new]==['ready','approach','insert','seat','hold']
    assert rows[1]['phase']=='entry' and rows[3]['phase']=='rise'
    assert not meta['commands_changed']
    with pytest.raises(ValueError):evaluation_trace(plan,rows)


def test_unknown_legacy_phase_fails_closed():
    with pytest.raises(ValueError,match='Unknown'):
        evaluation_trace(dict(parameters={'trajectory_mode':'staged6d'}),[dict(phase='mystery')],True)
