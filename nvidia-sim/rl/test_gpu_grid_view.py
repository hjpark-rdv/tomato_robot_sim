from types import SimpleNamespace
import pytest
import gpu_grid_view as module
from gpu_grid_view import DisplayCadence, GridView


def test_refresh_budget_throttles_rendering_without_sleeping_physics():
    cadence=DisplayCadence(10,budget=.2)
    cadence.finish(0.,.08)
    assert not cadence.due(.1)
    assert cadence.due(.4)
    assert cadence.due(.1,force=True)
    cadence.finish(1.,1.001)
    assert not cadence.due(1.05)
    assert cadence.due(1.1)


@pytest.mark.parametrize('hz',[60,120,960])
def test_render_displays_current_state_and_uses_actual_physics_time(monkeypatch,hz):
    # A refresh must not replay the stale 960-Hz assumption of 16 steps/frame.
    viewer=GridView.__new__(GridView)
    calls=[]
    viewer.world=SimpleNamespace(slots=[None],step_timings={'steps':2},
        sim=SimpleNamespace(render=lambda:calls.append('render')))
    viewer.physics_dt=1./hz;viewer.cadence=DisplayCadence(10)
    viewer.last_state_step=0;viewer.request_capture=False;viewer.output=None
    viewer.header=SimpleNamespace(text='');viewer.footer=SimpleNamespace(text='')
    viewer.base=0;viewer.count=1;viewer.finished=False;viewer.frames=0
    viewer.timings=dict(frames=0,state_updates=0,geometry_s=0.,render_s=0.,total_s=0.)
    viewer._update_geometry=lambda:calls.append('geometry')
    now=[0.]
    monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    viewer.update(physics_step=True)
    assert calls==['geometry','render']
    assert f'Simulated {2/hz:.3f} s' in viewer.footer.text
    now[0]=.2;viewer.update()
    assert calls==['geometry','render','render']  # Idle UI avoids reskinning.
    viewer.update(force=True)
    assert calls[-2:]==['geometry','render']  # Capture/reset can force same-step state.
