from types import SimpleNamespace

import pytest

from gpu_step_profile import NativeStepProfile


def test_native_calls_are_forwarded_once_and_context_restored():
    calls = []
    native = SimpleNamespace(simulate=lambda *a, **k: calls.append((a, k)) or 7,
                             fetch_results=lambda: 9, untouched='native')
    context = SimpleNamespace(_physx_sim_interface=native)
    profile = NativeStepProfile(context)
    profile.start()
    assert context._physx_sim_interface.simulate(.01, current_time=3) == 7
    assert context._physx_sim_interface.fetch_results() == 9
    assert context._physx_sim_interface.untouched == 'native'
    profile.stop()
    assert context._physx_sim_interface is native
    assert calls == [((.01,), {'current_time': 3})]
    assert profile.timings['simulate']['calls'] == 1
    assert profile.timings['fetch_results']['calls'] == 1
    assert all(r['wall_s'] >= 0 for r in profile.timings.values())


def test_native_errors_are_not_swallowed():
    def fail():
        raise RuntimeError('native failure')
    profile = NativeStepProfile(SimpleNamespace(_physx_sim_interface=SimpleNamespace(fetch_results=fail)))
    with pytest.raises(RuntimeError, match='native failure'):
        profile.fetch_results()
    assert profile.timings['fetch_results']['calls'] == 1
