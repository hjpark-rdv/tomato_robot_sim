import pytest
from gpu_replication import environment_id_bits
from gpu_physics_errors import invalid_physics_message


def test_environment_bounds_id_space_includes_source_and_all_clones():
    for count in (1,16,64,128,1000,1024,32768):
        bits=environment_id_bits(count)
        assert 4<=bits<=16 and 2**bits>count
    with pytest.raises(ValueError):environment_id_bits(0)
    with pytest.raises(ValueError):environment_id_bits(65536)


def test_native_capacity_or_cuda_errors_invalidate_data_but_renderer_noise_does_not():
    error='PhysX error: The application needs to increase foundLostPairsCapacity, otherwise, the simulation will miss interactions'
    assert invalid_physics_message('omni.physx.plugin',error)
    assert invalid_physics_message('omni.physics.tensors.plugin','CUDA error 700')
    assert not invalid_physics_message('renderer.plugin','buffer overflow')
    assert not invalid_physics_message('omni.physx.plugin','CUDA kernel compilation completed')
