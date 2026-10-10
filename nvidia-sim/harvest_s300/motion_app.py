"""Application-only bootstrap options for the existing native motion executor.

The installed Isaac 6.1 official ``test_headless_no_rendering.py`` provides the
physics-only bootstrap: explicitly ``experience=None`` and a small extension
list. An empty experience string instead auto-loads the regular RTX app.
No scene geometry, solver, contacts, controller, or simulation API changes here.
"""
import os
from pathlib import Path


def configure_scene_device(stage, device, scene_path='/World/PhysicsScene', *, broadphase='auto'):
    """Configure native dynamics and the independently selected broadphase.

    SimulationManager.set_device relies on a populated USD-stage event cache.
    The official minimal app can open a stage before that cache is populated;
    a CPU tensor backend alone then does not turn authored GPU dynamics off.
    Apply the same native attributes as the SDK's CPU/GPU device setter and
    report the effective composed values. This edits the runtime stage only.

    ``auto`` preserves the existing CPU/MBP and CUDA/GPU combinations. PhysX
    also supports CPU dynamics with GPU broadphase; that opt-in combination
    does not move contact generation or constraint solving onto the GPU. Set
    these attributes after SimulationManager.set_device and before native
    scene initialization, because the device setter resets both attributes.

    Official independent-pipeline description:
    https://nvidia-omniverse.github.io/PhysX/physx/5.7.0/docs/GPURigidBodies.html
    """
    from pxr import PhysxSchema, UsdPhysics
    if device not in ('cpu', 'cuda:0'):
        raise ValueError('Unsupported motion device: '+str(device))
    if broadphase not in ('auto', 'MBP', 'SAP', 'GPU'):
        raise ValueError('Unsupported motion broadphase: '+str(broadphase))
    prim = stage.GetPrimAtPath(scene_path)
    if not prim or not prim.IsA(UsdPhysics.Scene):
        raise ValueError('Missing native physics scene: '+scene_path)
    api = PhysxSchema.PhysxSceneAPI.Apply(prim)
    gpu = device.startswith('cuda')
    expected_broadphase = ('GPU' if gpu else 'MBP') if broadphase == 'auto' else broadphase
    api.CreateEnableGPUDynamicsAttr(gpu)
    api.CreateBroadphaseTypeAttr(expected_broadphase)
    actual = dict(scene=scene_path, enabled_gpu_dynamics=api.GetEnableGPUDynamicsAttr().Get(),
                  broadphase_type=api.GetBroadphaseTypeAttr().Get(),
                  requested_device=device, requested_broadphase=broadphase,
                  dynamics_backend='GPU' if gpu else 'CPU',
                  broadphase_backend='GPU' if expected_broadphase == 'GPU' else 'CPU')
    if actual['enabled_gpu_dynamics'] != gpu or actual['broadphase_type'] != expected_broadphase:
        raise RuntimeError('Native scene device mismatch: '+str(actual))
    return actual


def configure_gpu_pair_capacities(stage, capacity, scene_path='/World/PhysicsScene'):
    """Set native GPU aggregate-pair allocation limits before scene initialization.

    These two capacities reserve buffers; they do not change collision geometry,
    contact policy, or solver parameters. Record the effective composed values so
    an override in a stronger layer cannot silently defeat the requested limits.
    """
    from pxr import PhysxSchema, UsdPhysics
    if isinstance(capacity, bool) or not isinstance(capacity, int) or not 0 < capacity <= 2**31-1:
        raise ValueError('GPU pair capacity must be a positive integer at most 2147483647')
    prim = stage.GetPrimAtPath(scene_path)
    if not prim or not prim.IsA(UsdPhysics.Scene):
        raise ValueError('Missing native physics scene: '+scene_path)
    api = PhysxSchema.PhysxSceneAPI.Apply(prim)
    api.CreateGpuFoundLostAggregatePairsCapacityAttr(capacity)
    api.CreateGpuTotalAggregatePairsCapacityAttr(capacity)
    actual = dict(scene=scene_path, requested_capacity=capacity,
                  found_lost_aggregate_pairs_capacity=api.GetGpuFoundLostAggregatePairsCapacityAttr().Get(),
                  total_aggregate_pairs_capacity=api.GetGpuTotalAggregatePairsCapacityAttr().Get())
    if (actual['found_lost_aggregate_pairs_capacity'] != capacity
            or actual['total_aggregate_pairs_capacity'] != capacity):
        raise RuntimeError('Native GPU pair capacity mismatch: '+str(actual))
    return actual


def configure_contact_readback(settings, device, backend):
    """Restore contact-backend readback after the final SDK device selection.

    The SDK's CUDA device setter suppresses native readback. Collider contact
    callbacks require it enabled, including after a cold stage reopen. Only the
    opt-in CUDA tensor backend suppresses it; CPU always retains readback.
    """
    if device not in ('cpu', 'cuda:0'):
        raise ValueError('Unsupported motion device: '+str(device))
    if backend not in ('callback', 'tensor'):
        raise ValueError('Unsupported motion contact backend: '+str(backend))
    expected = device == 'cuda:0' and backend == 'tensor'
    path = '/physics/suppressReadback'
    settings.set_bool(path, expected)
    actual = dict(requested_device=device, contact_backend=backend,
                  suppress_readback=settings.get_as_bool(path))
    if actual['suppress_readback'] != expected:
        raise RuntimeError('Native contact readback mismatch: '+str(actual))
    return actual


# Reuse the official no-rendering sample's PhysX/Fabric/USD bootstrap. The sample
# spells the loop extension omni.kit.loop; this installation provides its Isaac
# implementation as omni.kit.loop-isaac. The existing executor also needs the
# SimulationManager and the SDK's Torch archive.
PHYSICS_ONLY_EXTENSIONS = (
    'omni.physx',
    'omni.physx.tensors',
    'omni.physx.fabric',
    'omni.warp.core',
    'usdrt.scenegraph',
    'omni.kit.telemetry',
    'omni.kit.loop-isaac',
    'omni.kit.usd.mdl',
    'omni.usd.metrics.assembler.ui',
    'omni.hydra.usdrt_delegate',
    'isaacsim.core.simulation_manager',
    'omni.isaac.ml_archive',
)

# Preserve the corresponding regular Isaac base experience's lifecycle/timeline
# settings. These do not select a solver, timestep, collision policy, or device.
# base.kit authors these physics settings below [settings.persistent], hence the
# persistent paths here rather than introducing unrelated global settings.
PHYSICS_ONLY_APP_ARGS = (
    '--/app/settings/persistent=false',
    '--/app/events/qualifiedNames=true',
    '--/app/runLoops/main/manualModeEnabled=true',
    '--/app/runLoops/main/rateLimitEnabled=false',
    '--/app/player/useFixedTimeStepping=false',
    '--/app/settings/fabricDefaultStageFrameHistoryCount=3',
    '--/app/useFabricSceneDelegate=true',
    '--/persistent/physics/resetOnStop=true',
    '--/persistent/physics/visualizationDisplayJoints=false',
    '--/persistent/physics/visualizationSimulationOutput=false',
)


def motion_app_options(*, headless, width=1280, height=800,
                       performance='optimized', physics_only=False, sdk_path=None):
    """Return launch options without importing or starting Kit.

    CPU device selection remains in the existing executor. A physics-only caller
    must not request a GUI; use the separate measured-pose viewer instead. The
    normal path deliberately retains the previous exact SimulationApp config.

    Physics-only excludes RTX rendering extensions. GPU Foundation is still a
    required PhysX/USDRT dependency, so this does not promise zero GPU memory.
    """
    config = dict(headless=headless, width=width, height=height,
                  renderer='RaytracedLighting',
                  disable_viewport_updates=headless and performance == 'optimized')
    if not physics_only:
        return {'config': config}
    if not headless:
        raise ValueError('physics-only requires headless; use the separate live viewer for display')
    root = sdk_path or os.environ.get('ISAAC_PATH')
    if not root:
        raise ValueError('ISAAC_PATH must identify the installed Isaac SDK for physics-only bootstrap')
    root = Path(root).resolve()
    folders = [root/'exts', root/'extscache', root/'extsDeprecated']
    missing = [str(p) for p in folders if not p.is_dir()]
    if missing:
        raise ValueError('Missing Isaac extension folders: '+', '.join(missing))
    extra = []
    for folder in folders:
        extra.extend(('--ext-folder', str(folder)))
    for extension in PHYSICS_ONLY_EXTENSIONS:
        extra.extend(('--enable', extension))
    extra.extend(PHYSICS_ONLY_APP_ARGS)
    config.update(create_new_stage=False, multi_gpu=False,
                  disable_viewport_updates=False, extra_args=extra)
    # Explicit None is essential: SimulationApp(experience="") loads base.python.
    return {'config': config, 'experience': None}


def create_motion_app(simulation_app_class, *, headless, width=1280, height=800,
                      performance='optimized', physics_only=False):
    """Create the usual SimulationApp or its opt-in official physics bootstrap."""
    options = motion_app_options(headless=headless, width=width, height=height,
                                 performance=performance, physics_only=physics_only)
    config = options['config']
    if physics_only:
        return simulation_app_class(config, experience=None)
    return simulation_app_class(config)
