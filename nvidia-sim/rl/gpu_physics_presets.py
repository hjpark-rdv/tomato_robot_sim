"""Explicit physical models for throughput experiments, not calibrated plants."""
PRESETS = {
    'reference960': dict(physics_hz=960, elastic_joint_armature=1e-5, position_iterations=64),
    'practical120': dict(physics_hz=120, elastic_joint_armature=3e-4, position_iterations=64),
    'practical60': dict(physics_hz=60, elastic_joint_armature=5e-4, position_iterations=64),
    # Keep the practical60 plant inertia while halving the contact timestep.
    # Candidate 49 crossed a rachis at 60 Hz; practical120 uses another inertia.
    'contact120': dict(physics_hz=120, elastic_joint_armature=5e-4, position_iterations=64),
}


def resolve(args):
    """Explicit numerical flags override the preset and are saved in config."""
    for key,value in PRESETS[args.physics_preset].items():
        if getattr(args,key) is None:setattr(args,key,value)
