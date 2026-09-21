"""Fail closed on native PhysX errors that otherwise only appear in Kit logs."""


def invalid_physics_message(source,message):
    source,message=source.lower(),message.lower()
    return ('physx' in source or 'physics.tensors' in source) and any(text in message for text in
        ('physx error:', 'simulation will miss interactions', 'cuda error', 'cuda kernel launch failed', 'buffer overflow'))
