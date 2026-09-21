"""Share read-only articulation tensor queries between per-environment adapters."""


class BatchReadCache:
    # Only row-indexed, argument-free tensor reads; metadata stays on local views.
    GETTERS = frozenset(('get_dof_positions','get_dof_velocities','get_link_transforms',
                         'get_link_velocities','get_root_transforms','get_root_velocities'))

    def __init__(self, view):
        self.view = view
        self.values = {}

    def invalidate(self):
        self.values.clear()

    def get(self, name, index):
        if name not in self.values:
            self.values[name] = getattr(self.view, name)()
        return self.values[name][index:index + 1]


class BatchSliceView:
    """Delegate writes to the original view, share reads on the batch view.

    Every write invalidates the entire group's cache, including writes during
    reset. Environment indices passed to local setters keep their old meaning.
    """
    def __init__(self, original, cache, index):
        self.original, self.cache, self.index = original, cache, index

    def __getattr__(self, name):
        if name in self.cache.GETTERS:
            return lambda: self.cache.get(name, self.index)
        value = getattr(self.original, name)
        if callable(value) and (name.startswith('set_') or name.startswith('apply_')):
            def write(*args, **kwargs):
                self.cache.invalidate()
                return value(*args, **kwargs)
            return write
        return value
