"""Scoped compatibility settings for Isaac's supported Newton USD importer."""

from __future__ import annotations

from contextlib import contextmanager


def restore_loop_endpoint_roots(builder) -> list[dict]:
    """Represent non-articulated dynamic loop endpoints with free world roots.

    OpenUSD excludes these bodies from the articulation tree. The source rigid
    body still moves freely subject to its separate loop joint; retaining only
    the loop joint would leave no joint-to-world creation path in SolverMuJoCo.
    Existing articulation members and existing free world roots are untouched.
    """
    owned, loop_endpoints = set(), set()
    for parent, child, articulation in zip(builder.joint_parent, builder.joint_child,
                                           builder.joint_articulation, strict=True):
        endpoints = {int(x) for x in (parent, child) if x >= 0}
        if articulation >= 0 or parent == -1:
            owned.update(endpoints)
        else:
            loop_endpoints.update(endpoints)
    restored = []
    for body in sorted(loop_endpoints - owned):
        if builder.body_mass[body] <= 0:
            continue
        label = str(builder.body_label[body])
        joint = builder.add_joint_free(child=body, parent=-1,
                                       label=label + "/native_import_free_root")
        builder.add_articulation([joint])
        restored.append({"body_index": body, "body": label, "joint_index": joint})
    return restored


@contextmanager
def preserve_unordered_bodies(builder_type, enabled: bool):
    """Keep loop-only body endpoints using the public USD parser option.

    Isaac6.1's stage does not expose ``bodies_follow_joint_ordering`` in its
    config, although its bundled Newton add_usd API supports it. Scoping this
    override avoids editing either installed packages or source USD joints.
    """
    audit = {"enabled": bool(enabled), "bodies_follow_joint_ordering": not enabled,
             "added_free_world_roots": []}
    if not enabled:
        yield audit
        return
    original = builder_type.add_usd

    def add_usd(builder, *args, **kwargs):
        kwargs["bodies_follow_joint_ordering"] = False
        result = original(builder, *args, **kwargs)
        audit["added_free_world_roots"].extend(restore_loop_endpoint_roots(builder))
        return result

    builder_type.add_usd = add_usd
    try:
        yield audit
    finally:
        builder_type.add_usd = original
