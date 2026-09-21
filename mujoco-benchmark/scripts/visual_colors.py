"""Fallback display colours, classified by the mesh rather than its fruit parent."""
FRUIT = [0.8, 0.07, 0.03]
GREEN = [0.22, 0.45, 0.08]


def is_calyx_or_pedicel(path):
    name = path.rsplit('/', 1)[-1]
    return name.startswith(('TRUSS_Calyx_', 'TRUSS_Pedicel_')) or name == 'PedicelCollider'


def fallback_color(path):
    if is_calyx_or_pedicel(path):
        return GREEN.copy()
    return (FRUIT if '/Tomato_' in path else GREEN).copy()


def shape_color(shape):
    color = shape['color']
    # Older reference bundles assigned fruit red to every mesh under Tomato_NN.
    # Keep those measured input bundles intact; correct only their display colour.
    if is_calyx_or_pedicel(shape['path']) and color == FRUIT:
        return GREEN.copy()
    return color
