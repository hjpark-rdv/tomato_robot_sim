"""Stage-wide units belong to each asset's root layer, not its sublayers."""
from pxr import UsdGeom, UsdPhysics


def stage_metrics(stage):
    return dict(meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(stage)),
                up_axis=str(UsdGeom.GetStageUpAxis(stage)),
                kilograms_per_unit=float(UsdPhysics.GetStageKilogramsPerUnit(stage)),
                time_codes_per_second=float(stage.GetTimeCodesPerSecond()),
                frames_per_second=float(stage.GetFramesPerSecond()))


def copy_stage_metrics(source, destination):
    values = stage_metrics(source)
    UsdGeom.SetStageMetersPerUnit(destination, values['meters_per_unit'])
    UsdGeom.SetStageUpAxis(destination, values['up_axis'])
    UsdPhysics.SetStageKilogramsPerUnit(destination, values['kilograms_per_unit'])
    destination.SetTimeCodesPerSecond(values['time_codes_per_second'])
    destination.SetFramesPerSecond(values['frames_per_second'])
    return values


def require_motion_units(stage):
    values = stage_metrics(stage)
    if (values['meters_per_unit'], values['up_axis'], values['kilograms_per_unit']) != (1., 'Z', 1.):
        raise ValueError('Motion adapter requires explicit metre/Z/kg stage metrics; '
                         'create a separate corrected asset, preserve the original: ' + str(values))
    return values
