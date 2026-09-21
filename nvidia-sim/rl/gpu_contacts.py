"""GPU contact API experiments, separate from production label generation."""
from pxr import Usd, UsdPhysics


def inspect_contact_views(world):
    root=world.slots[0].root
    api=world.sim.physics_sim_view
    ring=root+'/Robot/link6/tcp/tomato_gripper/RingCollision/segment_20'
    fruit=world.slots[0].target_spec['path']
    reports=[]
    world._diagnostic_contact_views=[]
    for sensor,filters in [(ring,[fruit+'/FruitCollider']),
                           (root+'/Robot/link6',[fruit]),
                           (root+'/Robot/link6',[fruit+'/FruitCollider'])]:
        try:
            view=api.create_rigid_contact_view(sensor,filters,max_contact_data_count=1024)
            world._diagnostic_contact_views.append(view)
            reports.append(dict(sensor=sensor,filters=filters,sensor_count=view.sensor_count,
                filter_count=view.filter_count,sensor_paths=view.sensor_paths,filter_paths=view.filter_paths))
        except Exception as e:
            reports.append(dict(sensor=sensor,filters=filters,error=str(e)))
    return reports
