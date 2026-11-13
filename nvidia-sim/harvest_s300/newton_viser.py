"""Read-only Viser display of the existing Newton latest-frame stream.

No Isaac/Kit dependency, simulator stepping, trajectory replay, or video writer.
Only the browser acknowledgement requests render images (kept in memory).
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import threading
import time

import numpy as np

from motion_live import MotionLiveReader
from newton_live import _write_json, write_display_ack, write_interactive_start


def body_mapping(model, live_report):
    mapping = live_report['mapping']
    if mapping.get('bodies_with_frame_offsets'):
        raise ValueError('Native mesh viewer requires identical USD/native body frames')
    result = {}
    for path, row in mapping['usd_to_native'].items():
        body = model.body(row['name'])
        if body.id in result:
            raise ValueError('Two display paths map to the same native body')
        result[body.id] = path
    required = {int(model.geom_bodyid[i]) for i in range(model.ngeom)} - {0}
    missing = required - set(result)
    if missing:
        raise ValueError('Visible body mapping missing: ' + str(sorted(missing)))
    return result


def geom_mesh(model, i, *, local_transform=True):
    """Preserve compiled mesh vertices and local geom transform, in Z-up metres."""
    import mujoco as mj
    import trimesh
    from scipy.spatial.transform import Rotation
    kind = model.geom_type[i]
    size = model.geom_size[i]
    if kind == mj.mjtGeom.mjGEOM_MESH:
        m = int(model.geom_dataid[i])
        va, vn = model.mesh_vertadr[m], model.mesh_vertnum[m]
        fa, fn = model.mesh_faceadr[m], model.mesh_facenum[m]
        mesh = trimesh.Trimesh(model.mesh_vert[va:va+vn].copy(),
                               model.mesh_face[fa:fa+fn].copy(), process=False)
    elif kind == mj.mjtGeom.mjGEOM_BOX:
        mesh = trimesh.creation.box(extents=2*size)
    elif kind == mj.mjtGeom.mjGEOM_SPHERE:
        mesh = trimesh.creation.icosphere(subdivisions=2, radius=size[0])
    elif kind == mj.mjtGeom.mjGEOM_ELLIPSOID:
        mesh = trimesh.creation.icosphere(subdivisions=2); mesh.apply_scale(size)
    elif kind == mj.mjtGeom.mjGEOM_CAPSULE:
        mesh = trimesh.creation.capsule(height=2*size[1], radius=size[0])
    elif kind == mj.mjtGeom.mjGEOM_CYLINDER:
        mesh = trimesh.creation.cylinder(height=2*size[1], radius=size[0])
    elif kind == mj.mjtGeom.mjGEOM_PLANE:
        mesh = trimesh.creation.box(extents=[max(2*size[0], 10), max(2*size[1], 10), .002])
        mesh.apply_translation([0, 0, -.001])
    else:
        raise ValueError(f'Unsupported display geometry {kind}: {model.geom(i).name}')
    q = model.geom_quat[i]
    transform = np.eye(4)
    transform[:3,:3] = Rotation.from_quat(q[[1,2,3,0]]).as_matrix()
    transform[:3,3] = model.geom_pos[i]
    if local_transform:
        mesh.apply_transform(transform)
    authored = getattr(model, 'vertex_color_lookup', {}).get(model.geom(i).name)
    if authored is not None:
        if np.asarray(authored).shape != (len(mesh.vertices), 4):
            raise ValueError('Authored vertex color count mismatch: '+model.geom(i).name)
        mesh.visual.vertex_colors = authored
    else:
        mesh.visual.vertex_colors = np.tile(np.clip(geom_color(model,i)*255, 0, 255).astype(np.uint8), (len(mesh.vertices),1))
    return mesh


def geom_color(model, index):
    material = int(model.geom_matid[index])
    return model.mat_rgba[material] if material >= 0 else model.geom_rgba[index]


def populate_scene(server,model,*,include_static=True,include_dynamic=True,static_root='/farm',shadows=True):
    """Shared full-triangle browser geometry for single and patrol viewers."""
    import trimesh
    handles = {}
    # Static full-farm instances share one mesh upload; no triangle decimation.
    static = {}
    for i in range(model.ngeom) if include_static else ():
        if model.geom_bodyid[i] != 0 or geom_color(model,i)[3] <= 0: continue
        if model.geom(i).name in getattr(model, 'vertex_color_lookup', {}):
            server.scene.add_mesh_trimesh(f'{static_root}/colored_g{i}',
                geom_mesh(model,i,local_transform=False),
                wxyz=model.geom_quat[i],position=model.geom_pos[i],cast_shadow=shadows,receive_shadow=shadows)
            continue
        key = (int(model.geom_type[i]), int(model.geom_dataid[i]), tuple(model.geom_size[i]))
        static.setdefault(key, []).append(i)
    for index,ids in enumerate(static.values()):
        mesh = geom_mesh(model, ids[0], local_transform=False)
        server.scene.add_batched_meshes_simple(f'{static_root}/g{index}',mesh.vertices.astype(np.float32),mesh.faces,
            model.geom_quat[ids],model.geom_pos[ids],
            batched_colors=np.clip(np.array([geom_color(model,i)[:3] for i in ids])*255,0,255).astype(np.uint8),
            batched_opacities=np.array([geom_color(model,i)[3] for i in ids]),lod='off',side='double',
            cast_shadow=shadows,receive_shadow=shadows)
    for bid in sorted({int(x) for x in model.geom_bodyid} - {0}) if include_dynamic else ():
        meshes = [geom_mesh(model,i) for i in range(model.ngeom)
                  if model.geom_bodyid[i] == bid and geom_color(model,i)[3] > 0]
        if not meshes:
            continue
        handles[bid] = server.scene.add_mesh_trimesh(f'/bodies/b{bid}', trimesh.util.concatenate(meshes),
            cast_shadow=shadows,receive_shadow=shadows)
    return handles


def run(args):
    import mujoco as mj
    import trimesh
    import viser
    reader = MotionLiveReader(args.trial)
    reader.verify_source()
    live = json.loads((args.trial/'newton_live.json').read_text())
    inputs = Path(os.environ['S300_VISER_INPUTS']).resolve(strict=True)
    provenance = json.loads((inputs/'inputs.json').read_text())
    from newton_inputs import sha256
    if reader.metadata['physical_model_sha256'] != sha256(inputs/'input_model.mjb'):
        raise ValueError('Viewer/physics input model mismatch')
    # Full visual export, while the producer can use its validated render-free MJB.
    export = Path(provenance['model_directory'])
    source = json.loads((export/'export.json').read_text())
    xml = export/'scene.xml'
    if sha256(xml) != source['model_sha256']:
        raise ValueError('Display export XML hash mismatch')
    model = mj.MjModel.from_xml_path(str(xml))
    mapping = body_mapping(model, live)
    port = int(os.environ.get('S300_VISER_PORT', '8091'))
    server = viser.ViserServer(host='0.0.0.0', port=port, label='Farmily · Newton GPU', verbose=False)
    if server.get_port() != port:
        server.stop()
        raise RuntimeError(f'Viewer port {port} is busy; specify another --viewer-port')
    server.scene.set_up_direction('+z')
    server.scene.add_grid('/ground-grid', plane='xy', width=10, height=20)
    status = server.gui.add_markdown('**Newton GPU · 실시간 물리**\n\n장면을 준비하고 있습니다.')
    start_button = server.gui.add_button('시작 · 1배속', disabled=True)
    close_button = server.gui.add_button('뷰어 닫기')
    quit_event = threading.Event()
    close_button.on_click(lambda _: quit_event.set())
    control_root = args.trial
    current = {'snapshot': None}
    def start_or_repeat(_):
        snapshot = current['snapshot']
        if snapshot is not None and snapshot['status'] in ('completed','stopped'):
            _write_json(args.trial/'interactive_restart.json',dict(requested=True))
            start_button.disabled=True
        else:
            write_interactive_start(args.trial,reader.metadata['model_sha256'],
                                    render_updates=1,source='Viser browser START')
            start_button.disabled=True
    start_button.on_click(start_or_repeat)
    handles = populate_scene(server,model)
    indices = {bid: reader.body_paths.index(path) for bid,path in mapping.items()}
    def update(snapshot):
        with server.atomic():
            for bid,handle in handles.items():
                if bid == 0: continue
                pose = snapshot['poses'][indices[bid]]
                handle.position = pose[:3]
                handle.wxyz = pose[[6,3,4,5]]
    first = reader.read()
    if first is None: raise RuntimeError('Missing initial measured frame')
    update(first)
    fruits = [indices[bid] for bid in indices if 'Fruit_' in model.body(bid).name]
    focus = first['poses'][fruits,:3].mean(axis=0) if fruits else first['poses'][:,:3].mean(axis=0)
    @server.on_client_connect
    def on_connect(client):
        client.camera.position = focus + [1.0, -.65, .4]
        client.camera.look_at = focus
        client.camera.up_direction = (0.,0.,1.)
    # The first actual browser render must finish before interactive physics starts.
    print(f'Newton live viewer: http://localhost:{port}', flush=True)
    status.content = '**Newton GPU · 준비 완료**\n\n카메라를 조작한 뒤 시작 버튼을 누르세요.'
    ready_written = False
    autostart = False
    last_seq, displayed = -1, 0
    last_ack = 0.
    started = time.monotonic()
    while not quit_event.is_set() and (args.seconds == 0 or time.monotonic()-started < args.seconds):
        next_file=control_root/'viser_next_trial.json'
        if next_file.exists():
            next_trial=Path(json.loads(next_file.read_text())['trial'])
            if next_trial != args.trial:
                candidate=MotionLiveReader(next_trial)
                if (candidate.metadata['model_sha256']!=reader.metadata['model_sha256'] or
                    candidate.metadata['physical_model_sha256']!=reader.metadata['physical_model_sha256'] or
                    candidate.body_paths!=reader.body_paths):
                    raise ValueError('Persistent display model/body mapping changed')
                candidate.verify_source()
                args.trial=next_trial;args.ready_file=next_trial/'newton_viewer_ready.json'
                reader=candidate;ready_written=False;last_seq=-1;displayed=0
                autostart=True
                start_button.label='시작 · 1배속';start_button.disabled=True
        snapshot = reader.read()
        current['snapshot']=snapshot
        if snapshot is not None and snapshot['sequence'] != last_seq:
            update(snapshot); last_seq = snapshot['sequence']; displayed += 1
            status.content = f"**Newton GPU · {snapshot['status']}**\n\n시뮬레이션 {snapshot['time']:.2f}초 · 그리퍼 {snapshot['gap']*1000:.1f}mm"
            if snapshot['status'] in ('completed','stopped'):
                start_button.label='초기화 후 다시 실행';start_button.disabled=False
        clients = list(server.get_clients().values())
        if snapshot is not None and clients and (not ready_written or time.monotonic()-last_ack > 1.):
            # A real client reply, not a websocket enqueue, is evidence of rendering.
            try:
                clients[0].get_render(height=240, width=320, timeout=5.)
            except (RuntimeError, TimeoutError):
                # A closed/slow browser never stops or restarts the producer.
                last_ack = time.monotonic()
                continue
            write_display_ack(args.trial, snapshot, displayed_count=displayed,
                              render_updates=displayed, timeline_time_s=0., physics_steps=0)
            last_ack = time.monotonic()
            if not ready_written:
                _write_json(args.ready_file, dict(pid=os.getpid(), physics_steps=0,
                    model_sha256=reader.metadata['model_sha256'], initial_viewport_frame_verified=True,
                    viewer='viser', url=f'http://localhost:{port}', frame_verification='browser get_render reply'))
                start_button.disabled = False
                ready_written = True
                if autostart:
                    write_interactive_start(args.trial,reader.metadata['model_sha256'],
                        render_updates=displayed,source='Viser browser requested repeat; initial frame verified')
                    start_button.disabled=True;autostart=False
        time.sleep(1./args.fps)
    server.stop()


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial', type=Path)
    p.add_argument('--fps', type=float, default=15.)
    p.add_argument('--seconds', type=float, default=0.)
    p.add_argument('--ready-file', type=Path, required=True)
    p.add_argument('--camera', default='entry')
    p.add_argument('--interactive', action='store_true')
    p.add_argument('--review-light', action='store_true')
    p.add_argument('--camera-test-motion', action='store_true')
    p.add_argument('--capture', type=Path)
    args = p.parse_args()
    if args.camera_test_motion:
        p.error('Use a moving browser camera for Viser tests; Isaac camera test is not interchangeable')
    run(args)
