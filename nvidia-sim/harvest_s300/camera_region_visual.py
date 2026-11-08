"""Show a chosen harvest group on its existing visible-pixel GT observation.

This module needs only NumPy/Pillow; it never opens a simulator or changes a
source RGB image, mask, recognition overlay, or three-panel analysis image.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from recording_storage import load_array, original_bytes
from PIL import Image, ImageDraw, ImageFont


REGION_COLOR = (255, 40, 215)
ANALYSIS_HEADER_PX = 30


def _json_value(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'Cannot serialize {type(value).__name__}')


def _inside_boundary(mask):
    """One-pixel four-neighbor contour contained entirely in the input mask."""
    interior = mask.copy()
    interior[0, :] = interior[-1, :] = False
    interior[:, 0] = interior[:, -1] = False
    interior[1:, :] &= mask[:-1, :]
    interior[:-1, :] &= mask[1:, :]
    interior[:, 1:] &= mask[:, :-1]
    interior[:, :-1] &= mask[:, 1:]
    return mask & ~interior


def annotate_region(directory, group):
    """Write harvest_region.png, analysis_region.png and region.json.

    Group names are joined to accepted camera observations by exact name, then
    to the CURRENT observation by exact stable instance ID and object key.
    An object absent from the current mask stays absent; no bounding box,
    hidden silhouette, camera projection, or inferred fruit center is painted.
    """
    directory = Path(directory)
    group = json.loads(json.dumps(group, default=_json_value, allow_nan=False))
    names = group.get('fruits', [])
    if not isinstance(names, list) or any(not isinstance(name, str) for name in names):
        raise ValueError('Selected fruit names must be a list of strings')
    if len(names) != len(set(names)):
        raise ValueError('Duplicate selected fruit name')
    camera_region = group.get('camera_region') or {}
    accepted = camera_region.get('accepted_observations', [])
    if not isinstance(accepted, list):
        raise ValueError('Accepted camera observations must be a list')

    objects_path = directory / 'objects.json'
    observed = json.loads(objects_path.read_text())
    rows = observed.get('objects', [])
    if not isinstance(rows, list):
        raise ValueError('Visible objects list required')
    by_id = {}
    for row in rows:
        identity = row['instance_id']
        if identity in by_id:
            raise ValueError('Duplicate visible instance ID')
        by_id[identity] = row

    instance = load_array(directory / 'instance.npy')
    if instance.ndim != 2 or not np.issubdtype(instance.dtype, np.integer):
        raise ValueError('Two-dimensional integer visible instance mask required')
    recognition = Image.open(directory / 'recognition.png').convert('RGB')
    w, h = recognition.size
    if instance.shape != (h, w):
        raise ValueError('Recognition and instance mask dimensions differ')
    analysis = Image.open(directory / 'analysis.png').convert('RGB')
    if analysis.size != (w * 3, h + ANALYSIS_HEADER_PX):
        raise ValueError('Expected three aligned panels below a 30-pixel header')

    selected = {}
    for row in accepted:
        name = row.get('name')
        if name not in names:
            continue
        identity = row.get('instance_id')
        if (isinstance(identity, bool) or not isinstance(identity, int) or
                not 0 < identity <= np.iinfo(np.uint32).max or not isinstance(row.get('key'), str)):
            raise ValueError('Exact accepted fruit name/key/instance ID required')
        parts = name.rsplit('_Fruit_', 1)
        if len(parts) != 2 or not row['key'].endswith(f'/{parts[0]}/Fruit_{parts[1]}'):
            raise ValueError('Selected fruit name and accepted object key disagree')
        if name in selected or any(item['instance_id'] == identity for item in selected.values()):
            raise ValueError('Duplicate accepted selected fruit mapping')
        current = by_id.get(identity)
        if current is not None and (current.get('key') != row['key'] or current.get('class_name') != 'fruit'):
            raise ValueError('Selected fruit identity disagrees with current visible object')
        if (current is not None and group.get('truss_key') is not None and
                current.get('truss') != group['truss_key']):
            raise ValueError('Selected fruit belongs to a different truss')
        selected[name] = row

    visible = []
    boundary = np.zeros(instance.shape, bool)
    for name in names:
        row = selected.get(name)
        if row is None:
            continue
        identity = row['instance_id']
        mask = instance == identity
        if not np.any(mask):
            continue
        if identity not in by_id:
            raise ValueError('Selected visible mask ID is absent from objects.json')
        boundary |= _inside_boundary(mask)
        ys, xs = np.where(mask)
        visible.append(dict(name=name, key=row['key'], instance_id=identity,
                            visible_pixels=int(mask.sum()),
                            bbox_xyxy=[int(xs.min()), int(ys.min()), int(xs.max() + 1), int(ys.max() + 1)]))

    pixels = np.array(recognition)
    pixels[boundary] = REGION_COLOR
    highlighted = Image.fromarray(pixels)
    draw = ImageDraw.Draw(highlighted)
    size = max(8, min(11, w // 40))
    try:
        font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', size)
    except OSError:
        font = ImageFont.load_default()
    label = f'Harvest region: {len(visible)}'
    # The legend is separate from the one-pixel fruit contours. Keep it small.
    box = draw.textbbox((4, 3), label, font=font)
    draw.rectangle((2, 1, min(w - 1, box[2] + 3), box[3] + 2), fill=(18, 22, 25))
    draw.text((4, 3), label, font=font, fill=REGION_COLOR)
    analysis.paste(highlighted, (w, ANALYSIS_HEADER_PX))

    observation = camera_region.get('observation') or {}
    metadata = dict(schema='s300-visible-harvest-region-v1',
        timestamp_s=observed.get('timestamp_s'),
        selected_fruits=names, selected_instance_ids=[selected[name]['instance_id'] for name in names if name in selected],
        visible_selected_instance_ids=[row['instance_id'] for row in visible],
        visible_selected_objects=visible, visible_selected_count=len(visible),
        currently_absent_selected_fruits=[name for name in names if name not in {row['name'] for row in visible}],
        unmapped_selected_fruits=[name for name in names if name not in selected],
        group=group, truss=group.get('truss_key'), target_aim_world_m=group.get('aim_world_m'),
        belt_aim_world_m=group.get('group_selection', {}).get('belt_aim_world_m'),
        selection_observation=observation,
        selection_observation_path=observation.get('observation_path') or observation.get('objects_path') or observation.get('path'),
        visible_pixels_only=True, hidden_masks_fabricated=False,
        contour='one-pixel inner boundary of existing selected instance masks only',
        legend='Harvest region count is the number of selected fruit currently visible',
        color_rgb=list(REGION_COLOR), boundary_pixels=int(boundary.sum()), training_eligible=False,
        source_sha256={name: hashlib.sha256(original_bytes(directory / name)).hexdigest()
                       for name in ('recognition.png', 'analysis.png', 'objects.json', 'instance.npy')})
    highlighted.save(directory / 'harvest_region.png')
    analysis.save(directory / 'analysis_region.png')
    (directory / 'region.json').write_text(json.dumps(metadata, indent=2, allow_nan=False) + '\n')
    return dict(recognition='harvest_region.png', analysis='analysis_region.png',
                region='region.json', visible_selected_count=len(visible),
                selected_instance_ids=metadata['selected_instance_ids'],
                visible_selected_instance_ids=metadata['visible_selected_instance_ids'])
