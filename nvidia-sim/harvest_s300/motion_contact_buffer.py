"""Buffer native contact values while preserving the per-step validity guard.

Contact records retain exact native collider IDs, signed separation, impulse,
timestamp and phase. Normal callbacks avoid force norms, event dictionaries and
impulse lists; legacy event formatting occurs after the measured physics loop.
Raw buffering preserves native physics; an optional pair-specific stop threshold
changes diagnostic acceptance only, never contact generation or response.
"""
from __future__ import annotations

import math
import sys
from typing import NamedTuple


ACTIVE_PATH_MARKERS = ('/ActuatedTool/', '/HarvestRobot/', '/ElasticTarget/', '/ElasticNeighbors/')


class RawContact(NamedTuple):
    t: float
    phase: str
    collider0: int
    collider1: int
    separation_m: float
    impulse_ns: tuple[float, float, float]


class FramedRawContact(NamedTuple):
    t: float
    phase: str
    collider0: int
    collider1: int
    separation_m: float
    impulse_ns: tuple[float, float, float]
    position_world_m: tuple[float, float, float]
    normal_world: tuple[float, float, float]


def _native_slice_triplet(value, native_float3_type):
    """Copy one exact carb.Float3 through its native three-value slice.

    The installed binding declares Float3[slice] -> list[float]. Unlike its
    Python sequence iteration, the slice copies all three C++ components in one
    binding call. Unknown vector classes and subclasses retain the original
    indexed/float conversion. Never retain a callback-owned vector or view.
    """
    if native_float3_type is not None and type(value) is native_float3_type:
        return tuple(value[:])
    return (float(value[0]),float(value[1]),float(value[2]))


class BufferedContactRecorder:
    """Native callback adapter with a default 0.5 mm active-contact guard.

    A supplied pair_limit overrides only the diagnostic stop threshold. All raw
    records, including tolerated overlaps, are retained.

    Call ``clear_step`` immediately before each simulate/fetch operation. During
    callback delivery call ``record(headers, data, t=..., phase=...)``. Afterwards
    ``step_invalid_events`` returns the same violating legacy events as the
    original callback/guard. ``materialize`` preserves full contact order and
    values for the existing contacts.json and summary code after the loop.

    Initialization callbacks may be collected before the first ``clear_step``;
    their timestamp/phase are retained and can be materialized independently.
    The decoder must return the same exact collider paths as the native runner.
    """
    def __init__(self, path_decoder, hz, *, active_path_markers=ACTIVE_PATH_MARKERS,
                 penetration_limit_m=0.0005, capture_frames=False, pair_limit=None,
                 native_triplet_slice=False):
        hz=float(hz)
        if not math.isfinite(hz) or hz<=0:
            raise ValueError('Physics frequency must be positive and finite')
        limit=float(penetration_limit_m)
        if not math.isfinite(limit) or limit<0:
            raise ValueError('Penetration limit must be nonnegative and finite')
        self.hz=hz
        self.penetration_limit_m=limit
        self.capture_frames=bool(capture_frames)
        self._pair_limit=pair_limit
        self._pair_limits={}
        self._decoder=path_decoder
        self._markers=tuple(active_path_markers)
        self._records=[]
        self._actor_paths={}
        self._pair_active={}
        self._step_invalid=[]
        self._step_start=0
        self.step_minimum_separation_m=None
        # Default remains the existing callback loop. Opt-in changes value
        # copying only; native generation/response, samples and stop stay exact.
        self._native_triplet_slice=bool(native_triplet_slice)
        self._native_float3_type=None
        if self._native_triplet_slice:
            carb=sys.modules.get('carb')
            self._native_float3_type=None if carb is None else getattr(carb,'Float3',None)

    @property
    def record_count(self):
        return len(self._records)

    @property
    def raw_records(self):
        """Immutable raw evidence; IDs and impulses are not converted to forces."""
        return tuple(self._records)

    @property
    def step_contact_count(self):
        return len(self._records)-self._step_start

    @property
    def step_has_invalid_contact(self):
        return bool(self._step_invalid)

    def clear_step(self):
        self._step_start=len(self._records)
        self._step_invalid.clear()
        self.step_minimum_separation_m=None

    def _path(self, collider):
        if collider not in self._actor_paths:
            self._actor_paths[collider]=str(self._decoder(collider))
        return self._actor_paths[collider]

    def _active(self, a, b):
        key=(a,b)
        if key not in self._pair_active:
            left,right=self._path(a),self._path(b)
            self._pair_active[key]=any(marker in left or marker in right for marker in self._markers)
        return self._pair_active[key]

    def record(self, headers, data, *, t, phase):
        """Copy raw native data before callback-owned objects are reused."""
        if self._native_triplet_slice:
            return self._record_native_slices(headers,data,t=t,phase=phase)
        t=float(t)
        for header in headers:
            a,b=int(header.collider0),int(header.collider1)
            active=self._active(a,b)
            limit=self.penetration_limit_m
            if self._pair_limit is not None:
                if (a,b) not in self._pair_limits:
                    value=float(self._pair_limit(self._path(a),self._path(b)))
                    if not math.isfinite(value) or value<0:raise ValueError('Invalid pair penetration limit')
                    self._pair_limits[a,b]=value
                limit=self._pair_limits[a,b]
            offset,count=int(header.contact_data_offset),int(header.num_contact_data)
            for i in range(offset,offset+count):
                contact=data[i]
                separation=float(contact.separation)
                impulse=contact.impulse
                raw_impulse=(float(impulse[0]),float(impulse[1]),float(impulse[2]))
                if self.capture_frames:
                    try:
                        position,normal=contact.position,contact.normal
                    except AttributeError as exc:
                        raise AttributeError('Native ContactData.position/normal unavailable with capture_frames=True') from exc
                    raw=FramedRawContact(t,phase,a,b,separation,raw_impulse,
                        (float(position[0]),float(position[1]),float(position[2])),
                        (float(normal[0]),float(normal[1]),float(normal[2])))
                else:
                    raw=RawContact(t,phase,a,b,separation,raw_impulse)
                index=len(self._records)
                self._records.append(raw)
                if self.step_minimum_separation_m is None or separation<self.step_minimum_separation_m:
                    self.step_minimum_separation_m=separation
                if active and separation < -limit:
                    self._step_invalid.append(index)

    def _record_native_slices(self,headers,data,*,t,phase):
        """Lossless opt-in vector-copy loop; each point still drives the guard.

        Header ordering, overlapping header ranges and zero-point headers are
        retained. No native ContactDataVector is kept beyond callback lifetime.
        Attribute errors preserve the same already-copied prefix as default.
        """
        t=float(t)
        records=self._records;append=records.append;invalid_append=self._step_invalid.append
        capture=self.capture_frames;native_type=self._native_float3_type
        triplet=_native_slice_triplet
        for header in headers:
            a,b=int(header.collider0),int(header.collider1)
            active=self._active(a,b)
            limit=self.penetration_limit_m
            if self._pair_limit is not None:
                if (a,b) not in self._pair_limits:
                    value=float(self._pair_limit(self._path(a),self._path(b)))
                    if not math.isfinite(value) or value<0:raise ValueError('Invalid pair penetration limit')
                    self._pair_limits[a,b]=value
                limit=self._pair_limits[a,b]
            offset,count=int(header.contact_data_offset),int(header.num_contact_data)
            for i in range(offset,offset+count):
                contact=data[i]
                separation=float(contact.separation)
                raw_impulse=triplet(contact.impulse,native_type)
                if capture:
                    try:
                        position,normal=contact.position,contact.normal
                    except AttributeError as exc:
                        raise AttributeError('Native ContactData.position/normal unavailable with capture_frames=True') from exc
                    raw=FramedRawContact(t,phase,a,b,separation,raw_impulse,
                        triplet(position,native_type),triplet(normal,native_type))
                else:
                    raw=RawContact(t,phase,a,b,separation,raw_impulse)
                index=len(records)
                append(raw)
                if self.step_minimum_separation_m is None or separation<self.step_minimum_separation_m:
                    self.step_minimum_separation_m=separation
                if active and separation < -limit:
                    invalid_append(index)

    def _event(self, raw):
        # Keep the original norm expression and units exactly, including zero
        # impulses and signed separations. Callback impulses are N*s, not N.
        event=dict(t=raw.t,phase=raw.phase,a=self._path(raw.collider0),b=self._path(raw.collider1),
                   separation_m=raw.separation_m,
                   force_n=math.sqrt(sum(float(x)**2 for x in raw.impulse_ns))*self.hz,
                   impulse_ns=list(raw.impulse_ns))
        if isinstance(raw,FramedRawContact):
            event.update(position_world_m=list(raw.position_world_m),normal_world=list(raw.normal_world))
        return event

    def step_invalid_events(self):
        """Format only violating events so the current per-step stop stays exact."""
        return [self._event(self._records[i]) for i in self._step_invalid]

    def materialize(self, start=0, end=None):
        """Return ordered legacy dictionaries without changing raw evidence."""
        end=len(self._records) if end is None else end
        if not 0<=start<=end<=len(self._records):
            raise ValueError('Contact materialization bounds are invalid')
        return [self._event(self._records[i]) for i in range(start,end)]

    def metadata(self):
        result=dict(backend='buffered_native_callback',raw_contact_points=len(self._records),
                    cached_collider_paths=len(self._actor_paths),cached_ordered_pairs=len(self._pair_active),
                    raw_units=dict(separation='m',impulse='N*s',timestamp='physics seconds'),
                    force_metric='maximum norm of one contact point impulse / physics dt; not summed force',
                    per_step_guard=dict(scope='exact collider paths; active robot/tool/plant',
                                        separation_below_m=-self.penetration_limit_m,
                                        pair_override=self._pair_limit is not None),
                    native_break_events='unchanged existing native subscription',
                    force_formatting='deferred until invalid event or post-loop materialization',
                    capture_frames=self.capture_frames,
                    contact_frame=None if not self.capture_frames else dict(
                        position='native world-space contact point, m',
                        normal='native world-space normal from second shape to first; not reoriented or normalized',
                        source_fields=['ContactData.position','ContactData.normal'],
                        source_documentation='https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/_build/physx/latest/struct_px_contact_pair_point.html'),
                    native_equivalence_validation='pending')
        if self._native_triplet_slice:
            result['native_triplet_copy']=dict(mode='exact_carb_Float3_slice_v1',
                native_type_available=self._native_float3_type is not None,
                unknown_vectors='original indexed float conversion',
                retained_callback_owned_vectors=False,contact_points_omitted=0,
                native_prefix_equivalence_validation='pending')
        return result
