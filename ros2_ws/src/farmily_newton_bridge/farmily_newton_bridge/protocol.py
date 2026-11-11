"""Local framed JSON + numeric arrays; no pickle or ROS Python ABI dependency."""
import json
import math
import struct
import numpy as np

MAX_BYTES = 32 * 1024 * 1024
HEADER = struct.Struct('!II')


def ros_instance_id(value):
    """Preserve uint32 identity bits in ROS's standard signed32SC1 encoding."""
    value=int(value)
    if not 0<=value<2**32:raise ValueError('Expected uint32 source instance ID')
    return value if value<2**31 else value-2**32


def encode(message, arrays=None):
    meta = dict(message); chunks = []; descriptors = []
    for name, value in (arrays or {}).items():
        value = np.asarray(value)
        if value.dtype.kind not in 'uif' or value.dtype.itemsize not in (1, 2, 4, 8):
            raise ValueError('Only numeric image arrays are permitted')
        value = np.ascontiguousarray(value, dtype=value.dtype.newbyteorder('<'))
        raw = value.tobytes()
        descriptors.append(dict(name=name, dtype=value.dtype.str, shape=list(value.shape), size=len(raw)))
        chunks.append(raw)
    meta['arrays'] = descriptors
    header = json.dumps(meta, allow_nan=False, separators=(',', ':')).encode()
    body = b''.join(chunks)
    if len(header) + len(body) > MAX_BYTES:raise ValueError('Telemetry packet too large')
    return HEADER.pack(len(header), len(body)) + header + body


def exact(sock, count):
    result = bytearray()
    while len(result) < count:
        part = sock.recv(count-len(result))
        if not part:raise EOFError('Telemetry producer disconnected')
        result.extend(part)
    return bytes(result)


def receive(sock):
    size, body_size = HEADER.unpack(exact(sock, HEADER.size))
    if size < 2 or size + body_size > MAX_BYTES:raise ValueError('Invalid telemetry frame size')
    meta = json.loads(exact(sock, size)); body = exact(sock, body_size)
    arrays = {}; offset = 0
    for desc in meta.pop('arrays', []):
        dtype = np.dtype(desc['dtype']); shape = desc['shape']
        if dtype.kind not in 'uif' or dtype.itemsize not in (1, 2, 4, 8):raise ValueError('Invalid numeric dtype')
        if len(shape) > 3 or any(not isinstance(n, int) or n < 0 for n in shape):raise ValueError('Invalid array shape')
        count = math.prod(shape); length = count*dtype.itemsize
        if length != desc['size'] or offset+length > len(body) or desc['name'] in arrays:
            raise ValueError('Invalid telemetry array length/name')
        arrays[desc['name']] = np.frombuffer(body, dtype=dtype, count=count, offset=offset).reshape(shape)
        offset += length
    if offset != len(body):raise ValueError('Unclaimed telemetry payload')
    return meta, arrays
