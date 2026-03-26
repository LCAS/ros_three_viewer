"""Efficient PointCloud2 decoder for ros2_web_viewer.

Decodes sensor_msgs/PointCloud2 to packed binary float32 arrays:
  N × [x, y, z, r, g, b]  (6 × float32 = 24 bytes per point)

Colour sources (in priority order):
  1. rgb / rgba field in the cloud
  2. intensity field → viridis colourmap
  3. z height → viridis colourmap (sentinel: r=g=b=-1 sent; JS applies height colour)
"""

import struct
import math


# ---------------------------------------------------------------------------
# Viridis colourmap (polynomial approximation)
# ---------------------------------------------------------------------------

def _viridis(t: float):
    """Return (r, g, b) ∈ [0,1] for t ∈ [0,1]."""
    t = max(0.0, min(1.0, t))
    if t < 0.25:
        s = t / 0.25
        return (0.267 + s * 0.005, 0.005 + s * 0.357, 0.329 + s * 0.246)
    elif t < 0.5:
        s = (t - 0.25) / 0.25
        return (0.272 + s * 0.136, 0.362 + s * 0.271, 0.575 - s * 0.061)
    elif t < 0.75:
        s = (t - 0.5) / 0.25
        return (0.408 + s * 0.379, 0.633 + s * 0.182, 0.514 - s * 0.265)
    else:
        s = (t - 0.75) / 0.25
        return (0.787 + s * 0.166, 0.815 + s * 0.123, 0.249 - s * 0.249)


# ---------------------------------------------------------------------------
# Field metadata helper
# ---------------------------------------------------------------------------

def _field_map(msg):
    """Return {name: (offset, datatype)} from PointCloud2 fields."""
    return {f.name: (f.offset, f.datatype) for f in msg.fields}


_STRUCT_FMT = {1: 'b', 2: 'B', 3: 'h', 4: 'H', 5: 'i', 6: 'I', 7: 'f', 8: 'd'}


def _read_field(data, offset, datatype):
    fmt = _STRUCT_FMT.get(datatype, 'f')
    size = struct.calcsize(fmt)
    if offset + size > len(data):
        return 0.0
    return struct.unpack_from(fmt, data, offset)[0]


# ---------------------------------------------------------------------------
# Main decoder
# ---------------------------------------------------------------------------

def decode_pointcloud2(msg, max_points: int = 10000) -> bytes:
    """Decode PointCloud2 → packed bytes: N × [x,y,z,r,g,b] as float32."""

    fields = _field_map(msg)
    step = msg.point_step
    raw = bytes(msg.data)
    n_total = msg.width * msg.height

    # Stride for downsampling
    stride = max(1, n_total // max_points)

    has_x = 'x' in fields
    has_y = 'y' in fields
    has_z = 'z' in fields
    has_rgb = 'rgb' in fields
    has_rgba = 'rgba' in fields
    has_intensity = 'intensity' in fields

    x_off = fields['x'][0] if has_x else 0
    y_off = fields['y'][0] if has_y else 4
    z_off = fields['z'][0] if has_z else 8

    # Precompute z range for normalisation (quick pass)
    z_min, z_max = float('inf'), float('-inf')
    if not (has_rgb or has_rgba) and has_z:
        for i in range(0, n_total, stride * 4):
            base = i * step
            if base + step > len(raw):
                break
            z = struct.unpack_from('f', raw, base + z_off)[0]
            if z == z:  # not NaN
                z_min = min(z_min, z)
                z_max = max(z_max, z)
        if z_min == float('inf'):
            z_min, z_max = 0.0, 1.0

    result = bytearray()

    for i in range(0, n_total, stride):
        base = i * step
        if base + step > len(raw):
            break

        x = struct.unpack_from('f', raw, base + x_off)[0]
        y = struct.unpack_from('f', raw, base + y_off)[0]
        z = struct.unpack_from('f', raw, base + z_off)[0]

        # Skip NaN / inf
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue

        # ---- Colour ----
        if has_rgb or has_rgba:
            key = 'rgb' if has_rgb else 'rgba'
            rgb_int = struct.unpack_from('I', raw, base + fields[key][0])[0]
            r = float((rgb_int >> 16) & 0xFF) / 255.0
            g = float((rgb_int >> 8) & 0xFF) / 255.0
            b = float(rgb_int & 0xFF) / 255.0
        elif has_intensity:
            intensity = _read_field(raw, base + fields['intensity'][0],
                                    fields['intensity'][1])
            t = max(0.0, min(1.0, intensity / 255.0))
            r, g, b = _viridis(t)
        else:
            # Height-based via viridis
            z_range = z_max - z_min
            t = (z - z_min) / z_range if z_range > 1e-6 else 0.5
            r, g, b = _viridis(t)

        result += struct.pack('ffffff', x, y, z, r, g, b)

    return bytes(result)
