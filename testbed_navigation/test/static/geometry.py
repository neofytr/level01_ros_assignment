import struct
import xml.etree.ElementTree as ET
from functools import lru_cache


def _xyz(element):
    origin = element.find('origin')
    return [float(v) for v in (origin.get('xyz') if origin is not None else '0 0 0').split()]


def _bounds(path):
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    axes = list(zip(*(struct.unpack_from('<3f', data, 96 + i * 50 + v * 12)
                      for i in range(count) for v in range(3))))
    return [min(a) for a in axes], [max(a) for a in axes]


@lru_cache(maxsize=None)
def _urdf(description):
    return ET.parse(description / 'urdf' / 'testbed.xacro').getroot()


def part_geometry(description):
    parts = {}
    for link in _urdf(description).iter('link'):
        mesh = link.find('visual/geometry/mesh')
        if mesh is None:
            continue
        scale = [float(v) for v in mesh.get('scale').split()]
        lo, hi = _bounds(description / 'meshes' / mesh.get('filename').rsplit('/', 1)[1])
        offset = _xyz(link.find('visual'))
        parts[link.get('name')] = (
            [(lo[a] + hi[a]) / 2 * scale[a] + offset[a] for a in range(3)],
            [(hi[a] - lo[a]) / 2 * scale[a] for a in range(3)],
        )
    return parts


def in_base_link(description):
    parts = part_geometry(description)
    placed = {}
    for joint in _urdf(description).iter('joint'):
        child = joint.find('child').get('link')
        if joint.find('parent').get('link') == 'base_link' and child in parts:
            placed[child] = [o + c for o, c in zip(_xyz(joint), parts[child][0])]
    return placed


def joint_origin(description, name):
    return _xyz(_urdf(description).find(f"joint[@name='{name}']"))
