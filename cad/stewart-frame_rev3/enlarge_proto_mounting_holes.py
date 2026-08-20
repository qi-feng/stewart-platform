"""Enlarge only the four protoboard corner holes from 2.4 to 3.2 mm."""

import struct
from pathlib import Path

import numpy as np


SOURCE = Path(__file__).with_name('proto-board.stl')
OUTPUT = Path(__file__).with_name('proto-board_m3_3.2mm.stl')
CENTERS = [(-15.9, -22.25), (15.9, -22.25), (-15.9, 22.25), (15.9, 22.25)]
SCALE = 1.6 / 1.2


data = SOURCE.read_bytes()
count = struct.unpack_from('<I', data, 80)[0]
triangles = np.empty((count, 3, 3), dtype=np.float64)
for index in range(count):
    values = struct.unpack_from('<12fH', data, 84 + 50 * index)
    triangles[index] = np.array(values[3:12]).reshape(3, 3)

for center_x, center_y in CENTERS:
    dx = triangles[:, :, 0] - center_x
    dy = triangles[:, :, 1] - center_y
    radius = np.hypot(dx, dy)
    boundary = (radius > 1.18) & (radius < 1.22)
    triangles[:, :, 0][boundary] = center_x + dx[boundary] * SCALE
    triangles[:, :, 1][boundary] = center_y + dy[boundary] * SCALE

with OUTPUT.open('wb') as stream:
    stream.write(b'Protoboard with four 3.2 mm M3 mounting holes'.ljust(80, b' '))
    stream.write(struct.pack('<I', count))
    for triangle in triangles:
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        normal /= np.linalg.norm(normal) or 1.0
        stream.write(struct.pack('<12fH', *(list(normal) + list(triangle.flat)), 0))

print(OUTPUT)
