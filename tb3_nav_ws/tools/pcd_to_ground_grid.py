#!/usr/bin/env python3
"""Conservative 2D ground planning layer derived from a Fast-LIO2 XYZ... PCD.

Free cells require observed ground; empty PCD cells remain unknown. This is NOT
an inverse sensor model: for full free-space maps record poses and raw scans.
"""
import argparse
from pathlib import Path
import numpy as np
import yaml


def read_xyz(path):
    with open(path, 'rb') as stream:
        meta = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD DATA header missing')
            parts = line.decode('ascii').strip().split()
            if parts and not parts[0].startswith('#'):
                meta[parts[0]] = parts[1:]
            if parts and parts[0] == 'DATA':
                break
        if meta['DATA'] != ['binary']:
            raise ValueError('Only uncompressed binary PCD is supported')
        fields = meta['FIELDS']
        sizes = list(map(int, meta['SIZE']))
        kinds = meta['TYPE']
        counts = list(map(int, meta['COUNT']))
        if any(c != 1 for c in counts):
            raise ValueError('Multi-count fields are not supported')
        dtype = np.dtype([(name, {'F': 'f', 'I': 'i', 'U': 'u'}[kind] + str(size))
                          for name, kind, size in zip(fields, kinds, sizes)])
        pts = np.frombuffer(stream.read(), dtype=dtype)
        if len(pts) != int(meta['POINTS'][0]):
            raise ValueError('Truncated or malformed PCD')
        return np.stack([pts[k] for k in ('x', 'y', 'z')], axis=1)


def create_grid(points, resolution, floor_z=None):
    xyz = points[np.isfinite(points).all(axis=1)]
    if len(xyz) < 200:
        raise ValueError('Too few finite PCD points to create a map')
    z = xyz[:, 2]
    if floor_z is None:
        # Most frequent low horizontal plane, rather than the median of walls.
        low = z[z < np.quantile(z, 0.40)]
        bins = np.arange(low.min() - 0.025, low.max() + 0.05, 0.025)
        hist, edges = np.histogram(low, bins=bins)
        floor_z = float((edges[hist.argmax()] + edges[hist.argmax() + 1]) / 2)
    origin = np.floor((xyz[:, :2].min(axis=0) - 0.3) / resolution) * resolution
    end = np.ceil((xyz[:, :2].max(axis=0) + 0.3) / resolution) * resolution
    width, height = np.ceil((end - origin) / resolution).astype(int) + 1
    if width * height > 4_000_000:
        raise ValueError('Map extent too large; clip/filter the PCD first')
    ix, iy = np.floor((xyz[:, :2] - origin) / resolution).astype(int).T
    floor = np.abs(z - floor_z) < 0.085
    obstacle = (z > floor_z + 0.20) & (z < floor_z + 0.90)
    ground_votes = np.zeros((height, width), dtype=np.int32)
    obstacle_votes = np.zeros_like(ground_votes)
    np.add.at(ground_votes, (iy[floor], ix[floor]), 1)
    np.add.at(obstacle_votes, (iy[obstacle], ix[obstacle]), 1)
    # At 10 cm resolution, a few returns are evidence of ground; dilate no
    # further than one neighboring cell, then let all observed obstacles win.
    evidence = ground_votes >= 2
    supported = evidence.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx or dy:
                y0, y1 = max(0, dy), min(height, height + dy)
                x0, x1 = max(0, dx), min(width, width + dx)
                supported[y0:y1, x0:x1] |= evidence[y0-dy:y1-dy, x0-dx:x1-dx]
    grid = np.full((height, width), 205, dtype=np.uint8)  # Nav2 unknown
    grid[supported] = 254  # free
    grid[obstacle_votes >= 2] = 0  # occupied wins, including over free floor
    return grid, origin, floor_z


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd', type=Path)
    parser.add_argument('output', type=Path, help='YAML output path; writes matching PGM')
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--floor-z', type=float, default=None,
                        help='Override detected floor elevation in PCD/map frame')
    a = parser.parse_args()
    if not 0.04 <= a.resolution <= 0.25:
        parser.error('resolution must be between 0.04 and 0.25 m')
    grid, origin, floor_z = create_grid(read_xyz(a.pcd), a.resolution, a.floor_z)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    image = a.output.with_suffix('.pgm')
    # PGM starts at top-left; YAML origin names bottom-left in ROS map frame.
    with image.open('wb') as f:
        h, w = grid.shape
        f.write(f'P5\n{w} {h}\n255\n'.encode('ascii'))
        f.write(grid[::-1].tobytes())
    a.output.write_text(yaml.safe_dump({
        'image': image.name, 'mode': 'trinary', 'resolution': a.resolution,
        'origin': [float(origin[0]), float(origin[1]), 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }, sort_keys=False))
    print(f'PCD floor_z={floor_z:.3f} m; grid {w}x{h} at {a.resolution} m; '
          f'free={(grid == 254).sum()} occupied={(grid == 0).sum()} '
          f'unknown={(grid == 205).sum()}')
    print(f'Saved {a.output} and {image}; review in RViz before navigation.')


if __name__ == '__main__':
    main()
