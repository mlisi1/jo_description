#!/usr/bin/env python3
"""Convert the AgileX Bunker Pro 2.0 STEP model into the STL meshes used by the URDF.

The STEP file only contains planar faces bounded by straight edges (a faceted
B-rep), so it can be triangulated without a CAD kernel: every ADVANCED_FACE is
projected on its PLANE, triangulated with earcut and lifted back to 3D.

Meshes are written in metres, in the CAD frame (origin on the ground, at the
longitudinal centre of the tracks, X forward, Z up). The URDF places them with
the CAD-frame offsets defined in urdf/robot_core.xacro.

Usage:
    python3 scripts/step_to_meshes.py AgileX_BUNKER_PRO_2_0.step meshes/

Requires: numpy, shapely, trimesh, mapbox_earcut.
"""
import os
import re
import sys

import numpy as np
import trimesh
from shapely.geometry import Polygon

# mesh file -> STEP solid names (regex). One file per link and material.
GROUPS = {
    # chassis link
    'body.stl': r'SCOCCA_(SUPERIORE|INFERIORE)',
    'body_sides.stl': r'FIANCO_SUPERIORE_(SX|DX)',
    'body_dark.stl': r'PARAURTI_ANTERIORE|TAPPO_.*|Q1_.*|Q3_.*|Q4_.*',
    'rails.stl': r'BARRA_.*',
    'estop.stl': r'Q5_STOP_EMERGENZA_FUNGO',
    'estop_base.stl': r'Q5_STOP_EMERGENZA_BASE',
    'panel_green.stl': r'Q2_INTERRUTTORE|Q6_LED_STATO',
}
for side, tag in (('left', 'SX'), ('right', 'DX')):
    GROUPS.update({
        # back_<side>_roller link
        f'track_{side}.stl': rf'CINGOLO_{tag}',
        f'wheels_{side}.stl': rf'(RUOTA|RUOTINO)_{tag}_\d',
        # <side>_cover link
        f'cover_{side}.stl': rf'(FIANCATA|COPERCHIO_OVALE)_{tag}',
        f'track_frame_{side}.stl': rf'TELAIO_CINGOLO_{tag}',
        f'hubs_{side}.stl': rf'(MOZZO|COPRIMOZZO)_{tag}_\d',
    })


class Step:
    def __init__(self, path):
        txt = open(path).read()
        data = txt.split('DATA;', 1)[1].split('ENDSEC;', 1)[0]
        self.e = {}
        for m in re.finditer(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)', data, re.S):
            self.e[int(m.group(1))] = m.group(2).replace('\n', '')

    @staticmethod
    def refs(s):
        return [int(x) for x in re.findall(r'#(\d+)', s)]

    def kind(self, i):
        return self.e[i].split('(', 1)[0].strip()

    def point(self, i):
        nums = re.findall(r'\(([^()]*)\)\s*\)\s*$', self.e[i])[0]
        return np.array([float(x) for x in nums.split(',')])

    def loop(self, i):
        pts = []
        for oe in self.refs(self.e[i]):
            ec = self.refs(self.e[oe])[0]
            v1, v2 = self.refs(self.e[ec])[:2]
            a = self.point(self.refs(self.e[v1])[0])
            b = self.point(self.refs(self.e[v2])[0])
            pts.append(a if self.e[oe].strip().endswith('.T.)') else b)
        return np.array(pts)

    def solid(self, i):
        V, F, nv = [], [], 0
        for face in self.refs(self.e[self.refs(self.e[i])[0]]):
            fr = self.refs(self.e[face])
            bounds, surf = fr[:-1], fr[-1]
            if self.kind(surf) != 'PLANE':
                raise ValueError(f'#{surf}: only planar faces are supported')
            o, z, x = (self.point(j) for j in self.refs(self.e[self.refs(self.e[surf])[0]]))
            n = z / np.linalg.norm(z)
            if not self.e[face].strip().endswith('.T.)'):
                n = -n
            x = x - n * x.dot(n)
            x /= np.linalg.norm(x)
            y = np.cross(n, x)
            outer, holes = None, []
            for b in bounds:
                p3 = self.loop(self.refs(self.e[b])[0])
                p2 = np.c_[(p3 - o) @ x, (p3 - o) @ y]
                if self.kind(b) == 'FACE_OUTER_BOUND':
                    outer = p2
                else:
                    holes.append(p2)
            if outer is None:
                holes.sort(key=lambda p: -Polygon(p).area)
                outer = holes.pop(0)
            poly = Polygon(outer, holes).buffer(0)
            for pg in ([poly] if poly.geom_type == 'Polygon' else list(poly.geoms)):
                v2, f = trimesh.creation.triangulate_polygon(pg, engine='earcut')
                v3 = o + v2[:, :1] * x + v2[:, 1:2] * y
                t = v3[f]
                flip = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]) @ n < 0
                f[flip] = f[flip][:, ::-1]
                V.append(v3)
                F.append(f + nv)
                nv += len(v3)
        return trimesh.Trimesh(np.vstack(V) / 1000.0, np.vstack(F))  # mm -> m

    def solids(self):
        for i, s in self.e.items():
            if s.startswith('MANIFOLD_SOLID_BREP'):
                yield re.findall(r"'([^']*)'", s)[0], i


def main(step_path, out_dir):
    step = Step(step_path)
    solids = dict(step.solids())
    used = set()
    os.makedirs(out_dir, exist_ok=True)
    for fname, pattern in GROUPS.items():
        names = [n for n in solids if re.fullmatch(pattern, n)]
        if not names:
            raise SystemExit(f'{fname}: no solid matches {pattern}')
        used.update(names)
        mesh = trimesh.util.concatenate([step.solid(solids[n]) for n in names])
        mesh.merge_vertices()
        mesh.export(os.path.join(out_dir, fname))
        print(f'{fname:24s} {len(mesh.faces):6d} tris  <- {", ".join(names)}')
    missing = set(solids) - used
    if missing:
        raise SystemExit(f'solids not assigned to any mesh: {sorted(missing)}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
