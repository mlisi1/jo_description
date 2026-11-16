#!/usr/bin/env python3
"""Convert the robot + platform STEP assembly into the STL meshes used by the URDF.

The assembly frame is the Bunker CAD frame (origin on the ground at the
longitudinal centre of the tracks, X forward, Z up). Meshes of the robot and of
the rack are written in that frame; sensor meshes are written in their own
sensor frame, so the URDF can place them directly in the sensor links.

Sensor frames come from the axis triads in the assembly ("TERNA - <name>"):
red arm = X, green = Y, blue = Z. They are printed w.r.t. base_link so they can
be copied into the xacro files.

Usage:
    python3 scripts/step_to_meshes.py BUNKER_PRO2_rack_sensori_ASSIEME_2.step meshes/

Requires: cadquery-ocp, numpy, trimesh, fast_simplification
    (pip install cadquery-ocp numpy trimesh fast_simplification)
"""
import os
import re
import sys

import numpy as np
import trimesh
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.OCP.collections import Sequence_TDF_Label
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
from OCP.XCAFDoc import XCAFDoc_DocumentTool

# base_link in the CAD frame (top deck of the Bunker, centre of the upper shell); keep in
# sync with cad2base_x / cad2base_z in urdf/robot_core.xacro
BASE_LINK = np.array([0.038, 0.0, 0.454])

LIN_DEFLECTION = 0.4   # mm
ANG_DEFLECTION = 0.35  # rad

BUNKER = r'AgileX BUNKER PRO 2\.0'
# mesh file -> (regex on "<product>/<solid>", frame[, target faces]). frame: 'cad' or a triad name.
# Target faces decimates the mesh (needs fast_simplification).
def P(product_regex):
    """all solids of the matching products"""
    return rf'({product_regex})/.*'


GROUPS = {
    # chassis link
    'body.stl': (rf'{BUNKER}/SCOCCA_(SUPERIORE|INFERIORE)', 'cad'),
    'body_sides.stl': (rf'{BUNKER}/FIANCO_SUPERIORE_(SX|DX)', 'cad'),
    'body_dark.stl': (rf'{BUNKER}/(PARAURTI_ANTERIORE|TAPPO_.*|Q1_.*|Q3_.*|Q4_.*)', 'cad'),
    'rails.stl': (P(r'GUIDA LINEARE .*'), 'cad'),
    'estop.stl': (rf'{BUNKER}/Q5_STOP_EMERGENZA_FUNGO', 'cad'),
    'estop_base.stl': (rf'{BUNKER}/Q5_STOP_EMERGENZA_BASE', 'cad'),
    'panel_green.stl': (rf'{BUNKER}/(Q2_INTERRUTTORE|Q6_LED_STATO)', 'cad'),
    # rack link (sensorized platform)
    'rack.stl': (P(r'RACK - (?!Porta - vetro|Porta - strisce|Etichetta logo)[^/]*'
                   r'|TAPPO (FRONT|BACK)[^/]*|SERRATURA TAPPO [^/]*|GOPRO MOUNT [^/]*|STAFFA D456 [^/]*'
                   r'|ADATTATORE IMU[^/]*'), 'cad'),
    'rack_glass.stl': (P(r'RACK - Porta - vetro[^/]*'), 'cad'),
    'rack_red.stl': (P(r'RACK - (Porta - strisce rosse|Etichetta logo - riquadro rosso)'), 'cad'),
    'rack_label.stl': (P(r'RACK - Etichetta logo'), 'cad'),
    'imu_shelf.stl': (P(r'RIPIANO LEGNO[^/]*'), 'cad'),
    # velodyne_holder link
    'lidar_mount.stl': (P(r'PROFILATO HFSB8-8080[^/]*|SUPPORTO LIDAR[^/]*'), 'cad'),
    'lidar_profiles.stl': (P(r'PROFILATO HFS8-4040[^/]*'), 'cad'),
    # sensor links (sensor frame)
    'vlp16.stl': (P(r'VELODYNE PUCK[^/]*'), 'velodyne'),
    'mti680g.stl': (P(r'MTi-680G - (?!CE logo|Xsens text|.*sticker)[^/]*'), 'imu_mti680g', 6000),
    'd456.stl': (P(r'REALSENSE D456 FRONT'), 'd456_front_link', 4000),
}
for side, tag in (('left', 'SX'), ('right', 'DX')):
    GROUPS.update({
        f'track_{side}.stl': (rf'{BUNKER}/CINGOLO_{tag}', 'cad'),
        f'wheels_{side}.stl': (rf'{BUNKER}/(RUOTA|RUOTINO)_{tag}_\d', 'cad'),
        f'cover_{side}.stl': (rf'{BUNKER}/(FIANCATA|COPERCHIO_OVALE)_{tag}', 'cad'),
        f'track_frame_{side}.stl': (rf'{BUNKER}/TELAIO_CINGOLO_{tag}', 'cad'),
        f'hubs_{side}.stl': (rf'{BUNKER}/(MOZZO|COPRIMOZZO)_{tag}_\d', 'cad'),
    })
# parts deliberately not exported (frames, duplicate camera, decorations)
IGNORE = r'(TERNA - .*|REALSENSE D456 BACK/.*|MTi-680G - (CE logo|Xsens text|.*sticker).*)'


def step_entities(path):
    data = open(path).read().split('DATA;', 1)[1].split('ENDSEC;', 1)[0]
    return {int(m.group(1)): m.group(2).replace('\n', '')
            for m in re.finditer(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|$)', data, re.S)}


def refs(s):
    return [int(x) for x in re.findall(r'#(\d+)', s)]


def product_solids(E):
    """product name -> ordered [(solid name, rgb or None)] of its shape representation."""
    colour = {}
    for s in E.values():
        if s.startswith('STYLED_ITEM'):
            r = refs(s)
            stack = [r[0]]
            while stack:
                j = stack.pop()
                if E[j].startswith('COLOUR_RGB'):
                    colour[r[-1]] = tuple(float(v) for v in re.findall(r'[-\d.eE]+', E[j].split(',', 1)[1]))
                    break
                stack += refs(E[j])
    name = {}
    for i, s in E.items():
        if s.startswith('PRODUCT_DEFINITION('):
            prod = refs(E[refs(s)[0]])[0]
            name[i] = re.findall(r"'([^']*)'", E[prod])[0]
    out = {}
    for s in E.values():
        if s.startswith('SHAPE_DEFINITION_REPRESENTATION'):
            pds, rep = refs(s)
            r = refs(E[pds])
            if r and r[0] in name:
                items = [j for j in refs(E[rep]) if E[j].startswith('MANIFOLD_SOLID_BREP')]
                out[name[r[0]]] = [(re.findall(r"'([^']*)'", E[j])[0], colour.get(j)) for j in items]
    return out


def load_parts(path):
    """[(product name, [(solid name, colour, TopoDS_Solid in assembly frame)])]"""
    names = product_solids(step_entities(path))
    doc = TDocStd_Document(TCollection_ExtendedString('doc'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(path) != 1:
        raise SystemExit(f'cannot read {path}')
    reader.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def label_name(lab):
        a = TDataStd_Name()
        return a.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), a) else ''

    parts = []

    def walk(lab, loc):
        if st.IsReference_s(lab):
            ref = TDF_Label()
            st.GetReferredShape_s(lab, ref)
            walk(ref, loc * st.GetLocation_s(lab))
        elif st.IsAssembly_s(lab):
            seq = Sequence_TDF_Label()
            st.GetComponents_s(lab, seq)
            for i in range(1, seq.Length() + 1):
                walk(seq.Value(i), loc)
        else:
            prod = label_name(lab)
            shape = st.GetShape_s(lab).Moved(loc)
            solids = []
            ex = TopExp_Explorer(shape, TopAbs_SOLID)
            while ex.More():
                solids.append(ex.Current())
                ex.Next()
            if not solids:  # surface model (e.g. the official Velodyne model)
                solids = [shape]
            info = names.get(prod, [])
            if len(info) != len(solids):  # cannot pair names: name solids by index
                info = [(f'{k}', None) for k in range(len(solids))]
            parts.append((prod, [(n, c, s) for (n, c), s in zip(info, solids)]))

    roots = Sequence_TDF_Label()
    st.GetFreeShapes(roots)
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location())
    return parts


def mesh_of(shape):
    BRepMesh_IncrementalMesh(shape, LIN_DEFLECTION, False, ANG_DEFLECTION, True)
    V, F, n = [], [], 0
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        face = TopoDS.Face(ex.Current())
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            v = np.array([tri.Node(i).Transformed(trsf).Coord() for i in range(1, tri.NbNodes() + 1)])
            f = np.array([tri.Triangle(i).Get() for i in range(1, tri.NbTriangles() + 1)]) - 1
            if face.Orientation() == TopAbs_REVERSED:
                f = f[:, ::-1]
            V.append(v)
            F.append(f + n)
            n += len(v)
        ex.Next()
    if not V:
        return None
    return trimesh.Trimesh(np.vstack(V) / 1000.0, np.vstack(F), process=False)  # mm -> m


def triad_frame(solids):
    """4x4 pose (m) of an axis triad given its (name, colour, solid) arms: red=X, green=Y, blue=Z."""
    arms = {}
    for _, colour, solid in solids:
        m = mesh_of(solid)
        lo, hi = m.bounds
        axis = int(np.argmax(hi - lo))
        arms[int(np.argmax(colour))] = (axis, lo, hi)  # 0 red, 1 green, 2 blue
    origin = np.zeros(3)
    R = np.zeros((3, 3))
    for k, (axis, lo, hi) in arms.items():
        # the arm starts at the origin, where the other arms are
        others = np.array([(a[1] + a[2]) / 2 for j, a in arms.items() if j != k])
        start_lo = np.abs(others[:, axis] - lo[axis]).min() < np.abs(others[:, axis] - hi[axis]).min()
        R[axis, k] = 1.0 if start_lo else -1.0
        origin[axis] = lo[axis] if start_lo else hi[axis]  # arms cover the 3 axes
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = origin
    return T


def main(step_path, out_dir):
    parts = load_parts(step_path)
    frames = {}
    for prod, solids in parts:
        if prod.startswith('TERNA - '):
            frames[prod[len('TERNA - '):]] = triad_frame(solids)

    print('Sensor frames w.r.t. base_link  (xyz [m], yaw [deg]):')
    for name, T in frames.items():
        p = T[:3, 3] - BASE_LINK
        yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
        print(f'  {name:24s} xyz = {p[0]: .4f} {p[1]: .4f} {p[2]: .4f}   yaw = {yaw:.1f}   Z axis = {T[:3, 2]}')

    used = set()
    os.makedirs(out_dir, exist_ok=True)
    for fname, (pattern, frame, *max_faces) in GROUPS.items():
        meshes = []
        for pi, (prod, solids) in enumerate(parts):
            for si, (sname, _, solid) in enumerate(solids):
                if re.fullmatch(pattern, f'{prod}/{sname}'):
                    if (pi, si) in used:
                        raise SystemExit(f'{prod}/{sname} matched by more than one mesh group')
                    used.add((pi, si))
                    m = mesh_of(solid)
                    if m is not None:
                        meshes.append(m)
        if not meshes:
            raise SystemExit(f'{fname}: nothing matches {pattern}')
        mesh = trimesh.util.concatenate(meshes)
        if frame != 'cad':
            mesh.apply_transform(np.linalg.inv(frames[frame]))
        mesh.merge_vertices()
        if max_faces and len(mesh.faces) > max_faces[0]:
            mesh = mesh.simplify_quadric_decimation(face_count=max_faces[0])
        mesh.export(os.path.join(out_dir, fname))
        print(f'{fname:24s} {len(mesh.faces):7d} tris  frame={frame}')

    left = [f'{p}/{s}' for pi, (p, solids) in enumerate(parts) for si, (s, _, _) in enumerate(solids)
            if (pi, si) not in used and not re.fullmatch(IGNORE, f'{p}/{s}')]
    if left:
        raise SystemExit(f'solids not assigned to any mesh: {left}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    main(sys.argv[1], sys.argv[2])
