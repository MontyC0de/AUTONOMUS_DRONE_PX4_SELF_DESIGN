"""Decimate the frame DAE for Gazebo/GitHub, keeping one material per CAD colour.

Parts smaller than MIN_PART (SMD components, pins, connectors: ~75 % of the
triangles) are dropped; they are invisible at simulator scale and turn into
shards when decimated. The rest is decimated gently.

Writes the COLLADA with pycollada directly: Z_UP, opaque phong materials with the
CAD colours and smooth normals split at sharp edges (> 30 deg), so Gazebo renders
it like the CAD.

Development tool, not needed to run the simulation. Dependencies:
    pip install trimesh pycollada fast-simplification numpy lxml scipy networkx

usage:
    python3 simulation/tools/decimate_frame.py \
        simulation/cad_source/mi_frame_full_2M_tris.dae \
        simulation/models/mi_drone_base/meshes/mi_frame_full.dae 150000
"""
import re
import sys
from collections import defaultdict

import collada
import fast_simplification
import numpy as np
import trimesh
from collada import geometry, material, scene, source

MIN_PART = 0.003   # m, largest dimension of the smallest part kept
CREASE = np.radians(30)

src, dst, target = sys.argv[1], sys.argv[2], int(sys.argv[3])

loaded = trimesh.load(src, force="scene")
groups = defaultdict(list)
dropped = 0
for m in loaded.dump():
    if m.extents.max() < MIN_PART:
        dropped += len(m.faces)
        continue
    groups[tuple(int(c) for c in m.visual.material.baseColorFactor)].append(m)

total = sum(len(m.faces) for ms in groups.values() for m in ms)
ratio = min(1.0, target / total)
print(f"dropped {dropped} triangles of parts < {MIN_PART * 1000:.0f} mm")
print(f"kept parts: {total} triangles, keep ratio {ratio:.4f}")


def simplify(mesh):
    """Decimate one CAD part (never below a floor) and smooth-shade it."""
    mesh = trimesh.Trimesh(mesh.vertices, mesh.faces, process=True)
    keep = max(min(len(mesh.faces), 48), int(len(mesh.faces) * ratio))
    if keep < len(mesh.faces):
        v, f = fast_simplification.simplify(
            mesh.vertices.astype(np.float32),
            mesh.faces.astype(np.int64),
            target_reduction=1.0 - keep / len(mesh.faces),
        )
        mesh = trimesh.Trimesh(v, f, process=True)
    mesh.update_faces(mesh.nondegenerate_faces())
    # Split vertices along edges sharper than CREASE, average normals elsewhere
    return trimesh.graph.smooth_shade(mesh, angle=CREASE)


dae = collada.Collada()
dae.assetInfo.upaxis = collada.asset.UP_AXIS.Z_UP
dae.assetInfo.unitname = "meter"
dae.assetInfo.unitmeter = 1.0
nodes = []
tris_out = 0
for i, (color, meshes) in enumerate(groups.items()):
    parts = [simplify(m) for m in meshes]
    offsets = np.cumsum([0] + [len(p.vertices) for p in parts[:-1]])
    vertices = np.vstack([p.vertices for p in parts])
    normals = np.vstack([p.vertex_normals for p in parts])
    faces = np.vstack([p.faces + o for p, o in zip(parts, offsets)])
    tris_out += len(faces)

    rgb = [c / 255.0 for c in color[:3]]
    effect = material.Effect(f"effect{i}", [], "phong",
                             emission=(0.0, 0.0, 0.0, 1.0),
                             ambient=(*[c * 0.6 for c in rgb], 1.0),
                             diffuse=(*rgb, 1.0),
                             specular=(0.15, 0.15, 0.15, 1.0),
                             shininess=20.0, double_sided=True)
    mat = material.Material(f"material{i}", f"material{i}", effect)
    dae.effects.append(effect)
    dae.materials.append(mat)

    verts = source.FloatSource(f"verts{i}", np.round(vertices, 5).ravel(), ("X", "Y", "Z"))
    norms = source.FloatSource(f"normals{i}", np.round(normals, 3).ravel(), ("X", "Y", "Z"))
    geom = geometry.Geometry(dae, f"geometry{i}", f"part{i}", [verts, norms])
    inputs = source.InputList()
    # Separate index columns for VERTEX and NORMAL: gz-common's ColladaLoader
    # segfaults when both inputs share offset 0
    inputs.addInput(0, "VERTEX", f"#verts{i}")
    inputs.addInput(1, "NORMAL", f"#normals{i}")
    indices = np.column_stack([faces.ravel(), faces.ravel()]).ravel()
    geom.primitives.append(geom.createTriangleSet(indices, inputs, f"matref{i}"))
    dae.geometries.append(geom)
    nodes.append(scene.Node(f"node{i}", children=[
        scene.GeometryNode(geom, [scene.MaterialNode(f"matref{i}", mat, inputs=[])])]))

root = scene.Scene("scene", nodes)
dae.scenes.append(root)
dae.scene = root
dae.write(dst)

# pycollada always writes <transparent>/<transparency>; drop them so every
# loader treats the materials as opaque (COLLADA default)
with open(dst) as fh:
    text = fh.read()
text = re.sub(r"\s*<transparent>.*?</transparent>", "", text, flags=re.S)
text = re.sub(r"\s*<transparency>.*?</transparency>", "", text, flags=re.S)
with open(dst, "w") as fh:
    fh.write(text)
print("output triangles:", tris_out)
