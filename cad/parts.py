"""Parametric CAD for the two custom parts (units: mm).

Run:  python cad/parts.py
Writes STEP (importable into Fusion 360) to cad/ and STL to the description package.
Dimensions must stay in sync with the constants at the top of robot.urdf.xacro.
"""
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parent.parent
MESH_DIR = ROOT / "src" / "planguard_description" / "meshes"

# Chassis: 400 x 300 x 120, origin at the geometric center.
CH_L, CH_W, CH_H = 400.0, 300.0, 120.0
CH_FILLET = 12.0
CH_CHAMFER = 3.0

# L-bracket, origin at the bottom-center of the mounting flange's reference point
# (x = 0 is the lidar axis, z = 0 is the chassis top surface).
POST_X = (-70.0, -60.0)
POST_W = 30.0
ARM_X = (-70.0, 40.0)
ARM_W = 50.0
ARM_Z = (65.0, 75.0)
FLANGE_X = (-90.0, -40.0)
FLANGE_W = 50.0
FLANGE_T = 5.0
HOLE_D = 4.5


def chassis() -> cq.Workplane:
    body = (
        cq.Workplane("XY")
        .box(CH_L, CH_W, CH_H)
        .edges("|Z")
        .fillet(CH_FILLET)
        .faces(">Z")
        .edges()
        .chamfer(CH_CHAMFER)
    )
    # bracket mounting holes (M4 clearance) under the flange
    holes = [(-80.0, -15.0), (-80.0, 15.0), (-50.0, -15.0), (-50.0, 15.0)]
    body = body.faces(">Z").workplane(centerOption="CenterOfBoundBox").pushPoints(holes).hole(HOLE_D, 15.0)
    return body


def _slab(x0, x1, w, z0, z1) -> cq.Workplane:
    return cq.Workplane("XY").box(x1 - x0, w, z1 - z0).translate(((x0 + x1) / 2, 0, (z0 + z1) / 2))


def lidar_bracket() -> cq.Workplane:
    flange = _slab(*FLANGE_X, FLANGE_W, 0.0, FLANGE_T)
    post = _slab(*POST_X, POST_W, 0.0, ARM_Z[1])
    arm = _slab(*ARM_X, ARM_W, *ARM_Z)
    part = flange.union(post).union(arm)
    part = part.faces(">Z").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0)).hole(6.0, 20.0)
    holes = [(-80.0, -15.0), (-80.0, 15.0), (-50.0, -15.0), (-50.0, 15.0)]
    part = part.faces("<Z").workplane(centerOption="ProjectedOrigin", origin=(0, 0, 0)).pushPoints(
        [(x, -y) for x, y in holes]
    ).hole(HOLE_D, FLANGE_T)
    return part


def main() -> None:
    MESH_DIR.mkdir(parents=True, exist_ok=True)
    for name, part in (("chassis", chassis()), ("lidar_bracket", lidar_bracket())):
        cq.exporters.export(part, str(ROOT / "cad" / f"{name}.step"))
        cq.exporters.export(part, str(MESH_DIR / f"{name}.stl"), tolerance=0.2, angularTolerance=0.2)
        bb = part.val().BoundingBox()
        print(f"{name}: x[{bb.xmin:.0f},{bb.xmax:.0f}] y[{bb.ymin:.0f},{bb.ymax:.0f}] z[{bb.zmin:.0f},{bb.zmax:.0f}] mm")


if __name__ == "__main__":
    main()
