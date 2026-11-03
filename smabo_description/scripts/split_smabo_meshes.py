"""Split the smabo-hardware STEP assemblies into per-link STL meshes.

  smabo_base.step            -> meshes/smabo_base/        base_link (shared by every configuration)
  smabo_base_breadboard.step -> meshes/smabo_breadboard/  breadboard_link (rear breadboard mounts)
  smabo_arm.step             -> meshes/smabo_arm/         arm_base_link, arm_link_1..4
  smabo_hand.step            -> meshes/smabo_hand/        hand_mount_link, left/right_hand_link
  smabo_tt_motor.step        -> meshes/smabo_tt_motor/    chassis_link, left/right_wheel_link
  smabo_neck.step            -> meshes/smabo_neck/        pan_stand_link, pan_link
Parts that belong to another module (the shared base, the breadboard mounts) are
skipped in each assembly.

Each mesh is in metres and expressed in its link frame, matching urdf/*.xacro:
  * axes follow ROS (x = smabo front = smartphone side, z = up), i.e. the STEP
    frame turned 180 deg about z,
  * base_link sits at the bottom centre of the base box; moving links have their
    origin on the joint axis,
  * STEP modelling errors are corrected with SHIFT (moved so the servo horns are
    concentric with the servo shafts):
      arm  : sg90_4 / sg90_5 / hand lowered 1 mm
      hand : sg90_base + arm_hand moved 6.9 mm toward the centre of the plate
  * servos and motors that are not in the STEP are added as simplified shapes
    (SG90, MG996R, TT gear motor, wheel, ball caster) to the link they are
    screwed into; positions are fitted to the pockets of the brackets. The
    caster height is chosen so the base stands level on wheels and caster.
Needs OCP (cadquery-ocp) and trimesh.

    python3 split_smabo_meshes.py ../meshes
"""
import os, sys
import numpy as np, trimesh
from trimesh.transformations import translation_matrix as T, rotation_matrix as R
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.BRepBuilderAPI import BRepBuilderAPI_Copy

STEP_DIR = os.path.expanduser("~/github/smabo-repos/smabo-hardware/step/assembly")
STEP_TO_ROS = R(np.pi, [0, 0, 1])                                 # x: front (smartphone side)

# ---- simplified parts (mm, local frame: shaft along +z, body toward -x) ----
def box(size, at):
    return trimesh.creation.box(size, T(at))

def cyl(r, z0, z1, at=(0, 0)):
    return trimesh.creation.cylinder(r, z1 - z0, sections=48, transform=T((at[0], at[1], (z0 + z1) / 2)))

def sg90(off=4.85):
    """SG90: body 22.5x12.2x22.7, tabs 32.2, shaft tip (origin) 30 above the bottom."""
    L, W, H, TIP = 22.5, 12.2, 22.7, 30.0
    zb = -TIP
    return trimesh.util.concatenate([
        box((L, W, H), (-off, 0, zb + H/2)),
        box((32.2, W, 2.5), (-off, 0, zb + 15.9 + 1.25)),
        cyl(5.9, zb + H, zb + H + 4.0),
        cyl(2.45, zb + H + 4.0, 0),
    ])

def mg996r(off=10.0):
    """MG996R: body 40.7x19.7x36, tabs 54, shaft tip (origin) 42.9 above the bottom."""
    L, W, H, TIP = 40.7, 19.7, 36.0, 42.9
    zb = -TIP
    return trimesh.util.concatenate([
        box((L, W, H), (-off, 0, zb + H/2)),
        box((54.0, W, 2.5), (-off, 0, zb + 26.5 + 1.25)),
        cyl(6.5, zb + H, zb + H + 3.0),
        cyl(2.95, zb + H + 3.0, 0),
    ])

def tt_motor(top=1):
    """TT gear motor (SunFounder drawing): 64.4 long, gearbox 22.3 wide x 18.7 thick,
    shaft 11.0 from the gearbox end. Origin on the shaft axis at the body mid-plane,
    shaft along +z (9.1 out of the body), motor end toward -x. The 17.5 wide motor
    end is flush with the gearbox side on local +y (top=1) or -y (top=-1), as the
    brackets hold it."""
    th = 18.7
    return trimesh.util.concatenate([
        box((42.1, 22.3, th), (11.0 - 42.1/2, 0, 0)),             # gearbox
        box((22.3, 17.5, th), (11.0 - 42.1 - 22.3/2, top * (22.3 - 17.5)/2, 0)),   # motor end
        box((5.0, 5.0, 2.0), (11.0 + 2.5, 0, 0)),                 # mounting ear
        cyl(2.7, th/2, th/2 + 9.1),                               # output shaft
    ])

def caster(h):
    """Stick-on ball caster (Dokodemo caster, sticker included), simplified: a
    housing and a ball, h tall. Origin on the mounting face, ball toward -z."""
    ball = 5.0
    return trimesh.util.concatenate([
        cyl(9.0, -(h - ball), 0),
        trimesh.creation.icosphere(subdivisions=2, radius=ball).apply_translation((0, 0, -(h - ball))),
    ])

def wheel(r=30.0, w=26.0):
    """Wheel centred on the origin, axis along z (radius from smabo-esp32 config)."""
    return cyl(r, -w/2, w/2)

def place(m, origin, z_dir, body_dir=None):
    """Put a local part with its +z along z_dir and its -x (body side) along body_dir."""
    z = np.array(z_dir, float)
    x = -np.array(body_dir, float) if body_dir is not None else np.cross([0, 0, 1] if abs(z[2]) < 0.9 else [1, 0, 0], z)
    M = np.eye(4); M[:3, 0] = x; M[:3, 2] = z; M[:3, 1] = np.cross(z, x); M[:3, 3] = origin
    return m.copy().apply_transform(M)

BASE_PARTS = ["base", "cover_back_bread_board", "cover_smartphone_base",
              "smartphone_long", "smartphone_short"]
BREADBOARD_PARTS = ["bread_board_stopper", "pca9685_stopper", "mini_bread_board_stand"]

# mesh dir -> STEP file; links {link: (frame origin in STEP mm, part names)};
# parts to skip; shift {part: STEP mm}; extras {link: [simplified parts in STEP mm]}.
# A part name matches every instance ("arm_hand") or one instance ("arm_hand#2").
ASSEMBLIES = {
    "smabo_base": dict(
        step="smabo_base.step",
        links={"base_link": ((50.0, 50.0, 0.0), BASE_PARTS)},
    ),
    "smabo_breadboard": dict(
        step="smabo_base_breadboard.step",
        links={"breadboard_link": ((50.0, 50.0, 0.0), BREADBOARD_PARTS)},
        skip=BASE_PARTS,
    ),
    "smabo_arm": dict(
        step="smabo_arm.step",
        links={
            "arm_base_link": ((40.0, 50.0, 90.0), ["sg90_1_L", "sg90_1_R", "pca9685_stopper"]),
            "arm_link_1": ((40.0, 50.0, 105.0), ["sg90_2", "sg90_3_concave", "sg90_3_convex"]),
            "arm_link_2": ((40.0, 50.0, 141.5), ["sg90_4_concave", "sg90_4_convex"]),
            "arm_link_3": ((40.0, 50.0, 193.0), ["sg90_5_concave", "sg90_5_convex"]),
            "arm_link_4": ((27.5, 54.8, 236.45), ["sg90_5_hand"]),
        },
        skip=BASE_PARTS,
        shift={"sg90_4": (0, 0, -1.0), "sg90_5": (0, 0, -1.0)},
        extras={
            "arm_base_link": [place(sg90(4.85), (40.0, 50.0, 108.75), (0, 0, 1), (1, 0, 0))],
            "arm_link_1": [place(sg90(4.5), (40.0, 28.75, 141.5), (0, -1, 0), (0, 0, -1))],
            "arm_link_2": [place(sg90(4.5), (40.0, 29.75, 193.0), (0, -1, 0), (0, 0, -1))],
            "arm_link_3": [place(sg90(5.3), (26.0, 54.8, 236.45), (-1, 0, 0), (0, -1, 0))],
        },
    ),
    "smabo_hand": dict(
        step="smabo_hand.step",
        links={
            "hand_mount_link": ((50.0, 50.0, 0.0), ["sg90_plate_1", "sg90_plate_2"]),
            "left_hand_link": ((49.7, -2.0, 50.0), ["sg90_base#1", "arm_hand#1"]),
            "right_hand_link": ((50.3, 102.0, 50.0), ["sg90_base#2", "arm_hand#2"]),
        },
        skip=BASE_PARTS + BREADBOARD_PARTS,
        shift={"sg90_base#1": (-6.9, 0, 0), "arm_hand#1": (-6.9, 0, 0),
               "sg90_base#2": (6.9, 0, 0), "arm_hand#2": (6.9, 0, 0)},
        extras={"hand_mount_link": [
            place(sg90(4.85), (49.7, -5.75, 50.0), (0, -1, 0), (-1, 0, 0)),
            place(sg90(4.85), (50.3, 105.75, 50.0), (0, 1, 0), (1, 0, 0)),
        ]},
    ),
    "smabo_tt_motor": dict(
        step="smabo_tt_motor.step",
        links={
            "chassis_link": ((50.0, 50.0, 0.0), ["tt_motor_bottom", "tt_motor_stopper1",
                                                 "tt_motor_stopper2", "dc_motor_caster"]),
            "left_wheel_link": ((71.5, -15.0, -19.85), []),
            "right_wheel_link": ((71.5, 115.0, -19.85), []),
        },
        skip=BASE_PARTS + BREADBOARD_PARTS,
        extras={
            "chassis_link": [place(tt_motor(1), (71.5, 10.0, -19.85), (0, -1, 0), (-1, 0, 0)),
                             place(tt_motor(-1), (71.5, 90.0, -19.85), (0, 1, 0), (-1, 0, 0)),
                             # under dc_motor_caster (bottom z -39) down to the wheel bottom (-19.85 - 30)
                             place(caster(-39.0 - (-19.85 - 30.0)), (9.0, 50.0, -39.0), (0, 0, 1))],
            "left_wheel_link": [place(wheel(), (71.5, -15.0, -19.85), (0, 1, 0))],
            "right_wheel_link": [place(wheel(), (71.5, 115.0, -19.85), (0, 1, 0))],
        },
    ),
    "smabo_neck": dict(
        step="smabo_neck.step",
        links={
            "pan_stand_link": ((50.0, 50.0, -58.8), ["pan_stand_left", "pan_stand_right"]),
            "pan_link": ((50.0, 50.0, -15.0), ["pan_plate_top", "pan_adapter"]),
        },
        skip=BASE_PARTS + BREADBOARD_PARTS,
        extras={"pan_stand_link": [place(mg996r(), (50.0, 50.0, -8.0), (0, 0, 1), (-1, 0, 0))]},
    ),
}

def matches(key, prefixes):
    name = key.split("#")[0]
    return any(p == key or name == p or name.startswith(p + "_") for p in prefixes)

def nm(l):
    n = TDataStd_Name()
    return n.Get().ToExtString() if l.FindAttribute(TDataStd_Name.GetID_s(), n) else "?"

def tess(shape, defl, ang):
    shape = BRepBuilderAPI_Copy(shape).Shape()          # fresh triangulation per resolution
    BRepMesh_IncrementalMesh(shape, defl, False, ang, True)
    verts, tris = [], []
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = TopoDS.Face_s(exp.Current()); loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation_s(face, loc)
        if tri is not None:
            trsf = loc.Transformation(); base = len(verts)
            for i in range(1, tri.NbNodes()+1):
                p = tri.Node(i).Transformed(trsf); verts.append((p.X(), p.Y(), p.Z()))
            rev = face.Orientation() == 1
            for i in range(1, tri.NbTriangles()+1):
                a, b, c = tri.Triangle(i).Get()
                if rev: a, c = c, a
                tris.append((base+a-1, base+b-1, base+c-1))
        exp.Next()
    return trimesh.Trimesh(np.array(verts), np.array(tris), process=True)

def split(out, step, links, skip=(), shift={}, extras={}):
    doc = TDocStd_Document(TCollection_ExtendedString("d"))
    r = STEPCAFControl_Reader(); r.ReadFile(os.path.join(STEP_DIR, step)); r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    free = TDF_LabelSequence(); st.GetFreeShapes(free)
    comps = TDF_LabelSequence(); st.GetComponents_s(free.Value(1), comps)
    parts = {k: {"visual": [], "collision": []} for k in links}
    count = {}
    for i in range(1, comps.Length()+1):
        c = comps.Value(i); name = nm(c).split(" v")[0].replace(" ", "_")
        if name == "smabo_base": continue                     # sub-assembly node
        count[name] = count.get(name, 0) + 1; key = f"{name}#{count[name]}"
        if matches(key, skip): continue                       # belongs to another module
        link = next((l for l, (_, pre) in links.items() if matches(key, pre)), None)
        if link is None: raise SystemExit(f"{step}: unassigned part {key}")
        d = next((np.array(v) for p, v in shift.items() if matches(key, [p])), np.zeros(3))
        print(f"  {key:40s} -> {link}")
        for kind, (defl, ang) in (("visual", (0.2, 0.4)), ("collision", (1.0, 0.8))):
            m = tess(st.GetShape_s(c), defl, ang); m.apply_translation(d); parts[link][kind].append(m)
    for link, (origin, _) in links.items():
        for kind, ms in parts[link].items():
            m = trimesh.util.concatenate(ms + list(extras.get(link, [])))
            m.apply_translation(-np.array(origin)); m.apply_transform(STEP_TO_ROS); m.apply_scale(0.001)
            os.makedirs(f"{out}/{kind}", exist_ok=True)
            m.export(f"{out}/{kind}/{link}.stl")
            print(f"{kind:9s} {link:16s} tris={len(m.faces):6d} bounds(m)={np.round(m.bounds, 4).tolist()}")

if __name__ == "__main__":
    only = sys.argv[2:] or list(ASSEMBLIES)
    for mesh_dir in only:
        print("==", mesh_dir)
        split(os.path.join(sys.argv[1], mesh_dir), **ASSEMBLIES[mesh_dir])
