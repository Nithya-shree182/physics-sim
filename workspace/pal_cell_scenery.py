"""The room around the cell: a fenced robot cell on a factory floor, a branded sign, and the
telemetry monitor on its stand.

Built to the reference the project owner supplied on 2026-11-08: a guarded UR10e cell with yellow
fence posts and wire mesh, an aluminium-extrusion conveyor, a wide monitor on a floor stand showing
live telemetry, and the depth of a real building behind it (roof trusses, strip lights, machinery).

Split out of pal_cell.py so that file stays about the robot's work. Nothing here has a collider:
none of it is ever touched, and collision shapes would only give the solver more pairs to test.

The monitor is the exception to "set dressing": it is a real readout, fed from the running
simulation. Everything animated is refreshed on rendered frames only, never per physics step —
doing it per step held the cell to half of real time, and looking prims up through USD each frame
cost another 15 s of a 24 s cycle (both measured 2026-11-08).
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

# ── palette ───────────────────────────────────────────────────────────────────────────────────
FLOOR_COL = (0.55, 0.56, 0.58)
WALL = (0.72, 0.74, 0.77)
ALU = (0.70, 0.72, 0.76)          # aluminium extrusion
STEEL = (0.60, 0.63, 0.68)
DARK = (0.17, 0.18, 0.21)
MESH = (0.26, 0.28, 0.32)
FENCE_YELLOW = (0.93, 0.76, 0.10)
LANE_YELLOW = (0.88, 0.74, 0.14)
SCREEN_BG = (0.04, 0.06, 0.11)
NTT_BLUE = (0.22, 0.45, 0.72)
BOARD_BLUE = (0.045, 0.145, 0.33)     # the facility board: NTT DATA's navy, not black
CYAN = (0.20, 0.78, 0.95)
AMBER = (0.98, 0.70, 0.15)
GREEN = (0.25, 0.80, 0.42)
RED = (0.92, 0.28, 0.26)
MAGENTA = (0.85, 0.40, 0.80)
TRACE_COLS = (CYAN, AMBER, GREEN, MAGENTA, (0.45, 0.65, 0.95), (0.95, 0.55, 0.35))
MACHINE_COLS = ((0.38, 0.40, 0.44), (0.30, 0.33, 0.38), (0.44, 0.46, 0.50))

LOGO_CANDIDATES = ("/workspace/assets_all/brand/ntt_data_logo.png",
                   str(Path(__file__).resolve().parents[1] / "assets" / "brand" / "ntt_data_logo.png"),
                   str(Path(__file__).resolve().parents[1] / "dashboard" / "assets" / "ntt_data_logo.png"))


def find_logo():
    for c in LOGO_CANDIDATES:
        if Path(c).exists():
            return c
    return None


SUBTITLE_CANDIDATES = ("/workspace/assets_all/brand/sign_india.png",
                       str(Path(__file__).resolve().parents[1] / "assets" / "brand" / "sign_india.png"))


def find_subtitle():
    """The "India Innovation Centre" line, made by tools/make_sign_texture.py."""
    for c in SUBTITLE_CANDIDATES:
        if Path(c).exists():
            return c
    return None


TELEMETRY_CANDIDATES = ("/workspace/assets_all/brand/sign_telemetry.png",
                        str(Path(__file__).resolve().parents[1] / "assets" / "brand" / "sign_telemetry.png"))


def find_telemetry_word():
    """The word on the monitor's header bar. The brand mark belongs on the facility sign; the
    screen says what it is showing."""
    for c in TELEMETRY_CANDIDATES:
        if Path(c).exists():
            return c
    return None


def cube(stage, path, size, color, translate=(0, 0, 0), rotate=None):
    """An axis-aligned box. Returns (prim, scaleOp, translateOp) so animated ones never need a
    USD lookup again."""
    from pxr import Gf, Sdf, UsdGeom
    if not Sdf.Path.IsValidPathString(path)[0]:
        raise ValueError(f"{path!r} is not a usable USD prim path")
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(1.0)
    x = UsdGeom.Xformable(g)
    tr = x.AddTranslateOp()
    tr.Set(Gf.Vec3d(*[float(v) for v in translate]))
    if rotate:
        x.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rotate]))
    sc = x.AddScaleOp()
    sc.Set(Gf.Vec3f(*[float(v) for v in size]))
    g.CreateDisplayColorAttr([Gf.Vec3f(*[float(c) for c in color])])
    return g, sc, tr


def cyl(stage, path, radius, height, color, translate=(0, 0, 0), rotate=None):
    from pxr import Gf, UsdGeom
    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateRadiusAttr(float(radius)); g.CreateHeightAttr(float(height)); g.CreateAxisAttr("Z")
    x = UsdGeom.Xformable(g)
    x.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translate]))
    rop = x.AddRotateXYZOp()
    rop.Set(Gf.Vec3f(*[float(v) for v in (rotate or (0, 0, 0))]))
    g.CreateDisplayColorAttr([Gf.Vec3f(*[float(c) for c in color])])
    return g, rop


def logo_quad(stage, path, width, height, translate, rotate=None, emissive=True, image=None):
    """A quad carrying a transparent PNG, facing -y before any rotation.

    The PNG is a white logo on transparency, so its alpha drives opacity: on a dark plate it reads
    as a lit sign. A missing file falls back to a plain blue plate and says so, because the scene
    must not die over a texture."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    img = image or find_logo()
    w, h = width / 2.0, height / 2.0
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(-w, 0, -h), Gf.Vec3f(w, 0, -h), Gf.Vec3f(w, 0, h), Gf.Vec3f(-w, 0, h)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, -1, 0)] * 4)
    mesh.CreateDoubleSidedAttr(True)
    x = UsdGeom.Xformable(mesh)
    x.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translate]))
    if rotate:
        x.AddRotateXYZOp().Set(Gf.Vec3f(*[float(v) for v in rotate]))
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                 UsdGeom.Tokens.varying)
    st.Set([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)])
    if img is None:
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*NTT_BLUE)])
        print(f"[cell] sign texture not found for {path}; using a plain plate", flush=True)
        return mesh

    mat = UsdShade.Material.Define(stage, f"{path}_Mat")
    sh = UsdShade.Shader.Define(stage, f"{path}_Mat/Surface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    reader = UsdShade.Shader.Define(stage, f"{path}_Mat/StReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    tex = UsdShade.Shader.Define(stage, f"{path}_Mat/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(img)
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    tex.CreateOutput("a", Sdf.ValueTypeNames.Float)
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    if emissive:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(tex.ConnectableAPI(), "a")
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    return mesh


# ── the building ──────────────────────────────────────────────────────────────────────────────
def build_factory(stage, root, floor_z, cell_x=(-1.9, 2.9), cell_y=(-1.6, 1.75)):
    """Floor markings, roof structure, lighting and some plant in the distance.

    Depth is what makes a render look like a photograph of a factory rather than a grey room, so
    the far wall sits well back and there is machinery, racking and columns between here and it."""
    roof = floor_z + 4.0
    cube(stage, f"{root}/FarWall", (26.0, 0.10, 5.0), WALL, (2.0, 11.0, floor_z + 2.5))
    cube(stage, f"{root}/SideWall", (0.10, 24.0, 5.0), WALL, (-9.0, 2.0, floor_z + 2.5))

    # yellow lane markings on the floor: the cell's own border, and a walkway beyond it
    for i, x in enumerate(cell_x):
        cube(stage, f"{root}/LaneX{i}", (0.09, cell_y[1] - cell_y[0] + 0.4, 0.004), LANE_YELLOW,
             (x, (cell_y[0] + cell_y[1]) / 2, floor_z + 0.003))
    for i, y in enumerate(cell_y):
        cube(stage, f"{root}/LaneY{i}", (cell_x[1] - cell_x[0] + 0.4, 0.09, 0.004), LANE_YELLOW,
             ((cell_x[0] + cell_x[1]) / 2, y, floor_z + 0.003))
    for i in range(10):                                        # walkway hatching further out
        cube(stage, f"{root}/Hatch{i}", (0.07, 1.5, 0.004), LANE_YELLOW,
             (-3.6 + i * 0.55, 3.4, floor_z + 0.003), (0, 0, 32))

    # roof: trusses across the bay, purlins along it, and rows of strip lights
    for i in range(9):
        y = -1.5 + i * 1.55
        cube(stage, f"{root}/Truss{i}", (22.0, 0.11, 0.30), (0.74, 0.75, 0.78), (2.0, y, roof))
        cube(stage, f"{root}/TrussLow{i}", (22.0, 0.08, 0.10), (0.68, 0.70, 0.73), (2.0, y, roof - 0.45))
    for i in range(7):
        cube(stage, f"{root}/Purlin{i}", (0.10, 20.0, 0.14), (0.70, 0.72, 0.75),
             (-6.0 + i * 2.6, 3.5, roof + 0.22))
    for r in range(4):
        for c in range(6):
            cube(stage, f"{root}/Light{r}{c}", (1.45, 0.17, 0.07), (0.98, 0.98, 0.96),
                 (-4.2 + c * 2.0, 0.6 + r * 2.3, roof - 0.62))

    # structural columns, and plant in the distance
    for i, (x, y) in enumerate(((-5.2, 4.2), (3.6, 4.2), (-5.2, 8.4), (3.6, 8.4))):
        cube(stage, f"{root}/Column{i}", (0.30, 0.30, 4.6), (0.78, 0.79, 0.81), (x, y, floor_z + 2.3))
    rng = np.random.default_rng(11)
    for i in range(10):
        w = float(rng.uniform(0.7, 1.9)); d = float(rng.uniform(0.6, 1.4)); h = float(rng.uniform(1.0, 2.4))
        x = float(rng.uniform(-8.0, 8.0)); y = float(rng.uniform(4.6, 10.2))
        cube(stage, f"{root}/Machine{i}", (w, d, h), MACHINE_COLS[i % len(MACHINE_COLS)],
             (x, y, floor_z + h / 2))
        if i % 3 == 0:
            cube(stage, f"{root}/MachineTop{i}", (w * 0.5, d * 0.5, 0.25), (0.86, 0.72, 0.20),
                 (x, y, floor_z + h + 0.12))


def build_fence(stage, root, floor_z, x0=-1.9, x1=2.9, y_far=1.75, x_left=-1.9, y0=-1.6,
                height=1.95, panel=1.20):
    """Guard fencing: yellow posts with dark wire mesh between them.

    Only the far side and the left run are built. The camera looks in across the open side, which
    is how a real cell is photographed and keeps the mesh from sitting between you and the robot."""
    def post(name, x, y):
        cube(stage, f"{root}/{name}", (0.075, 0.075, height), FENCE_YELLOW, (x, y, floor_z + height / 2))
        cube(stage, f"{root}/{name}Foot", (0.20, 0.20, 0.025), (0.40, 0.42, 0.46), (x, y, floor_z + 0.012))

    def mesh_panel(name, cx, cy, length, along_x):
        """A frame plus a grid of thin bars: enough to read as welded mesh at this distance."""
        sx, sy = (length, 0.02) if along_x else (0.02, length)
        cube(stage, f"{root}/{name}T", (sx, sy, 0.045), MESH, (cx, cy, floor_z + height - 0.10))
        cube(stage, f"{root}/{name}B", (sx, sy, 0.045), MESH, (cx, cy, floor_z + 0.16))
        n_v, n_h = 9, 5      # enough to read as welded mesh without flooding the renderer
        for i in range(n_v):
            f = (i + 0.5) / n_v - 0.5
            px = cx + (f * length if along_x else 0.0)
            py = cy + (0.0 if along_x else f * length)
            cube(stage, f"{root}/{name}V{i}", (0.012, 0.012, height - 0.32), MESH,
                 (px, py, floor_z + 0.16 + (height - 0.32) / 2))
        for j in range(n_h):
            z = floor_z + 0.20 + j * (height - 0.40) / (n_h - 1)
            cube(stage, f"{root}/{name}H{j}", (sx, sy, 0.012), MESH, (cx, cy, z))

    n_far = max(1, int(round((x1 - x0) / panel)))
    for i in range(n_far + 1):
        post(f"PostF{i}", x0 + i * (x1 - x0) / n_far, y_far)
    for i in range(n_far):
        a = x0 + i * (x1 - x0) / n_far
        mesh_panel(f"MeshF{i}", a + (x1 - x0) / n_far / 2, y_far, (x1 - x0) / n_far - 0.08, True)

    n_left = max(1, int(round((y_far - y0) / panel)))
    for i in range(n_left + 1):
        post(f"PostL{i}", x_left, y0 + i * (y_far - y0) / n_left)
    for i in range(n_left):
        a = y0 + i * (y_far - y0) / n_left
        mesh_panel(f"MeshL{i}", x_left, a + (y_far - y0) / n_left / 2, (y_far - y0) / n_left - 0.08, False)


def build_sign(stage, root, floor_z, x=-0.62, y=1.70, z=1.50, width=1.00):
    """The NTT DATA mark on a dark board bolted to the fence, with the site name under it."""
    h = width * 184.0 / 900.0                                  # the logo PNG's own aspect
    sub_w = width * 0.80
    sub_h = sub_w * 210.0 / 1400.0                             # the generated text's aspect
    gap = 0.045
    plate_h = h + sub_h + gap + 0.17
    cz = floor_z + z
    cube(stage, f"{root}/SignPlate", (width + 0.17, 0.035, plate_h), BOARD_BLUE,
         (x, y - 0.05, cz))
    logo_z = cz + plate_h / 2 - 0.075 - h / 2
    sub_z = logo_z - h / 2 - gap - sub_h / 2
    logo_quad(stage, f"{root}/SignLogo", width, h, (x, y - 0.075, logo_z))
    # a hairline between the mark and the site name, as on a real facility board
    cube(stage, f"{root}/SignRule", (width * 0.72, 0.02, 0.004), (0.42, 0.52, 0.66),
         (x, y - 0.072, logo_z - h / 2 - gap * 0.45))
    logo_quad(stage, f"{root}/SignSubtitle", sub_w, sub_h, (x, y - 0.075, sub_z),
              image=find_subtitle())
    cube(stage, f"{root}/SignEdge", (width + 0.21, 0.02, 0.022), NTT_BLUE,
         (x, y - 0.06, cz - plate_h / 2 - 0.016))


def build_bollards(stage, root, floor_z, spots=((2.55, -1.35), (2.55, 0.25))):
    for i, (x, y) in enumerate(spots):
        cyl(stage, f"{root}/Bollard{i}", 0.055, 0.85, FENCE_YELLOW, (x, y, floor_z + 0.425))
        cyl(stage, f"{root}/BollardCap{i}", 0.058, 0.10, DARK, (x, y, floor_z + 0.86))


# ── the telemetry monitor ─────────────────────────────────────────────────────────────────────
class TelemetryScreen:
    """A wide monitor on a floor stand, angled at the camera, carrying live readouts.

    A trace per joint of its torque against its own limit, a second panel tracing the cell's
    throughput, a bar per joint and a row of status lamps — all fed from the running simulation.
    Drawn with scaled boxes rather than a rendered texture, so there is no image to generate and
    no file to write per frame."""

    # Everything drawn on the screen sits this far in FRONT of the monitor's centre. The bezel is
    # 45 mm thick and centred on that point, so a depth of a few millimetres buries the whole
    # readout inside it and the screen renders as a blank slab (seen 2026-11-08).
    FACE = -0.034
    N_TRACE = 24
    TRACE_EVERY = 4            # traces advance every fourth drawn frame: ~7 Hz, still live

    def __init__(self, stage, root, dof, floor_z, x=1.95, y=-0.75, yaw=-24.0,
                 w=1.32, h=0.80, z_centre=1.35):
        self.stage, self.dof = stage, dof
        self.w, self.h, self.yaw = w, h, yaw
        p = f"{root}/Monitor"
        self.root = p
        cz = floor_z + z_centre

        # stand: base plate, pole, head
        cube(stage, f"{p}/Base", (0.70, 0.50, 0.035), (0.30, 0.32, 0.36), (x, y, floor_z + 0.018), (0, 0, yaw))
        cyl(stage, f"{p}/Pole", 0.045, z_centre - 0.10, (0.38, 0.40, 0.44), (x, y, floor_z + (z_centre - 0.10) / 2))
        cube(stage, f"{p}/Yoke", (0.10, 0.12, 0.16), (0.30, 0.32, 0.36), (x, y, cz - h / 2 - 0.02), (0, 0, yaw))
        cube(stage, f"{p}/Bezel", (w + 0.05, 0.045, h + 0.05), (0.07, 0.08, 0.10), (x, y, cz), (0, 0, yaw))

        # Everything on the screen face is laid out in the monitor's own frame and then rotated
        # with it, so the panel can be angled at the camera without hand-placing each element.
        self._yaw = np.radians(yaw)
        self._origin = np.array([x, y, cz])
        self._depth = -0.03

        self._face(stage, f"{p}/Panel", (w, 0.012, h), SCREEN_BG, (0.0, 0.0), self.FACE)
        self._face(stage, f"{p}/Header", (w, 0.010, 0.09), (0.07, 0.11, 0.19), (0.0, h / 2 - 0.055), self.FACE - 0.008)
        self._face(stage, f"{p}/HeaderRule", (w, 0.008, 0.006), NTT_BLUE, (0.0, h / 2 - 0.101), self.FACE - 0.010)
        lw = 0.46
        self._logo_on_face(stage, f"{p}/HeaderTitle", lw, lw * 200.0 / 900.0,
                           (-w / 2 + 0.29, h / 2 - 0.055), image=find_telemetry_word())

        gw = w - 0.14
        self.traces, self.hist, self._trace_cols = [], [], []
        tops = (h / 2 - 0.155, h / 2 - 0.475)
        for g, ztop in enumerate(tops):
            self._face(stage, f"{p}/GraphBg{g}", (gw + 0.03, 0.009, 0.285), (0.06, 0.09, 0.14),
                       (0.0, ztop - 0.142), self.FACE - 0.006)
            for k in range(4):
                self._face(stage, f"{p}/Grid{g}{k}", (gw, 0.007, 0.0035), (0.13, 0.17, 0.23),
                           (0.0, ztop - 0.03 - k * 0.075), self.FACE - 0.009)
            n_series = dof if g == 0 else 1
            bw = gw / self.N_TRACE
            series = []
            for sIdx in range(n_series):
                col = TRACE_COLS[sIdx % len(TRACE_COLS)] if g == 0 else AMBER
                bars = []
                for i in range(self.N_TRACE):
                    fx = -gw / 2 + bw * (i + 0.5)
                    _, sc, tr = self._face(stage, f"{p}/Tr{g}_{sIdx}_{i}", (bw * 0.60, 0.006, 0.01), col,
                                           (fx, ztop - 0.275), self.FACE - 0.012 - 0.0012 * sIdx)
                    bars.append((sc, tr, fx, ztop - 0.275, bw * 0.60, self.FACE - 0.012 - 0.0012 * sIdx))
                series.append(bars)
            self.traces.append(series)
            self.hist.append([[0.0] * self.N_TRACE for _ in range(n_series)])

        # a bar per joint along the bottom
        self.joint_bars = []
        bz = -h / 2 + 0.105
        bw2 = (w - 0.26) / max(dof, 1)
        self._face(stage, f"{p}/JointBg", (w - 0.10, 0.009, 0.185), (0.06, 0.09, 0.14), (0.0, bz + 0.015), self.FACE - 0.006)
        for j in range(dof):
            jx = -(w - 0.26) / 2 + bw2 * (j + 0.5)
            self._face(stage, f"{p}/JointSlot{j}", (bw2 * 0.52, 0.007, 0.135), (0.10, 0.14, 0.20),
                       (jx, bz + 0.035), self.FACE - 0.009)
            _, sc, tr = self._face(stage, f"{p}/JointBar{j}", (bw2 * 0.52, 0.005, 0.01),
                                   TRACE_COLS[j % len(TRACE_COLS)], (jx, bz - 0.032), self.FACE - 0.013)
            self.joint_bars.append((sc, tr, jx, bz - 0.032, bw2 * 0.52, self.FACE - 0.013))

        self.lamps = []
        for i in range(4):
            g, _, _ = self._face(stage, f"{p}/Lamp{i}", (0.045, 0.006, 0.045), (0.15, 0.19, 0.25),
                                 (w / 2 - 0.10 - i * 0.075, h / 2 - 0.055), self.FACE - 0.012)
            self.lamps.append(g)

        self._rng = np.random.default_rng(7)
        self._aux = 0.35
        self._frame = 0

    # -- placing things on the angled face -----------------------------------------------------
    def _world(self, fx, fz, depth):
        """Face coordinates (across, up) plus a depth offset -> world, through the monitor's yaw."""
        c, s = np.cos(self._yaw), np.sin(self._yaw)
        local = np.array([fx, depth, 0.0])
        return self._origin + np.array([c * local[0] - s * local[1], s * local[0] + c * local[1], fz])

    def _face(self, stage, path, size, color, fxz, depth):
        return cube(stage, path, size, color, self._world(fxz[0], fxz[1], depth), (0, 0, self.yaw))

    def _logo_on_face(self, stage, path, w, h, fxz, image=None):
        return logo_quad(stage, path, w, h, self._world(fxz[0], fxz[1], self.FACE - 0.014),
                         (0, 0, self.yaw), image=image)

    def _grow(self, bar, frac, full):
        """Grow a bar upward from its baseline, in the monitor's own frame."""
        from pxr import Gf
        sc, tr, fx, base_fz, width, depth = bar
        hgt = max(0.0035, full * float(np.clip(frac, 0.0, 1.0)))
        sc.Set(Gf.Vec3f(width, 0.006, hgt))
        tr.Set(Gf.Vec3d(*self._world(fx, base_fz + hgt / 2, depth)))

    # -- live update ---------------------------------------------------------------------------
    def update(self, tau_frac, throughput, t):
        from pxr import Gf
        self._frame += 1
        tau_frac = np.clip(np.asarray(tau_frac, float), 0.0, 1.0)
        worst = float(tau_frac.max()) if tau_frac.size else 0.0
        self._aux = float(np.clip(0.90 * self._aux + 0.10 * (throughput + self._rng.normal(0, 0.05)), 0.0, 1.0))

        if self._frame % self.TRACE_EVERY == 0:
            for sIdx, bars in enumerate(self.traces[0]):          # one trace per joint
                h = self.hist[0][sIdx]
                h.pop(0); h.append(float(tau_frac[sIdx]) if sIdx < tau_frac.size else 0.0)
                for i, bar in enumerate(bars):
                    self._grow(bar, h[i], 0.235)
            h = self.hist[1][0]
            h.pop(0); h.append(self._aux)
            for i, bar in enumerate(self.traces[1][0]):
                self._grow(bar, h[i], 0.235)

        for j, bar in enumerate(self.joint_bars):
            self._grow(bar, float(tau_frac[j]) if j < tau_frac.size else 0.0, 0.125)

        lamp_state = (GREEN if worst < 0.55 else AMBER if worst < 0.85 else RED,
                      CYAN if throughput > 0.05 else (0.15, 0.19, 0.25),
                      GREEN,
                      AMBER if int(t * 2) % 2 else (0.15, 0.19, 0.25))
        for g, col in zip(self.lamps, lamp_state):
            g.CreateDisplayColorAttr([Gf.Vec3f(*[float(c) for c in col])])


class ControlBox:
    """The cell's control station: a small cabinet beside the robot with lamps and buttons.

    Industrial cells always have one, and it gives the scene something that blinks at human scale
    rather than only on the big monitor. The lamps are driven from the run: power, cycle running,
    vacuum on, and a fault lamp."""

    def __init__(self, stage, root, floor_z, x=-0.46, y=-0.30, z_top=0.34, yaw=18.0):
        p = f"{root}/ControlBox"
        self.stage = stage
        bw, bd, bh = 0.30, 0.22, 0.42
        cz = floor_z + 0.62
        cube(stage, f"{p}/Post", (0.06, 0.06, 0.62), (0.38, 0.40, 0.44), (x, y, floor_z + 0.31))
        cube(stage, f"{p}/Foot", (0.20, 0.20, 0.02), (0.32, 0.34, 0.38), (x, y, floor_z + 0.01))
        cube(stage, f"{p}/Body", (bw, bd, bh), (0.42, 0.45, 0.50), (x, y, cz + bh / 2), (0, 0, yaw))
        cube(stage, f"{p}/Face", (bw - 0.03, 0.012, bh - 0.04), (0.20, 0.22, 0.26),
             (x, y - bd / 2 + 0.004, cz + bh / 2), (0, 0, yaw))
        # the red mushroom e-stop, and two momentary buttons
        cyl(stage, f"{p}/EStopRing", 0.038, 0.012, (0.92, 0.80, 0.12),
            (x - 0.07, y - bd / 2 - 0.004, cz + bh - 0.10), (90, 0, 0))
        cyl(stage, f"{p}/EStop", 0.028, 0.026, (0.82, 0.12, 0.10),
            (x - 0.07, y - bd / 2 - 0.016, cz + bh - 0.10), (90, 0, 0))
        for i, (dx, col) in enumerate(((0.03, (0.20, 0.70, 0.32)), (0.09, (0.25, 0.27, 0.31)))):
            cyl(stage, f"{p}/Button{i}", 0.017, 0.016, col,
                (x + dx, y - bd / 2 - 0.010, cz + bh - 0.10), (90, 0, 0))
        self.lamps = []
        for i in range(4):
            g, _, _ = cube(stage, f"{p}/Lamp{i}", (0.026, 0.010, 0.026), (0.18, 0.20, 0.24),
                           (x - 0.10 + i * 0.065, y - bd / 2 - 0.008, cz + 0.08), (0, 0, yaw))
            self.lamps.append(g)
        # a stack light on top, the thing you actually read across a factory floor
        for i, col in enumerate(((0.85, 0.18, 0.16), (0.92, 0.74, 0.14), (0.22, 0.72, 0.34))):
            g, _, _ = cube(stage, f"{p}/Stack{i}", (0.055, 0.055, 0.055), col,
                           (x, y, cz + bh + 0.04 + i * 0.058), (0, 0, yaw))
            self.lamps.append(g)
        cyl(stage, f"{p}/StackCap", 0.032, 0.02, (0.26, 0.28, 0.32), (x, y, cz + bh + 0.21))

    def update(self, running, vacuum, worst_frac, t):
        from pxr import Gf
        off = (0.17, 0.19, 0.23)
        blink = int(t * 2) % 2 == 0
        states = [
            GREEN,                                           # power
            CYAN if running else off,                        # cycle running
            AMBER if vacuum else off,                        # vacuum
            RED if worst_frac > 0.85 else off,               # overload warning
            RED if worst_frac > 0.85 and blink else (0.30, 0.10, 0.10),   # stack: red
            AMBER if 0.55 < worst_frac <= 0.85 else (0.30, 0.26, 0.10),   # stack: amber
            GREEN if worst_frac <= 0.55 else (0.12, 0.26, 0.16),          # stack: green
        ]
        for g, col in zip(self.lamps, states):
            g.CreateDisplayColorAttr([Gf.Vec3f(*[float(c) for c in col])])


def relight(stage, floor_z):
    """Soften the Panda-era studio lighting for a room this size.

    The old key light was aimed at a single arm on a table; in a building with a roof it throws
    hard diagonal bars of shadow across everything. More ambient, less key, plus a couple of
    broad overheads over the cell itself."""
    from pxr import Gf, UsdGeom, UsdLux
    dome = UsdLux.DomeLight.Get(stage, "/World/Lights/DomeLight")
    if dome:
        dome.CreateIntensityAttr(620.0)
        dome.CreateColorAttr(Gf.Vec3f(0.93, 0.95, 1.0))
    key = UsdLux.DistantLight.Get(stage, "/World/Lights/KeyLight")
    if key:
        # A hard, bright key threw sharp bars of shadow through the roof trusses and blew the
        # white surfaces out. Dimmer, and with a wide angular size so its shadows are soft.
        key.CreateIntensityAttr(150.0)
        key.CreateAngleAttr(14.0)
    for i, (x, y) in enumerate(((0.3, -0.3), (1.4, 0.8))):
        r = UsdLux.RectLight.Define(stage, f"/World/Lights/CellPanel{i}")
        r.CreateIntensityAttr(1200.0)
        r.CreateWidthAttr(1.8); r.CreateHeightAttr(1.1)
        r.CreateColorAttr(Gf.Vec3f(1.0, 0.99, 0.96))
        xf = UsdGeom.Xformable(r)
        xf.AddTranslateOp().Set(Gf.Vec3d(x, y, floor_z + 3.1))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(90.0, 0.0, 0.0))
