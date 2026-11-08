"""Independent URDF kinematics and gravity, plain numpy (no PhysX, no Pinocchio).

Used by tools/check_run.py (host) and pal_selftest.py (in the sim) to recompute quantities the
simulator also reports, so a plant that disagrees with the URDF cannot pass unnoticed.
`mass_scale` scales the moving arm links the same way pal_plant.Plant.set_inertials does.

Arm-agnostic since 2026-11-08. Nothing here is named after a particular robot any more: the root
link, the actuated joints, the links that `mass_scale` applies to and the tool frame are all read
out of the URDF (and, where it needs to pick between several candidates, out of `pal_robot`). The
Nothing about the arm is hard-coded: the root, the arm joints, the driven links and the tool
frame are all inferred from the URDF, so the same code reads any arm's.
"""
import xml.etree.ElementTree as ET

import numpy as np

import pal_robot

G = 9.81
ARM = pal_robot.active().dof      # the active arm's joint count; prefer Urdf.dof, which is
                                  # read from the URDF being parsed rather than from the default


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @
            np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @
            np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def rot_axis(a, th):
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


class Urdf:
    def __init__(self, path, mass_scale=1.0, payload_kg=0.0, payload_offset=None,
                 arm_joints=None, tool_link=None):
        """Read a URDF's kinematics and inertials.

        `arm_joints` names the actuated joints, in order; `tool_link` is the frame the tool offset
        and the payload hang from. Both are inferred when left out: the actuated joints are the
        revolute ones along the longest chain from the root, and the tool is that chain's last
        link. `mass_scale` multiplies the mass and inertia of every link driven by an arm joint,
        which is the plant/model mismatch pal_plant.set_inertials applies."""
        root = ET.parse(path).getroot()
        self.joints = []
        for j in root.findall("joint"):
            o, ax = j.find("origin"), j.find("axis")
            self.joints.append({
                "name": j.get("name"), "type": j.get("type"),
                "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                "xyz": np.array([float(x) for x in o.get("xyz").split()]) if o is not None and o.get("xyz") else np.zeros(3),
                "rpy": [float(x) for x in o.get("rpy").split()] if o is not None and o.get("rpy") else [0, 0, 0],
                "axis": np.array([float(x) for x in ax.get("xyz").split()]) if ax is not None else np.array([0., 0, 1])})

        links = {l.get("name") for l in root.findall("link")}
        children = {j["child"] for j in self.joints}
        roots = sorted(links - children)
        if len(roots) != 1:
            raise ValueError(f"{path}: expected exactly one root link, found {roots}")
        self.root = roots[0]
        self.parent_of = {j["child"]: j["parent"] for j in self.joints}
        self.joint_of_child_name = {j["child"]: j for j in self.joints}

        if arm_joints is None:
            arm_joints = self._infer_arm_joints()
        self.arm_joints = tuple(arm_joints)
        by_name = {j["name"]: j for j in self.joints}
        missing = [n for n in self.arm_joints if n not in by_name]
        if missing:
            raise ValueError(f"{path}: no such joint(s) {missing}")
        self.dof = len(self.arm_joints)
        self.jindex = {n: k for k, n in enumerate(self.arm_joints)}
        # The links an arm joint drives directly: each arm joint's child. This is what a
        # plant-vs-model mass mismatch is applied to, so it must be the driven links and not the
        # whole URDF — scaling a fixed base would change nothing in g(q) and everything in a
        # total-mass check.
        self.moving_links = tuple(by_name[n]["child"] for n in self.arm_joints)
        # The tool frame: what the tool offset and the payload hang from. Callers should pass it
        # (pal_robot.RobotSpec.tool_frame); inferred otherwise as the last driven link.
        self.tool = tool_link or self.moving_links[-1]
        if payload_offset is None:
            payload_offset = tuple(pal_robot.active().tool_offset)

        self.mass, self.com, self.inertia = {}, {}, {}
        for l in root.findall("link"):
            it = l.find("inertial")
            if it is None:
                continue
            o = it.find("origin")
            nm = l.get("name")
            i_el = it.find("inertia")
            Ixx, Iyy, Izz = (float(i_el.get(k)) for k in ("ixx", "iyy", "izz"))
            Ixy, Ixz, Iyz = (float(i_el.get(k)) for k in ("ixy", "ixz", "iyz"))
            s = mass_scale if nm in self.moving_links else 1.0
            self.inertia[nm] = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]]) * s
            self.mass[nm] = float(it.find("mass").get("value")) * s
            self.com[nm] = (np.array([float(x) for x in o.get("xyz").split()])
                            if o is not None and o.get("xyz") else np.zeros(3))
            # A URDF may rotate the inertia frame relative to the link (the UR10e does on three
            # does not). Rotate the tensor into link axes so every consumer sees one convention.
            if o is not None and o.get("rpy"):
                Rin = rpy(*[float(x) for x in o.get("rpy").split()])
                self.inertia[nm] = Rin @ self.inertia[nm] @ Rin.T
        self.payload = (payload_kg, np.asarray(payload_offset, float))

    def _infer_arm_joints(self):
        """Revolute joints along the longest chain out of the root, in order from the base."""
        kids = {}
        for j in self.joints:
            kids.setdefault(j["parent"], []).append(j)
        best = []

        def walk(link, path):
            nonlocal best
            if len(path) > len(best):
                best = list(path)
            for j in kids.get(link, ()):
                walk(j["child"], path + [j])

        walk(self.root, [])
        return tuple(j["name"] for j in best if j["type"] == "revolute")

    def com_positions(self, q, aux=0.0):
        """World COM of every massive link for arm angles q and an open-aux offset."""
        pose = self._poses(q, aux)
        out = {name: (m, pose[name][1] + pose[name][0] @ self.com[name]) for name, m in self.mass.items()}
        if self.payload[0] > 0:
            R, p = pose[self.tool]
            out["payload"] = (self.payload[0], p + R @ self.payload[1])
        return out

    def _poses(self, q, aux=0.0):
        return self._forward(q, aux)[0]

    def _forward(self, q, aux=0.0):
        """Link poses and world joint axes/origins for arm angles q: (pose, axis_w, origin_w, joint_of_child)."""
        pose = {self.root: (np.eye(3), np.zeros(3))}
        axis_w, origin_w, joint_of_child = {}, {}, {}
        todo = list(self.joints)
        while todo:
            rest = []
            for j in todo:
                if j["parent"] not in pose:
                    rest.append(j); continue
                R, p = pose[j["parent"]]
                Rj = R @ rpy(*j["rpy"]); pj = p + R @ j["xyz"]
                k = self.jindex.get(j["name"])
                if k is not None:                       # a driven arm joint
                    axis_w[k] = Rj @ (j["axis"] / np.linalg.norm(j["axis"])); origin_w[k] = pj
                    if j["type"] == "prismatic":
                        pj = pj + Rj @ (j["axis"] * q[k])
                    else:
                        Rj = Rj @ rot_axis(j["axis"], q[k])
                elif j["type"] == "prismatic" and aux is not None:
                    pj = pj + Rj @ (j["axis"] * aux)   # a non-arm prismatic joint, held open
                pose[j["child"]] = (Rj, pj)
                joint_of_child[j["child"]] = j
            if len(rest) == len(todo):
                break
            todo = rest
        return pose, axis_w, origin_w, joint_of_child

    def mass_matrix(self, q, aux=0.0, armature=0.0):
        """Joint-space mass matrix M(q) of the 7 arm joints, composite-rigid-body from the URDF.

        Sum over every massive link of  m Jv^T Jv + Jw^T (R I R^T) Jw,  with the full 3x3 inertia
        tensor (off-diagonal terms included). The gripper and fingers ride on link 7, so they enter
        through joints 1-7 only. `armature` (kg*m^2) is added to every diagonal term, which is how
        the plant's dynamics include the reflected rotor inertia.
        """
        pose, axis_w, origin_w, joint_of_child = self._forward(q, aux)
        parent_of = self.parent_of

        def ancestors(link):
            ks = []
            while link in parent_of:
                k = self.jindex.get(joint_of_child[link]["name"])
                if k is not None:
                    ks.append(k)
                link = parent_of[link]
            return ks

        n = self.dof
        M = np.zeros((n, n))
        bodies = [(nm, self.mass[nm], self.inertia[nm], self.com[nm]) for nm in self.mass]
        if self.payload[0] > 0:
            bodies.append(("payload", self.payload[0], np.zeros((3, 3)), self.payload[1]))
        for name, m, I, c in bodies:
            host = self.tool if name == "payload" else name
            R, p = pose[host]
            pc = p + R @ c
            Jv, Jw = np.zeros((3, n)), np.zeros((3, n))
            for k in ancestors(host):
                Jw[:, k] = axis_w[k]
                Jv[:, k] = np.cross(axis_w[k], pc - origin_w[k])
            Iw = R @ I @ R.T
            M += m * Jv.T @ Jv + Jw.T @ Iw @ Jw
        return M + armature * np.eye(n)

    def tool_jacobian(self, q, aux=0.0, offset=None):
        """Tool-centre-point pose and geometric Jacobian: (R 3x3, p 3, J 6xdof, rows [linear; angular], world frame).

        The TCP is `offset` along the tool frame's own axes — for the UR10e 0.195 m down the flange's
        z (the fingertip centre), for the UR10e whatever the mounted tool adds below the flange.
        It is the same point the payload is attached at."""
        if offset is None:
            offset = self.payload[1]
        pose, axis_w, origin_w, _ = self._forward(q, aux)
        R, p0 = pose[self.tool]
        p = p0 + R @ np.asarray(offset, float)
        J = np.zeros((6, self.dof))
        for k in range(self.dof):
            J[:3, k] = np.cross(axis_w[k], p - origin_w[k])
            J[3:, k] = axis_w[k]
        return R, p, J

    def gravity(self, q, aux=0.0):
        """tau_g = d(U)/dq with U = sum m g z, by central differences."""
        def U(qq):
            return sum(m * G * c[2] for m, c in self.com_positions(qq, aux).values())
        tau = np.zeros(self.dof)
        for i in range(self.dof):
            e = np.zeros(self.dof); e[i] = 1e-6
            tau[i] = (U(q + e) - U(q - e)) / 2e-6
        return tau


