"""Independent URDF kinematics and gravity, plain numpy (no PhysX, no Pinocchio).

Used by tools/check_run.py (host) and pal_selftest.py (in the sim) to recompute quantities the
simulator also reports, so a plant that disagrees with the URDF cannot pass unnoticed.
`mass_scale` scales arm links 1-7 the same way pal_plant.Plant.set_inertials does.
"""
import xml.etree.ElementTree as ET

import numpy as np

G = 9.81
ARM = 7
ARM_LINKS = tuple(f"panda_link{i}" for i in range(1, 8))


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
    def __init__(self, path, mass_scale=1.0, payload_kg=0.0, payload_offset=(0.0, 0.0, 0.1034)):
        root = ET.parse(path).getroot()
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
            self.inertia[nm] = np.array([[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]]) * (
                mass_scale if nm in ARM_LINKS else 1.0)
            self.mass[nm] = float(it.find("mass").get("value")) * (mass_scale if nm in ARM_LINKS else 1.0)
            self.com[l.get("name")] = (np.array([float(x) for x in o.get("xyz").split()])
                                       if o is not None and o.get("xyz") else np.zeros(3))
        self.payload = (payload_kg, np.asarray(payload_offset, float))
        self.joints = []
        for j in root.findall("joint"):
            o, ax = j.find("origin"), j.find("axis")
            self.joints.append({
                "name": j.get("name"), "type": j.get("type"),
                "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                "xyz": np.array([float(x) for x in o.get("xyz").split()]) if o is not None and o.get("xyz") else np.zeros(3),
                "rpy": [float(x) for x in o.get("rpy").split()] if o is not None and o.get("rpy") else [0, 0, 0],
                "axis": np.array([float(x) for x in ax.get("xyz").split()]) if ax is not None else np.array([0., 0, 1])})

    def com_positions(self, q, finger):
        """World COM of every massive link for arm angles q and an open-finger offset."""
        pose = {"panda_link0": (np.eye(3), np.zeros(3))}
        out = {}
        todo = list(self.joints)
        while todo:
            rest = []
            for j in todo:
                if j["parent"] not in pose:
                    rest.append(j); continue
                R, p = pose[j["parent"]]
                Rj = R @ rpy(*j["rpy"]); pj = p + R @ j["xyz"]
                if j["type"] == "revolute":
                    Rj = Rj @ rot_axis(j["axis"], q[int(j["name"][-1]) - 1])
                elif j["type"] == "prismatic":
                    pj = pj + Rj @ (j["axis"] * finger)
                pose[j["child"]] = (Rj, pj)
            if len(rest) == len(todo):
                break
            todo = rest
        for name, m in self.mass.items():
            R, p = pose[name]
            out[name] = (m, p + R @ self.com[name])
        if self.payload[0] > 0:
            R, p = pose["panda_hand"]
            out["payload"] = (self.payload[0], p + R @ self.payload[1])
        return out

    def _forward(self, q, finger=0.04):
        """Link poses and world joint axes/origins for arm angles q: (pose, axis_w, origin_w, joint_of_child)."""
        pose = {"panda_link0": (np.eye(3), np.zeros(3))}
        axis_w, origin_w, joint_of_child = {}, {}, {}
        todo = list(self.joints)
        while todo:
            rest = []
            for j in todo:
                if j["parent"] not in pose:
                    rest.append(j); continue
                R, p = pose[j["parent"]]
                Rj = R @ rpy(*j["rpy"]); pj = p + R @ j["xyz"]
                if j["type"] == "revolute":
                    k = int(j["name"][-1]) - 1
                    axis_w[k] = Rj @ (j["axis"] / np.linalg.norm(j["axis"])); origin_w[k] = pj
                    Rj = Rj @ rot_axis(j["axis"], q[k])
                elif j["type"] == "prismatic":
                    pj = pj + Rj @ (j["axis"] * finger)
                pose[j["child"]] = (Rj, pj)
                joint_of_child[j["child"]] = j
            if len(rest) == len(todo):
                break
            todo = rest
        return pose, axis_w, origin_w, joint_of_child

    def mass_matrix(self, q, finger=0.04, armature=0.0):
        """Joint-space mass matrix M(q) of the 7 arm joints, composite-rigid-body from the URDF.

        Sum over every massive link of  m Jv^T Jv + Jw^T (R I R^T) Jw,  with the full 3x3 inertia
        tensor (off-diagonal terms included). The gripper and fingers ride on link 7, so they enter
        through joints 1-7 only. `armature` (kg*m^2) is added to every diagonal term, which is how
        the plant's dynamics include the reflected rotor inertia.
        """
        pose, axis_w, origin_w, joint_of_child = self._forward(q, finger)
        parent_of = {j["child"]: j["parent"] for j in self.joints}

        def ancestors(link):
            ks = []
            while link in parent_of:
                j = joint_of_child[link]
                if j["type"] == "revolute":
                    ks.append(int(j["name"][-1]) - 1)
                link = parent_of[link]
            return ks

        M = np.zeros((ARM, ARM))
        bodies = [(n, self.mass[n], self.inertia[n], self.com[n]) for n in self.mass]
        if self.payload[0] > 0:
            bodies.append(("payload", self.payload[0], np.zeros((3, 3)), self.payload[1]))
        for name, m, I, c in bodies:
            host = "panda_hand" if name == "payload" else name
            R, p = pose[host]
            pc = p + R @ c
            Jv, Jw = np.zeros((3, ARM)), np.zeros((3, ARM))
            for k in ancestors(host):
                Jw[:, k] = axis_w[k]
                Jv[:, k] = np.cross(axis_w[k], pc - origin_w[k])
            Iw = R @ I @ R.T
            M += m * Jv.T @ Jv + Jw.T @ Iw @ Jw
        return M + armature * np.eye(ARM)

    def tool_jacobian(self, q, finger=0.04, offset=(0.0, 0.0, 0.1034)):
        """Tool-centre-point pose and geometric Jacobian: (R 3x3, p 3, J 6x7 with rows [linear; angular], world frame).

        The TCP is `offset` along the panda_hand z axis (0.1034 m, the fingertip centre), the same point the payload is attached at."""
        pose, axis_w, origin_w, _ = self._forward(q, finger)
        R, p0 = pose["panda_hand"]
        p = p0 + R @ np.asarray(offset, float)
        J = np.zeros((6, ARM))
        for k in range(ARM):
            J[:3, k] = np.cross(axis_w[k], p - origin_w[k])
            J[3:, k] = axis_w[k]
        return R, p, J

    def gravity(self, q, finger):
        """tau_g = d(U)/dq with U = sum m g z, by central differences."""
        def U(qq):
            return sum(m * G * c[2] for m, c in self.com_positions(qq, finger).values())
        tau = np.zeros(ARM)
        for i in range(ARM):
            e = np.zeros(ARM); e[i] = 1e-6
            tau[i] = (U(q + e) - U(q - e)) / 2e-6
        return tau


