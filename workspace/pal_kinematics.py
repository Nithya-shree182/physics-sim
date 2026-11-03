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
        self.mass, self.com = {}, {}
        for l in root.findall("link"):
            it = l.find("inertial")
            if it is None:
                continue
            o = it.find("origin")
            nm = l.get("name")
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

    def gravity(self, q, finger):
        """tau_g = d(U)/dq with U = sum m g z, by central differences."""
        def U(qq):
            return sum(m * G * c[2] for m, c in self.com_positions(qq, finger).values())
        tau = np.zeros(ARM)
        for i in range(ARM):
            e = np.zeros(ARM); e[i] = 1e-6
            tau[i] = (U(q + e) - U(q - e)) / 2e-6
        return tau


