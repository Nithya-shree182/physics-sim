#!/usr/bin/env python3
"""Sanity-check a captured telemetry run, independently of the simulator's own bookkeeping.

    .venv-tools/bin/python tools/check_run.py data/slow_reference_0kg_1000hz.parquet

Three checks, each with its tolerance stated in the output rather than buried:

 1. SCHEMA     schema/validate.py exits 0.
 2. GRAVITY    g(q) the simulator reports, against g(q) computed here from the URDF with plain
               numpy forward kinematics (no PhysX, no Pinocchio) at the LOGGED q. This catches
               a plant whose masses/COMs do not match the URDF the analysis side models.
 3. IDENTITY   the proposal's O2 test on simulator terms: tau_measured vs M qdd + C + g,
               worst joint as % of the joint's torque limit, against the 5% pass condition
               (proposal section 9). Simulator terms only, so it tests the plant, not the
               analysis side's model.

A --urdf option points check 2 at a different URDF; running it against a copy with one link
mass raised 15% makes check 2 FAIL (0.56 N*m on joint 4), which is the proof the check can fail.

Exit code 0 only if every check passes. Nothing here is a stored result: it recomputes from
the file each time.
"""
import argparse
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets/franka_panda/panda_arm_hand.urdf"
TAU_MAX = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])    # Franka datasheet limits
GRAVITY_TOL_NM = 0.15        # [CHOICE] abs N*m; sag of the stiff drive explains ~0.1 on loaded joints
IDENTITY_TOL_PCT = 5.0       # proposal section 9, O2
G = 9.81
ARM = 7


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
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.mass, self.com = {}, {}
        for l in root.findall("link"):
            it = l.find("inertial")
            if it is None:
                continue
            o = it.find("origin")
            self.mass[l.get("name")] = float(it.find("mass").get("value"))
            self.com[l.get("name")] = (np.array([float(x) for x in o.get("xyz").split()])
                                       if o is not None and o.get("xyz") else np.zeros(3))
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("parquet", type=Path)
    ap.add_argument("--urdf", type=Path, default=URDF, help="URDF for the independent gravity check")
    ap.add_argument("--samples", type=int, default=40, help="rows used for the URDF gravity check")
    a = ap.parse_args()
    ok = True

    r = subprocess.run([sys.executable, str(ROOT / "schema/validate.py"), str(a.parquet)],
                       capture_output=True, text=True)
    passed = r.returncode == 0
    ok &= passed
    print(f"1 SCHEMA    {'PASS' if passed else 'FAIL'}   (schema/validate.py exit {r.returncode})")
    if not passed:
        print(r.stdout[-600:], r.stderr[-300:])

    d = pd.read_parquet(a.parquet)
    n = len(d)
    arr = lambda p: np.stack([d[f"{p}_{i}"].to_numpy() for i in range(ARM)], axis=1)
    q, qd, tm, g_sim, c = arr("q"), arr("qd"), arr("tau_measured"), arr("g"), arr("coriolis")
    M = np.stack([[d[f"M_{r_}_{c_}"].to_numpy() for c_ in range(ARM)] for r_ in range(ARM)], axis=0)  # (7,7,n)
    dt = float(d["dt"].iloc[0])
    finger = float(d["finger_pos_0"].mean())

    urdf = Urdf(a.urdf)
    rows = np.linspace(int(0.1 * n), n - 1, a.samples).astype(int)
    g_urdf = np.array([urdf.gravity(q[k], finger) for k in rows])
    err = np.abs(g_urdf - g_sim[rows])
    worst = err.max(axis=0)
    passed = bool((worst < GRAVITY_TOL_NM).all())
    ok &= passed
    print(f"2 GRAVITY   {'PASS' if passed else 'FAIL'}   URDF-numpy vs simulator g(q), worst abs error per joint "
          f"[N*m]: {np.round(worst, 3).tolist()}  (tolerance {GRAVITY_TOL_NM})")

    qdd = np.vstack([np.zeros((1, ARM)), np.diff(qd, axis=0) / dt])      # backward difference
    res = np.array([tm[k] - (M[:, :, k] @ qdd[k] + c[k] + g_sim[k]) for k in range(int(0.1 * n), n)])
    rms_pct = 100 * np.sqrt((res ** 2).mean(axis=0)) / TAU_MAX
    passed = bool((rms_pct < IDENTITY_TOL_PCT).all())
    ok &= passed
    print(f"3 IDENTITY  {'PASS' if passed else 'FAIL'}   tau_measured vs M qdd + C + g, RMS % of torque limit per "
          f"joint: {np.round(rms_pct, 2).tolist()}  (limit {IDENTITY_TOL_PCT}%)")

    print("\nOVERALL", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
