"""Inverse kinematics for the Franka arm: Cartesian tool targets in, joint configurations out. Pure numpy, no Isaac import.

Damped least squares on the 6-D pose error of the tool-centre point (the fingertip centre, 0.1034 m along the hand's z axis), using
the analytic Jacobian from pal_kinematics. The arm has seven joints for a six-dimensional target, so the spare degree of freedom is
used to stay near a reference posture (null-space bias), which keeps successive solutions on the same elbow branch.

This is what lets a *task* (go above the cube, descend, grasp, lift, carry, place) be expressed as joint waypoints. The trajectory
styles in pal_trajectories (trapezoidal, S-curve, torque-aware) then move between those waypoints in joint space, and the same
computed-torque controller tracks them, whatever the style or payload.
"""
import numpy as np

import pal_config as cfg

TOOL_OFFSET = (0.0, 0.0, 0.1034)           # the Franka TCP; the physical grasp uses the pad centre instead (pal_tasks.GRASP_TOOL_OFFSET)
DOWN = np.diag([1.0, -1.0, -1.0])         # tool z axis pointing straight down (a rotation of pi about x)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def orientation_error(R, R_target):
    """Rotation vector (small-angle) taking frame R onto R_target, in the world frame."""
    return 0.5 * sum(np.cross(R[:, i], R_target[:, i]) for i in range(3))


def solve(urdf, p_target, R_target, q_seed, q_ref=None, tol_pos=1e-9, tol_rot=1e-8, iters=200, lam=0.03, step_max=0.15,
          null_gain=0.1, offset=TOOL_OFFSET):
    """Return (q, converged, (position error m, rotation error rad)). Joint limits are enforced by clipping each step.

    Works for a redundant arm (the Panda's 7 joints, where the null-space bias holds the elbow on
    one branch) and for a square one (the UR10e's 6, where the null space is empty and the bias
    term falls out of the arithmetic by itself)."""
    n = urdf.dof
    q = np.array(q_seed, float)
    q_ref = cfg.HOME_JOINTS if q_ref is None else q_ref
    q_min, q_max = cfg.Q_MIN, cfg.Q_MAX
    for _ in range(iters):
        R, p, J = urdf.tool_jacobian(q, offset=offset)
        e = np.concatenate([p_target - p, orientation_error(R, R_target)])
        ep, er = np.linalg.norm(e[:3]), np.linalg.norm(e[3:])
        if ep < tol_pos and er < tol_rot:
            return q, True, (ep, er)
        # Exact pseudo-inverse away from singularities (damping makes J+J an inexact projector, so the posture bias leaks into the
        # task error and the solver stalls); damped least squares only when the smallest singular value is small.
        if np.linalg.svd(J, compute_uv=False)[-1] > 0.02:
            Jp = np.linalg.pinv(J)
        else:
            Jp = J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(6))
        dq = Jp @ e + (np.eye(n) - Jp @ J) @ (null_gain * (q_ref - q))
        m = np.abs(dq).max()
        if m > step_max:
            dq *= step_max / m
        q = np.clip(q + dq, q_min + 1e-3, q_max - 1e-3)
    R, p, _ = urdf.tool_jacobian(q, offset=offset)
    return q, False, (float(np.linalg.norm(p_target - p)), float(np.linalg.norm(orientation_error(R, R_target))))


def tool_target_from_home(urdf):
    """The home pose's tool orientation, which has the gripper pointing down; targets keep it so the hand stays level."""
    R, p, _ = urdf.tool_jacobian(cfg.HOME_JOINTS)
    return R, p


def solve_path(urdf, points, R_target, q_seed, **kw):
    """IK for a sequence of tool positions, each warm-started from the previous solution (so the elbow does not flip).
    Raises if any point is unreachable. Returns an (n, 7) array of joint configurations."""
    q, out = np.array(q_seed, float), []
    for i, p in enumerate(points):
        q, ok, err = solve(urdf, p, R_target, q, **kw)
        if not ok:
            raise ValueError(f"IK did not converge at path point {i} ({np.round(p, 3).tolist()}), error {err}")
        out.append(q.copy())
    return np.array(out)
