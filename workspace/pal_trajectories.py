"""Reference trajectories. Pure numpy, no Isaac, so everything here is unit-tested.

Every trajectory is a callable  traj(t) -> (q, qd, qdd)  of 7-vectors, with analytic
derivatives, plus a `.duration` (None for endless ones).

  multisine(frac)      the 3-harmonic reference used for O2/O4/O5, amplitude set by `frac` of
                       the joint velocity limits (0.20 = inside the <=50% regime)
  hold(q)              stay at a pose
  cycle(q0, q1, kind)  O6: out-and-back between two joint configurations with one of the
                       three styles the proposal compares (section 7.6):
      trapezoidal      velocity-limit / acceleration-limit trapezoid
      s_curve          jerk-limited 7-segment profile
      torque_aware     quintic (minimum-jerk) with a duration CHOSEN to respect a torque
                       budget; the o6 / o6pick suites run it over a grid of durations

Joints move together: each style is a scalar profile s(t) in [0, 1] scaled by the joint
displacement, with the scalar limits set by the most demanding joint.
"""
import numpy as np

import pal_config as cfg

ARM = cfg.DOF
# Multisine base frequencies, one per joint. The seven are the Panda's, chosen so no two joints
# share a harmonic; a different arm takes the first `ARM` of them, which keeps that property.
F0_ALL = np.array([.55, .45, .60, .50, .75, .70, .80])
F0 = F0_ALL[:ARM]
_K = sum(1.0 / (h + 1) for h in range(3))

# [CHOICE] kinematic limits for the O6 profiles. Velocity limits are the datasheet's (pal_config).
# The acceleration limit is a conservative 10 rad/s^2. The jerk limit is a SMOOTHNESS limit of
# 100 rad/s^3, far below the 7500 rad/s^3 Franka publishes as a hardware limit: at 7500 the jerk
# phases last ~1 ms and an S-curve is indistinguishable from a trapezoid (measured, first O6 run),
# so the hardware figure cannot be what an S-curve is limiting.
A_MAX = np.full(ARM, 10.0)
J_MAX = np.full(ARM, 100.0)


class Traj:
    def __init__(self, fn, duration=None):
        self.fn, self.duration = fn, duration

    def __call__(self, t):
        return self.fn(t)


def multisine(frac=0.20):
    A = frac * cfg.QD_MAX / (2 * np.pi * F0 * _K)

    def fn(t):
        q = cfg.HOME_JOINTS.copy(); qd = np.zeros(ARM); qdd = np.zeros(ARM)
        for h in range(3):
            w = 2 * np.pi * F0 * (h + 1); a = A / (h + 1) ** 2
            q += a * np.sin(w * t); qd += a * w * np.cos(w * t); qdd -= a * w * w * np.sin(w * t)
        return q, qd, qdd
    return Traj(fn)


def hold(q):
    q = np.asarray(q, float).copy()
    return Traj(lambda t: (q, np.zeros(ARM), np.zeros(ARM)))


# ---- scalar profiles s(t): 0 -> 1, rest to rest -----------------------------------------------
def _trap_scalar(vmax, amax):
    """Trapezoid for unit distance. Returns (T, fn(t) -> s, sd, sdd)."""
    if vmax * vmax / amax >= 1.0:            # never reaches vmax: triangle
        ta = np.sqrt(1.0 / amax); vp = amax * ta; tv = 0.0
    else:
        ta = vmax / amax; vp = vmax; tv = (1.0 - vmax * ta) / vmax
    T = 2 * ta + tv

    def fn(t):
        t = np.clip(t, 0.0, T)
        if t < ta:
            return 0.5 * amax * t * t, amax * t, amax
        if t < ta + tv:
            return 0.5 * amax * ta * ta + vp * (t - ta), vp, 0.0
        u = T - t
        return 1.0 - 0.5 * amax * u * u, amax * u, -amax
    return T, fn


def _scurve_scalar(vmax, amax, jmax):
    """Jerk-limited 7-segment profile for unit distance, built by integrating the jerk."""
    def phases(vp):
        if vp * jmax >= amax * amax:
            tj = amax / jmax; ta = vp / amax + tj
        else:
            tj = np.sqrt(vp / jmax); ta = 2 * tj
        return tj, ta, 0.5 * vp * ta               # tj, accel time, accel distance

    tj, ta, da = phases(vmax)
    if 2 * da > 1.0:                                # no cruise: lower the peak velocity
        lo, hi = 1e-6, vmax
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if 2 * phases(mid)[2] > 1.0: hi = mid
            else: lo = mid
        vp = lo; tj, ta, da = phases(vp)
    else:
        vp = vmax
    tv = (1.0 - 2 * da) / vp
    T = 2 * ta + tv
    dt = 2e-5
    jerk_segments = [(tj, +1), (ta - 2 * tj, 0), (tj, -1), (tv, 0), (tj, -1), (ta - 2 * tj, 0), (tj, +1)]
    j = np.concatenate([np.full(max(int(round(d / dt)), 0), s * jmax) for d, s in jerk_segments])
    a = np.cumsum(j) * dt
    v = np.cumsum(a) * dt
    s = np.cumsum(v) * dt
    scale = 1.0 / s[-1]                             # absorb the grid's small integration error
    s, v, a = s * scale, v * scale, a * scale
    T = len(s) * dt
    grid = np.arange(len(s)) * dt

    def fn(t):
        t = float(np.clip(t, 0.0, T - dt))
        return (float(np.interp(t, grid, s)), float(np.interp(t, grid, v)),
                float(np.interp(t, grid, a)))
    return T, fn


def _quintic_scalar(T):
    def fn(t):
        u = np.clip(t / T, 0.0, 1.0)
        return (10 * u**3 - 15 * u**4 + 6 * u**5,
                (30 * u**2 - 60 * u**3 + 30 * u**4) / T,
                (60 * u - 180 * u**2 + 120 * u**3) / T**2)
    return T, fn


def _leg_profile(d, kind, T, vscale, ascale):
    """Scalar profile for a move of joint displacements d: (leg time, fn(t) -> s, sd, sdd). The limits come from the most demanding joint."""
    ad = np.abs(d)
    mv = ad > 1e-9
    v_s = float(np.min(vscale * cfg.QD_MAX[mv] / ad[mv]))
    a_s = float(np.min(ascale * A_MAX[mv] / ad[mv]))
    j_s = float(np.min(J_MAX[mv] / ad[mv]))
    if kind == "trapezoidal":
        return _trap_scalar(v_s, a_s)
    if kind == "s_curve":
        return _scurve_scalar(v_s, a_s, j_s)
    if kind == "torque_aware":
        if T is None:
            raise ValueError("torque_aware needs T")
        return _quintic_scalar(T)
    raise ValueError(kind)


def leg(q0, q1, kind, T=None, vscale=1.0, ascale=1.0):
    """One-way rest-to-rest move q0 -> q1 in the given style. The Traj has .duration."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = q1 - q0
    Tleg, prof = _leg_profile(d, kind, T, vscale, ascale)

    def fn(t):
        s, sd, sdd = prof(t)
        return q0 + s * d, sd * d, sdd * d
    return Traj(fn, duration=Tleg)


def dwell(q, seconds):
    q = np.asarray(q, float).copy()
    return Traj(lambda t: (q, np.zeros(ARM), np.zeros(ARM)), duration=seconds)


def from_samples(q, hz=1000.0):
    """A Traj from joint samples on the physics grid; qd and qdd by central differences (the IK path is smooth to ~1e-9)."""
    q = np.asarray(q, float)
    dt = 1.0 / hz
    qd = np.gradient(q, dt, axis=0)
    qdd = np.gradient(qd, dt, axis=0)
    n = len(q)

    def fn(t):
        i = int(min(max(round(t * hz), 0), n - 1))
        return q[i], qd[i], qdd[i]
    return Traj(fn, duration=(n - 1) * dt)


def sequence(parts):
    """Play Trajs one after another. Returns a Traj whose .starts gives each part's start time."""
    starts = np.concatenate([[0.0], np.cumsum([p.duration for p in parts])])

    def fn(t):
        i = int(min(np.searchsorted(starts, t, side="right") - 1, len(parts) - 1))
        return parts[i](min(t - starts[i], parts[i].duration))
    tr = Traj(fn, duration=float(starts[-1]))
    tr.starts = starts
    return tr


def cycle(q0, q1, kind, T=None, dwell=0.2, vscale=1.0, ascale=1.0):
    """Out-and-back q0 -> q1 -> q0 with a dwell at each end. `T` is the duration of ONE leg
    (torque_aware only). vscale/ascale shrink the kinematic limits (1.0 = full)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    d = q1 - q0
    Tleg, leg_fn = _leg_profile(d, kind, T, vscale, ascale)
    leg = leg_fn
    total = 2 * Tleg + 2 * dwell

    def fn(t):
        if t < dwell:
            return q0.copy(), np.zeros(ARM), np.zeros(ARM)
        t1 = t - dwell
        if t1 < Tleg:
            s, sd, sdd = leg(t1); sgn, base = 1.0, q0
        elif t1 < Tleg + dwell:
            return q1.copy(), np.zeros(ARM), np.zeros(ARM)
        elif t1 < 2 * Tleg + dwell:
            s, sd, sdd = leg(t1 - Tleg - dwell); sgn, base = -1.0, q1
        else:
            return q0.copy(), np.zeros(ARM), np.zeros(ARM)
        return base + sgn * s * d, sgn * sd * d, sgn * sdd * d
    tr = Traj(fn, duration=total)
    tr.leg_time = Tleg
    return tr
