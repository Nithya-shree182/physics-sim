"""Sensor model for the logged telemetry. Idealised by default: noise is OFF unless asked for.

WHY OFF BY DEFAULT (decided 2026-11-08). This project is a digital twin for studying how
physics parameters move the residual, not an attempt to reproduce what a particular real
controller can measure. Adding noise by default meant every energy, power and torque channel
carried an injected error that was not physics, which made the numbers harder to read for no
gain. So the twin is given clean joint sensing and noise became a study parameter: dial it in
when the question is "how good must the sensing be before detection fails?", and leave it out
otherwise. Nothing about the plant changes either way — noise is added to what is LOGGED, the
way a real sensor corrupts a measurement, never to the state the controller sees.

The simulator's own decomposition (g, C, M, J) is left clean even when noise is on, because it
is a cross-check rather than a sensor reading.

TWO CONSEQUENCES, both deliberate:

  1. With noise off, two healthy runs with the same seed are bit-identical, so a false-alarm
     RATE cannot be measured from repeats. That measurement needs noise, which is what
     `reference()` below is for.
  2. A real UR has no joint torque sensor at all; its reported torque is derived from motor
     current. Clean joint torque sensing is therefore a capability the real arm does not have.
     That is stated here rather than hidden: the sensing model is a study parameter, not a
     fidelity claim (docs/UR10E_PORT.md).

    PAL_SENSOR_NOISE=1      # the reference sigmas below
    PAL_SENSOR_NOISE=0.5    # half of them
    PAL_SENSOR_NOISE=0      # off (the default)
"""
from __future__ import annotations

import os

import numpy as np

import pal_config as cfg

# [CHOICE] Reference sigmas, used when noise is switched on. The torque figure holds the
# noise-to-limit ratio this project used on the Franka Panda (0.02 N*m against an 87 N*m
# limit) at the UR10e's much larger torques, so detection results stay comparable across the
# arm change. q is an encoder step's order; qd is what differencing one gives.
REFERENCE_SIGMA = {"q": 2e-5,                                     # rad
                   "qd": 1e-3,                                    # rad/s
                   "tau": 0.02 / 87.0 * float(np.max(cfg.TAU_MAX))}   # N*m -> 0.076 on a UR10e

IDEAL_SIGMA = {"q": 0.0, "qd": 0.0, "tau": 0.0}

NOISY_FIELDS = ("q", "qd", "tau_measured", "tau_applied")


def scale_from_env(default: float = 0.0) -> float:
    """$PAL_SENSOR_NOISE as a multiplier on REFERENCE_SIGMA. 0 (the default) means ideal."""
    try:
        return max(0.0, float(os.environ.get("PAL_SENSOR_NOISE", default)))
    except ValueError:
        print(f"[sensors] PAL_SENSOR_NOISE={os.environ['PAL_SENSOR_NOISE']!r} is not a number; "
              f"using {default}", flush=True)
        return default


def reference(scale: float = 1.0) -> dict:
    """The reference sigmas, scaled. `reference(0)` is the ideal sensor."""
    return {k: v * float(scale) for k, v in REFERENCE_SIGMA.items()}


class SensorNoise:
    """Additive white noise on the logged state and torque. A sigma of zero is a no-op.

    `dof` defaults to the active arm's, so one object works for any arm; the row is only
    touched for the columns it actually has.
    """

    def __init__(self, seed, sigma=None, scale=None, dof=None):
        self.seed = int(seed)
        self.dof = int(dof if dof is not None else cfg.DOF)
        if sigma is None:
            sigma = reference(scale_from_env() if scale is None else scale)
        self.sigma = dict(IDEAL_SIGMA, **sigma)
        self.rng = np.random.default_rng(self.seed)
        self.ideal = all(v == 0.0 for v in self.sigma.values())

    def __bool__(self):
        """False for an ideal sensor, so `if noise:` reads as "is any noise injected"."""
        return not self.ideal

    def apply(self, row):
        """Corrupt one telemetry row in place. Does nothing if every sigma is zero."""
        if self.ideal:
            return
        per_field = {"q": self.sigma["q"], "qd": self.sigma["qd"],
                     "tau_measured": self.sigma["tau"], "tau_applied": self.sigma["tau"]}
        for field, sigma in per_field.items():
            if not sigma:
                continue
            for i in range(self.dof):
                key = f"{field}_{i}"
                if key in row:                 # a row need not carry every field
                    row[key] += self.rng.normal(0.0, sigma)

    def manifest(self):
        return {"seed": self.seed, "dof": self.dof, "sigma": self.sigma,
                "ideal": self.ideal,
                "applied_to": list(NOISY_FIELDS) if not self.ideal else [],
                "note": ("ideal sensing: no noise injected" if self.ideal else
                         "[CHOICE] representative sigmas, not identified constants")}


def from_env(seed=0, dof=None) -> SensorNoise:
    """The sensor this process should use, honouring $PAL_SENSOR_NOISE. Always returns an
    object, so callers never branch on None; an ideal one is a cheap no-op."""
    return SensorNoise(seed, scale=scale_from_env(), dof=dof)
