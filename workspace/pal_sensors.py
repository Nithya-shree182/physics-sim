"""Sensor noise for the logged telemetry, so repeated healthy runs differ.

The simulation is deterministic: two healthy runs are bit-identical, so a false-alarm RATE
cannot be measured. Noise with a per-run seed fixes that without touching the physics.

Noise is added to what is LOGGED (joint position, velocity and the two torque readings), the
way a real sensor corrupts a measurement. The controller inside the sim keeps the true state,
and the simulator's own decomposition (g, C, M, J) is left clean because it is a cross-check,
not a sensor.

The sigmas are [CHOICE] representative values for a Franka-class arm, NOT identified
constants: nobody measured them on this project. They are recorded in every run manifest so
a reader can see exactly what was injected.
"""
import numpy as np

ARM = 7
DEFAULT_SIGMA = {"q": 2e-5,        # rad, order of an encoder step
                 "qd": 1e-3,       # rad/s
                 "tau": 0.02}      # N*m, torque-sensor level


class SensorNoise:
    def __init__(self, seed, sigma=None):
        self.seed = int(seed)
        self.sigma = dict(DEFAULT_SIGMA, **(sigma or {}))
        self.rng = np.random.default_rng(self.seed)

    def apply(self, row):
        """Corrupt one telemetry row in place."""
        for i in range(ARM):
            row[f"q_{i}"] += self.rng.normal(0.0, self.sigma["q"])
            row[f"qd_{i}"] += self.rng.normal(0.0, self.sigma["qd"])
            row[f"tau_measured_{i}"] += self.rng.normal(0.0, self.sigma["tau"])
            row[f"tau_applied_{i}"] += self.rng.normal(0.0, self.sigma["tau"])

    def manifest(self):
        return {"seed": self.seed, "sigma": self.sigma,
                "applied_to": ["q", "qd", "tau_measured", "tau_applied"],
                "note": "representative values, not identified constants"}
