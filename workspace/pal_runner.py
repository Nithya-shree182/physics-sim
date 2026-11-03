"""Scenario runner: one plant, many runs, all in effort mode.

Why effort mode (docs/REVIEW.md F2): tau_measured is the NET joint torque and cannot show
friction, backlash or a locked joint. The torque we APPLY can, so the arm is driven by explicit
torque commands and `tau_applied` is logged.

The controller is computed-torque: tau = M (qdd_des + Kp e + Kd ed) + C + g, using the plant's
own M, C, g as feedforward (a perfect-knowledge controller). Any unmodelled load (a payload
the model does not know about, friction, a fault) therefore appears as the difference between
the applied torque and the nominal model, which is exactly what the residual reads.

A Scenario fixes everything about one run; nothing is stored between runs except the plant,
whose inertials and friction are rewritten and read back at the start of each one.
"""
from dataclasses import dataclass, field

import numpy as np

import pal_config as cfg
from pal_faults import FaultInjector, FaultSpec
from pal_sensors import SensorNoise
from pal_telemetry import TelemetryWriter

ARM = 7
KP_NORMAL, KD_NORMAL = 400.0, 40.0     # acceleration-space feedback gains
KP_LOW, KD_LOW = 40.0, 12.0             # "low-gain torque mode" for the validation push (proposal 7.7): ten times
                                        # softer than normal but critically damped. Kp = 4 let a 10 N push drive the
                                        # arm to its velocity limit, where the residual is not valid (docs/VALIDATION.md)
SETTLE_S = 0.4


@dataclass
class PushSpec:
    force: tuple           # world-frame N at the hand
    start_s: float
    end_s: float


@dataclass
class Scenario:
    name: str
    traj: object                         # a pal_trajectories.Traj
    traj_type: str = "slow_reference"    # schema traj_type label
    duration_s: float = 6.0
    payload_kg: float = 0.0
    mass_scale: float = 1.05
    fault: FaultSpec = field(default_factory=FaultSpec)
    noise_seed: int = None               # None = no sensor noise
    gains: str = "normal"                # "normal" | "low"
    push: PushSpec = None
    tau_limit: bool = True               # clip the command to the datasheet torque limits
    diagnostics: bool = False            # also return net = M qdd + C + g and the Jacobian per step
    payload_events: tuple = ()           # ((t_s, kg), ...): payload changes mid-run, e.g. a pickup at 3 s


class Runner:
    def __init__(self, plant, out_dir, mj_every=10):
        self.p, self.out_dir, self.mj_every = plant, out_dir, mj_every
        self.hz, self.dt = plant.hz, plant.dt
        self.inj = FaultInjector(plant, plant.hz)
        plant.to_effort_mode()
        self.art = plant.art
        self._tau = np.zeros(9, np.float32)

    @staticmethod
    def _v(x, n=ARM):
        return np.asarray(x).flatten()[:n]

    def _command(self, traj, t, kp, kd, tau_limit):
        art = self.art
        q, qd = self._v(art.get_joint_positions()), self._v(art.get_joint_velocities())
        g = self._v(art.get_generalized_gravity_forces())
        c = self._v(art.get_coriolis_and_centrifugal_forces())
        M = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
        qs, qds, qdds = traj(t)
        tau = M @ (qdds + kp * (qs - q) + kd * (qds - qd)) + c + g
        if tau_limit:
            tau = np.clip(tau, -cfg.TAU_MAX, cfg.TAU_MAX)
        return tau, qs, qds, qdds

    def _apply(self, tau):
        self._tau[:ARM] = tau
        self.art.set_joint_efforts(self._tau.reshape(1, 9))

    def run(self, sc, write=True):
        p, art, hz = self.p, self.art, self.hz
        p.set_inertials(sc.payload_kg, sc.mass_scale)
        p.set_friction(p.base_friction)
        p.push((0.0, 0.0, 0.0))
        self.inj.set_spec(sc.fault)
        kp, kd = (KP_LOW, KD_LOW) if sc.gains == "low" else (KP_NORMAL, KD_NORMAL)

        q0, qd0, _ = sc.traj(0.0)
        art.set_joint_positions(np.concatenate([q0, [.04, .04]]))
        art.set_joint_velocities(np.concatenate([qd0, [0., 0.]]).reshape(1, -1))
        for _ in range(int(SETTLE_S * hz)):
            tau, *_ = self._command(sc.traj, 0.0, KP_NORMAL, KD_NORMAL, sc.tau_limit)
            self._apply(tau); p.world.step(render=False)

        noise = SensorNoise(sc.noise_seed) if sc.noise_seed is not None else None
        run_id = f"{sc.name}_{int(hz)}hz"
        f = sc.fault
        writer = TelemetryWriter(
            art, run_id, out_dir=self.out_dir, ee_body_index=cfg.EE_JACOBIAN_ROW,
            ee_prim=f"{cfg.ROBOT_PRIM}/panda_hand", mj_every=self.mj_every, noise=noise,
            labels={"traj_type": sc.traj_type, "drive_mode": "torque", "payload_kg": sc.payload_kg,
                    "payload_attached": sc.payload_kg > 0, "fault_type": f.type,
                    "fault_joint": f.joint, "fault_param": f.param, "attack_type": "none"},
            manifest=self._manifest(sc))
        steps = int(sc.duration_s * hz)
        qdd_last = np.zeros(ARM)
        qd_prev = self._v(art.get_joint_velocities())
        log = {"q": [], "qd": [], "qdd": [], "cmd": [], "q_des": [], "net": [], "J": []}
        for k in range(steps):
            t = k / hz
            active = self.inj.update(t)
            for ev_t, ev_kg in sc.payload_events:
                if k == int(round(ev_t * hz)):
                    p.set_inertials(ev_kg, sc.mass_scale)
                    writer.labels["payload_kg"] = ev_kg
                    writer.labels["payload_attached"] = ev_kg > 0
            if sc.push is not None:
                on = sc.push.start_s <= t < sc.push.end_s
                p.push(sc.push.force if on else (0.0, 0.0, 0.0))
            tau, qs, qds, qdds = self._command(sc.traj, t, kp, kd, sc.tau_limit)
            tau = self.inj.command(tau, qdd_last)
            self._apply(tau)
            p.world.step(render=False)
            qd_now = self._v(art.get_joint_velocities())
            qdd_last = (qd_now - qd_prev) / self.dt; qd_prev = qd_now
            if write:
                writer.capture(k, (k + 1) / hz, self.dt, q_cmd=qs, qd_cmd=qds, qdd_cmd=qdds,
                               push_force=(sc.push.force if (sc.push and sc.push.start_s <= t < sc.push.end_s)
                                           else (0.0, 0.0, 0.0)) if sc.push else None,
                               fault_active=active)
            if sc.diagnostics:
                g1 = self._v(art.get_generalized_gravity_forces())
                c1 = self._v(art.get_coriolis_and_centrifugal_forces())
                M1 = np.asarray(art.get_mass_matrices()).reshape(9, 9)[:ARM, :ARM]
                log["net"].append(M1 @ qdd_last + c1 + g1)
                J = np.asarray(art.get_jacobians())
                log["J"].append(J.reshape(J.shape[0], -1, J.shape[-2], J.shape[-1])[0, cfg.EE_JACOBIAN_ROW][:, :ARM])
            log["q"].append(self._v(art.get_joint_positions())); log["qd"].append(qd_now.copy())
            log["cmd"].append(tau.copy()); log["q_des"].append(qs)
        p.push((0.0, 0.0, 0.0)); self.inj.clear()
        out = {k: np.array(v) for k, v in log.items()}
        # Measured, not asserted: the speed the run actually reached, per joint and overall, as a
        # fraction of the datasheet velocity limit. The analysis side applies its 50% band from this.
        frac = np.max(np.abs(out["qd"]), axis=0) / cfg.QD_MAX
        writer.manifest_extra["speed_fraction"] = float(frac.max())
        writer.manifest_extra["speed_fraction_per_joint"] = [round(float(x), 4) for x in frac]
        if sc.payload_events:
            writer.manifest_extra["payload_schedule"] = [[0.0, sc.payload_kg]] + [[t, kg] for t, kg in sc.payload_events]
        if write:
            writer.close()
        out["speed_fraction"] = float(frac.max())
        return out

    def _manifest(self, sc):
        import os
        f = sc.fault
        return {"git_commit": os.environ.get("PAL_GIT_COMMIT", "uncommitted"),
                "machine": os.environ.get("PAL_MACHINE", "omen-5090"),
                "isaac_sim_version": "6.0.1", "physics_hz": self.hz,
                "solver": {"type": cfg.SOLVER_TYPE, "position_iterations": cfg.POSITION_ITERATIONS,
                           "velocity_iterations": cfg.VELOCITY_ITERATIONS},
                "asset": {"usd": cfg.ROBOT_USD_SUBPATH, "urdf": cfg.URDF_PATH,
                          "inertials_authored_from_urdf": True},
                "armature": cfg.ARMATURE.tolist(), "gravity": [0.0, 0.0, cfg.GRAVITY], "seed": 0,
                "friction_source": cfg.FRICTION_SOURCE,
                "plant_friction_scale": cfg.PLANT_FRICTION_SCALE,
                "friction_values": self.p.base_friction.tolist(),
                "friction_units": "columns: static N*m, dynamic N*m, viscous N*m*s/rad",
                "torque_convention": cfg.TORQUE_CONVENTION, "finger_q_ref": cfg.FINGER_OPEN,
                "plant_link_mass_scale": sc.mass_scale,
                "control": {"mode": "effort", "law": "computed torque with the plant's own M, C, g",
                            "kp": KP_LOW if sc.gains == "low" else KP_NORMAL,
                            "kd": KD_LOW if sc.gains == "low" else KD_NORMAL},
                "payload_attachment": {"body": "panda_hand", "offset_m": [0, 0, 0.1034],
                                       "model": "point mass folded into the hand's composite"},
                "fault": {"type": f.type, "joint": f.joint, "param": f.param, "onset_s": f.onset_s},
                "scenario": sc.name}
