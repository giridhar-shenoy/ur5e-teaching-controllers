"""bench_handguide: what a wrong payload model costs the operator of the hand-guiding controller.

The sim is launched with the *actual* payload (xacro payload_mass, Gazebo only); this driver sets
the controllers' *model* payload (default 1.0 kg) and, per repetition:

1. holds the arm with the tracking controller at HOME + a seeded random offset (|.| <= 0.05 rad)
   until its integrator has converged (3 s, integral time 0.5 s): its torque is then exactly
   what the simulated arm needs to stay put, tau_hold = g_true(q);
2. switches to gravity_comp_handguide_controller, whose model torque at rest is
   tau_hg = g_model(q) (published as reference.effort), and lets go for 1 s;
3. computes, with Pinocchio on the robot_description:
   - effort to hold: the tool force an operator must apply to keep the arm still,
     F = J(q)^-T (tau_hold - tau_hg). Its vertical component should be (m_actual - m_model) g.
     (Measured statically on purpose: an estimate from the release acceleration breaks down as
     soon as the joint damping acts within the fit window, which it does on the light wrists.)
   - drift: TCP linear speed |J(q) qd| over [0.9, 1.0] s and TCP displacement at 1 s. Not 2 s:
     with a 1 kg error the arm travels tens of centimetres within 2 s and leaves the region
     (and, sinking, approaches the floor), so later numbers stop describing drift.

    ros2 run ur5e_teaching_bench bench_handguide --out results/handguide.csv \
        --actual-payload 2.0 --model-payload 1.0 --reps 3 --seed 1
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import pinocchio as pin
import rclpy
from rcl_interfaces.srv import GetParameters

from ur5e_teaching_bench.common import (
    HOME,
    JOINTS,
    BenchNode,
    append_rows,
    git_commit,
    world_step_size,
)

CONTROLLER = "gravity_comp_handguide_controller"
DRIFT_S = 1.0
HOLD_SETTLE_S = 3.0
HOLD_AVERAGE_S = 0.3
G = 9.81


def robot_model(node: BenchNode) -> pin.Model:
    client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    client.wait_for_service(timeout_sec=10.0)
    future = client.call_async(GetParameters.Request(names=["robot_description"]))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return pin.buildModelFromXML(future.result().values[0].string_value)


def to_pin(model: pin.Model, q_ctrl: np.ndarray) -> np.ndarray:
    q = np.zeros(model.nq)
    for j, name in enumerate(JOINTS):
        q[model.joints[model.getJointId(name)].idx_q] = q_ctrl[j]
    return q


def analyse(model: pin.Model, hold: dict, rec: dict) -> dict:
    data = model.createData()
    tool = model.getFrameId("tool0")

    def tcp(qc):
        pin.framesForwardKinematics(model, data, to_pin(model, qc))
        return data.oMf[tool].translation.copy()

    def jac(qc):
        return pin.computeFrameJacobian(
            model, data, to_pin(model, qc), tool, pin.LOCAL_WORLD_ALIGNED
        )

    # Effort to hold: tracking-controller steady-state torque vs hand-guiding torque at rest.
    tau_hold = np.mean(hold["effort"], axis=0)
    q_hold = np.mean(hold["position"], axis=0)
    # The hand-guiding controller's model torque g_model(q) [+ C(q,qd)qd, ~0 at rest], i.e. its
    # command without the damping term, at the first cycle after the switch.
    tau_hg = rec["model_effort"][0]
    J = jac(q_hold)
    # J is 6x6 in controller order after to_pin/back: map the torque gap to a tool wrench.
    idx = [model.joints[model.getJointId(n)].idx_v for n in JOINTS]
    wrench = np.linalg.solve(J[:, idx].T, tau_hold - tau_hg)

    t = rec["stamps"] - rec["stamps"][0]
    q, qd = rec["position"], rec["velocity"]
    end = t <= DRIFT_S
    late = (t >= DRIFT_S - 0.1) & end
    speeds = [np.linalg.norm((jac(q[k])[:, idx] @ qd[k])[:3]) for k in np.flatnonzero(late)]
    disp = np.linalg.norm(tcp(q[np.flatnonzero(end)[-1]]) - tcp(q[0]))
    return {
        "drift_speed_mps": float(np.mean(speeds)),
        "drift_disp_m": float(disp),
        "hold_force_x_n": float(wrench[0]),
        "hold_force_y_n": float(wrench[1]),
        "hold_force_z_n": float(wrench[2]),
        "hold_samples": int(len(hold["stamps"])),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", required=True)
    parser.add_argument("--actual-payload", type=float, required=True)
    parser.add_argument("--model-payload", type=float, default=1.0)
    parser.add_argument("--reps", type=int, default=3)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args(argv)

    rclpy.init()
    node = BenchNode("bench_handguide")
    node.wait_for_clock()
    rng = np.random.default_rng(args.seed)
    commit = git_commit()
    step = world_step_size()
    rate = node.get_param("controller_manager", "update_rate")
    node.switcher.ensure_configured(CONTROLLER)  # its parameter services exist only once loaded
    backend = node.get_param(CONTROLLER, "dynamics_backend")
    model = robot_model(node)  # kinematics only; the payload does not change J
    expected_z = (args.actual_payload - args.model_payload) * G

    tracker = "pid_ff_trajectory_controller"
    node.switcher.ensure_configured(tracker)
    node.set_params(tracker, {"payload_mass": args.model_payload})
    for rep in range(args.reps):
        offset = rng.uniform(-0.05, 0.05, size=6)
        node.home(HOME + offset, settle_s=HOLD_SETTLE_S - HOLD_AVERAGE_S)
        node.start_recording(tracker)
        node.sleep_sim(HOLD_AVERAGE_S)
        hold = node.stop_recording()
        node.set_params(CONTROLLER, {"payload_mass": args.model_payload})
        node.start_recording(CONTROLLER)
        node.switcher.activate_exclusive(CONTROLLER)
        node.sleep_sim(DRIFT_S + 0.2)
        rec = node.stop_recording()
        r = analyse(model, hold, rec)
        append_rows(
            args.out,
            [
                {
                    "bench": "handguide",
                    "git_commit": commit,
                    "sim_step_s": step,
                    "update_rate_hz": rate,
                    "dynamics_backend": backend,
                    "seed": args.seed,
                    "rep": rep,
                    "model_payload_kg": args.model_payload,
                    "actual_payload_kg": args.actual_payload,
                    "payload_error_kg": args.actual_payload - args.model_payload,
                    "drift_speed_1s_mps": f"{r['drift_speed_mps']:.6e}",
                    "drift_disp_1s_m": f"{r['drift_disp_m']:.6e}",
                    "hold_force_x_n": f"{r['hold_force_x_n']:.4f}",
                    "hold_force_y_n": f"{r['hold_force_y_n']:.4f}",
                    "hold_force_z_n": f"{r['hold_force_z_n']:.4f}",
                    "expected_hold_force_z_n": f"{expected_z:.4f}",
                    "hold_samples": r["hold_samples"],
                }
            ],
        )
        node.get_logger().info(
            f"actual {args.actual_payload} kg / model {args.model_payload} kg rep {rep}: "
            f"drift {r['drift_speed_mps']:.4f} m/s, hold Fz {r['hold_force_z_n']:.2f} N "
            f"(expected {expected_z:.2f} N)"
        )
    node.home()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
