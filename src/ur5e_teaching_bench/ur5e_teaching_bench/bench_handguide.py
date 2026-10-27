"""bench_handguide: what a wrong payload model costs the operator of the hand-guiding controller.

The sim is launched with the *actual* payload (xacro payload_mass, Gazebo only); this driver sets
the controller's *model* payload (default 1.0 kg) and, per repetition:

1. homes the arm with the tracking controller to HOME + a seeded random offset (|.| <= 0.05 rad),
2. switches to gravity_comp_handguide_controller and lets go with zero applied force,
3. records 2 s of joint state and computes, with Pinocchio on the robot_description:
   - drift: TCP linear speed |J(q) qd| averaged over [1.9, 2.0] s and TCP displacement at 2 s;
   - effort to move: the force the operator must apply at tool0 to hold the arm still,
     F = -J(q)^-T M(q) qdd over the first 40 ms after release (qd ~ 0, so damping and Coriolis
     do not contribute yet). M uses the actual payload, because that is the arm being held.
     Its vertical component is reported with the expectation (m_actual - m_model) * g.

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
DRIFT_S = 2.0
RELEASE_WINDOW_S = 0.04
G = 9.81
PAYLOAD_COM_Z = 0.05  # same default as the xacro and the controller
CUBE_SIDE = 0.06


def robot_model(node: BenchNode) -> pin.Model:
    client = node.create_client(GetParameters, "/robot_state_publisher/get_parameters")
    client.wait_for_service(timeout_sec=10.0)
    future = client.call_async(GetParameters.Request(names=["robot_description"]))
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    return pin.buildModelFromXML(future.result().values[0].string_value)


def with_payload(model: pin.Model, mass: float) -> pin.Model:
    """Copy of `model` with the payload cube on tool0, matching the Gazebo payload link."""
    m = model.copy()
    if mass <= 0.0:
        return m
    tool = m.getFrameId("tool0")
    i = mass * CUBE_SIDE**2 / 6.0
    payload = pin.Inertia(mass, np.array([0.0, 0.0, PAYLOAD_COM_Z]), np.eye(3) * i)
    joint = m.frames[tool].parentJoint
    m.inertias[joint] = m.inertias[joint] + m.frames[tool].placement.act(payload)
    return m


def to_pin(model: pin.Model, q_ctrl: np.ndarray) -> np.ndarray:
    q = np.zeros(model.nq)
    for j, name in enumerate(JOINTS):
        q[model.joints[model.getJointId(name)].idx_q] = q_ctrl[j]
    return q


def analyse(model: pin.Model, rec: dict) -> dict:
    data = model.createData()
    tool = model.getFrameId("tool0")
    t = rec["stamps"] - rec["stamps"][0]
    q, qd = rec["position"], rec["velocity"]

    def tcp(qc):
        pin.framesForwardKinematics(model, data, to_pin(model, qc))
        return data.oMf[tool].translation.copy()

    def jac(qc):
        return pin.computeFrameJacobian(
            model, data, to_pin(model, qc), tool, pin.LOCAL_WORLD_ALIGNED
        )

    end = t <= DRIFT_S
    late = (t >= DRIFT_S - 0.1) & end
    speeds = [np.linalg.norm((jac(q[k]) @ qd[k])[:3]) for k in np.flatnonzero(late)]
    disp = np.linalg.norm(tcp(q[np.flatnonzero(end)[-1]]) - tcp(q[0]))

    # Release acceleration from a straight-line fit of qd over the first 40 ms.
    early = t <= RELEASE_WINDOW_S
    qdd = np.polyfit(t[early], qd[early], 1)[0]
    q0 = to_pin(model, q[0])
    M = pin.crba(model, data, q0)
    M = np.triu(M) + np.triu(M, 1).T
    qdd_pin = to_pin(model, qdd)
    tau = M @ qdd_pin  # net generalised force at release
    J = pin.computeFrameJacobian(model, data, q0, tool, pin.LOCAL_WORLD_ALIGNED)
    wrench = np.linalg.solve(J.T, tau)  # equivalent wrench at tool0 acting on the arm
    hold = -wrench  # what the operator must apply to cancel it
    return {
        "drift_speed_mps": float(np.mean(speeds)),
        "drift_disp_m": float(disp),
        "hold_force_x_n": float(hold[0]),
        "hold_force_y_n": float(hold[1]),
        "hold_force_z_n": float(hold[2]),
        "release_samples": int(early.sum()),
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
    model = with_payload(robot_model(node), args.actual_payload)
    expected_z = (args.actual_payload - args.model_payload) * G

    for rep in range(args.reps):
        offset = rng.uniform(-0.05, 0.05, size=6)
        node.home(HOME + offset)
        node.set_params(CONTROLLER, {"payload_mass": args.model_payload})
        node.start_recording(CONTROLLER)
        node.switcher.activate_exclusive(CONTROLLER)
        node.sleep_sim(DRIFT_S + 0.2)
        rec = node.stop_recording()
        r = analyse(model, rec)
        append_rows(
            args.out,
            [
                {
                    "bench": "handguide",
                    "git_commit": commit,
                    "sim_step_s": step,
                    "update_rate_hz": rate,
                    "seed": args.seed,
                    "rep": rep,
                    "model_payload_kg": args.model_payload,
                    "actual_payload_kg": args.actual_payload,
                    "payload_error_kg": args.actual_payload - args.model_payload,
                    "drift_speed_2s_mps": f"{r['drift_speed_mps']:.6e}",
                    "drift_disp_2s_m": f"{r['drift_disp_m']:.6e}",
                    "hold_force_x_n": f"{r['hold_force_x_n']:.4f}",
                    "hold_force_y_n": f"{r['hold_force_y_n']:.4f}",
                    "hold_force_z_n": f"{r['hold_force_z_n']:.4f}",
                    "expected_hold_force_z_n": f"{expected_z:.4f}",
                    "release_samples": r["release_samples"],
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
