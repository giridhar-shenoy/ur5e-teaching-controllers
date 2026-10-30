# Controllers

All three controllers are C++17 `ros2_control` plugins on the **effort** interface: (a) joint
trajectory tracking with three control laws (PID, PID + RNEA feedforward, computed torque), (b)
gravity-compensated hand guiding, and (c) Cartesian TCP jogging with damped least squares. All dynamics come
from the `robot_description` URDF through the `DynamicsModel` interface, implemented by Pinocchio
(default) and orocos KDL (`dynamics_backend` parameter, see [below](#dynamics-backends)); no
UR5e mass, length or inertia appears in controller code. The configured gains live in
`src/ur5e_teaching_bringup/config/controllers.yaml`.

Notation: $q, \dot q \in \mathbb{R}^6$ measured joint positions/velocities,
$M(q)$ mass matrix, $C(q,\dot q)\dot q$ Coriolis/centrifugal torque, $g(q)$ gravity torque,
$\Delta t = 2$ ms controller period.

## (a) PidFfTrajectoryController — mode `track`

**Input.** `trajectory_msgs/JointTrajectory` on `~/joint_trajectory` (topic only; the
FollowJointTrajectory action is listed under "What's next"). Points are mapped by joint name;
trajectories with non-finite values or targets outside the URDF limits are rejected. A new
message pre-empts the running trajectory and starts from the current desired position at zero
velocity (so pre-empting at speed steps $\dot q_{des}$ to zero — a known v0.1 limitation). The
profile is built in the subscription callback and handed to `update()` through a
`RealtimeThreadSafeBox`; it is dropped if its start is more than `max_start_deviation` from the
current desired state when the realtime loop picks it up.

**Re-profiling.** Each segment between waypoints is a rest-to-rest trapezoid with the "1/3
rule" (accelerate, cruise, decelerate for $T/3$ each). For a joint distance $D$ in time $T$:

$$
v_{peak} = \frac{3}{2}\frac{D}{T}, \qquad a = \frac{9}{2}\frac{D}{T^2},
\qquad T_j^{min} = \max\left(\frac{3}{2}\frac{|D|}{v_{max}},\ \sqrt{\frac{9}{2}\frac{|D|}{a_{max}}}\right).
$$

All joints share $T = \max(T_{requested}, \max_j T_j^{min})$, so they start and stop together
and each stays inside its own $v_{max}, a_{max}$ (parameters). Segments are only ever stretched,
never shortened. Waypoints are reached at rest (no via-point blending, a deliberate v0.1 choice:
it keeps the profile closed-form and the unit tests exact).

**Three control laws.** The same controller (same trajectory input, profiles, limits and
state output) runs one of three laws, chosen by `control_law` and `feedforward` and switchable at
runtime; `bench_tracking` compares them on one path. With $e = q_{des} - q$:

| Variant | Parameters | Law |
|---|---|---|
| PID | `control_law: pid_ff`, `feedforward: false` | $\tau = K_p e + K_d \dot e + K_i \int e\,dt$ |
| PID + RNEA FF | `control_law: pid_ff`, `feedforward: true` (default) | $\tau = K_p e + K_d \dot e + K_i \int e\,dt + \mathrm{RNEA}(q_{des}, \dot q_{des}, \ddot q_{des})$ |
| Computed torque | `control_law: computed_torque` | $\tau = \mathrm{RNEA}(q, \dot q, \ddot q_{des} + K_p' e + K_d' \dot e)$ |

Every model term comes out of one recursive Newton–Euler pass over the URDF's links, never from
an explicit $M$ or $C$:

$$
\mathrm{RNEA}(q, \dot q, a) = \underbrace{M(q)\,a}_{\text{inertia}}
+ \underbrace{C(q,\dot q)\,\dot q}_{\text{Coriolis, centrifugal}}
+ \underbrace{g(q)}_{\text{gravity}}.
$$

The forward pass propagates each link's velocity and acceleration from the base (gravity enters
as a base acceleration of $-g$), the backward pass sums each link's $m\,a$ and
$I\dot\omega + \omega \times I\omega$ back to the joints, and a joint's torque is the
projection of the wrench it carries onto its axis. The velocity products in the forward pass
are the Coriolis and centrifugal terms. The model payload (`payload_mass`, `payload_com`) is
just more inertia on the last link; a change arrives as a rebuilt model, as described for (b).

*PID* is feedback only: the integrator has to build up the gravity torque before the arm holds
its pose, and during motion the inertial and Coriolis torques show up as tracking error.

*PID + RNEA feedforward* evaluates the model on the **desired** state, so the model supplies the
torque the reference needs and the PID only corrects what the model gets wrong. Because it is a
pure function of the reference, it feeds back no measurement noise. The feedback gains are fixed
per joint and were tuned on the diagonal of $M(q)$ at one pose (below), so the stiffness the
arm actually has varies with configuration and with the coupling the diagonal ignores.

*Computed torque* (inverse dynamics control, as in Spong, Hutchinson and Vidyasagar, *Robot
Modeling and Control*) evaluates the model on the **measured** state and feeds the error back
inside it. If the model is exact, $M(q)\ddot q + C\dot q + g = \tau$ gives
$\ddot q = \ddot q_{des} + K_p' e + K_d' \dot e$, i.e. for every joint

$$
\ddot e + K_d' \dot e + K_p' e = 0, \qquad K_p' = \omega^2,\ K_d' = 2\zeta\omega,
$$

decoupled from the others and independent of configuration and payload: the gains are in
acceleration units and the model scales them by $M(q)$. `ct_omega` and `ct_zeta` set $\omega$
and $\zeta$ per joint. The shipped $\omega$ = 20 rad/s (base three) and 30 rad/s (wrist) is
the PID's design bandwidth, so the laws are compared at the same nominal stiffness. In the 2 ms
explicit loop $K_d'\Delta t = 2\zeta\omega\Delta t \le 0.12$, and unlike the PID gains these
do not have to shrink for the light wrist_3 link. The command is clamped to the same torque
limits. The law lives in `src/control/computed_torque.hpp` so the controller and its
closed-loop unit test (`test/test_computed_torque.cpp`, both backends, exact-model plant) run
the same code.

**Why computed torque has no integral term.** With an exact model, the linearised error
dynamics above have no constant disturbance left for an integrator to remove, so an integral
would only add a third pole, overshoot and windup under the torque limits. With a wrong model,
what is left is $M^{-1}(q)\,\delta(q, \dot q, \ddot q)$, where $\delta$ is the torque error.
That residual changes with configuration, velocity and acceleration, so an integrator in
acceleration units only cancels it at rest, and the PID law's integrator already does that
job. Keeping the outer loop a pure PD also keeps the benchmark honest: the computed-torque
column shows what the model buys on its own, with nothing integrating the model's mistakes away.

**What the benchmark shows** (table in [results.md](results.md)): both model-based laws beat PID
by more than an order of magnitude, and with exact models and the same nominal bandwidth they
track the path about equally well when the machine is quiet. Under CPU contention they do not
degrade equally. A check run while a build was running had computed torque about 2× behind PID +
feedforward on every joint ([what_broke.md](what_broke.md)). An offline replica on an exact
rigid-body plant with a 2 ms hold shows the same direction when an actuation delay is added:
computed torque feeds the measured state back through the whole inverse dynamics, so any lag in
that state reaches the torque directly, while PID + feedforward evaluates its model on the
reference. The replica's errors are an order of magnitude below Gazebo's for both laws, so what
Gazebo adds on top is not identified.

**Saturation and anti-windup.** $\tau$ is clamped to the URDF effort limit times
`effort_limit_scale`. The integral uses conditional integration: it is frozen whenever the
unsaturated command already exceeds the limit in the direction the error would push it, and
$|K_i \int e|$ is clamped to `i_clamp`. Both behaviours are unit-tested
(`test/test_control_laws.cpp`). Switching `control_law` resets the integrators.

**Gains.** Critically damped PD per joint at ~20 rad/s (base) and ~30 rad/s (wrist) on the
diagonal of $M(q)$ at the home pose: $K_p = M_{jj}\omega^2$, $K_d = 2M_{jj}\omega$. The wrist_3
inertia is ~3·10⁻⁴ kg m², so its gains are tiny; with $K_d \Delta t / M_{jj} > 1$ the explicit
2 ms loop would go unstable. Integral time $K_p/K_i = 0.5$ s.

## (b) GravityCompHandGuidingController — mode `handguide`

**Control law.**

$$
\tau = \hat g(q) + \hat C(q,\dot q)\dot q - D\dot q + \tau_{fric}(\dot q) + \tau_{lim}(q),
$$

with hats marking model quantities that include the *configured* payload. The Coriolis term is
optional (`coriolis_compensation`; the backend's $C\dot q + g$: Pinocchio's `nonLinearEffects`,
or KDL's `ChainDynParam::JntToCoriolis` plus `JntToGravity`). $D$ is a
diagonal viscous damping for stability and feel. The friction hook
$\tau_{fric} = F_v\dot q + F_c\tanh(\dot q / 0.01)$ is zero by default because Gazebo joints
have no friction; on a real arm it is where identified friction goes.

**Runtime payload.** `payload_mass` and `payload_com` can be set while the controller runs
(`ros2 param set`). The payload is a solid 6 cm cube, CoM 3 cm along tool z by default, the same
shape and place as the Gazebo payload link. The model is never changed inside `update()`: the
parameter listener's user callback (the executor thread that serves `set_parameters`) clones the
payload-free model, adds the payload to the body that carries `tool0` (KDL: the last chain
segment, after which its solvers are rebuilt) and queues the finished model in a
`realtime_tools::RealtimeThreadSafeBox`. `update()` takes it with a non-blocking `try_get` and
swaps a `shared_ptr`; replaced models stay referenced on the non-realtime side and are released
there, so the loop never allocates, rebuilds or frees a model. $\hat g$ and $\hat C$ change on
the first cycle after the model is ready. The other runtime parameters are picked up without
blocking (`try_update_params`) and adopted only as a whole set whose per-joint arrays still have
one entry per joint.

**What the operator feels.** With a perfect model the arm floats and a push $F$ at the tool
moves it against the damping only. With a payload error $\Delta m = m_{actual} - m_{model}$ the
arm sinks ($\Delta m > 0$) or rises ($\Delta m < 0$), and holding it still costs a force of
$\Delta m\,g$ at the payload. `bench_handguide` measures that force statically, as the
converged tracking controller's hold torque minus the hand-guiding model torque mapped through
$J^{-T}$, and compares it with $\Delta m\,g$ (method in [results.md](results.md)).

**Safety.**

- *Joint-limit repulsion*: within `limit_margin` of a URDF position limit a spring of stiffness
  `limit_stiffness` pushes the joint back (zero in the interior).
- *Speed cap*: if any $|\dot q_j|$ exceeds `max_joint_velocity[j]` the controller writes its
  gravity-compensation-plus-damping torque one last time and returns `ERROR`; the controller
  manager deactivates it and activates its
  `fallback_controllers` (configured: the tracking controller), which holds the joints where
  the trip happened. Verified in simulation by lowering the cap at runtime and pushing the
  wrist. Without a fallback the last torque would stay applied while the arm moves away from the
  pose it was computed for; on a real arm the brakes engage.

**Hand-over between controllers.** On deactivation neither controller touches the command
interface: the last torque stays until the next controller writes. Writing zero there (the
first version) dropped the arm under gravity for one 2 ms cycle at every switch, which the
hand-guiding benchmark exposed as a 0.05 rad/s jolt at the elbow.

## (c) TcpJogController — mode `jog`

**Input.** `geometry_msgs/TwistStamped` on `~/twist`: the linear and angular velocity of
`tip_frame` (tool0), expressed in the URDF root frame (`header.frame_id` empty or
`command_frame`, default `world`; anything else is refused, not reinterpreted). Commands must keep
coming: without one for `command_timeout` (0.1 s) the commanded twist is zero (deadman).

**Inverse differential kinematics.** Each cycle, on the *reference* configuration $q_{des}$:

$$
\dot q = J^T \left(J J^T + \lambda^2 I\right)^{-1} v, \qquad
\lambda^2 = \begin{cases} \lambda_{max}^2 \left(1 - (w/w_0)^2\right) & w < w_0 \\ 0 & w \ge w_0
\end{cases}, \qquad w = \sqrt{\det J J^T}
$$

the damped least-squares solution with the damping scheduled on Yoshikawa's manipulability
(Nakamura and Hanafusa; Chiaverini). Far from a singularity it is the exact inverse; near one
it gives up some TCP accuracy to keep $\|\dot q\| \le \|v\| / (2\lambda)$ instead of
letting the joint speeds blow up. On the UR5e $w$ is about 0.08 at home and 0.004 at
`wrist_2_joint` = 0.05 rad (the wrist singularity is at 0); the shipped $w_0 = 0.02$ and
$\lambda_{max} = 0.05$ bound the joint speed to 0.5 rad/s for a 5 cm/s command at the
singularity itself. $J$ comes from the dynamics backend (either one), the 6×6 solve is
fixed-size (LDLT), and nothing in the loop allocates.

**Limits, in order, each as one scalar on the whole joint velocity** (so the TCP keeps moving in
the commanded direction, only slower): the joint speed limits, the joint acceleration limits on
the change of $\dot q$, and a stop `limit_margin` before any URDF position limit a joint is
moving towards. The reference integrates the result, $q_{des} \mathrel{+}= \dot q\,\Delta t$,
and the joint loop around it is the tracking controller's PID + RNEA feedforward on
$(q_{des}, \dot q_{des}, \ddot q_{des})$. Integrating a reference instead of commanding joint
velocities directly keeps the arm on the effort interface (there is no way back to position on
gz_ros2_control 3.0.9, see the README) and means a deadman stop holds the pose it stopped at.

**Output.** `~/controller_state` as for (a), plus `~/jog_state`
(`std_msgs/Float64MultiArray`): `[t, t_cmd, w, λ, scale, v_cmd(6), v_meas(6)]`, where $t$ is the
update time, $t_{cmd}$ the stamp of the command in use (both sim time in Gazebo, so latency can
be measured without wall clocks), scale the product of the three limit scalings and
$v_{meas} = J(q)\dot q$ from the measured state. `bench_jog` measures command-to-motion latency
(to 50 % and 90 % of the commanded speed), velocity tracking while streaming, and the stop time
after the last command.

## Dynamics backends

`src/dynamics/dynamics_model.hpp` is the interface the controllers use: `gravity`, `rnea`,
`nonlinear_effects`, `mass_matrix`, `tip_jacobian` (rows [linear; angular], reference point at
the tip, URDF-root orientation, i.e. Pinocchio's `LOCAL_WORLD_ALIGNED`), the tip pose, the URDF
limits and `set_payload`. Every query is allocation-free and writes into buffers sized at
construction; `set_payload` and `clone` may allocate and only run outside the loop.

| | Pinocchio (`pinocchio`, default) | orocos KDL (`kdl`) |
|---|---|---|
| Model | `buildModelFromXML`, whole URDF | `kdl_parser` tree, chain `base_link → tool0` |
| $g(q)$ | `computeGeneralizedGravity` | `ChainDynParam::JntToGravity` |
| RNEA | `rnea` | `ChainIdSolver_RNE::CartToJnt` |
| $C\dot q + g$ | `nonLinearEffects` | `ChainDynParam::JntToCoriolis` + `JntToGravity` |
| $M(q)$ | `crba` (upper triangle, mirrored) | `ChainDynParam::JntToMass` |
| Tip Jacobian | `computeFrameJacobian`, `LOCAL_WORLD_ALIGNED` | `ChainJntToJacSolver`, rotated from base_link to the root |
| Tip pose | `framesForwardKinematics` | `ChainFkSolverPos_recursive`, composed with root → base_link |
| Payload | added to the wrist_3 joint body through the tip frame's placement | added to the last segment's inertia (tip frame), solvers rebuilt |
| Limits | from the URDF via Pinocchio | from the URDF via `urdf::Model` (kdl_parser does not carry them) |

Conventions that had to line up, and are checked rather than assumed: KDL computes in the chain
root (`base_link`), so the wrapper rotates gravity into it and the Jacobian and tip pose out of
it, composing the fixed joints from the URDF root (`world`); `KDL::RigidBodyInertia(m, c, I_c)`
takes the rotational inertia about the centre of mass, and a segment's inertia is expressed in
its tip frame; KDL solvers hold a reference to their chain, so a copy of the model has to
rebuild them.

**Gravity.** One read-only parameter, `gravity_z` (default −9.81 m/s²), feeds both backends.
It must equal the simulator's gravity: SDF's default is −9.8, and in v0.1 that 0.01 m/s²
difference made a correctly modelled arm drift in hand guiding (4 mm/s, 0.3 mm/s after the fix)
until the worlds set `<gravity>0 0 -9.81</gravity>` explicitly. `test_gravity_consistency.py` keeps the worlds, the
controllers' YAML and the parameter default in step.

**Equivalence.** `test/test_dynamics_equivalence.cpp` evaluates both backends on 200 random
states (q inside the URDF position limits, q̇ inside the velocity limits, |q̈| ≤ 10 rad/s²) at
0 kg and 1 kg payload, plus a 1 kg off-axis payload on the rotated `ft_frame` and gravity −9.8.
Gravity, RNEA and $C\dot q + g$ must agree within 1e-6 N·m per joint, $M$ within 1e-6, the
Jacobian within 1e-9 and the tip pose within 1e-9; the worst deviations it prints are at the
rounding level (1e-14 N·m), which is what two exact recursions on the same inertias should give.
The mock-hardware test runs both controllers on the KDL backend and compares their torques with
Pinocchio's, and `scripts/ci_smoke.sh` (in CI with `SMOKE_BACKEND=kdl` too) runs both in
Gazebo.

**Cost.** `bench_dynamics_timing` (table in [results.md](results.md#dynamics-timing--details))
times single calls: Pinocchio is faster on both $g(q)$ and RNEA, and on this 6-DoF arm neither
uses more than a fraction of a percent of the 2 ms cycle.

## Pinocchio identities

The Pinocchio backend (Pinocchio 4.1) is also tested against identities that must hold for any
rigid-body model (`test/test_pinocchio_model.cpp`), so the equivalence test compares KDL with a
reference that is itself checked: $g = \mathrm{RNEA}(q,0,0)$,
$\mathrm{RNEA}(q,0,a) - g = M a$, $M = M^T \succ 0$, the Jacobian equals the finite difference
of the tip position, and adding a payload $m$ at the tip adds $J^T [0,0,mg]^T$ to $g(q)$.
