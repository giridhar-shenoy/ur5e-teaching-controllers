# Controllers

All three controllers are C++17 `ros2_control` plugins on the **effort** interface: joint trajectory
tracking with three control laws (PID, PID + RNEA feedforward, computed torque), gravity-compensated
hand guiding, and Cartesian TCP jogging with damped least squares. All dynamics come from the
`robot_description` URDF through the `DynamicsModel` interface, implemented by Pinocchio (default)
and orocos KDL (`dynamics_backend` parameter, see [Two dynamics backends](#two-dynamics-backends));
no UR5e mass, length or inertia appears in controller code. The configured gains live in
`src/ur5e_teaching_bringup/config/controllers.yaml` and are worked through in
[Gains and bandwidths](#gains-and-bandwidths).

| Mode | Controller | Claims | What it does |
|---|---|---|---|
| `track` | `PidFfTrajectoryController` | `*/effort` | Trapezoidal re-profiled trajectories with one of three laws: PID with anti-windup, PID + RNEA feedforward, or computed torque (`control_law`) |
| `handguide` | `GravityCompHandGuidingController` | `*/effort` | $g(q) + C(q,\dot q)\dot q - D\dot q$, runtime payload, joint-limit repulsion, speed cap |
| `jog` | `TcpJogController` | `*/effort` | TCP twist → damped least squares with manipulability-scheduled damping, speed/acceleration/position limits, deadman |

Notation: $q, \dot q \in \mathbb{R}^6$ measured joint positions/velocities,
$M(q)$ mass matrix, $C(q,\dot q)\dot q$ Coriolis/centrifugal torque, $g(q)$ gravity torque,
$\Delta t = 2$ ms controller period.

## Tracking controller

`PidFfTrajectoryController`, mode `track`.

**Input.** `trajectory_msgs/JointTrajectory` on `~/joint_trajectory` (topic only; the
FollowJointTrajectory action is listed under [Not done yet](#not-done-yet)). Points are mapped by
joint name; trajectories with non-finite values or targets outside the URDF limits are rejected. A
new message pre-empts the running trajectory and starts from the current desired position at zero
velocity (so pre-empting at speed steps $\dot q_{des}$ to zero — a known v0.1 limitation,
[what_broke.md F13](what_broke.md#f13-trajectories-stop-at-every-waypoint-and-pre-emption-is-abrupt)).
The profile is built in the subscription callback and handed to `update()` through a
`RealtimeThreadSafeBox`; it is dropped if its start is more than `max_start_deviation` from the
current desired state when the realtime loop picks it up.

**Re-profiling.** Each segment between waypoints is a rest-to-rest trapezoid with the "1/3
rule" (accelerate, cruise, decelerate for $T/3$ each). For a joint distance $D$ in time $T$:

$$
v_{peak} = \frac{3}{2}\frac{D}{T}, \qquad a = \frac{9}{2}\frac{D}{T^2},
\qquad T_j^{min} = \max\left(\frac{3}{2}\frac{|D|}{v_{max}},\ \sqrt{\frac{9}{2}\frac{|D|}{a_{max}}}\right).
$$

All joints share $T = \max(T_{requested}, \max_j T_j^{min})$, so they start and stop together and
each stays inside its own $v_{max}, a_{max}$ (parameters). Segments are only ever stretched, never
shortened. Waypoints are reached at rest (no via-point blending, a deliberate v0.1 choice: it keeps
the profile closed-form and the unit tests exact). The desired acceleration steps at the four
corners of every segment (infinite jerk); what that does to the max-error statistic is
[what_broke.md F10](what_broke.md#f10-the-max-error-statistic-is-unstable-between-runs-because-the-trapezoid-steps-the-desired-acceleration-at-segment-corners).

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

The forward pass propagates each link's velocity and acceleration from the base (gravity enters as a
base acceleration of $-g$), the backward pass sums each link's $m\,a$ and $I\dot\omega + \omega
\times I\omega$ back to the joints, and a joint's torque is the projection of the wrench it carries
onto its axis. The velocity products in the forward pass are the Coriolis and centrifugal terms. The
model payload (`payload_mass`, `payload_com`) is just more inertia on the last link; a change
arrives as a rebuilt model, as described under [Hand guiding](#hand-guiding).

*PID* is feedback only: the integrator has to build up the gravity torque before the arm holds its
pose, and during motion the inertial and Coriolis torques show up as tracking error
([what_broke.md F2](what_broke.md#f2-without-feedforward-pid-sags-first-and-tracks-late)).

*PID + RNEA feedforward* evaluates the model on the **desired** state, so the model supplies the
torque the reference needs and the PID only corrects what the model gets wrong. Because it is a pure
function of the reference, it feeds back no measurement noise. The feedback gains are fixed per
joint and were chosen on the diagonal of $M(q)$ at one pose
([Gains and bandwidths](#gains-and-bandwidths)), so the stiffness the arm actually has varies with
configuration and with the coupling the diagonal ignores.

*Computed torque* (inverse dynamics control, as in Spong, Hutchinson and Vidyasagar, *Robot
Modeling and Control*) evaluates the model on the **measured** state and feeds the error back
inside it. If the model is exact, $M(q)\ddot q + C\dot q + g = \tau$ gives
$\ddot q = \ddot q_{des} + K_p' e + K_d' \dot e$, i.e. for every joint

$$
\ddot e + K_d' \dot e + K_p' e = 0, \qquad K_p' = \omega^2,\ K_d' = 2\zeta\omega,
$$

decoupled from the others and independent of configuration and payload: the gains are in
acceleration units and the model scales them by $M(q)$. `ct_omega` and `ct_zeta` set $\omega$ and
$\zeta$ per joint; the shipped values are $\omega$ = 20 rad/s (base three) and 30 rad/s (wrist),
$\zeta = 1$. On the base joints that is close to the PID's PD bandwidth, so there the two laws are
compared at roughly the same stiffness; on the wrist joints the PD loops are faster than 30 rad/s
([Gains and bandwidths](#gains-and-bandwidths)), so the comparison is not at equal stiffness there.
In the 2 ms explicit loop $K_d'\Delta t = 2\zeta\omega\Delta t \le 0.12$, and unlike the PID gains
these do not have to shrink for the light wrist_3 link. The command is clamped to the same torque
limits. The law lives in `src/control/computed_torque.hpp` so the controller and its closed-loop
unit test (`test/test_computed_torque.cpp`, both backends, exact-model plant) run the same code.

**Why computed torque has no integral term.** With an exact model, the linearised error
dynamics above have no constant disturbance left for an integrator to remove, so an integral
would only add a third pole, overshoot and windup under the torque limits. With a wrong model,
what is left is $M^{-1}(q)\,\delta(q, \dot q, \ddot q)$, where $\delta$ is the torque error.
That residual changes with configuration, velocity and acceleration, so an integrator in
acceleration units only cancels it at rest, and the PID law's integrator already does that
job. Keeping the outer loop a pure PD also keeps the benchmark honest: the computed-torque
column shows what the model buys on its own, with nothing integrating the model's mistakes away.

**What the benchmark shows** (tables in [results.md](results.md#tracking-full-tables)): both
model-based laws are about two orders of magnitude more accurate than PID alone, and with exact
models they track the path about equally well; which of the two comes out ahead changes from run to
run. Both residuals are set mainly by a timing offset between the reference and the measured state,
not by either law ([what_broke.md](what_broke.md#timing-offset-between-reference-and-state)), so
this benchmark cannot rank them. Two earlier observations pointed to computed torque being the more
delay-sensitive law: a one-repetition check made while a build was running, and an offline replica
of the loop on an exact rigid-body plant with a 2 ms hold and an added actuation delay. No script in
this repository reproduces either; both are recorded in the
[what_broke.md appendix](what_broke.md#appendix-hand-recorded-runs-not-regenerated-by-any-script).
The direction is still what the structure predicts: computed torque feeds the measured state back
through the whole inverse dynamics, so any lag in that state reaches the torque directly, while
PID + feedforward evaluates its model on the reference
([what_broke.md F3](what_broke.md#f3-computed-torque-buys-nothing-over-pid--feedforward-here-and-is-more-fragile)).

**Saturation and anti-windup.** $\tau$ is clamped to the URDF effort limit times
`effort_limit_scale`. The integral uses conditional integration: it is frozen whenever the
unsaturated command already exceeds the limit in the direction the error would push it, and
$|K_i \int e|$ is clamped to `i_clamp`. Both behaviours are unit-tested
(`test/test_control_laws.cpp`). Switching `control_law` resets the integrators.

**What it does not handle.** A stale or non-finite joint state: `update()` ignores the return value
of `read_state()`, and a NaN passes through the saturation
([what_broke.md F5](what_broke.md#f5-no-controller-handles-a-stale-or-nan-joint-state--the-first-thing-that-would-bite-on-hardware)).
There are no path or goal tolerances and no tracking-error abort; a blocked arm saturates.

## Hand guiding

`GravityCompHandGuidingController`, mode `handguide`.

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

**What the operator feels.** With a perfect model the arm floats and a push $F$ at the tool moves it
against the damping only. With a payload error $\Delta m = m_{actual} - m_{model}$ the arm sinks
($\Delta m > 0$) or rises ($\Delta m < 0$), and holding it still costs a force of $\Delta m\,g$ at
the payload. `bench_handguide` measures that force statically, as the converged tracking
controller's hold torque minus the hand-guiding model torque mapped through $J^{-T}$, and compares
it with $\Delta m\,g$ (method in [results.md](results.md#hand-guiding)). Because the simulator and
the model share the URDF and $g$, that force equals $\Delta m\,g$ by construction: it checks that
the payload enters the model in the right place, it does not measure anything that could fail. The
drift after release is the measured part, and it depends on the damping chosen
([what_broke.md F1](what_broke.md#f1-a-wrong-payload-model-makes-the-arm-drift-and-the-operator-pays-for-it)).

**Safety.**

- *Joint-limit repulsion*: within `limit_margin` of a URDF position limit a spring of stiffness
  `limit_stiffness` pushes the joint back (zero in the interior).
- *Speed cap*: if any $|\dot q_j|$ exceeds `max_joint_velocity[j]` the controller writes its
  gravity-compensation-plus-damping torque one last time and returns `ERROR`; the controller manager
  deactivates it and activates its `fallback_controllers` (configured: the tracking controller). The
  fallback's `on_activate` sets $q_{des} = q$, $\dot q_{des} = 0$, so it stops the arm where the
  trip happened, but by braking: its damping term asks for $K_d \dot q$, which at the shoulder_lift
  cap is the joint's full effort limit
  ([what_broke.md F11](what_broke.md#f11-the-speed-cap-is-a-hard-stop-and-its-fallback-brakes-hard-see-f12),
  [F12](what_broke.md#f12-the-speed-cap-fallback-is-a-full-torque-brake-not-a-gentle-hold)). No test
  covers this path; it was checked by hand by lowering the cap at runtime (appendix of
  what_broke.md). Without a fallback the last torque would stay applied while the arm moves away
  from the pose it was computed for; on a real arm the brakes engage.

**Hand-over between controllers.** On deactivation neither controller touches the command interface:
the last torque stays until the next controller writes. Writing zero there (the first version)
dropped the arm under gravity for one 2 ms cycle at every switch, which the hand-guiding benchmark
exposed as a jolt at the elbow (numbers in the
[what_broke.md appendix](what_broke.md#appendix-hand-recorded-runs-not-regenerated-by-any-script)).
This relies on gz_ros2_control holding the last command; a real UR on direct torque does not
([what_broke.md F4](what_broke.md#f4-gazebos-effort-interface-is-kinder-than-a-real-arms)).

**What it does not handle.** A stale or non-finite joint state, as for the tracking controller; a
NaN velocity does not trip the speed cap either
([what_broke.md F5](what_broke.md#f5-no-controller-handles-a-stale-or-nan-joint-state--the-first-thing-that-would-bite-on-hardware)).

## TCP jogging

`TcpJogController`, mode `jog`.

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

the damped least-squares solution (Wampler; Siciliano et al., *Robotics: Modelling, Planning and
Control*) with the damping scheduled on Yoshikawa's manipulability measure $w$. The form of the
schedule is Chiaverini's, who applies it to the smallest singular value $\sigma_{min}$ of $J$;
Nakamura and Hanafusa schedule $\lambda = \lambda_0 (1 - w/w_0)^2$ on $w$. Chiaverini's form on
Yoshikawa's $w$ is a variant of both, not either paper. Far from a singularity it is the exact
inverse; near one it gives up some TCP accuracy to keep $\|\dot q\| \le \|v\| / (2\lambda)$ instead
of letting the joint speeds blow up. On the UR5e $w$ is about 0.08 at home and 0.004 at
`wrist_2_joint` = 0.05 rad (the wrist singularity is at 0); the shipped $w_0 = 0.02$ and
$\lambda_{max} = 0.05$ bound the joint speed to 0.5 rad/s for a 5 cm/s command at the singularity
itself. $J$ comes from the dynamics backend (either one), the 6×6 solve is fixed-size (LDLT), and
nothing in the loop allocates.

*Units of $w$.* $J$ has three linear rows (metres per radian) and three angular rows, so $w$ mixes
metres and radians and scales with the cube of the length unit: $w_0 = 0.02$ is a value in metres
and would be $2 \cdot 10^7$ in millimetres. For this square $J$, $w$ does not change with the tool
offset (moving the reference point multiplies $J$ by a block-triangular matrix with determinant 1),
but how close to singular a given $w$ is does: $\sigma_{min}$ changes with the offset while the
product of the singular values does not. A schedule on $\sigma_{min}$ would be the unit-consistent
alternative.

**Limits, in order, each as one scalar on the whole joint velocity** (so the TCP keeps moving in the
commanded direction, only slower): the joint speed limits, the joint acceleration limits on the
change of $\dot q$, and a stop `limit_margin` before any URDF position limit a joint is moving
towards. The position-limit stop is applied after the acceleration limit, so at the margin $\dot q$
can drop to zero within one cycle and the resulting $\ddot q_{des}$ step goes into the feedforward
(a one-cycle torque spike). The reference integrates the result, $q_{des} \mathrel{+}= \dot
q\,\Delta t$, and the joint loop around it is the tracking controller's PID + RNEA feedforward on
$(q_{des}, \dot q_{des}, \ddot q_{des})$. Integrating a reference instead of commanding joint
velocities directly keeps the arm on the effort interface (there is no way back to position on
gz_ros2_control 3.0.9, see
[what_broke.md F6](what_broke.md#f6-switching-a-joint-from-effort-back-to-position-drops-the-arm-gz_ros2_control-309))
and means a deadman stop holds the pose it stopped at. The reference is integrated open loop, with
no bound on $|q_{des} - q|$: if the arm is blocked, the reference keeps going and the torque rises
to the effort limit
([what_broke.md F5](what_broke.md#f5-no-controller-handles-a-stale-or-nan-joint-state--the-first-thing-that-would-bite-on-hardware)).

**Output.** `~/controller_state` as for the tracking controller, plus `~/jog_state`
(`std_msgs/Float64MultiArray`): `[t, t_cmd, w, λ, scale, v_cmd(6), v_meas(6)]`, where $t$ is the
update time, $t_{cmd}$ the stamp of the command in use (both sim time in Gazebo, so latency can be
measured without wall clocks), scale the product of the three limit scalings and $v_{meas} =
J(q)\dot q$ from the measured state.

**Benchmark: written, not yet published.** `bench_jog` (`scripts/bench_jog.sh`) measures
command-to-motion latency (to 50 % and 90 % of the commanded speed), velocity tracking while
streaming, and the stop time after the last command. Its numbers are not published anywhere yet:
`scripts/regenerate_all.sh` does not call it, `results/` has no jog CSV, and
[results.md](results.md) has no jog section. The controller's runtime evidence so far is the
mock-hardware tests and the smoke test's jog leg (the arm moves under a streamed twist and stops
after the deadman), which does not check that the TCP moved along the commanded axis.

## Two dynamics backends

Both controllers get their dynamics through one interface with two implementations, chosen per
controller by the read-only `dynamics_backend` parameter: `pinocchio` (the default) or `kdl`
(`sim.launch.py dynamics_backend:=kdl` switches every controller). KDL (`ChainIdSolver_RNE` for
RNEA, `ChainDynParam` for gravity, mass matrix and Coriolis terms) is what most ros_control-era arm
stacks used; Pinocchio is the default here because the same model also gives frame Jacobians,
placements and analytical derivatives, and it is the faster of the two. Both are built from the same
URDF.

`src/dynamics/dynamics_model.hpp` is the interface the controllers use: `gravity`, `rnea`,
`nonlinear_effects`, `mass_matrix`, `tip_jacobian` (rows [linear; angular], reference point at the
tip, URDF-root orientation, i.e. Pinocchio's `LOCAL_WORLD_ALIGNED`), the tip pose, the URDF limits
and `set_payload`. Every query is allocation-free and writes into buffers sized at construction;
`set_payload` and `clone` may allocate and only run outside the loop. A runtime payload change
therefore rebuilds the model outside the realtime loop (for KDL that means new solvers) and
`update()` only swaps a pointer ([Hand guiding](#hand-guiding)).

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

**Gravity.** One read-only parameter, `gravity_z` (default −9.81 m/s²), feeds both backends. It must
equal the simulator's gravity: SDF's default is −9.8, and in v0.1 that 0.01 m/s² difference made a
correctly modelled arm drift in hand guiding until the worlds set `<gravity>0 0 -9.81</gravity>`
explicitly (drift before and after in the
[what_broke.md appendix](what_broke.md#appendix-hand-recorded-runs-not-regenerated-by-any-script)).
`test_gravity_consistency.py` keeps the worlds, the controllers' YAML and the parameter default in
step.

**Equivalence.** `test/test_dynamics_equivalence.cpp` evaluates both backends on 200 random states
(q inside the URDF position limits, q̇ inside the velocity limits, |q̈| ≤ 10 rad/s²) at 0 kg and
1 kg payload, plus a 1 kg off-axis payload on the rotated `ft_frame` and gravity −9.8. Gravity, RNEA
and $C\dot q + g$ must agree within 1e-6 N·m per joint, $M$ within 1e-6, the Jacobian within 1e-9
and the tip pose within 1e-9; the worst deviations it prints are at the rounding level (of order
1e-14 N·m), which is what two exact recursions on the same inertias should give. The mock-hardware
test runs both controllers on the KDL backend and compares their torques with Pinocchio's, and
`scripts/ci_smoke.sh` (in CI with `SMOKE_BACKEND=kdl` too) runs both in Gazebo.

**Cost.** `bench_dynamics_timing` (table in [results.md](results.md#dynamics-timing)) times single
calls: Pinocchio is faster on both $g(q)$ and RNEA, and on this 6-DoF arm neither uses more than a
fraction of a percent of the 2 ms cycle.

### Pinocchio identities

The Pinocchio backend (Pinocchio 4.1) is also tested against identities that must hold for any
rigid-body model (`test/test_pinocchio_model.cpp`), so the equivalence test compares KDL with a
reference that is itself checked: $g = \mathrm{RNEA}(q,0,0)$,
$\mathrm{RNEA}(q,0,a) - g = M a$, $M = M^T \succ 0$, the Jacobian equals the finite difference
of the tip position, and adding a payload $m$ at the tip adds $J^T [0,0,mg]^T$ to $g(q)$.

## Gains and bandwidths

The tracking controller's PID gains (`kp`, `kd`, `ki` in `controllers.yaml`; the jog controller's
joint loop uses the same values) were chosen per joint on the diagonal of $M(q)$ at the home pose
$q = [0, -1.57, 1.57, -1.57, -1.57, 0]$, with integral time $K_p/K_i = 0.5$ s. The design note said
"critically damped PD at about 20 rad/s (base) and 30 rad/s (wrist), $K_p = M_{jj}\omega^2$, $K_d =
2M_{jj}\omega$". The shipped gains do not follow that formula on the wrist joints. Reading them back
with

$$
\omega_j = \sqrt{K_{p,j}/M_{jj}}, \qquad \zeta_j = \frac{K_{d,j}}{2\sqrt{K_{p,j} M_{jj}}}
$$

(the PD part only; the integral adds a slow pole near $K_i/K_p$ = 2 rad/s), with $M_{jj}$ from
Pinocchio's `crba` on the URDF at the home pose:

| Joint | $M_{jj}$ (kg m²) | $K_p$ (N·m/rad) | $K_d$ (N·m·s/rad) | $\omega$ (rad/s) | $\zeta$ | $K_d \Delta t / M_{jj}$ | CT $\omega$ (rad/s) |
|---|---|---|---|---|---|---|---|
| shoulder_pan | 1.06 | 600 | 50 | 23.8 | 0.99 | 0.094 | 20 |
| shoulder_lift | 2.59 | 1200 | 100 | 21.5 | 0.90 | 0.077 | 20 |
| elbow | 0.881 | 500 | 40 | 23.8 | 0.95 | 0.091 | 20 |
| wrist_1 | 0.0232 | 30 | 1.5 | 35.9 | 0.90 | 0.129 | 30 |
| wrist_2 | 0.00535 | 8 | 0.3 | 38.7 | 0.72 | 0.112 | 30 |
| wrist_3 | 0.000258 | 0.5 | 0.02 | 44.1 | 0.88 | 0.155 | 30 |

So the base joints are close to the design (21–24 rad/s, $\zeta$ 0.90–0.99) but the wrist PD loops
run at 36–44 rad/s, not 30, and wrist_2 is clearly underdamped ($\zeta$ = 0.72). Two consequences:

- The computed-torque law uses $\omega$ = 20 / 30 rad/s, so the two laws are compared at roughly
  equal stiffness on the base joints only. On the wrists the PD loop of PID + feedforward is
  $(36/30)^2$ to $(44/30)^2$, i.e. 1.4–2.2 times, stiffer than computed torque, which favours PID +
  feedforward there.
- The explicit 2 ms loop needs $K_d \Delta t / M_{jj} < 1$. The largest value is wrist_3's, about
  0.16, six times inside the bound: wrist_3's gains are tiny because its inertia is (the last link
  plus whatever is bolted to the flange), not because the loop is at a stability edge
  ([what_broke.md F9](what_broke.md#f9-wrist_3-is-the-worst-joint-because-of-that-timing-floor-not-because-of-a-stability-limit)).
  The YAML comment rounds wrist_3's $M_{jj}$ to $3 \cdot 10^{-4}$ kg m², which gives 40.8 rad/s,
  $\zeta$ = 0.82 and 0.133; the table uses the model's value.

The payload changes wrist_3 more than any other joint. The 1 kg, 6 cm model payload cube sits on the
wrist_3 axis and adds $m s^2/6 = 6 \cdot 10^{-4}$ kg m² to $M_{66}$, 3.3 times the bare value; with
the same fixed gains wrist_3's PD loop drops to about 24 rad/s and $\zeta \approx 0.48$. Computed
torque scales its acceleration-unit gains by $M(q)$ and does not have this problem, as long as its
model payload is right.

Hand guiding's viscous damping `damping` obeys the same explicit-loop bound, $D_j \Delta t / M_{jj}
< 1$: at the shipped values it is largest on wrist_2 (0.56) and wrist_3 (0.39) with no payload.
Computed torque's $K_d' \Delta t = 2\zeta\omega\Delta t$ is 0.08 on the base joints and 0.12 on the
wrists, independent of the inertia.

## Not done yet

Longer-term work that has not been started (moved here from the README; the four next steps are in
the README's "What I'd do next"):

- Publish `bench_jog`'s command-to-motion latency for the `TcpJogController` and add a
  singularity-approach benchmark (damped vs undamped joint speeds). `bench_jog` exists; its results
  are not published and `regenerate_all.sh` does not run it.
- An admittance controller on the wrist F/T sensor (`tcp_fts_sensor`): the hand-guiding law that
  would run on a real UR without the effort interface, commanding a TCP velocity from the measured
  external force.
- MPC for tracking. It needs a QP solved inside the 2 ms loop (a real-time-safe solver,
  warm-starting, a fallback when it misses the deadline), which is a separate piece of work from the
  controllers here.
- A system-identification experiment in simulation: excite the simulated arm, fit the inertial
  parameters, and measure how far the fitted model's feedforward is from the URDF's.
- An IBVS (image-based visual servoing) demo.
- Quintic or other jerk-limited profiles with via-point blending (README "What I'd do next", item 4;
  [what_broke.md F10](what_broke.md#f10-the-max-error-statistic-is-unstable-between-runs-because-the-trapezoid-steps-the-desired-acceleration-at-segment-corners)).
- A FollowJointTrajectory action server for the tracking controller.
- The full tracking grid: five speeds × four variants × five repetitions.
- A `ur5e_teaching_msgs` package.
