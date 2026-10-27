# Controllers

Both controllers are C++17 `ros2_control` plugins on the **effort** interface. All dynamics come
from the `robot_description` URDF through Pinocchio (`src/dynamics/pinocchio_model.cpp`); no
UR5e mass, length or inertia appears in controller code. The configured gains live in
`src/ur5e_teaching_bringup/config/controllers.yaml`.

Notation: $q, \dot q \in \mathbb{R}^6$ measured joint positions/velocities,
$M(q)$ mass matrix, $C(q,\dot q)\dot q$ Coriolis/centrifugal torque, $g(q)$ gravity torque,
$\Delta t = 2$ ms controller period.

## (a) PidFfTrajectoryController — mode `track`

**Input.** `trajectory_msgs/JointTrajectory` on `~/joint_trajectory` (topic only; the
FollowJointTrajectory action is listed under "What's next"). A new message pre-empts the
running trajectory and starts from the current desired state.

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

**Control law.**

$$
\tau = K_p e + K_d \dot e + K_i \int e\,dt + \tau_{ff}, \qquad e = q_{des} - q,
\qquad \tau_{ff} = \mathrm{RNEA}(q_{des}, \dot q_{des}, \ddot q_{des})
= M(q_{des})\ddot q_{des} + C(q_{des},\dot q_{des})\dot q_{des} + g(q_{des}).
$$

The feedforward is evaluated on the *desired* state, so it is a pure function of the reference
and does not feed measurement noise back. `feedforward:=false` turns it off for the comparison
benchmark. The model payload (`payload_mass`, `payload_com`) is added to the tool body.

**Saturation and anti-windup.** $\tau$ is clamped to the URDF effort limit times
`effort_limit_scale`. The integral uses conditional integration: it is frozen whenever the
unsaturated command already exceeds the limit in the direction the error would push it, and
$|K_i \int e|$ is clamped to `i_clamp`. Both behaviours are unit-tested
(`test/test_control_laws.cpp`).

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
optional (`coriolis_compensation`, computed with Pinocchio's `nonLinearEffects`). $D$ is a
diagonal viscous damping for stability and feel. The friction hook
$\tau_{fric} = F_v\dot q + F_c\tanh(\dot q / 0.01)$ is zero by default because Gazebo joints
have no friction; on a real arm it is where identified friction goes.

**Runtime payload.** `payload_mass` and `payload_com` can be set while the controller runs
(`ros2 param set`). The new values are pushed into the inertia of the body that carries
`tool0` (payload = solid 6 cm cube, the same shape the Gazebo payload link uses), so $\hat g$
and $\hat C$ change on the next cycle.

**What the operator feels.** With a perfect model the arm floats and a push $F$ at the tool
moves it against the damping only. With a payload error $\Delta m = m_{actual} - m_{model}$ the
arm sinks ($\Delta m > 0$) or rises ($\Delta m < 0$), and holding it still costs a force of
$\Delta m\,g$ at the payload. `bench_handguide` measures exactly that force from the
release acceleration ($F = -J^{-T} M \ddot q$) and compares it with $\Delta m\,g$.

**Safety.**

- *Joint-limit repulsion*: within `limit_margin` of a URDF position limit a spring of stiffness
  `limit_stiffness` pushes the joint back (zero in the interior).
- *Speed cap*: if any $|\dot q_j|$ exceeds `max_joint_velocity[j]` the controller writes zero
  torque and returns `ERROR`, and the controller manager deactivates it. In simulation the arm
  then falls; on a real arm this is where the brakes engage. Re-home with mode `track`.

## Dynamics wrapper — `PinocchioModel`

Thin layer over Pinocchio 4.1: `buildModelFromXML(robot_description)`, a map from the
controller's joint order to Pinocchio's, and allocation-free `gravity`, `rnea`,
`nonlinear_effects`, `mass_matrix`, `tip_jacobian` (LOCAL_WORLD_ALIGNED, rows
[linear; angular]) and `set_payload`. It is tested against identities that must hold for any
rigid-body model (`test/test_pinocchio_model.cpp`): $g = \mathrm{RNEA}(q,0,0)$,
$\mathrm{RNEA}(q,0,a) - g = M a$, $M = M^T \succ 0$, the Jacobian equals the finite difference
of the tip position, and adding a payload $m$ at the tip adds $J^T [0,0,mg]^T$ to $g(q)$.
