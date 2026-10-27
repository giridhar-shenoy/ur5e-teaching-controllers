# MORNING — ur5e-teaching-controllers

## WAITING FOR GIRI

### 1. GitHub Actions does not start on this private repo (billing)

- **What**: every CI run on `giridhar-shenoy/ur5e-teaching-controllers` ends in ~3 s with the
  annotation *"The job was not started because recent account payments have failed or your
  spending limit needs to be increased. Please check the 'Billing & plans' section in your
  settings"*. First seen on run
  [40591497563](https://github.com/giridhar-shenoy/ur5e-teaching-controllers/actions/runs/40591497563).
- **Why it needs you**: it is an account billing setting. The other way to get free minutes
  is to make the repo public, and changing visibility was explicitly not approved for tonight.
- **Recommendation**: fix the payment method / raise the Actions spending limit (private repos on
  the free plan get 2,000 min/month; one run of this workflow should take ~10–15 min once the
  image layers are cached), or flip the repo to public when you are ready to publish it. Then
  re-run the latest workflow (`gh run rerun <id>` or push any commit).
- **What it blocks**: the acceptance items "CI run on GitHub is green" (plumbing and feature).
  Meanwhile I run the same steps natively on the laptop (pre-commit, rosdep check, colcon build,
  colcon test, ci_smoke.sh) and paste their outputs in CHECKPOINTS.md. The Docker image build
  itself cannot be verified locally (Docker is not installed).

### 2. Git author e-mail on every commit is the old work address

- **What**: the machine's global git config is `Giridhar J Shenoy <the old work e-mail>`, so
  every commit in this repo carries that address. The repo is private for now.
- **Why it needs you**: identity and history are yours to decide; changing it afterwards means
  rewriting history, which I must not do.
- **Recommendation**: if you want your personal address on public portfolio repos, set
  `git config user.email` in this repo before the next commits; whether to rewrite the existing
  ones before going public is your call (it is a force-push).
- **Blocks**: nothing technically.

### 3. Facts in the README / what_broke.md only you can confirm

- **README "What transfers to a real UR"** says the stock `ur_robot_driver` path is position /
  velocity, not joint torque, as your brief states. Newer PolyScope releases may offer some
  direct-torque mode; please confirm the sentence matches what you want to claim.
- **Blocks**: publishing (not the code).

### 4. Branch protection and visibility

- Not enabled / not changed (not approved for tonight). Recommendation: once CI runs, protect
  `main` with the `build-test` check required, then make the repo public when you are happy
  with the README.
