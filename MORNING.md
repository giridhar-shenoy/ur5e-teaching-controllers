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
