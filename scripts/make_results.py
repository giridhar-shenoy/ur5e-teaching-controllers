#!/usr/bin/env python3
"""Turn results/*.csv into docs/results.md, the README's generated blocks and the result figures.

Every number in docs/results.md and in the README blocks is computed here from the CSVs the
benchmark scripts write, plus inputs read from the bench source and controllers.yaml; no number
is typed in by hand. src/ur5e_teaching_bench/test/test_results_sync.py checks that the README
blocks and docs/results.md are exactly what the CSVs produce.

    python3 scripts/make_results.py                     # rewrite docs/results.md, README, PNGs
    python3 scripts/make_results.py --check --no-plots  # exit 1 if any text is out of date
    python3 scripts/make_results.py --readme /tmp/README.md   # write the blocks into another file

Inputs: results/{tracking,handguide,dynamics_timing}.csv are the published single run of each
benchmark; results/runs_2026-10-31/run*_tracking.csv are repeated tracking runs of the same code
(one CSV per run, same schema) and loadavg.csv the machine load during each of them.

The README must contain each marker pair below exactly once, in this order:
<!-- headline:begin/end -->, <!-- breath:begin/end -->, <!-- results:begin/end -->.
"""

from __future__ import annotations

import argparse
import ast
import csv
import math
import re
import sys
import textwrap
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
RUNS_DIR = RESULTS / "runs_2026-10-31"
DOCS = ROOT / "docs"
ASSETS = DOCS / "assets"
README = ROOT / "README.md"
BENCH_PKG = ROOT / "src" / "ur5e_teaching_bench" / "ur5e_teaching_bench"
CONTROLLERS_YAML = ROOT / "src" / "ur5e_teaching_bringup" / "config" / "controllers.yaml"
TRACKING_CONTROLLER = "pid_ff_trajectory_controller"

# Links from docs/results.md into docs/what_broke.md (headings owned by that file).
TIMING_OFFSET_DOC = "what_broke.md#timing-offset-between-reference-and-state"
APPENDIX_DOC = "what_broke.md#appendix-hand-recorded-runs-not-regenerated-by-any-script"

# README blocks, in the order they must appear; each marker pair exactly once.
BLOCKS = ("headline", "breath", "results")
NOMINAL = 1.0  # the speed scale the headline numbers are quoted at
NUMBER_WORDS = ("zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine")

# Reference palette (dataviz skill, validated light/dark): slot 1 blue, slot 2 orange, slot 3
# aqua (the three validate all-pairs in both modes; light aqua is below 3:1 on the surface, so
# every mark carries a direct value label and the tables are the table view).
THEMES = {
    "light": {
        "surface": "#fcfcfb",
        "ink": "#0b0b0b",
        "ink2": "#52514e",
        "muted": "#898781",
        "grid": "#e1e0d9",
        "axis": "#c3c2b7",
        "s1": "#2a78d6",
        "s2": "#eb6834",
        "s3": "#1baf7a",
    },
    "dark": {
        "surface": "#1a1a19",
        "ink": "#f0efec",
        "ink2": "#c3c2b7",
        "muted": "#898781",
        "grid": "#2c2c2a",
        "axis": "#383835",
        "s1": "#3987e5",
        "s2": "#d95926",
        "s3": "#199e70",
    },
}


# ---------------------------------------------------------------------------- formatting
def read_csv(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pm(values, scale=1.0, digits=2) -> str:
    v = np.asarray(values, dtype=float) * scale
    return f"{v.mean():.{digits}f} ± {v.std(ddof=1) if v.size > 1 else 0.0:.{digits}f}"


def sig(x: float, n: int = 2) -> str:
    """x to n significant figures, never in exponent notation (296 stays 296)."""
    if x == 0 or not math.isfinite(x):
        return f"{x:g}"
    decimals = max(0, n - 1 - math.floor(math.log10(abs(x))))
    return f"{x:.{decimals}f}"


def span(lo: float, hi: float, fmt) -> str:
    """'lo–hi', or a single value when both ends print the same."""
    a, b = fmt(lo), fmt(hi)
    return a if a == b else f"{a}–{b}"


def spread(values) -> str:
    """'lo–hi' of a between-run spread, with enough decimals (2 to 4) that the two ends differ:
    a model-based tracking error is never quoted as one sub-millirad number."""
    lo, hi = float(np.min(values)), float(np.max(values))
    for k in (2, 3, 4):
        text = span(lo, hi, lambda x, k=k: f"{x:.{k}f}")
        if "–" in text:
            break
    return text


def f2(x: float) -> str:
    return f"{x:.2f}"


def f3(x: float) -> str:
    return f"{x:.3f}"


def number_word(n: int) -> str:
    return NUMBER_WORDS[n] if 0 <= n < len(NUMBER_WORDS) else str(n)


def about(x: float) -> int:
    """Round a ratio for prose: to 10 above 50, to 5 above 10, else to 1."""
    step = 10 if x >= 50 else 5 if x >= 10 else 1
    return int(step * math.floor(x / step + 0.5))


def times_range(lo: float, hi: float) -> str:
    a, b = about(lo), about(hi)
    return f"about {a}" if a == b else f"between about {a} and {b}"


def orders_of_magnitude(lo: float, hi: float) -> str:
    a, b = round(math.log10(lo)), round(math.log10(hi))
    unit = "order" if max(a, b) == 1 else "orders"
    if a == b:
        return f"about {number_word(a)} {unit} of magnitude"
    return f"{number_word(a)} to {number_word(b)} {unit} of magnitude"


def stamp(rows: list[dict]) -> str:
    commits = sorted({r["git_commit"] for r in rows})
    seeds = sorted({r["seed"] for r in rows})
    backends = sorted({r["dynamics_backend"] for r in rows if r.get("dynamics_backend")})
    return (
        f"commit `{', '.join(commits)}`, seed {', '.join(seeds)}, physics step "
        f"{rows[0]['sim_step_s']} s, controller update rate {rows[0]['update_rate_hz']} Hz, "
        f"dynamics backend {', '.join(backends) or 'pinocchio'}"
    )


# ---------------------------------------------------------------------------- bench inputs
def module_constants(path: Path, names: set[str]) -> dict:
    """Literal module-level constants of a Python file, read without importing it (the bench
    modules import rclpy). `np.array([...])` is read as its list argument."""
    found = {}
    for node in ast.parse(path.read_text()).body:
        if not (isinstance(node, ast.Assign) and len(node.targets) == 1):
            continue
        target = node.targets[0]
        if not (isinstance(target, ast.Name) and target.id in names):
            continue
        value = node.value
        if isinstance(value, ast.Call) and value.args:
            value = value.args[0]
        found[target.id] = ast.literal_eval(value)
    missing = names - found.keys()
    if missing:
        raise SystemExit(f"make_results: cannot read {sorted(missing)} from {path}")
    return found


@dataclass(frozen=True)
class Profile:
    """The tracking bench's reference: waypoint offsets from the start pose, rest-to-rest 1/3-rule
    trapezoids (profiles/trapezoidal.hpp), `segment_s` per segment at 1×, error window from the
    start of motion to `settle_s` after its end (bench_tracking.py)."""

    joints: tuple[str, ...]
    offsets: np.ndarray  # (waypoints, joints)
    segment_s: float
    settle_s: float

    @classmethod
    def from_bench(cls) -> Profile:
        bench = module_constants(
            BENCH_PKG / "bench_tracking.py", {"PATH_OFFSETS", "NOMINAL_SEGMENT_S", "SETTLE_S"}
        )
        common = module_constants(BENCH_PKG / "common.py", {"JOINTS"})
        return cls(
            tuple(common["JOINTS"]),
            np.asarray(bench["PATH_OFFSETS"], dtype=float),
            float(bench["NOMINAL_SEGMENT_S"]),
            float(bench["SETTLE_S"]),
        )

    @property
    def distances(self) -> np.ndarray:
        return np.diff(np.vstack([np.zeros(len(self.joints)), self.offsets]), axis=0)

    def duration(self, speed: float) -> float:
        return len(self.offsets) * self.segment_s / speed

    def rms_velocity(self, speed: float) -> np.ndarray:
        """RMS of q̇_des per joint over the bench's error window. A 1/3-rule segment of distance
        D and duration T peaks at 1.5 D/T for the middle third and ramps linearly in the outer
        two, so the integral of q̇² over it is (1.5 D/T)² · T · (1/3 + 2/9) = 1.25 D²/T."""
        t = self.segment_s / speed
        window = self.duration(speed) + self.settle_s
        return np.sqrt(1.25 * (self.distances**2).sum(axis=0) / t / window)

    def check(self, rows: list[dict]) -> None:
        """RMS(q̇_des) above assumes no segment was stretched by a velocity or acceleration limit,
        i.e. every run took the requested time."""
        for r in rows:
            speed = float(r["speed_scale"])
            requested, profiled = float(r["requested_s"]), float(r["profiled_s"])
            if abs(requested - self.duration(speed)) > 1e-3 or abs(profiled - requested) > 0.01:
                raise SystemExit(
                    "make_results: a tracking row's duration does not match the bench path "
                    f"({requested} s requested, {profiled} s profiled at {speed:g}×); RMS(q̇_des) "
                    "would need the controller's limits"
                )


def controller_param(name: str) -> object:
    config = yaml.safe_load(CONTROLLERS_YAML.read_text())
    return config[TRACKING_CONTROLLER]["ros__parameters"][name]


def kp_by_joint() -> dict[str, float]:
    joints = controller_param("joints")
    return dict(zip(joints, (float(k) for k in controller_param("kp")), strict=True))


# ---------------------------------------------------------------------------- tracking
# Tracking variants in display order: (CSV value, label). v0.1 CSVs have a `feedforward` column
# (0 = pid, 1 = pid_ff) instead of `variant`.
VARIANTS = (("pid", "PID"), ("pid_ff", "PID + RNEA FF"), ("computed_torque", "Computed torque"))
LABEL = dict(VARIANTS)


def variant_of(row: dict) -> str:
    if row.get("variant"):
        return row["variant"]
    return "pid_ff" if int(row["feedforward"]) else "pid"


def present_variants(keyed) -> list[str]:
    have = {v for _, v in keyed}
    return [v for v, _ in VARIANTS if v in have]


def tracking(rows: list[dict]):
    """worst[(speed, variant)] = list over reps of max-over-joints RMSE (and max |e|)."""
    per_rep = defaultdict(list)
    for r in rows:
        per_rep[(float(r["speed_scale"]), variant_of(r), int(r["rep"]))].append(r)
    worst_rmse, worst_max, per_joint = defaultdict(list), defaultdict(list), defaultdict(list)
    for (speed, variant, _rep), joints in sorted(per_rep.items()):
        worst_rmse[(speed, variant)].append(max(float(j["rmse_rad"]) for j in joints))
        worst_max[(speed, variant)].append(max(float(j["max_abs_err_rad"]) for j in joints))
        for j in joints:
            per_joint[(speed, variant, j["joint"])].append(
                (float(j["rmse_rad"]), float(j["max_abs_err_rad"]))
            )
    return worst_rmse, worst_max, per_joint


def worst_means(rows: list[dict]) -> dict[tuple[float, str], float]:
    """Worst-joint RMSE [rad] per (speed, variant), mean over the repetitions of one run."""
    worst_rmse, _, _ = tracking(rows)
    return {k: float(np.mean(v)) for k, v in worst_rmse.items()}


def offsets_ms(rows: list[dict], profile: Profile) -> dict[tuple[float, str], np.ndarray]:
    """RMSE ÷ RMS(q̇_des) per joint [ms], per (speed, variant); RMSE is the mean over reps."""
    _, _, per_joint = tracking(rows)
    out = {}
    for speed, variant in {(s, v) for s, v, _ in per_joint}:
        rmse = np.array(
            [np.mean(np.asarray(per_joint[(speed, variant, j)])[:, 0]) for j in profile.joints]
        )
        out[(speed, variant)] = 1e3 * rmse / profile.rms_velocity(speed)
    return out


@dataclass
class Run:
    number: int
    csv_name: str
    rows: list[dict]
    load: dict | None

    @property
    def worst(self) -> dict[tuple[float, str], float]:
        return worst_means(self.rows)


def read_runs(directory: Path) -> list[Run]:
    def number(p: Path) -> int:
        return int(re.match(r"run(\d+)_tracking\.csv$", p.name).group(1))

    paths = sorted(directory.glob("run*_tracking.csv"), key=number)
    if not paths:
        raise SystemExit(f"make_results: no run*_tracking.csv in {directory}")
    load_csv = directory / "loadavg.csv"
    load = {r["csv"]: r for r in read_csv(load_csv)} if load_csv.exists() else {}
    return [Run(number(p), p.name, read_csv(p), load.get(p.name)) for p in paths]


def across_runs(runs: list[Run], speed: float, variant: str) -> np.ndarray:
    """Per-run worst-joint RMSE [mrad] for one condition, in run order (runs lacking it skipped)."""
    return np.array([1e3 * r.worst[(speed, variant)] for r in runs if (speed, variant) in r.worst])


def ratio_range(runs: list[Run], speed: float) -> tuple[float, float]:
    """Lowest and highest per-run PID ÷ PID + FF worst-joint RMSE at one speed."""
    ratios = [r.worst[(speed, "pid")] / r.worst[(speed, "pid_ff")] for r in runs]
    return min(ratios), max(ratios)


def spearman(x, y) -> float:
    rx = np.argsort(np.argsort(x)).astype(float)
    ry = np.argsort(np.argsort(y)).astype(float)
    return float(np.corrcoef(rx, ry)[0, 1])


# ---------------------------------------------------------------------------- handguide
def handguide(rows: list[dict]):
    by = defaultdict(list)
    for r in rows:
        by[(float(r["model_payload_kg"]), float(r["actual_payload_kg"]))].append(r)
    return dict(sorted(by.items(), key=lambda kv: kv[0][1] - kv[0][0]))


@dataclass(frozen=True)
class HandGuide:
    errors_kg: np.ndarray  # actual − model, one per condition
    force: np.ndarray  # mean hold F_z [N] per condition
    expected: np.ndarray  # Δm·g [N] per condition
    drift_mm: np.ndarray  # mean TCP displacement at 1 s [mm] per condition
    residual_n: float  # max |F_z − Δm·g| over every row
    printed_n: float  # the CSV's last printed digit of F_z
    rows: int

    @classmethod
    def from_rows(cls, rows: list[dict]) -> HandGuide:
        by = handguide(rows)
        decimals = max(len(r["hold_force_z_n"].partition(".")[2]) for r in rows)
        return cls(
            np.array([a - m for m, a in by]),
            np.array([np.mean([float(r["hold_force_z_n"]) for r in rs]) for rs in by.values()]),
            np.array([float(rs[0]["expected_hold_force_z_n"]) for rs in by.values()]),
            np.array(
                [1e3 * np.mean([float(r["drift_disp_1s_m"]) for r in rs]) for rs in by.values()]
            ),
            max(
                abs(float(r["hold_force_z_n"]) - float(r["expected_hold_force_z_n"])) for r in rows
            ),
            10.0**-decimals,
            len(rows),
        )

    @property
    def wrong(self) -> np.ndarray:
        return self.errors_kg != 0

    @property
    def g(self) -> float:
        """N per kg of payload error, from the CSV's own Δm·g column."""
        return float(np.mean(self.expected[self.wrong] / self.errors_kg[self.wrong]))

    @property
    def error_kg(self) -> str:
        sizes = sorted({abs(e) for e in self.errors_kg[self.wrong]})
        return " / ".join(f"{s:g}" for s in sizes)

    @property
    def hold_n(self) -> str:
        """|F_z| needed to hold the arm with a wrong payload model [N], as printed."""
        held = np.abs(self.force[self.wrong])
        return span(held.min(), held.max(), f2)

    @property
    def drift(self) -> str:
        """TCP displacement 1 s after release with a wrong payload model [mm], lowest–highest."""
        moved = self.drift_mm[self.wrong]
        return span(moved.min(), moved.max(), lambda x: f"{x:.0f}")


def handguide_table(by) -> list[str]:
    lines = [
        "| Model / actual payload (kg) | Payload error (kg) "
        "| TCP drift speed at 1 s (mm/s) "
        "| TCP drift in 1 s (mm) | Effort to hold, F_z (N) | Expected (Δm·g, N) "
        "| Regenerate with |",
        "|---|---|---|---|---|---|---|",
    ]
    for (model, actual), rs in by.items():
        expected = float(rs[0]["expected_hold_force_z_n"])
        lines.append(
            f"| {model:g} / {actual:g} | {actual - model:+g} | "
            f"{pm([r['drift_speed_1s_mps'] for r in rs], 1e3)} | "
            f"{pm([r['drift_disp_1s_m'] for r in rs], 1e3)} | "
            f"{pm([r['hold_force_z_n'] for r in rs])} | {expected:.2f} | "
            "`scripts/bench_handguide.sh` |"
        )
    return lines


# ---------------------------------------------------------------------------- dynamics timing
BACKENDS = (("pinocchio", "Pinocchio"), ("kdl", "KDL"))
FUNCTIONS = (("gravity", "g(q)"), ("rnea", "RNEA"))


def timing(rows: list[dict]) -> dict:
    """timing[(backend, function)] = CSV row (one row per pair, one run)."""
    return {(r["backend"], r["function"]): r for r in rows}


def us(row: dict, key: str) -> str:
    # The executable times each call in whole nanoseconds, so three decimals in µs are exact.
    return f"{float(row[key]):.3f}"


def cycle_ms(t: dict) -> str:
    """The control cycle the CSV's 'share of cycle' columns are relative to [ms]."""
    cycles = [float(r["median_us"]) / float(r["median_cycle_fraction"]) for r in t.values()]
    return f"{round(float(np.median(cycles)) / 1e3, 1):g}"


def ceil1(x: float) -> str:
    """x rounded up to one significant figure, for an 'at most' statement."""
    exp = 10.0 ** math.floor(math.log10(x))
    return sig(math.ceil(x / exp - 1e-9) * exp, 1)


def worst_p95_share(t: dict) -> float:
    return 100 * max(
        float(t[(b, f)]["p95_cycle_fraction"]) for b, _ in BACKENDS for f, _ in FUNCTIONS
    )


def timing_summary_table(t: dict) -> list[str]:
    cells = [
        f"{us(t[(b, f)], 'median_us')} / {us(t[(b, f)], 'p95_us')}"
        for b, _ in BACKENDS
        for f, _ in FUNCTIONS
    ]
    head = " | ".join(f"{bn} {fn}" for _, bn in BACKENDS for _, fn in FUNCTIONS)
    return [
        f"| Per call, median / p95 (µs) | {head} | Worst p95, share of the {cycle_ms(t)} ms cycle "
        "| Regenerate with |",
        "|---|---|---|---|---|---|---|",
        f"| UR5e, 6 DoF, one thread | {' | '.join(cells)} | {worst_p95_share(t):.4f} % "
        "| `scripts/bench_dynamics_timing.sh` |",
    ]


def timing_table(t: dict) -> list[str]:
    c = cycle_ms(t)
    lines = [
        "| Backend | Call | Median (µs) | p95 (µs) | Mean (µs) | Max (µs) "
        f"| Median, share of {c} ms | p95, share of {c} ms |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for b, bn in (*BACKENDS, ("none", "—")):
        for f, fn in (*FUNCTIONS, ("clock", "two clock reads only")):
            if (b, f) not in t:
                continue
            r = t[(b, f)]
            lines.append(
                f"| {bn} | {fn} | {us(r, 'median_us')} | {us(r, 'p95_us')} "
                f"| {float(r['mean_us']):.3f} | {float(r['max_us']):.2f} "
                f"| {100 * float(r['median_cycle_fraction']):.4f} % "
                f"| {100 * float(r['p95_cycle_fraction']):.4f} % |"
            )
    return lines


def timing_stamp(rows: list[dict]) -> str:
    r = rows[0]
    return (
        f"commit `{r['git_commit']}`, build type {r['build_type']}, CPU {r['cpu']}, "
        f"{int(r['calls']):,} timed calls per row, seed {r['seed']}"
    )


# ---------------------------------------------------------------------------- the data
@dataclass
class Data:
    published: list[dict]  # results/tracking.csv
    runs: list[Run]  # results/runs_*/run*_tracking.csv
    handguide_rows: list[dict]
    timing_rows: list[dict]
    profile: Profile
    kp: dict[str, float]

    @property
    def hg(self) -> HandGuide:
        return HandGuide.from_rows(self.handguide_rows)

    @property
    def t(self) -> dict:
        return timing(self.timing_rows)

    @property
    def n_runs(self) -> str:
        """'five runs' / 'one run'."""
        n = len(self.runs)
        return f"{number_word(n)} run{'s' if n != 1 else ''}"

    def law(self, variant: str, speed: float = NOMINAL) -> np.ndarray:
        return across_runs(self.runs, speed, variant)

    def offsets(self) -> tuple[dict, list[dict]]:
        """(published run, [each repeated run]) RMSE ÷ RMS(q̇_des) per joint [ms]."""
        return offsets_ms(self.published, self.profile), [
            offsets_ms(r.rows, self.profile) for r in self.runs
        ]

    def run_offsets(self, variant: str = "pid_ff") -> np.ndarray:
        """Per-run median over the joints of RMSE ÷ RMS(q̇_des) at 1× [ms]."""
        return np.array([np.median(r[(NOMINAL, variant)]) for r in self.offsets()[1]])

    def joint_agreement(self) -> tuple[float, float, float, float]:
        """Largest ÷ smallest of the joints' RMSE ÷ RMS(q̇_des), over the published run and every
        repeated run: (worst for FF and CT at 1×, worst for FF and CT at any speed, lowest and
        highest for PID at 1×)."""
        pub, runs = self.offsets()
        model, model_1x, pid = [], [], []
        for table in (pub, *runs):
            for (speed, variant), d in table.items():
                if variant == "pid":
                    if speed == NOMINAL:
                        pid.append(d.max() / d.min())
                    continue
                model.append(d.max() / d.min())
                if speed == NOMINAL:
                    model_1x.append(d.max() / d.min())
        return max(model_1x), max(model), min(pid), max(pid)

    def load_story(self) -> tuple[str, str]:
        """(sentence about load vs the PID + FF error, verdict): verdict 'no' when the load
        average does not explain the spread, 'yes' when it might, 'unknown' with < 3 logs."""
        loaded = [r for r in self.runs if r.load and (NOMINAL, "pid_ff") in r.worst]
        if len(loaded) < 3:
            return "Fewer than three runs have a load log, so load and error are not compared.", (
                "unknown"
            )
        load = np.array([float(r.load["load1_mean"]) for r in loaded])
        err = np.array([1e3 * r.worst[(NOMINAL, "pid_ff")] for r in loaded])
        rho = spearman(load, err)
        busy, quiet = loaded[int(np.argmax(load))], loaded[int(np.argmin(load))]
        detail = (
            f"the busiest run (run {busy.number}, mean load {float(busy.load['load1_mean']):.2f}) "
            f"gave {1e3 * busy.worst[(NOMINAL, 'pid_ff')]:.3f} mrad and the quietest (run "
            f"{quiet.number}, mean load {float(quiet.load['load1_mean']):.2f}) "
            f"{1e3 * quiet.worst[(NOMINAL, 'pid_ff')]:.3f} mrad"
        )
        if rho >= 0.5:
            return (
                "In these runs the PID + RNEA FF error does rise with the mean 1-minute load "
                f"average (rank correlation {rho:+.2f}; {detail}).",
                "yes",
            )
        verdict = (
            "does not order the runs"
            if rho > -0.5
            else "orders the runs the wrong way round for a load explanation"
        )
        return (
            f"The 1-minute load average {verdict}: the rank correlation between a run's mean "
            f"load and its PID + RNEA FF error is {rho:+.2f}; {detail}.",
            "no",
        )


def para(*parts: str) -> list[str]:
    """One prose paragraph, wrapped at 100 columns like the hand-written docs."""
    return textwrap.wrap(
        " ".join(parts), width=100, break_long_words=False, break_on_hyphens=False
    ) + [""]


def pid_text(d: Data) -> str:
    pid = d.law("pid")
    text = span(pid.min(), pid.max(), sig)
    return text if "–" in text else f"about {text}"


def every_run(d: Data) -> str:
    return "in every run" if len(d.runs) > 1 else "in the one run so far"


def law_comparison(d: Data, speed: float = NOMINAL) -> str:
    ff, ct = d.law("pid_ff", speed), d.law("computed_torque", speed)
    if np.all(ct < ff):
        return f"computed torque lower than PID plus feedforward {every_run(d)}"
    if np.all(ct > ff):
        return f"computed torque higher than PID plus feedforward {every_run(d)}"
    return "neither law ahead in every run"


def offset_phrase(d: Data) -> str:
    worst = max(d.run_offsets("pid_ff").max(), d.run_offsets("computed_torque").max())
    return "a fraction of a millisecond" if worst < 1.0 else f"up to {sig(worst)} ms"


def timing_explains(d: Data) -> bool:
    """The per-joint signature of a timing offset: at 1× every joint's RMSE ÷ RMS(q̇_des) is
    within a factor of two of every other's, for both model-based laws, in every run."""
    return d.joint_agreement()[0] < 2.0


def offset_varies(d: Data) -> bool:
    med = d.run_offsets("pid_ff")
    return len(med) > 1 and med.max() / med.min() > 1.1


# ---------------------------------------------------------------------------- README blocks
# The README's own prose defines PID, feedforward, computed torque, gains, payload model, RMSE,
# mrad and RNEA before these blocks, so the blocks do not define them again.
def headline(d: Data) -> str:
    """The README's 'What this is' ends with this; its only number is the PID ÷ FF ratio."""
    lo, hi = ratio_range(d.runs, NOMINAL)
    hg = d.hg
    exact = "exactly" if hg.residual_n <= hg.printed_n * 1.0001 else "about"
    timing = timing_explains(d)
    if timing:
        varies = offset_varies(d)
        why = (
            " Most of the error that remains is a timing artefact: the simulation compares the "
            f"commanded angle (the reference) with the measured angle {offset_phrase(d)} apart"
            + (", by an amount that changes from run to run," if varies else ",")
            + " so a moving joint shows an error proportional to its speed even when it is on the "
            "path."
            + (" That is why the factor is a range, and" if varies else "")
            + (" the" if varies else " The")
            + " true improvement is probably larger."
        )
    else:
        why = ""
    ct = ""
    if law_comparison(d).startswith("neither"):
        ct = " Computed torque did the same" + (
            "; the artefact hides any difference between the two." if timing else "."
        )
    return (
        "With the shipped PID gains, PID plus feedforward made the worst joint's root-mean-square "
        f"tracking error {times_range(lo, hi)} times smaller than PID alone.{why}{ct} By "
        "construction (it follows from how the controller is built, not from a measurement), a "
        f"payload model off by some mass leaves {exact} that weight for the operator to hold."
    )


def breath(d: Data) -> str:
    """The README's 'Results in one breath' paragraph: a few plain sentences per benchmark, then
    the caveat."""
    ff, ct = d.law("pid_ff"), d.law("computed_torque")
    lo, hi = ratio_range(d.runs, NOMINAL)
    hg, t = d.hg, d.t
    g = abs(float(controller_param("gravity_z")))
    rnea = float(t[("pinocchio", "rnea")]["median_us"])
    right = abs(hg.force[~hg.wrong]).max()
    residual = f"{hg.residual_n:.{decimals_of(hg.printed_n)}f}"
    clean = hg.residual_n <= hg.printed_n * 1.0001 and right < hg.printed_n
    link = "([run-to-run spread](docs/results.md#run-to-run-spread))"
    tracking = (
        f"Tracking: on one path through all {number_word(len(d.profile.joints))} joints, at the "
        f"speed the path was designed for, PID alone leaves an RMSE of {pid_text(d)} mrad on the "
        f"worst joint. PID plus feedforward brings it to {spread(ff)} mrad and computed torque to "
        f"{spread(ct)} mrad across {d.n_runs} of the same code: {orders_of_magnitude(lo, hi)} "
        f"lower, with {law_comparison(d)}."
    )
    guiding = (
        f"Hand guiding: with the payload model {hg.error_kg} kg wrong, holding the arm still takes "
        f"{hg.hold_n} N, the weight of {hg.error_kg} kg ({hg.error_kg} kg × {g:g} m/s²). That "
        "number is fixed in advance, by construction: the simulated arm and the controller use "
        "the same model and the same gravity, so the mass the model gets wrong is the only thing "
        "left unbalanced, and its weight is what the operator has to hold."
    )
    if clean:
        guiding += (
            " What the benchmark can still catch is a mistake in the controller: had it attached "
            "the payload to the wrong part of the arm, or had the switch from one controller to "
            f"the next jolted the arm, the force would miss {hg.hold_n} N, or would not be zero "
            f"with the correct model; it hits both to within {residual} N."
        )
    cost = (
        f"Computing the dynamics is cheap: one RNEA call takes {rnea:.2f} µs (microseconds; the "
        f"median call on the default backend), against {cycle_ms(t)} ms between control updates."
    )
    if len(d.runs) < 2:
        caveat = (
            "The caveat: only one repeated run is in place, so how much the feedforward number "
            f"moves between runs is not given yet {link}."
        )
    elif timing_explains(d):
        changes = "changes" if offset_varies(d) else "is about the same"
        verdict = d.load_story()[1]
        if verdict == "no":
            load = (
                " CPU load from other programs was the first suspect, but the load average (a "
                f"measure of how busy the computer was) logged during each of the {d.n_runs} does "
                "not line up with their errors"
            )
        elif verdict == "yes":
            load = (
                " The load average logged during the runs does rise with the error, so CPU load "
                "is not ruled out"
            )
        else:
            load = ""
        caveat = (
            "The caveat: the recorded reference and the measured joint angle are compared at "
            f"slightly mismatched instants, {offset_phrase(d)} apart, and that mismatch {changes} "
            "from run to run; it is what moves the feedforward and computed-torque numbers "
            f"between runs{'.' if load else ''}{load} {link}."
        )
    else:
        caveat = (
            "The caveat: the feedforward and computed-torque numbers move between runs of the "
            f"same code, for a reason not yet pinned down {link}."
        )
    return " ".join([tracking, guiding, cost, caveat])


def direction(force: float) -> str:
    """What the operator has to do against a vertical hold force F_z [N]."""
    return "pushing down" if force < 0 else "holding up"


def ceil_sig(x: float, n: int = 2) -> str:
    """x rounded up to n significant figures, for a 'within' statement."""
    exp = 10.0 ** (math.floor(math.log10(x)) - n + 1)
    return sig(math.ceil(x / exp - 1e-9) * exp, n)


def hero_note(wrong: list[tuple[float, float]]) -> str:
    """Point the reader from the hero GIF's take (record_mode.py) to the row with the same error."""
    hero = module_constants(BENCH_PKG / "record_mode.py", {"HERO_ACTUAL_KG", "HERO_MODEL_ERROR_KG"})
    real = float(hero["HERO_ACTUAL_KG"])
    model = real - float(hero["HERO_MODEL_ERROR_KG"])
    same = [k for k in wrong if math.isclose(k[1] - k[0], real - model)]
    if not same or (model, real) in wrong:
        return ""
    m, a = same[0]
    return (
        f". The GIF at the top (model {model:g} kg, real {real:g} kg) has the same "
        f"{real - model:g} kg error as the model {m:g} kg, real {a:g} kg row"
    )


def summary_table(d: Data) -> list[str]:
    """The README's simplified table (also docs/results.md's summary): the first data column is
    the case without the model or with it wrong, the second the case with the model."""
    ff, ct = d.law("pid_ff"), d.law("computed_torque")
    lo, hi = ratio_range(d.runs, NOMINAL)
    t = d.t
    orders = orders_of_magnitude(lo, hi)
    by = handguide(d.handguide_rows)
    wrong = [(m, a) for (m, a) in by if a != m]
    right = [(m, a) for (m, a) in by if a == m]

    def masses(m: float, a: float) -> str:
        return f"model {m:g} kg, real {a:g} kg"

    force = {k: float(np.mean([float(r["hold_force_z_n"]) for r in rs])) for k, rs in by.items()}
    moved = {
        k: 1e3 * float(np.mean([float(r["drift_disp_1s_m"]) for r in rs])) for k, rs in by.items()
    }
    hold_wrong = "<br>".join(
        f"{masses(*k)}: {abs(force[k]):.2f} N, {direction(force[k])}" for k in wrong
    )
    hold_right = "<br>".join(f"{masses(*k)}: {abs(force[k]):.2f} N" for k in right)
    drift_wrong = "<br>".join(
        f"{masses(*k)}: {'sinks' if force[k] > 0 else 'rises'} {moved[k]:.0f} mm" for k in wrong
    )
    drift_right = "<br>".join(f"{masses(*k)}: {abs(moved[k]):.2f} mm" for k in right)
    heaviest = max(wrong, key=lambda k: k[1])
    lightest = min(wrong, key=lambda k: k[1])
    if heaviest != lightest and moved[heaviest] < moved[lightest]:
        drift_note = (
            "The arm drifts until someone holds it; it moves less when the real payload is "
            f"{heaviest[1]:g} kg than when it is {lightest[1]:g} kg, because there is more mass to "
            "accelerate"
        )
    else:
        drift_note = "The arm drifts until someone holds it"
    p95 = max(float(t[(b, "rnea")]["p95_us"]) for b, _ in BACKENDS)
    cycle_us = 1e3 * float(cycle_ms(t))
    fraction = (
        "under a thousandth of the cycle"
        if p95 < cycle_us / 1000
        else f"{sig(100 * p95 / cycle_us, 1)} % of the cycle"
    )
    return [
        "| What was measured | Without the model, or with it wrong | With the model "
        "| What it means |",
        "|---|---|---|---|",
        "| **Tracking:** RMSE of the worst of the six joints, at the speed the path was designed "
        f"for (mrad) | PID alone: {pid_text(d)} "
        f"| PID plus feedforward: {spread(ff)}<br>computed torque: {spread(ct)}<br>"
        f"(lowest–highest of {d.n_runs}) "
        f"| {orders[0].upper()}{orders[1:]} less error, with {law_comparison(d)} |",
        f"| **Hand guiding:** force needed to hold the arm still | {hold_wrong} | {hold_right} "
        f"| Equal to Δm·g (the mass error times gravity), by construction{hero_note(wrong)} |",
        "| **Hand guiding:** how far the tool moves in the first second after letting go "
        f"| {drift_wrong} | {drift_right} | {drift_note} |",
        "| **Dynamics cost:** time for one RNEA call, median, in µs (microseconds) | — "
        f"| Pinocchio (the default): {float(t[('pinocchio', 'rnea')]['median_us']):.2f}<br>"
        f"KDL: {float(t[('kdl', 'rnea')]['median_us']):.2f} "
        f"| Far inside the {cycle_ms(t)} ms control cycle: 95 % of RNEA calls on either backend "
        f"finish within {ceil_sig(p95)} µs, {fraction} |",
    ]


def picture(name: str, alt: str, width: int, prefix: str) -> list[str]:
    return [
        "<picture>",
        f'  <source media="(prefers-color-scheme: dark)" srcset="{prefix}{name}_dark.png">',
        f'  <img alt="{alt}" src="{prefix}{name}_light.png" width="{width}">',
        "</picture>",
    ]


def tracking_caption(d: Data, spread_link: str) -> str:
    left, right = chart_scales(d)
    speeds = speeds_of(d)
    pid = [d.law("pid", s) for s in speeds]
    model_hi = max(d.law(v, s).max() for s in speeds for v in ("pid_ff", "computed_torque"))
    pid_lo, pid_hi = min(p.min() for p in pid), max(p.max() for p in pid)
    return (
        "Lower is better. 1× is the speed the path was designed for; "
        f"{' and '.join(f'{s:g}×' for s in speeds if s != NOMINAL)} run the same path slower "
        "or faster. Each bar is the worst joint's RMSE: the median of the "
        f"{d.n_runs}, printed above the bar, with a whisker (a thin line from the lowest to the "
        "highest run). "
        "The left panel is PID alone; the right panel shows PID plus feedforward and computed "
        f"torque on a scale {about(left / right)} times finer, because at the left panel's scale "
        f"they would barely show: every one is below {ceil1(model_hi)} mrad, against "
        f"{span(pid_lo, pid_hi, sig)} mrad for PID. The whiskers on the right are the "
        f"[run-to-run spread]({spread_link})."
    )


def tracking_alt(d: Data) -> str:
    speeds = speeds_of(d)
    pid = ", ".join(f"{sig(float(np.median(d.law('pid', s))))}" for s in speeds)
    model = [d.law(v, s) for s in speeds for v in ("pid_ff", "computed_torque")]
    return (
        "Two-panel bar chart of the worst joint's tracking error in mrad at "
        f"{', '.join(f'{s:g}' for s in speeds)} times the designed speed. Left, PID alone: "
        f"{pid} mrad. Right, on a finer scale, PID plus feedforward and computed torque: all "
        f"between {f2(min(m.min() for m in model))} and {f2(max(m.max() for m in model))} mrad "
        f"across the {number_word(len(speeds))} speeds, with whiskers (thin lines from the lowest "
        f"to the highest of the {d.n_runs})."
    )


def handguide_caption(d: Data) -> str:
    hg = d.hg
    up_when_light = bool(np.all(np.sign(hg.force[hg.wrong]) == np.sign(hg.errors_kg[hg.wrong])))
    sign = (
        "Positive means the operator has to push up, because the model is too light and the arm "
        "would sink; negative means push down. "
        if up_when_light
        else ""
    )
    return (
        f"The {number_word(len(hg.errors_kg))} measured points sit on the Δm·g line (mass error "
        f"times gravity, {hg.g:.2f} N per kg) to within "
        f"{hg.residual_n:.{decimals_of(hg.printed_n)}f} N. {sign}They sit on the line by "
        "construction, because the simulator and the controller use the same model and the same "
        "gravity; a controller that put the payload in the wrong place would miss it. On a real "
        "arm, friction and model error would move the points off the line."
    )


def signed(x: float, fmt: str) -> str:
    return "0" if x == 0 else format(x, fmt)


def signed_n(force: float) -> str:
    """'+9.81 N', '−9.81 N', '0.00 N' (typographic minus, no sign on zero)."""
    text = f"{force:+.2f}"
    return ("0.00" if float(text) == 0 else text.replace("-", "−")) + " N"


def handguide_alt(d: Data) -> str:
    hg = d.hg
    points = ", ".join(
        f"{signed_n(f)} at {signed(e, '+g').replace('-', '−')} kg"
        for e, f in zip(hg.errors_kg, hg.force, strict=True)
    )
    return (
        "Scatter of the vertical force needed to hold the arm still against payload-model error "
        f"(actual minus model mass): {points}, all on the dashed line force = error × "
        f"{hg.g:.2f} N per kg."
    )


def decimals_of(step: float) -> int:
    return max(0, -math.floor(math.log10(step) + 1e-9))


def results_block(d: Data) -> str:
    return "\n".join(
        [
            *summary_table(d),
            "",
            *picture("tracking_rmse", tracking_alt(d), 720, "docs/assets/"),
            "",
            tracking_caption(d, "docs/results.md#run-to-run-spread"),
            "",
            *picture("handguide", handguide_alt(d), 640, "docs/assets/"),
            "",
            handguide_caption(d),
            "",
            "Every speed and control law, the per-joint errors, each of the "
            f"{d.n_runs} with its machine load, the hand-guiding drift and the cost of each "
            "dynamics call on both backends are in [docs/results.md](docs/results.md).",
        ]
    )


# ---------------------------------------------------------------------------- docs/results.md
def speeds_of(d: Data) -> list[float]:
    return sorted({s for s, _ in d.runs[0].worst})


def runs_date() -> str:
    m = re.search(r"(\d{4})-(\d{2})-(\d{2})", RUNS_DIR.name)
    months = "Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec".split()
    return f"{int(m.group(3))} {months[int(m.group(2)) - 1]} {m.group(1)}" if m else RUNS_DIR.name


def digits_for(variant: str) -> int:
    """PID errors are mrad-sized; the model-based laws' need a third decimal."""
    return 2 if variant == "pid" else 3


def tracking_full_table(d: Data) -> list[str]:
    worst_rmse, worst_max, _ = tracking(d.published)
    lines = [
        "| Speed (× nominal) | Control law "
        "| Worst-joint RMSE, published run (mrad, mean ± std over reps) "
        f"| Worst-joint RMSE over the {d.n_runs} (mrad, lowest–highest run mean) "
        "| Worst max \\|e\\|, published run (mrad, mean ± std) | Regenerate with |",
        "|---|---|---|---|---|---|",
    ]
    for s in sorted({s for s, _ in worst_rmse}):
        for v in present_variants(worst_rmse):
            if (s, v) not in worst_rmse:
                continue
            k = digits_for(v)
            runs = d.law(v, s)
            spread = span(runs.min(), runs.max(), lambda x, k=k: f"{x:.{k}f}") if len(runs) else "—"
            lines.append(
                f"| {s:g} | {LABEL[v]} | {pm(worst_rmse[(s, v)], 1e3, k)} | {spread} "
                f"| {pm(worst_max[(s, v)], 1e3)} | `scripts/bench_tracking.sh` |"
            )
    return lines


def tracking_joint_table(per_joint) -> list[str]:
    joints = []
    for _s, _f, j in per_joint:
        if j not in joints:
            joints.append(j)
    lines = [
        "| Speed | Variant | " + " | ".join(f"{j} RMSE / max (mrad)" for j in joints) + " |",
        "|---|---|" + "---|" * len(joints),
    ]
    have = {k[1] for k in per_joint}
    for s in sorted({k[0] for k in per_joint}):
        for v in (v for v, _ in VARIANTS if v in have):
            cells = []
            for j in joints:
                vals = np.asarray(per_joint[(s, v, j)])
                cells.append(f"{pm(vals[:, 0], 1e3, digits_for(v))} / {pm(vals[:, 1], 1e3)}")
            lines.append(f"| {s:g} | {LABEL[v]} | " + " | ".join(cells) + " |")
    return lines


def run_table(d: Data) -> list[str]:
    pub_offsets, run_offsets = d.offsets()
    zones = sorted({r.load["started"][19:] for r in d.runs if r.load})
    started = f"Started (UTC{', '.join(zones)})" if zones else "Started"
    lines = [
        f"| Run | {started} | 1-min load average: start / mean / end "
        "| Heavy-process check at start "
        "| Worst-joint RMSE at 1×, PID (mrad) | PID + RNEA FF (mrad) | Computed torque (mrad) "
        "| RMSE ÷ RMS(q̇_des) at 1×, PID + RNEA FF (ms, median of the joints) "
        "| Computed torque (ms) |",
        "|---|---|---|---|---|---|---|---|---|",
    ]

    def cells(worst: dict, offsets: dict) -> str:
        return (
            f"{1e3 * worst[(NOMINAL, 'pid')]:.2f} | {1e3 * worst[(NOMINAL, 'pid_ff')]:.3f} "
            f"| {1e3 * worst[(NOMINAL, 'computed_torque')]:.3f} "
            f"| {np.median(offsets[(NOMINAL, 'pid_ff')]):.3f} "
            f"| {np.median(offsets[(NOMINAL, 'computed_torque')]):.3f}"
        )

    for run, offsets in zip(d.runs, run_offsets, strict=True):
        if run.load:
            when = run.load["started"][11:16]
            keys = ("load1_start", "load1_mean", "load1_end")
            load = " / ".join(f"{float(run.load[k]):.2f}" for k in keys)
            heavy = f"`{run.load.get('heavy_processes') or '—'}`"
        else:
            when, load, heavy = "—", "not logged", "—"
        lines.append(f"| {run.number} | {when} | {load} | {heavy} | {cells(run.worst, offsets)} |")
    lines.append(
        "| published (`results/tracking.csv`) | — | not logged | — "
        f"| {cells(worst_means(d.published), pub_offsets)} |"
    )
    return lines


def heavy_note(d: Data) -> str:
    """Name every run whose start-of-run heavy-process check did not come back `none`."""
    flagged = [r for r in d.runs if r.load and r.load.get("heavy_processes", "none") != "none"]
    if not flagged:
        return ""
    loads = sorted(float(r.load["load1_mean"]) for r in d.runs if r.load)
    notes = []
    for r in flagged:
        mean = float(r.load["load1_mean"])
        rank = (
            " the lowest of the runs"
            if mean == loads[0]
            else " the highest of the runs"
            if mean == loads[-1]
            else ""
        )
        notes.append(
            f"Run {r.number}: the heavy-process check at its start reported "
            f"`{r.load['heavy_processes']}`, and `loadavg.csv` does not record which process; "
            f"its mean load, {mean:.2f}, was{rank or ' within the range of the others'}."
        )
    return " ".join(notes)


def offset_table(d: Data) -> list[str]:
    joints = d.profile.joints
    pub, runs = d.offsets()

    def ratio(v: np.ndarray) -> str:
        r = v.max() / v.min()
        return f"{r:.{0 if r >= 10 else 2}f}"

    def row(label: str, values, last: str) -> str:
        return f"| {label} | " + " | ".join(values) + f" | {last} |"

    kp = np.array([d.kp[j] for j in joints])
    rms = d.profile.rms_velocity(NOMINAL)
    lines = [
        "| At 1× nominal speed | " + " | ".join(joints) + " | Largest ÷ smallest |",
        "|---|" + "---|" * (len(joints) + 1),
        row("Kp (N·m/rad, `controllers.yaml`)", [f"{k:g}" for k in kp], ratio(kp)),
        row("RMS(q̇_des) (rad/s, from the profile)", [f"{v:.3f}" for v in rms], ratio(rms)),
    ]
    for v in ("pid", "pid_ff", "computed_torque"):
        values = pub[(NOMINAL, v)]
        lines.append(
            row(f"{LABEL[v]}, published run (ms)", [sig(x, 3) for x in values], ratio(values))
        )
    for v in ("pid_ff", "computed_torque"):
        per_run = np.array([r[(NOMINAL, v)] for r in runs])  # (runs, joints)
        ratios = per_run.max(axis=1) / per_run.min(axis=1)
        lines.append(
            row(
                f"{LABEL[v]}, {d.n_runs}, lowest–highest (ms)",
                [span(c.min(), c.max(), f3) for c in per_run.T],
                f"{span(ratios.min(), ratios.max(), f2)} (per run)",
            )
        )
    return lines


def wrist_note(d: Data) -> str:
    """The joint with the lowest PID-only ratio sits on the same offset; the worst-joint PID
    number (another joint) hides that its PID-only RMSE moves between runs."""
    joints = d.profile.joints
    pub, _ = d.offsets()
    pid, ff = pub[(NOMINAL, "pid")], pub[(NOMINAL, "pid_ff")]
    k = int(np.argmin(pid))
    if pid[k] > 2 * float(np.median(ff)) or len(d.runs) < 2:
        return ""
    _, _, per_joint = tracking(d.published)
    worst = joints[
        int(np.argmax([np.mean(np.asarray(per_joint[(NOMINAL, "pid", j)])[:, 0]) for j in joints]))
    ]
    smallest = " (the joint with the smallest Kp)" if d.kp[joints[k]] == min(d.kp.values()) else ""
    moved = []
    for run in d.runs:
        _, _, pj = tracking(run.rows)
        moved.append(1e3 * np.mean(np.asarray(pj[(NOMINAL, "pid", joints[k])])[:, 0]))
    return (
        f"PID alone gives {joints[k]}{smallest} a ratio of {pid[k]:.2f} ms "
        "in the published run, close to the model-based laws': its PID-only error is mostly this "
        "offset too, so its PID-only RMSE at 1× moves between runs "
        f"({span(min(moved), max(moved), f2)} mrad over the {d.n_runs}) while the worst-joint PID "
        f"number, set by {worst}, does not."
    )


def run_spread_section(d: Data) -> list[str]:
    ff, ct, pid = d.law("pid_ff"), d.law("computed_torque"), d.law("pid")
    pub, _ = d.offsets()
    model_1x, model_any, pid_lo, pid_hi = d.joint_agreement()
    kp = np.array(list(d.kp.values()))
    load_sentence, load_verdict = d.load_story()
    reps = sorted({int(r["rep"]) for run in d.runs for r in run.rows})
    commits = sorted({r["git_commit"] for run in d.runs for r in run.rows})
    seeds = sorted({r["seed"] for run in d.runs for r in run.rows})
    run_med = d.run_offsets("pid_ff")
    pub_med = float(np.median(pub[(NOMINAL, "pid_ff")]))
    step_ms = 1e3 * float(d.published[0]["sim_step_s"])
    longest = max(run_med.max(), d.run_offsets("computed_torque").max())
    within_step = f", shorter than one {step_ms:g} ms physics step" if longest < step_ms else ""
    if len(d.runs) < 2:
        varies = ""
    elif run_med.max() / run_med.min() > 1.1:
        varies = ", and different in every run"
    else:
        varies = ", and about the same in every run"
    not_load = " and not CPU load" if load_verdict == "no" else ""
    pub_commits = sorted({r["git_commit"] for r in d.published})
    same = "the same commit" if pub_commits == commits else f"commit `{', '.join(pub_commits)}`"
    lines = [
        "## Run-to-run spread",
        "",
        *para(
            f"{d.n_runs.capitalize()} of the tracking benchmark on the same code (commit",
            f"`{', '.join(commits)}`, seed {', '.join(seeds)}, {len(reps)} repetitions per",
            f"condition, every speed and law) {'were' if len(d.runs) > 1 else 'was'} made one",
            f"after another on {runs_date()}, one CSV per run in `results/{RUNS_DIR.name}/`;",
            "`loadavg.csv` there logs the machine's 1-minute load average during each run. The",
            f"published single run in the tables above is a separate, earlier run of {same}.",
        ),
        *run_table(d),
        "",
    ]
    if len(d.runs) > 1:
        lines += para(
            f"Across the {d.n_runs} the PID-only worst-joint RMSE at 1× stays at",
            f"{span(pid.min(), pid.max(), f2)} mrad, while PID + RNEA FF spans",
            f"{span(ff.min(), ff.max(), f3)} mrad (highest run {ff.max() / ff.min():.1f} times the",
            f"lowest) and computed torque {span(ct.min(), ct.max(), f3)} mrad (highest run",
            f"{ct.max() / ct.min():.1f} times the lowest).",
            load_sentence,
            heavy_note(d),
        )
    lines += [
        "### RMSE ÷ RMS(q̇_des): a timing offset, not a control error"
        if timing_explains(d)
        else "### RMSE ÷ RMS(q̇_des) per joint",
        "",
        *para(
            "Divide each joint's RMSE by the RMS of its own reference velocity q̇_des. RMS(q̇_des)",
            "is not in the CSV; it is computed here from the bench's path (`PATH_OFFSETS`,",
            "`NOMINAL_SEGMENT_S` and `SETTLE_S` in `bench_tracking.py`) and the 1/3-rule",
            "trapezoid of `profiles/trapezoidal.hpp`, over the same window as the RMSE (every",
            "row's profiled duration matches the requested one, so no segment was stretched by a",
            "limit). If the residual came from the control law, this ratio would differ from joint",
            f"to joint: the {number_word(len(d.profile.joints))} joints differ by orders of",
            f"magnitude in inertia, and their Kp by a factor of {kp.max() / kp.min():.0f}.",
            "It does not. For PID + RNEA FF and computed torque at 1×, the largest of the joints'",
            f"ratios is at most {model_1x:.2f} times the smallest, in every run (at most",
            f"{model_any:.2f} at any speed), while for PID alone it is {pid_lo:.0f} to",
            f"{pid_hi:.0f} times. An error proportional to each",
            "joint's own reference speed, with one constant shared by every joint, is what a",
            "fixed offset in time between q_des and q produces: if the recorded q lags its",
            "reference by δ, then e ≈ q̇_des·δ, so the ratio is δ. At 1× it is",
            f"{pub_med:.2f} ms in the published run and {span(run_med.min(), run_med.max(), f2)}",
            f"ms across the {d.n_runs} (PID + RNEA FF, median over the joints){within_step}"
            f"{varies}.",
            f"That offset, not the control law{not_load}, sets the feedforward and",
            "computed-torque rows. Where it comes from (sampling, stamping, or which physics step",
            "the reference is evaluated at) is not measured yet; see",
            f"[what_broke.md]({TIMING_OFFSET_DOC}).",
            wrist_note(d),
            "Ratios and tables recorded on other days",
            "(29 Oct, v0.1) are not regenerated by any script and are in",
            f"[what_broke.md's appendix]({APPENDIX_DOC}).",
        ),
        *offset_table(d),
        "",
    ]
    return lines


def handguide_section(d: Data) -> list[str]:
    hg = d.hg
    reps = len({r["rep"] for r in d.handguide_rows})
    g = abs(float(controller_param("gravity_z")))
    return [
        "## Hand guiding",
        "",
        f"`results/handguide.csv`: {stamp(d.handguide_rows)}; {reps} repetitions per condition.",
        "",
        *handguide_table(handguide(d.handguide_rows)),
        "",
        *para(
            "Method: per repetition the tracking controller holds the arm at a seeded random",
            "offset (≤ 0.05 rad per joint) from home until its integrator has converged (3 s);",
            "its torque is then what the simulated arm needs to stay put. The hand-guiding",
            "controller then takes over with the model payload and nobody touches the arm.",
            "*Effort to hold* is the tool force an operator must apply to keep the arm still,",
            "F = J(q)⁻ᵀ (τ_hold − τ_handguide) at the release pose; its z component should",
            "equal (m_actual − m_model)·g. *Drift* is the TCP linear speed |J(q) q̇| over",
            "[0.9, 1.0] s after release and the TCP displacement at 1 s.",
        ),
        *para(
            "The measured F_z differs from Δm·g by at most",
            f"{hg.residual_n:.{decimals_of(hg.printed_n)}f} N over all {hg.rows} rows, the last",
            "digit the CSV records. That agreement holds by construction, so it is a placement",
            "check, not a measurement: Gazebo (DART) and Pinocchio are given the same URDF and",
            f"the same gravity ({g:g} m/s², the `gravity_z` parameter, set explicitly in the",
            "worlds), so the benchmark confirms that the payload model enters the dynamics at the",
            "right place and that a hand-over between controllers is seamless. On a real arm this",
            "is exactly where friction and model error would appear. Earlier versions of this",
            "benchmark showed an offset in the hold force; the two causes (SDF's default gravity",
            "of 9.8 m/s², and a controller that zeroed its torque on deactivation) are described",
            "in [what_broke.md](what_broke.md), and the hand-recorded offsets are in",
            f"[its appendix]({APPENDIX_DOC}).",
        ),
        *picture("handguide", handguide_alt(d), 640, "assets/"),
        "",
        handguide_caption(d),
        "",
    ]


def timing_section(d: Data) -> list[str]:
    if not d.timing_rows:
        return []
    t = d.t
    return [
        "## Dynamics timing",
        "",
        f"`results/dynamics_timing.csv`: {timing_stamp(d.timing_rows)}.",
        "",
        *timing_summary_table(t),
        "",
        *timing_table(t),
        "",
        *para(
            "Method: `bench_dynamics_timing` (C++, no ROS graph, no simulator, one thread) loads",
            "the simulation's URDF into each backend through the same `DynamicsModel` interface",
            "the controllers use, draws the states up front (q uniform inside the URDF position",
            "limits, q̇ inside the velocity limits, |q̈| ≤ 10 rad/s², seeded), makes 1 000 warm-up",
            "calls and then times every call on its own with `std::chrono::steady_clock`. Times",
            "include the wrapper's joint reordering and the timer itself (the *two clock reads",
            f"only* row); nothing is subtracted. *Share of {cycle_ms(t)} ms* is the time divided",
            "by one control cycle. `scripts/bench_dynamics_timing.sh` refuses to run next to a",
            "simulator, build, pip install or training run, and its log records the load average",
            "and the busiest processes at start; on the shared dev laptop it runs under the",
            "machine's heavy, build and Gazebo locks. Medians move from run to run; reruns",
            f"recorded by hand on 29 Oct are in [what_broke.md's appendix]({APPENDIX_DOC}). These",
            "are laptop numbers: CI runs the executable as a 1 000-call smoke test only, and",
            "runner timings are not published.",
        ),
    ]


def results_markdown(d: Data) -> str:
    _, _, per_joint = tracking(d.published)
    reps = len({r["rep"] for r in d.published})
    p = d.profile
    rate = d.published[0]["update_rate_hz"]
    lines = [
        "# Results",
        "",
        *para(
            "Every number on this page is computed by `scripts/make_results.py` from the CSVs in",
            "`results/`, which the benchmark scripts write; nothing is typed in by hand.",
            "Regenerate everything with `scripts/regenerate_all.sh`. Simulation only (Gazebo",
            "Jetty, DART); see [the README](../README.md#where-it-fails) for what does not",
            "transfer to a real UR5e.",
        ),
        "## Summary",
        "",
        *summary_table(d),
        "",
        *para(
            "Tracking cells are the lowest and highest per-run mean over the",
            f"{d.n_runs} in `results/{RUNS_DIR.name}/` ([run-to-run spread](#run-to-run-spread));",
            "the hand-guiding and timing cells come from the published single runs below.",
        ),
        "## Tracking, full tables",
        "",
        *para(
            f"`results/tracking.csv` (the published single run): {stamp(d.published)}; {reps}",
            f"repetitions per condition. Ranges: `results/{RUNS_DIR.name}/run*_tracking.csv`,",
            f"{d.n_runs} ([run-to-run spread](#run-to-run-spread)).",
        ),
        *tracking_full_table(d),
        "",
        *para(
            "Method: the arm is homed by the tracking controller itself (feedforward on) to a",
            "seeded random offset (≤ 0.05 rad per joint) from the home pose; for the PID-only",
            "variant feedforward is then switched off and the integrator given 3 s to carry",
            "gravity; the computed-torque variant switches the same controller to",
            "`control_law: computed_torque` (ω, ζ per joint from `controllers.yaml`), with no",
            f"integral term. A {len(p.offsets)}-waypoint path through all {len(p.joints)} joints",
            f"({p.segment_s:g} s per segment at 1×, each segment a rest-to-rest trapezoid) is sent",
            f"on `~/joint_trajectory`. The error is the controller's own `q_des − q` at {rate} Hz,",
            f"from trajectory start to {p.settle_s:g} s after the last reference motion. *Worst",
            "joint* is the joint with the largest RMSE in that repetition; a run's value is the",
            "mean over its repetitions. Tables published before this one (v0.1, and 29 Oct) came",
            "from single runs on other days; they are in",
            f"[what_broke.md's appendix]({APPENDIX_DOC}) and should be compared with this page",
            "only through the run-to-run spread below.",
        ),
        *picture("tracking_rmse", tracking_alt(d), 720, "assets/"),
        "",
        tracking_caption(d, "#run-to-run-spread"),
        "",
        *picture(
            "tracking_rmse_log",
            "The worst joint's tracking error of every run, per speed and control law, on a log "
            "scale, with the published run marked",
            640,
            "assets/",
        ),
        "",
        "The same data on a log scale: one dot per run, a hollow diamond for the published run.",
        "",
        "Per joint, published run, RMSE / max |e| in mrad, mean ± std over repetitions:",
        "",
        *tracking_joint_table(per_joint),
        "",
        *run_spread_section(d),
        *handguide_section(d),
        *timing_section(d),
    ]
    return "\n".join(lines).rstrip("\n") + "\n"


# ---------------------------------------------------------------------------- figures
def nice_ceil(x: float) -> float:
    exp = 10.0 ** math.floor(math.log10(x))
    return next(m * exp for m in (1, 1.2, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10) if m * exp >= x - 1e-12)


def chart_scales(d: Data) -> tuple[float, float]:
    speeds = speeds_of(d)
    pid = max(d.law("pid", s).max() for s in speeds)
    model = max(d.law(v, s).max() for s in speeds for v in ("pid_ff", "computed_torque"))
    return nice_ceil(1.15 * pid), nice_ceil(1.3 * model)


def chart_title(d: Data) -> str:
    ratios = [
        r.worst[(s, "pid")] / r.worst[(s, v)]
        for r in d.runs
        for s in speeds_of(d)
        for v in ("pid_ff", "computed_torque")
        if (s, v) in r.worst
    ]
    return (
        "The arm's model cuts the tracking error by "
        f"{orders_of_magnitude(min(ratios), max(ratios))}"
    )


def style_axes(ax, t):
    ax.set_facecolor(t["surface"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(t["axis"])
    ax.tick_params(colors=t["muted"], labelcolor=t["ink2"])
    ax.grid(axis="y", color=t["grid"], linewidth=0.8)
    ax.set_axisbelow(True)
    ax.xaxis.label.set_color(t["ink2"])
    ax.yaxis.label.set_color(t["ink2"])


def pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


SLOT = {v: f"s{k + 1}" for k, (v, _) in enumerate(VARIANTS)}  # fixed colour slot per law


def plot_tracking(d: Data, theme: str, path: Path):
    plt = pyplot()
    t = THEMES[theme]
    speeds = speeds_of(d)
    x = np.arange(len(speeds))
    left_top, right_top = chart_scales(d)
    fig, (a1, a2) = plt.subplots(
        1, 2, figsize=(7.2, 3.6), dpi=150, gridspec_kw={"width_ratios": [1, 1.7]}
    )
    fig.patch.set_facecolor(t["surface"])
    panels = ((a1, ("pid",), left_top, 0.36), (a2, ("pid_ff", "computed_torque"), right_top, 0.3))
    handles = []
    for ax, laws, top, w in panels:
        for k, v in enumerate(laws):
            vals = [d.law(v, s) for s in speeds]
            med = np.array([np.median(a) for a in vals])
            lo = med - np.array([a.min() for a in vals])
            hi = np.array([a.max() for a in vals]) - med
            xs = x + (k - (len(laws) - 1) / 2) * (w + 0.02)
            handles.append(ax.bar(xs, med, width=w, color=t[SLOT[v]], zorder=2))
            ax.errorbar(
                xs,
                med,
                yerr=[lo, hi],
                fmt="none",
                ecolor=t["ink2"],
                elinewidth=1,
                capsize=3,
                zorder=3,
            )
            for xi, m, h in zip(xs, med, hi, strict=True):
                ax.annotate(
                    f2(m) if top < 2 else f"{m:.1f}",
                    (xi, m + h),
                    xytext=(0, 3),
                    textcoords="offset points",
                    ha="center",
                    va="bottom",
                    fontsize=8,
                    color=t["ink"],
                )
        ax.set_ylim(0, top)
        ax.set_xticks(x, [f"{s:g}×" for s in speeds])
        ax.set_xlabel("speed (× the speed the path was designed for)", fontsize=8.5)
        ax.set_ylabel("worst-joint RMSE (mrad)")
        style_axes(ax, t)
    a1.set_title("PID alone", fontsize=9, loc="left", color=t["ink2"])
    a2.set_title(
        f"With the model (scale {about(left_top / right_top)}× finer)",
        fontsize=9,
        loc="left",
        color=t["ink2"],
    )
    leg = a2.legend(
        handles[1:],
        ["PID + feedforward", "computed torque"],
        frameon=False,
        fontsize=8,
        loc="upper left",
        ncol=2,
    )
    for text in leg.get_texts():
        text.set_color(t["ink2"])
    fig.suptitle(chart_title(d), x=0.02, ha="left", fontsize=10.5, color=t["ink"])
    fig.tight_layout()
    fig.savefig(path, facecolor=t["surface"])
    plt.close(fig)


def plot_tracking_log(d: Data, theme: str, path: Path):
    plt = pyplot()
    t = THEMES[theme]
    speeds = speeds_of(d)
    x = np.arange(len(speeds))
    pub = worst_means(d.published)
    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=150)
    fig.patch.set_facecolor(t["surface"])
    for k, (v, name) in enumerate(VARIANTS):
        xs = x + (k - 1) * 0.22
        for xi, s in zip(xs, speeds, strict=True):
            runs = d.law(v, s)
            ax.plot(
                np.full(len(runs), xi),
                runs,
                "o",
                color=t[SLOT[v]],
                markersize=6,
                markeredgecolor=t["surface"],
                markeredgewidth=1,
                label=name if xi == xs[0] else None,
                zorder=3,
            )
            if (s, v) in pub:
                ax.plot(
                    xi + 0.08,
                    1e3 * pub[(s, v)],
                    "D",
                    color=t[SLOT[v]],
                    markerfacecolor="none",
                    markersize=6,
                    markeredgewidth=1.2,
                    zorder=3,
                )
    ax.plot([], [], "D", color=t["ink2"], markerfacecolor="none", label="published run")
    ax.set_yscale("log")
    ax.set_xticks(x, [f"{s:g}× designed speed" for s in speeds])
    ax.set_xlim(-0.6, len(speeds) - 0.4)
    ax.set_ylabel("worst-joint RMSE (mrad, log)")
    ax.set_title(
        f"Each run's worst-joint RMSE ({d.n_runs}), log scale",
        fontsize=10,
        loc="left",
        color=t["ink"],
        pad=22,
    )
    style_axes(ax, t)
    leg = ax.legend(
        frameon=False,
        fontsize=8,
        loc="lower right",
        bbox_to_anchor=(1.0, 1.0),
        ncol=4,
        borderaxespad=0.2,
    )
    for text in leg.get_texts():
        text.set_color(t["ink2"])
    fig.tight_layout()
    fig.savefig(path, facecolor=t["surface"])
    plt.close(fig)


def plot_handguide(d: Data, theme: str, path: Path):
    plt = pyplot()
    t = THEMES[theme]
    hg = d.hg
    reach = 1.3 * max(abs(hg.errors_kg))
    fig, ax = plt.subplots(figsize=(6.4, 3.6), dpi=150)
    fig.patch.set_facecolor(t["surface"])
    ax.axhline(0, color=t["axis"], linewidth=0.8, zorder=1)
    ax.axvline(0, color=t["axis"], linewidth=0.8, zorder=1)
    line = np.array([-reach, reach])
    ax.plot(line, hg.g * line, color=t["muted"], linewidth=1.5, linestyle="--", zorder=2)
    mid = 0.4 * max(abs(hg.errors_kg))
    ax.annotate(
        f"expected: Δm·g, {hg.g:.2f} N per kg",
        (mid, hg.g * mid),
        xytext=(10, -6),
        textcoords="offset points",
        ha="left",
        va="top",
        fontsize=8,
        color=t["ink2"],
    )
    ax.plot(
        hg.errors_kg,
        hg.force,
        "o",
        color=t["s1"],
        markersize=9,
        markeredgecolor=t["surface"],
        markeredgewidth=2,
        zorder=3,
    )
    for e, f in zip(hg.errors_kg, hg.force, strict=True):
        ax.annotate(
            signed_n(f),
            (e, f),
            xytext=(-10, 6),
            textcoords="offset points",
            ha="right",
            va="bottom",
            fontsize=8.5,
            color=t["ink"],
        )
    ax.set_xlim(-reach, reach)
    ax.set_ylim(-hg.g * reach, hg.g * reach)
    ax.set_xticks(hg.errors_kg, [signed(e, "+g").replace("-", "−") for e in hg.errors_kg])
    ax.set_xlabel("payload-model error: actual − model mass (kg)")
    ax.set_ylabel("force to hold the arm still (N, + = up)")
    ax.set_title(
        f"Every kilogram of payload-model error costs {hg.g:.2f} N of holding force",
        fontsize=10.5,
        loc="left",
        color=t["ink"],
    )
    style_axes(ax, t)
    fig.tight_layout()
    fig.savefig(path, facecolor=t["surface"])
    plt.close(fig)


# ---------------------------------------------------------------------------- README
def markers(name: str) -> tuple[str, str]:
    return f"<!-- {name}:begin -->", f"<!-- {name}:end -->"


def check_markers(text: str, path: Path) -> None:
    """Each marker pair exactly once, begin before end, pairs in BLOCKS order."""
    problems, last = [], -1
    for name in BLOCKS:
        begin, end = markers(name)
        nb, ne = text.count(begin), text.count(end)
        if nb != 1 or ne != 1:
            problems.append(f"{begin} … {end} found {nb} / {ne} times (need exactly 1 / 1)")
            continue
        b, e = text.index(begin), text.index(end)
        if not last < b < e:
            problems.append(f"{begin} … {end} is out of order")
        last = e
    if problems:
        raise SystemExit(
            f"make_results: {path} needs the marker pairs {', '.join(BLOCKS)} in that order:\n  "
            + "\n  ".join(problems)
        )


def replace_block(text: str, name: str, body: str) -> str:
    begin, end = markers(name)
    head, rest = text.split(begin, 1)
    _, tail = rest.split(end, 1)
    return f"{head}{begin}\n{body}\n{end}{tail}"


def render_readme(path: Path, blocks: dict[str, str]) -> tuple[str, str]:
    """(current text, text with every generated block replaced)."""
    if not path.exists():
        raise SystemExit(f"make_results: {path} does not exist")
    text = path.read_text()
    check_markers(text, path)
    new = text
    for name in BLOCKS:
        new = replace_block(new, name, blocks[name])
    return text, new


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="only verify outputs are current")
    parser.add_argument("--no-plots", action="store_true")
    parser.add_argument(
        "--readme", type=Path, default=README, help="file holding the generated blocks"
    )
    args = parser.parse_args(argv)

    published = read_csv(RESULTS / "tracking.csv")
    profile = Profile.from_bench()
    runs = read_runs(RUNS_DIR)
    for rows in (published, *(r.rows for r in runs)):
        profile.check(rows)
    timing_csv = RESULTS / "dynamics_timing.csv"
    d = Data(
        published=published,
        runs=runs,
        handguide_rows=read_csv(RESULTS / "handguide.csv"),
        timing_rows=read_csv(timing_csv) if timing_csv.exists() else [],
        profile=profile,
        kp=kp_by_joint(),
    )
    full = results_markdown(d)
    blocks = {"headline": headline(d), "breath": breath(d), "results": results_block(d)}
    readme_now, readme_new = render_readme(args.readme, blocks)
    out = DOCS / "results.md"
    if args.check:
        stale = [
            p
            for p, ok in ((out, out.read_text() == full), (args.readme, readme_now == readme_new))
            if not ok
        ]
        if stale:
            print(
                "results out of date ("
                + ", ".join(str(p) for p in stale)
                + "): run scripts/make_results.py"
            )
            return 1
        print("results up to date")
        return 0
    out.write_text(full)
    args.readme.write_text(readme_new)
    if not args.no_plots:
        ASSETS.mkdir(parents=True, exist_ok=True)
        for theme in THEMES:
            plot_tracking(d, theme, ASSETS / f"tracking_rmse_{theme}.png")
            plot_tracking_log(d, theme, ASSETS / f"tracking_rmse_log_{theme}.png")
            plot_handguide(d, theme, ASSETS / f"handguide_{theme}.png")
    print(f"wrote {out.relative_to(ROOT)}, the README blocks in {args.readme} and the figures")
    return 0


if __name__ == "__main__":
    sys.exit(main())
