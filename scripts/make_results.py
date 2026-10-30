#!/usr/bin/env python3
"""Turn results/*.csv into docs/results.md, the README results block and the result figures.

Every number in docs/results.md and in the README table is computed here from the CSVs written by
the benchmark scripts; nothing is typed in by hand. tests/test_results_sync.py checks that the
README block and docs/results.md agree.

    python3 scripts/make_results.py            # rewrite docs/results.md, README block, PNGs
    python3 scripts/make_results.py --check    # exit 1 if any of them is out of date
"""

from __future__ import annotations

import argparse
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
DOCS = ROOT / "docs"
ASSETS = DOCS / "assets"
README = ROOT / "README.md"
BEGIN, END = "<!-- results:begin -->", "<!-- results:end -->"
HEAD_BEGIN, HEAD_END = "<!-- headline:begin -->", "<!-- headline:end -->"

# Reference palette (dataviz skill, validated light/dark): slot 1 blue, slot 2 orange, slot 3
# aqua (the three validate all-pairs in both modes; light aqua is below 3:1 on the surface, so
# every dot carries a direct value label and the tables are the table view).
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


def read_csv(path: Path) -> list[dict]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def pm(values, scale=1.0, digits=2) -> str:
    v = np.asarray(values, dtype=float) * scale
    return f"{v.mean():.{digits}f} ± {v.std(ddof=1) if v.size > 1 else 0.0:.{digits}f}"


def stamp(rows: list[dict]) -> str:
    commits = sorted({r["git_commit"] for r in rows})
    seeds = sorted({r["seed"] for r in rows})
    backends = sorted({r["dynamics_backend"] for r in rows if r.get("dynamics_backend")})
    return (
        f"commit `{', '.join(commits)}`, seed {', '.join(seeds)}, physics step "
        f"{rows[0]['sim_step_s']} s, controller update rate {rows[0]['update_rate_hz']} Hz, "
        f"dynamics backend {', '.join(backends) or 'pinocchio'}"
    )


# ---------------------------------------------------------------------------- tracking
# Tracking variants in display order: (CSV value, label). v0.1 CSVs have a `feedforward` column
# (0 = pid, 1 = pid_ff) instead of `variant`.
VARIANTS = (("pid", "PID"), ("pid_ff", "PID + RNEA FF"), ("computed_torque", "Computed torque"))
LABEL = dict(VARIANTS)


def variant_of(row: dict) -> str:
    if row.get("variant"):
        return row["variant"]
    return "pid_ff" if int(row["feedforward"]) else "pid"


def present_variants(worst_rmse) -> list[str]:
    have = {v for _, v in worst_rmse}
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


def tracking_summary_table(worst_rmse, worst_max) -> list[str]:
    speeds = sorted({s for s, _ in worst_rmse})
    variants = present_variants(worst_rmse)
    head = " | ".join(f"{LABEL[v]}: worst-joint RMSE (mrad)" for v in variants)
    head_max = " | ".join(f"{LABEL[v]}: worst max \\|e\\| (mrad)" for v in variants)
    lines = [
        f"| Speed (× nominal) | {head} | {head_max} | Regenerate with |",
        "|---|" + "---|" * (2 * len(variants)) + "---|",
    ]
    for s in speeds:
        rmse = " | ".join(pm(worst_rmse[(s, v)], 1e3) for v in variants)
        mx = " | ".join(pm(worst_max[(s, v)], 1e3) for v in variants)
        lines.append(f"| {s:g} | {rmse} | {mx} | `scripts/bench_tracking.sh` |")
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
                cells.append(f"{pm(vals[:, 0], 1e3)} / {pm(vals[:, 1], 1e3)}")
            lines.append(f"| {s:g} | {LABEL[v]} | " + " | ".join(cells) + " |")
    return lines


# ---------------------------------------------------------------------------- handguide
def handguide(rows: list[dict]):
    by = defaultdict(list)
    for r in rows:
        by[(float(r["model_payload_kg"]), float(r["actual_payload_kg"]))].append(r)
    return dict(sorted(by.items(), key=lambda kv: kv[0][1]))


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


# ---------------------------------------------------------------------------- figures
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


def plot_tracking(worst_rmse, theme: str, path: Path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = THEMES[theme]
    speeds = sorted({s for s, _ in worst_rmse})
    x = np.arange(len(speeds))
    variants = present_variants(worst_rmse)
    w = 0.6 / len(variants)
    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=150)
    fig.patch.set_facecolor(t["surface"])
    colors = (t["s1"], t["s2"], t["s3"])  # fixed slot per variant, in VARIANTS order
    slot = {v: k for k, (v, _) in enumerate(VARIANTS)}
    for k, variant in enumerate(variants):
        name, color = LABEL[variant], colors[slot[variant]]
        vals = [np.asarray(worst_rmse[(s, variant)]) * 1e3 for s in speeds]
        means = [v.mean() for v in vals]
        stds = [v.std(ddof=1) if v.size > 1 else 0.0 for v in vals]
        xs = x + (k - (len(variants) - 1) / 2) * w
        # Dots, not bars: on a log axis a bar's length has no meaning.
        ax.errorbar(
            xs,
            means,
            yerr=stds,
            fmt="o",
            color=color,
            markersize=8,
            label=name,
            ecolor=t["ink2"],
            elinewidth=1,
            capsize=3,
            markeredgecolor=t["surface"],
            markeredgewidth=1.5,
        )
        for xi, m, sd in zip(xs, means, stds, strict=True):
            ax.annotate(
                f"{m:.2g}",
                (xi, m + sd),
                xytext=(0, 3),
                textcoords="offset points",
                ha="center",
                va="bottom",
                fontsize=8,
                color=t["ink2"],
            )
    ax.set_yscale("log")
    ax.set_xticks(x, [f"{s:g}× nominal" for s in speeds])
    ax.set_xlim(-0.6, len(speeds) - 0.4)
    ax.set_ylabel("worst-joint RMSE (mrad, log)")
    ax.set_title(
        "Tracking error, mean ± std over repetitions",
        fontsize=10,
        loc="left",
        color=t["ink"],
        pad=22,
    )
    ax.margins(y=0.25)
    style_axes(ax, t)
    leg = ax.legend(
        frameon=False,
        fontsize=8,
        loc="lower right",
        bbox_to_anchor=(1.0, 1.0),
        ncol=len(variants),
        borderaxespad=0.2,
    )
    for text in leg.get_texts():
        text.set_color(t["ink2"])
    fig.tight_layout()
    fig.savefig(path, facecolor=t["surface"])
    plt.close(fig)


def plot_handguide(by, theme: str, path: Path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = THEMES[theme]
    err = np.array([actual - model for model, actual in by])
    force = [np.array([float(r["hold_force_z_n"]) for r in rs]) for rs in by.values()]
    drift = [np.array([float(r["drift_speed_1s_mps"]) for r in rs]) * 1e3 for rs in by.values()]
    expected = np.array([float(rs[0]["expected_hold_force_z_n"]) for rs in by.values()])

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.2, 3.2), dpi=150)
    fig.patch.set_facecolor(t["surface"])
    a1.plot(err, expected, color=t["muted"], linewidth=1.5, linestyle="--", label="expected Δm·g")
    a1.errorbar(
        err,
        [f.mean() for f in force],
        yerr=[f.std(ddof=1) for f in force],
        fmt="o",
        color=t["s1"],
        markersize=6,
        elinewidth=1,
        capsize=3,
        label="measured",
    )
    a1.set_xlabel("payload error, actual − model (kg)")
    a1.set_ylabel("force to hold the arm still, F_z (N)")
    a1.set_title(
        "Effort the operator spends on model error", fontsize=9, loc="left", color=t["ink"]
    )
    leg = a1.legend(frameon=False, fontsize=8)
    for text in leg.get_texts():
        text.set_color(t["ink2"])
    a2.errorbar(
        err,
        [d.mean() for d in drift],
        yerr=[d.std(ddof=1) for d in drift],
        fmt="o",
        color=t["s1"],
        markersize=6,
        elinewidth=1,
        capsize=3,
    )
    a2.set_xlabel("payload error, actual − model (kg)")
    a2.set_ylabel("TCP drift speed at 1 s (mm/s)")
    a2.set_title("Drift after letting go", fontsize=9, loc="left", color=t["ink"])
    for ax in (a1, a2):
        style_axes(ax, t)
        ax.set_xticks(err)
    fig.tight_layout()
    fig.savefig(path, facecolor=t["surface"])
    plt.close(fig)


# ---------------------------------------------------------------------------- dynamics timing
CYCLE_US = 2000.0  # 500 Hz controller cycle
BACKENDS = (("pinocchio", "Pinocchio"), ("kdl", "KDL"))
FUNCTIONS = (("gravity", "g(q)"), ("rnea", "RNEA"))


def timing(rows: list[dict]) -> dict:
    """timing[(backend, function)] = CSV row (one row per pair, one run)."""
    return {(r["backend"], r["function"]): r for r in rows}


def us(row: dict, key: str) -> str:
    # The executable times each call in whole nanoseconds, so three decimals in µs are exact.
    return f"{float(row[key]):.3f}"


def timing_summary_table(t: dict) -> list[str]:
    """One row for the README: median / p95 per call and the worst p95 share of the cycle."""
    cells = [
        f"{us(t[(b, f)], 'median_us')} / {us(t[(b, f)], 'p95_us')}"
        for b, _ in BACKENDS
        for f, _ in FUNCTIONS
    ]
    worst = max(float(t[(b, f)]["p95_cycle_fraction"]) for b, _ in BACKENDS for f, _ in FUNCTIONS)
    head = " | ".join(f"{bn} {fn}" for _, bn in BACKENDS for _, fn in FUNCTIONS)
    return [
        f"| Per call, median / p95 (µs) | {head} | Worst p95, share of the 2 ms cycle "
        "| Regenerate with |",
        "|---|---|---|---|---|---|---|",
        f"| UR5e, 6 DoF, one thread | {' | '.join(cells)} | {100 * worst:.4f} % "
        "| `scripts/bench_dynamics_timing.sh` |",
    ]


def timing_table(t: dict) -> list[str]:
    lines = [
        "| Backend | Call | Median (µs) | p95 (µs) | Mean (µs) | Max (µs) "
        "| Median, share of 2 ms | p95, share of 2 ms |",
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


def timing_section(rows: list[dict]) -> list[str]:
    if not rows:
        return []
    return [
        "## Dynamics timing — details",
        "",
        f"`results/dynamics_timing.csv`: {timing_stamp(rows)}.",
        "",
        "Method: `bench_dynamics_timing` (C++, no ROS graph, no simulator, one thread) loads",
        "the simulation's URDF into each backend through the same `DynamicsModel` interface the",
        "controllers use, draws the states up front (q uniform inside the URDF position limits,",
        "q̇ inside the velocity limits, |q̈| ≤ 10 rad/s², seeded), makes 1 000 warm-up calls",
        "and then times every call on its own with `std::chrono::steady_clock`. Times include",
        "the wrapper's joint reordering and the timer itself (the *two clock reads only* row);",
        "nothing is subtracted. *Share of 2 ms* is the time divided by one 500 Hz control cycle.",
        "`scripts/bench_dynamics_timing.sh` refuses to run next to a simulator, build, pip",
        "install or training run, and its log records the load average and the busiest",
        "processes at start; on the shared dev laptop it runs under the machine's heavy, build",
        "and Gazebo locks. In three back-to-back runs on 29 Oct the medians moved by up to ~12 %",
        "([what_broke.md](what_broke.md)). These are laptop numbers: CI runs the executable as a",
        "1 000-call smoke test only, and runner timings are not published.",
        "",
        *timing_table(timing(rows)),
        "",
    ]


# ---------------------------------------------------------------------------- outputs
def results_markdown(track_rows, hg_rows, timing_rows=()) -> tuple[str, str]:
    """(full docs/results.md, summary block shared with the README)."""
    worst_rmse, worst_max, per_joint = tracking(track_rows)
    by = handguide(hg_rows)
    timing_summary = (
        [
            "",
            "**Cost of one dynamics call** (gravity g(q) and RNEA on each backend, single "
            "thread; median / p95 over 100 000 timed calls):",
            "",
            *timing_summary_table(timing(timing_rows)),
        ]
        if timing_rows
        else []
    )
    summary = "\n".join(
        [
            "**Joint-space tracking** (PidFfTrajectoryController, 3-segment 6-joint path; worst "
            "joint per repetition, then mean ± std over repetitions):",
            "",
            *tracking_summary_table(worst_rmse, worst_max),
            "",
            "**Hand guiding with a wrong payload model** (GravityCompHandGuidingController, model "
            "payload fixed, actual Gazebo payload varied; mean ± std over repetitions):",
            "",
            *handguide_table(by),
            *timing_summary,
        ]
    )
    reps_t = len({r["rep"] for r in track_rows})
    reps_h = len({r["rep"] for r in hg_rows})
    full = "\n".join(
        [
            "# Results",
            "",
            "Every number on this page is computed by `scripts/make_results.py` from the CSVs in",
            "`results/`, which the benchmark scripts write. Regenerate everything with",
            "`scripts/regenerate_all.sh`. Simulation only (Gazebo Jetty, DART); see",
            "[the README](../README.md#where-it-fails) for what does not transfer to a real UR5e.",
            "",
            "## Summary",
            "",
            summary,
            "",
            "## Tracking — details",
            "",
            f"`results/tracking.csv`: {stamp(track_rows)}; {reps_t} repetitions per condition.",
            "",
            "Method: the arm is homed by the tracking controller itself (feedforward on) to a",
            "seeded random offset (≤ 0.05 rad per joint) from the home pose; for the PID-only",
            "variant feedforward is then switched off and the integrator given 3 s to carry",
            "gravity; the computed-torque variant switches the same controller to",
            "`control_law: computed_torque` (ω, ζ per joint from `controllers.yaml`), with no",
            "integral term. A 3-waypoint path (2.5 s per segment at 1×) is sent on",
            "`~/joint_trajectory`. The error is the controller's own `q_des − q` at 500 Hz,",
            "from trajectory start to 0.5 s after the last",
            'reference motion. "Worst joint" = the joint with the largest RMSE in that run.',
            "These rows replace v0.1's two-column table. Its PID + RNEA FF rows were several",
            "times lower; re-running the v0.1 code itself on 29 Oct reproduced this table's",
            "level, not its own, with no package changed in between (the A/B is in",
            "[what_broke.md](what_broke.md)), so the two tables should not be compared row by row.",
            "Whole-benchmark reruns reproduce the PID-only rows closely; the feedforward rows sit",
            "near the noise floor and vary more between reruns than the within-run std shows",
            "(the run log is in [what_broke.md](what_broke.md)).",
            "",
            "![tracking](assets/tracking_rmse_light.png)",
            "",
            "Per joint, RMSE / max |e| in mrad, mean ± std over repetitions:",
            "",
            *tracking_joint_table(per_joint),
            "",
            "## Hand guiding — details",
            "",
            f"`results/handguide.csv`: {stamp(hg_rows)}; {reps_h} repetitions per condition.",
            "",
            "Method: per repetition the tracking controller holds the arm at a seeded random",
            "offset (≤ 0.05 rad per joint) from home until its integrator has converged (3 s);",
            "its torque is then what the simulated arm needs to stay put. The hand-guiding",
            "controller then takes over with the model payload and nobody touches the arm.",
            "*Effort to hold* is the tool force an operator must apply to keep the arm still,",
            "F = J(q)⁻ᵀ (τ_hold − τ_handguide) at the release pose; its z component should",
            "equal (m_actual − m_model)·g. *Drift* is the TCP linear speed |J(q) q̇| over",
            "[0.9, 1.0] s after release and the TCP displacement at 1 s (by 2 s a 1 kg error has",
            "carried the arm tens of centimetres, towards the floor when it sinks).",
            "",
            "The hold force matches Δm·g to the printed precision because Gazebo (DART) and",
            "Pinocchio are given the same URDF and the same gravity (9.81 m/s², set explicitly in",
            "the world): the benchmark confirms that the payload model enters the dynamics at the",
            "right place and that a hand-over between controllers is seamless. On a real arm this",
            "is exactly where friction and model error would appear. Two earlier versions of this",
            "benchmark showed offsets of −0.5 N and −2.4 N; both were artefacts (SDF's default",
            "gravity of 9.8, and a controller that zeroed its torque on deactivation) and are",
            "described in [what_broke.md](what_broke.md).",
            "",
            "![handguide](assets/handguide_light.png)",
            "",
            *timing_section(list(timing_rows)),
        ]
    )
    return full, summary


def headline(track_rows, hg_rows) -> str:
    """The README's third sentence: one tracking number and one hand-guiding number."""
    worst_rmse, _, _ = tracking(track_rows)
    by = handguide(hg_rows)
    speed = 1.0 if (1.0, "pid_ff") in worst_rmse else sorted(worst_rmse)[0][0]
    over = [(m, a) for (m, a) in by if a - m > 0]
    model, actual = max(over, key=lambda k: k[1] - k[0]) if over else next(iter(by))
    rs = by[(model, actual)]
    ct = (
        f", computed torque {pm(worst_rmse[(speed, 'computed_torque')], 1e3)} mrad"
        if (speed, "computed_torque") in worst_rmse
        else ""
    )
    return (
        f"At {speed:g}× nominal speed the RNEA feedforward brings the worst-joint tracking RMSE "
        f"to {pm(worst_rmse[(speed, 'pid_ff')], 1e3)} mrad (PID alone: "
        f"{pm(worst_rmse[(speed, 'pid')], 1e3)} mrad{ct}), and in hand guiding a "
        f"{actual - model:g} kg "
        f"payload-model error costs the operator {pm([r['hold_force_z_n'] for r in rs])} N "
        f"of vertical force to hold the arm still "
        f"(Δm·g = {float(rs[0]['expected_hold_force_z_n']):.2f} N) and, let go, carries the "
        f"TCP {pm([r['drift_disp_1s_m'] for r in rs], 1e3, 0)} mm in one second."
    )


def replace_block(text: str, begin: str, end: str, body: str) -> str:
    if begin not in text or end not in text:
        return text
    head, rest = text.split(begin, 1)
    _, tail = rest.split(end, 1)
    return f"{head}{begin}\n{body}\n{end}{tail}"


def sync_readme(summary: str, head_sentence: str, check: bool) -> bool:
    if not README.exists():
        return True
    text = README.read_text()
    new = replace_block(text, BEGIN, END, summary)
    new = replace_block(new, HEAD_BEGIN, HEAD_END, head_sentence)
    if check:
        return new == text
    README.write_text(new)
    return True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="only verify outputs are current")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args(argv)

    track_rows = read_csv(RESULTS / "tracking.csv")
    hg_rows = read_csv(RESULTS / "handguide.csv")
    timing_csv = RESULTS / "dynamics_timing.csv"
    timing_rows = read_csv(timing_csv) if timing_csv.exists() else []
    full, summary = results_markdown(track_rows, hg_rows, timing_rows)
    full = full.rstrip("\n")  # exactly one trailing newline, as end-of-file-fixer wants
    head_sentence = headline(track_rows, hg_rows)
    out = DOCS / "results.md"
    if args.check:
        ok = out.read_text() == full + "\n" and sync_readme(summary, head_sentence, check=True)
        print("results up to date" if ok else "results out of date: run scripts/make_results.py")
        return 0 if ok else 1
    out.write_text(full + "\n")
    sync_readme(summary, head_sentence, check=False)
    if not args.no_plots:
        ASSETS.mkdir(parents=True, exist_ok=True)
        worst_rmse, _, _ = tracking(track_rows)
        for theme in THEMES:
            plot_tracking(worst_rmse, theme, ASSETS / f"tracking_rmse_{theme}.png")
            plot_handguide(handguide(hg_rows), theme, ASSETS / f"handguide_{theme}.png")
    print(f"wrote {out.relative_to(ROOT)}, README results block and figures")
    return 0


if __name__ == "__main__":
    sys.exit(main())
