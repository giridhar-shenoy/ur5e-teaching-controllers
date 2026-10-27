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

# Reference palette (dataviz skill, validated light/dark): slot 1 blue, slot 2 orange.
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
    return (
        f"commit `{', '.join(commits)}`, seed {', '.join(seeds)}, physics step "
        f"{rows[0]['sim_step_s']} s, controller update rate {rows[0]['update_rate_hz']} Hz"
    )


# ---------------------------------------------------------------------------- tracking
def tracking(rows: list[dict]):
    """worst[(speed, ff)] = list over reps of max-over-joints RMSE (and max |e|)."""
    per_rep = defaultdict(list)
    for r in rows:
        per_rep[(float(r["speed_scale"]), int(r["feedforward"]), int(r["rep"]))].append(r)
    worst_rmse, worst_max, per_joint = defaultdict(list), defaultdict(list), defaultdict(list)
    for (speed, ff, _rep), joints in sorted(per_rep.items()):
        worst_rmse[(speed, ff)].append(max(float(j["rmse_rad"]) for j in joints))
        worst_max[(speed, ff)].append(max(float(j["max_abs_err_rad"]) for j in joints))
        for j in joints:
            per_joint[(speed, ff, j["joint"])].append(
                (float(j["rmse_rad"]), float(j["max_abs_err_rad"]))
            )
    return worst_rmse, worst_max, per_joint


def tracking_summary_table(worst_rmse, worst_max) -> list[str]:
    speeds = sorted({s for s, _ in worst_rmse})
    lines = [
        "| Speed (× nominal) | PID: worst-joint RMSE (mrad) "
        "| PID + RNEA FF: worst-joint RMSE (mrad) "
        "| PID: worst max \\|e\\| (mrad) | PID + RNEA FF: worst max \\|e\\| (mrad) "
        "| Regenerate with |",
        "|---|---|---|---|---|---|",
    ]
    for s in speeds:
        lines.append(
            f"| {s:g} | {pm(worst_rmse[(s, 0)], 1e3)} | {pm(worst_rmse[(s, 1)], 1e3)} | "
            f"{pm(worst_max[(s, 0)], 1e3)} | {pm(worst_max[(s, 1)], 1e3)} | "
            "`scripts/bench_tracking.sh` |"
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
    for s in sorted({k[0] for k in per_joint}):
        for ff in (0, 1):
            cells = []
            for j in joints:
                vals = np.asarray(per_joint[(s, ff, j)])
                cells.append(f"{pm(vals[:, 0], 1e3)} / {pm(vals[:, 1], 1e3)}")
            name = "PID + RNEA FF" if ff else "PID"
            lines.append(f"| {s:g} | {name} | " + " | ".join(cells) + " |")
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
        "| TCP drift speed at 2 s (mm/s) "
        "| TCP drift in 2 s (mm) | Effort to hold, F_z (N) | Expected (Δm·g, N) "
        "| Regenerate with |",
        "|---|---|---|---|---|---|---|",
    ]
    for (model, actual), rs in by.items():
        expected = float(rs[0]["expected_hold_force_z_n"])
        lines.append(
            f"| {model:g} / {actual:g} | {actual - model:+g} | "
            f"{pm([r['drift_speed_2s_mps'] for r in rs], 1e3)} | "
            f"{pm([r['drift_disp_2s_m'] for r in rs], 1e3)} | "
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
    w = 0.3
    fig, ax = plt.subplots(figsize=(6.4, 3.4), dpi=150)
    fig.patch.set_facecolor(t["surface"])
    for k, (ff, name, color) in enumerate(((0, "PID", t["s1"]), (1, "PID + RNEA FF", t["s2"]))):
        vals = [np.asarray(worst_rmse[(s, ff)]) * 1e3 for s in speeds]
        means = [v.mean() for v in vals]
        stds = [v.std(ddof=1) if v.size > 1 else 0.0 for v in vals]
        xs = x + (k - 0.5) * w
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
        ncol=2,
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
    drift = [np.array([float(r["drift_speed_2s_mps"]) for r in rs]) * 1e3 for rs in by.values()]
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
    a2.set_ylabel("TCP drift speed at 2 s (mm/s)")
    a2.set_title("Drift after letting go", fontsize=9, loc="left", color=t["ink"])
    for ax in (a1, a2):
        style_axes(ax, t)
        ax.set_xticks(err)
    fig.tight_layout()
    fig.savefig(path, facecolor=t["surface"])
    plt.close(fig)


# ---------------------------------------------------------------------------- outputs
def results_markdown(track_rows, hg_rows) -> tuple[str, str]:
    """(full docs/results.md, summary block shared with the README)."""
    worst_rmse, worst_max, per_joint = tracking(track_rows)
    by = handguide(hg_rows)
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
            "gravity. A 3-waypoint path (2.5 s per segment at 1×) is sent on",
            "`~/joint_trajectory`. The error is the controller's own `q_des − q` at 500 Hz,",
            "from trajectory start to 0.5 s after the last",
            'reference motion. "Worst joint" = the joint with the largest RMSE in that run.',
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
            "Method: per repetition the arm is homed (seeded offset as above), the hand-guiding",
            "controller is activated with the model payload, and nobody touches it for 2 s.",
            "*Drift* is the TCP linear speed |J(q) q̇| over [1.9, 2.0] s and the TCP displacement",
            "at 2 s. *Effort to hold* is the force an operator would have to apply at tool0",
            "to keep the arm still: F = −J(q)⁻ᵀ M(q) q̈, with q̈ fitted over the first 40 ms",
            "after release (q̇ ≈ 0 there, so damping and Coriolis terms do not contribute) and",
            "M(q) from Pinocchio",
            "with the actual payload. Its z component should equal (m_actual − m_model)·g.",
            "",
            "![handguide](assets/handguide_light.png)",
            "",
        ]
    )
    return full, summary


def headline(track_rows, hg_rows) -> str:
    """The README's third sentence: one tracking number and one hand-guiding number."""
    worst_rmse, _, _ = tracking(track_rows)
    by = handguide(hg_rows)
    speed = 1.0 if (1.0, 1) in worst_rmse else sorted(worst_rmse)[0][0]
    over = [(m, a) for (m, a) in by if a - m > 0]
    model, actual = max(over, key=lambda k: k[1] - k[0]) if over else next(iter(by))
    rs = by[(model, actual)]
    return (
        f"At {speed:g}× nominal speed the RNEA feedforward brings the worst-joint tracking RMSE "
        f"to {pm(worst_rmse[(speed, 1)], 1e3)} mrad (PID alone: "
        f"{pm(worst_rmse[(speed, 0)], 1e3)} mrad), and in hand guiding a {actual - model:g} kg "
        f"payload-model error costs the operator {pm([r['hold_force_z_n'] for r in rs])} N "
        f"of vertical force to hold the arm still "
        f"(Δm·g = {float(rs[0]['expected_hold_force_z_n']):.2f} N)."
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
    full, summary = results_markdown(track_rows, hg_rows)
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
