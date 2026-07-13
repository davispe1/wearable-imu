"""
app/viewer.py — the Python visualiser for the gait-kinematics library.

Runs the full pipeline on a session (``kinematics.analyze_session``) and draws ONE figure that
shows what the rig measures, over a time window like the bundled P04 2-minute walk:

    1. Joint angles (hip / knee / ankle) vs time, with detected foot strikes and turns marked.
    2. The gait-event signal (shank sagittal angular velocity) with initial-contact, toe-off and
       mid-swing markers — the Salarian/Aminian detection in action.
    3. Gait-cycle-normalised joint angles (0–100 % stride): mean ± SD across all steady strides
       — the classic clinical gait plot.
    4. A panel of the spatiotemporal gait parameters.

It also (optionally) writes the intermediate CSV/JSON artefacts, so the same numbers can be
re-loaded without recomputing. OpenSim is never involved here — this is the kinematic product.

    python -m app.viewer <session_dir> [--t0 S --t1 S] [--mode 6D|9D|auto] [--save] [--no-show]
"""
from __future__ import annotations

import argparse
import os

import numpy as np

from kinematics import analyze_session
from kinematics import parameters as P

JCOLOR = {"hip": "#1f77b4", "knee": "#d62728", "ankle": "#2ca02c"}


# --------------------------------------------------------------------------- #
def _default_window(res, span_s=12.0):
    """A clean steady-walking window: ``span_s`` seconds from the first steady stride."""
    steady_idx = np.flatnonzero(res.steady_state)
    if not len(steady_idx):
        return 0.0, min(span_s, res.duration_s)
    t0 = float(res.t[steady_idx[0]])
    return t0, min(t0 + span_s, res.duration_s)


def _plot_angles(ax, res, t0, t1):
    """Joint angles vs time over [t0, t1] with foot strikes and turns marked."""
    t = res.t
    sel = (t >= t0) & (t <= t1)
    for j in res.joint_names:
        ax.plot(t[sel], res.joints[j]["flexion"][sel], color=JCOLOR.get(j), lw=1.6,
                label=f"{j} (ROM {res.joints[j]['params'].get('rom_cycle_deg', float('nan')):.0f}°)")
    # foot strikes (initial contact) as vertical guides
    fs_idx = np.asarray(res.events.get("foot_strike", []), int)
    for i in fs_idx:
        if i < len(t) and t0 <= t[i] <= t1:
            ax.axvline(t[i], color="0.6", ls="--", lw=0.8, alpha=0.7)
    # turnarounds shaded (if any fall in the window)
    for s, e, _deg in res.turnarounds:
        if t[min(e, len(t) - 1)] >= t0 and t[s] <= t1:
            ax.axvspan(t[s], t[min(e, len(t) - 1)], color="orange", alpha=0.12)
    ax.set_ylabel("flexion (deg)")
    ax.set_xlim(t0, t1)
    ax.set_title(f"Joint angles — {res.session_id}  ({res.side} leg, VQF "
                 f"{list(set(res.modes.values()))[0] if res.modes else '6D'}, "
                 f"{res.fs:.0f} Hz)  ·  dashed = initial contact")
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    ax.grid(alpha=0.25)


def _plot_events(ax, res, t0, t1):
    """Shank sagittal angular velocity with IC / TO / mid-swing markers over [t0, t1]."""
    t = res.t
    sel = (t >= t0) & (t <= t1)
    sig = np.asarray(res.events.get("sagittal_rate", np.zeros(len(t))))
    ax.plot(t[sel], np.degrees(sig[sel]), color="0.35", lw=1.2)
    marks = [("foot_strike", "initial contact", "#d62728", "v"),
             ("toe_off", "toe-off", "#1f77b4", "^"),
             ("mid_swing", "mid-swing", "#2ca02c", "o")]
    for key, label, color, mk in marks:
        idx = np.asarray(res.events.get(key, []), int)
        idx = idx[(idx < len(t))]
        idx = idx[(t[idx] >= t0) & (t[idx] <= t1)]
        ax.plot(t[idx], np.degrees(sig[idx]), mk, color=color, ms=6, ls="none", label=label)
    node = res.events.get("event_node", "shank")
    ax.set_ylabel(f"{node} sagittal\nrate (deg/s)")
    ax.set_xlabel("time (s)")
    ax.set_xlim(t0, t1)
    ax.legend(loc="upper right", fontsize=8, ncol=3)
    ax.grid(alpha=0.25)


def _plot_cycles(ax, res):
    """Gait-cycle-normalised joint angles (mean ± SD) across all steady strides."""
    strikes = res.events.get("foot_strike", np.array([], int))
    for j in res.joint_names:
        grid, cycles, mean, std = P.overlay_cycles(res.joints[j]["flexion"], strikes,
                                                    res.steady_state, res.fs)
        if mean is None:
            continue
        c = JCOLOR.get(j)
        ax.plot(grid, mean, color=c, lw=2.0, label=f"{j} (n={len(cycles)})")
        ax.fill_between(grid, mean - std, mean + std, color=c, alpha=0.15)
    ax.set_xlabel("gait cycle (%)  ·  0 = initial contact")
    ax.set_ylabel("flexion (deg)")
    ax.set_title("Mean gait cycle ± SD (steady strides)")
    ax.set_xlim(0, 100)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.25)


def _plot_params(ax, res):
    """Text panel: per-joint ROM/peaks + temporal & spatial gait parameters."""
    ax.axis("off")
    tp, sp = res.temporal, res.spatial
    lines = [r"$\bf{Temporal\ (steady\ straight\ walking)}$",
             f"cadence            {tp['cadence_steps_per_min']:6.1f} steps/min",
             f"stride time        {tp['stride_time_mean_s']:6.3f} ± {tp['stride_time_std_s']:.3f} s",
             f"step time          {tp['step_time_s']:6.3f} s",
             f"stance / swing     {tp['stance_pct']:5.1f}% / {tp['swing_pct']:.1f}%",
             f"stride-time CV     {tp['stride_time_cv_pct']:6.1f}%",
             f"steady strides     {tp['n_steady_strides']:6d}",
             "",
             r"$\bf{Spatial\ (foot\ ZUPT\ —\ estimate)}$",
             f"stride length    ~ {sp.get('stride_length_m_est', float('nan')):5.2f} m",
             f"walking speed    ~ {sp.get('walking_speed_mps_est', float('nan')):5.2f} m/s",
             "",
             r"$\bf{Per\ joint\ (cycle\text{-}averaged)}$",
             f"{'joint':6s}{'ROM':>8s}{'peak flex':>11s}{'peak ext':>10s}"]
    for j in res.joint_names:
        p = res.joints[j]["params"]
        lines.append(f"{j:6s}{p.get('rom_cycle_deg', p['rom_deg']):7.1f}°"
                     f"{p.get('peak_flexion_cycle_deg', p['peak_flexion_deg']):10.1f}°"
                     f"{p.get('peak_extension_cycle_deg', p['peak_extension_deg']):9.1f}°")
    lines += ["",
              r"$\bf{Bout}$",
              f"duration           {res.duration_s:6.1f} s",
              f"turnarounds        {len(res.turnarounds):6d}"]
    ax.text(0.0, 1.0, "\n".join(lines), va="top", ha="left", family="monospace",
            fontsize=9, transform=ax.transAxes)


# --------------------------------------------------------------------------- #
def make_figure(res, t0, t1, *, include_params=True, include_footnote=True):
    """Build the gait-kinematics figure.

    ``include_params`` adds the at-a-glance parameter text panel next to the graphs
    (the temporal/spatial/per-joint summary). ``include_footnote`` adds the methods/
    references line at the bottom. Both default to True for the CLI/PNG, which is a
    self-contained artefact. The GUI keeps the parameter panel but drops the footnote
    (references live in the docs, and the GUI also has a dedicated Parameters tab).
    """
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    if include_params:
        fig = plt.figure(figsize=(15, 9))
        gs = GridSpec(3, 2, figure=fig, height_ratios=[1.5, 1.0, 1.4], hspace=0.32, wspace=0.18)
        ax_angles = fig.add_subplot(gs[0, :])
        ax_events = fig.add_subplot(gs[1, :], sharex=ax_angles)
        ax_cycle = fig.add_subplot(gs[2, 0])
        ax_params = fig.add_subplot(gs[2, 1])
        _plot_angles(ax_angles, res, t0, t1)
        _plot_events(ax_events, res, t0, t1)
        _plot_cycles(ax_cycle, res)
        _plot_params(ax_params, res)
    else:
        fig = plt.figure(figsize=(13, 9))
        gs = GridSpec(3, 1, figure=fig, height_ratios=[1.5, 1.0, 1.4], hspace=0.32)
        ax_angles = fig.add_subplot(gs[0, 0])
        ax_events = fig.add_subplot(gs[1, 0], sharex=ax_angles)
        ax_cycle = fig.add_subplot(gs[2, 0])
        _plot_angles(ax_angles, res, t0, t1)
        _plot_events(ax_events, res, t0, t1)
        _plot_cycles(ax_cycle, res)

    fig.suptitle("Gait kinematics from 4 IMUs (pelvis · thigh · shank · foot) — VQF orientation "
                 "→ sagittal joint angles → gait parameters", fontsize=13, y=0.985)
    if include_footnote:
        fig.text(0.5, 0.005,
                 "Methods: VQF orientation (Laidig & Seel 2023) · functional-axis sagittal angles "
                 "(Seel et al. 2014) · gyroscope gait events (Aminian 2002, Salarian 2004) · "
                 "foot-ZUPT stride length (Mariani et al. 2010).",
                 ha="center", fontsize=7.5, color="0.4")
    return fig


# --------------------------------------------------------------------------- #
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Visualise gait kinematics for a session.")
    ap.add_argument("session", help="session folder (RF.csv/RS.csv/RT.csv/SA.csv or raw/data.csv)")
    ap.add_argument("--mode", choices=("6D", "9D", "auto"), default="6D")
    ap.add_argument("--side", choices=("right", "left"), default=None)
    ap.add_argument("--t0", type=float, default=None, help="window start (s); default: first steady")
    ap.add_argument("--t1", type=float, default=None, help="window end (s)")
    ap.add_argument("--full", action="store_true", help="time window = whole bout")
    ap.add_argument("--save", action="store_true", help="also write CSV/JSON artefacts + PNG")
    ap.add_argument("--no-show", action="store_true", help="do not open a window (just save the PNG)")
    args = ap.parse_args(argv)
    if not os.path.isdir(args.session):
        ap.error(f"not a directory: {args.session}")

    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    res = analyze_session(args.session, mode=args.mode, side=args.side)
    if args.full:
        t0, t1 = 0.0, res.duration_s
    else:
        d0, d1 = _default_window(res)
        t0 = args.t0 if args.t0 is not None else d0
        t1 = args.t1 if args.t1 is not None else d1

    fig = make_figure(res, t0, t1)

    out_dir = os.path.join(args.session, "results")
    os.makedirs(out_dir, exist_ok=True)
    png = os.path.join(out_dir, f"{res.session_id}_kinematics.png")
    fig.savefig(png, dpi=120, bbox_inches="tight")
    print(f"wrote {png}")
    if args.save:
        sid = res.session_id
        res.save_timeseries_csv(os.path.join(out_dir, f"{sid}_joint_angles.csv"))
        res.save_events_csv(os.path.join(out_dir, f"{sid}_gait_events.csv"))
        res.save_summary_json(os.path.join(out_dir, f"{sid}_gait_parameters.json"))
        print(f"wrote {sid}_joint_angles.csv, {sid}_gait_events.csv, {sid}_gait_parameters.json")

    if not args.no_show:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
