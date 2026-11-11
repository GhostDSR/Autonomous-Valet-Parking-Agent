"""Run all M4 experiments and write tables + figures.

    python -m m4_control.run_experiments            # from the folder that contains m4_control/
    python -m m4_control.run_experiments --quick    # fewer Monte-Carlo seeds

Outputs -> results/{tables,figures,logs}
"""
import argparse
import os
from dataclasses import replace

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

from .follower import FollowerConfig
from .lateral import PurePursuit, Stanley
from .metrics import summarize
from .scenarios import SCENARIOS
from .sim import Bicycle, VehicleParams, run_episode
from .pid import PID

OUT = "results/m4"
COL = {"Pure Pursuit": "#1f77b4", "Stanley": "#ff7f0e", "Stanley+FF": "#2ca02c"}
CTRLS = {
    "Pure Pursuit": lambda: PurePursuit(),
    "Stanley": lambda: Stanley(),
    "Stanley+FF": lambda: Stanley(use_ff=True),
}
plt.rcParams.update({"figure.dpi": 130, "axes.grid": True, "grid.alpha": 0.3,
                     "axes.spines.top": False, "axes.spines.right": False})


# ---------------------------------------------------------------- helpers
def md_table(df, digits=2):
    d = df.copy()
    for c in d.columns:
        if d[c].dtype == bool:
            d[c] = d[c].map(lambda v: "✓" if v else "✗")
        elif d[c].dtype == float:
            dg = 0 if "uccess" in str(c) else digits
            d[c] = d[c].map(lambda v, dg=dg: "—" if np.isnan(v) else f"{v:.{dg}f}")
    head = "| " + " | ".join(map(str, d.columns)) + " |"
    sep = "|" + "|".join(["---"] * len(d.columns)) + "|"
    rows = ["| " + " | ".join(map(str, r)) + " |" for r in d.values]
    return "\n".join([head, sep] + rows)


def save_table(df, name, md, title, note="", digits=2):
    os.makedirs(f"{OUT}/tables", exist_ok=True)
    df.to_csv(f"{OUT}/tables/{name}.csv", index=False)
    md.append(f"### {title}\n\n{md_table(df, digits)}\n" + (f"\n{note}\n" if note else ""))


def savefig(fig, name):
    os.makedirs(f"{OUT}/figures", exist_ok=True)
    fig.savefig(f"{OUT}/figures/{name}.png", bbox_inches="tight")
    plt.close(fig)


def car_polygon(x, y, yaw, L_rear=0.9, L_front=3.6, W=1.9):
    c = np.array([[-L_rear, -W / 2], [L_front, -W / 2], [L_front, W / 2], [-L_rear, W / 2]])
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    return c @ R.T + [x, y]


# ---------------------------------------------------------------- E1 baseline
def exp_baseline(md):
    rows, logs = [], {}
    for sname, sfn in SCENARIOS.items():
        path = sfn()
        os.makedirs(f"{OUT}/paths", exist_ok=True)
        path.to_csv(f"{OUT}/paths/{sname}.csv")
        for cname, cf in CTRLS.items():
            log = run_episode(path, cf())
            logs[(sname, cname)] = log
            rows.append({"scenario": sname, "controller": cname, **summarize(log, path)})
    df = pd.DataFrame(rows)

    os.makedirs(f"{OUT}/logs", exist_ok=True)
    for (sname, cname), lg in logs.items():           # raw logs (same columns as the Webots logger)
        pd.DataFrame({k: lg[k] for k in ["t", "x", "y", "yaw", "v", "delta", "v_ref", "accel",
                                         "cte", "head_err", "seg", "dir", "state"]}).to_csv(
            f"{OUT}/logs/sim_{sname}_{cname.replace(' ', '_').replace('+', 'p')}.csv", index=False)

    cols = ["scenario", "controller", "rms_cte_fwd", "rms_cte_rev", "max_cte_rev",
            "final_lat_cm", "final_long_cm", "final_yaw_deg", "time_s", "rms_steer_rate",
            "max_lat_acc", "success"]
    t = df[cols].rename(columns={
        "rms_cte_fwd": "RMS CTE fwd [m]", "rms_cte_rev": "RMS CTE rev [m]",
        "max_cte_rev": "Max CTE rev [m]", "final_lat_cm": "Final lat [cm]",
        "final_long_cm": "Final long [cm]", "final_yaw_deg": "Final yaw [deg]",
        "time_s": "Time [s]", "rms_steer_rate": "RMS steer rate [rad/s]",
        "max_lat_acc": "Max lat acc [m/s²]", "success": "Parked OK"})
    save_table(t, "E1_baseline", md, "E1 – Controller comparison (noise-free, 0.3 m / 0.05 rad start offset)",
               "CTE = true rear-axle cross-track error. Success: |lat| ≤ 20 cm, |long| ≤ 30 cm, |yaw| ≤ 3°.", 3)

    # --- figure: trajectories
    fig, axs = plt.subplots(1, 3, figsize=(17, 5))
    for ax, (sname, sfn) in zip(axs, SCENARIOS.items()):
        path = sfn()
        for sg in path.segments:
            ax.plot(sg.x[:sg.n_real], sg.y[:sg.n_real], "k--" if sg.direction > 0 else "k:",
                    lw=1.6, label="reference (fwd)" if sg.direction > 0 else "reference (rev)")
        tx, ty, tyaw = path.target_pose
        slot = car_polygon(tx, ty, tyaw, 1.2, 3.9, 2.6)
        ax.add_patch(Polygon(slot, fill=False, ec="gray", lw=2, ls="-"))
        for cname in CTRLS:
            lg = logs[(sname, cname)]
            ax.plot(lg["x"], lg["y"], color=COL[cname], lw=1.5, label=cname)
            ax.add_patch(Polygon(car_polygon(lg["x"][-1], lg["y"][-1], lg["yaw"][-1]),
                                 fill=False, ec=COL[cname], lw=1.2, alpha=0.8))
        ax.set_aspect("equal"); ax.set_title(f"Scenario: {sname}")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    h, l = axs[0].get_legend_handles_labels()
    fig.legend(h[:5], l[:5], loc="lower center", ncol=5, fontsize=9)
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    savefig(fig, "F1_trajectories")

    # --- figure: time series on the L-route
    fig, axs = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    for cname in CTRLS:
        lg = logs[("L_route", cname)]
        axs[0].plot(lg["t"], lg["cte"], color=COL[cname], label=cname)
        axs[1].plot(lg["t"], np.rad2deg(lg["head_err"]), color=COL[cname])
        axs[2].plot(lg["t"], np.rad2deg(lg["delta"]), color=COL[cname])
        axs[3].plot(lg["t"], lg["v"], color=COL[cname])
    lg = logs[("L_route", "Pure Pursuit")]
    axs[3].plot(lg["t"], lg["v_ref"], "k--", lw=1, label="speed reference")
    rev = lg["dir"] < 0
    for ax in axs:
        ax.fill_between(lg["t"], *ax.get_ylim(), where=rev, color="gray", alpha=0.12, lw=0)
    for ax, yl in zip(axs, ["CTE [m]", "heading err [deg]", "steering [deg]", "speed [m/s]"]):
        ax.set_ylabel(yl)
    axs[3].set_xlabel("time [s]  (grey = reversing phase of the Pure Pursuit run)")
    axs[0].legend(ncol=3, fontsize=8); axs[3].legend(fontsize=8)
    axs[0].set_title("L-route: tracking error, steering and speed")
    savefig(fig, "F2_timeseries_L_route")
    return logs


# ---------------------------------------------------------------- E2 parameter sweeps
def exp_sweeps(md, n_seeds):
    path = SCENARIOS["L_route"]()
    sigma = 0.05
    sweeps = {
        "Pure Pursuit": ("ld_min [m]", [0.5, 0.8, 1.2, 1.6, 2.0, 3.0],
                         lambda v: PurePursuit(ld_min=v, ld_max=max(4.0, v + 2))),
        "Stanley+FF": ("k (cross-track gain)", [0.3, 0.7, 1.5, 3.0, 5.0, 8.0],
                       lambda v: Stanley(k=v, use_ff=True)),
    }
    rows = []
    for cname, (pname, vals, mk) in sweeps.items():
        for v in vals:
            ms = [summarize(run_episode(path, mk(v), gps_sigma=sigma, yaw_sigma=0.01,
                                        speed_sigma=0.02, seed=s), path) for s in range(n_seeds)]
            d = pd.DataFrame(ms)
            rows.append({"controller": cname, "param": pname, "value": v,
                         "rms_cte_fwd": d.rms_cte_fwd.mean(), "rms_cte_rev": d.rms_cte_rev.mean(),
                         "max_cte_rev": d.max_cte_rev.mean(), "rms_steer_rate": d.rms_steer_rate.mean(),
                         "final_lat_cm": d.final_lat_cm.mean(), "success_%": 100 * d.success.mean()})
    df = pd.DataFrame(rows)
    save_table(df.rename(columns={"rms_cte_fwd": "RMS CTE fwd [m]", "rms_cte_rev": "RMS CTE rev [m]",
                                  "max_cte_rev": "Max CTE rev [m]", "rms_steer_rate": "RMS steer rate [rad/s]",
                                  "final_lat_cm": "Final lat [cm]", "success_%": "Success [%]"}),
               "E2_param_sweep", md, f"E2 – Look-ahead / gain sweep (L-route, GPS σ = {sigma} m, {n_seeds} seeds)",
               "", 3)

    fig, axs = plt.subplots(2, 3, figsize=(14, 6.5))
    for r, (cname, (pname, vals, _)) in enumerate(sweeps.items()):
        d = df[df.controller == cname]
        axs[r, 0].plot(d.value, d.rms_cte_fwd, "o-", color=COL[cname], label="RMS fwd")
        axs[r, 0].plot(d.value, d.rms_cte_rev, "s--", color=COL[cname], label="RMS rev")
        axs[r, 1].plot(d.value, d.rms_steer_rate, "o-", color=COL[cname])
        axs[r, 2].plot(d.value, d["success_%"], "o-", color=COL[cname])
        axs[r, 0].set_ylabel(f"{cname}\nCTE [m]"); axs[r, 1].set_ylabel("RMS steer rate [rad/s]")
        axs[r, 2].set_ylabel("success [%]"); axs[r, 0].legend(fontsize=8)
        for c in range(3):
            axs[r, c].set_xlabel(pname)
    axs[0, 1].set_title("steering activity"); axs[0, 0].set_title("tracking accuracy")
    axs[0, 2].set_title("parking success")
    fig.suptitle(f"Parameter sweeps (L-route, GPS σ={sigma} m)", y=1.0)
    fig.tight_layout()
    savefig(fig, "F3_param_sweeps")


# ---------------------------------------------------------------- E3 PID / speed control
def exp_pid(md):
    variants = {
        "P only":          dict(kp=1.5, ki=0.0, kd=0.0),
        "PI":              dict(kp=1.5, ki=0.8, kd=0.0),
        "PID":             dict(kp=1.5, ki=0.8, kd=0.15),
        "PI aggressive":   dict(kp=4.0, ki=4.0, kd=0.0),
        "PI sluggish":     dict(kp=0.5, ki=0.2, kd=0.0),
    }
    vp = VehicleParams()
    dt, T = 0.02, 14.0
    rows = []
    fig, ax = plt.subplots(figsize=(10, 4.5))
    tt = np.arange(0, T, dt)
    vref = np.where(tt < 1.0, 0.0, np.where(tt < 7.0, 2.0, 0.5))
    ax.plot(tt, vref, "k--", lw=1.2, label="reference")
    for name, g in variants.items():
        pid = PID(g["kp"], g["ki"], g["kd"], -vp.a_brk, vp.a_acc)
        car = Bicycle(0, 0, 0, vp)
        v = []
        for r in vref:
            a = pid.step(r - car.v, dt)
            car.step(0.0, a, 1, dt)
            v.append(car.v)
        v = np.array(v)
        seg = (tt >= 1.0) & (tt < 7.0)
        v1 = v[seg]
        t90 = tt[seg][np.argmax(v1 >= 0.9 * 2.0)] - 1.0 if (v1 >= 1.8).any() else np.nan
        os_ = max(v1.max() - 2.0, 0) / 2.0 * 100
        outside = np.abs(v1 - 2.0) > 0.04
        t_set = (tt[seg][np.where(outside)[0][-1]] - 1.0) if outside.any() else 0.0
        ss_err = 2.0 - v1[-int(1 / dt):].mean()
        if abs(ss_err) > 0.04:          # never settled inside the 2% band
            t_set = np.nan
        iae = np.sum(np.abs(vref[seg] - v1)) * dt
        rows.append({"variant": name, "kp": g["kp"], "ki": g["ki"], "kd": g["kd"],
                     "rise 90% [s]": t90, "overshoot [%]": os_, "settling 2% [s]": t_set,
                     "steady-state err [m/s]": ss_err, "IAE [m]": iae})
        ax.plot(tt, v, lw=1.5, label=name)
    ax.set_xlabel("time [s]"); ax.set_ylabel("speed [m/s]"); ax.legend(fontsize=8, ncol=2)
    ax.set_title("Speed PID – step response on the vehicle model (accel lag + drag + limits)")
    savefig(fig, "F4_pid_step")
    save_table(pd.DataFrame(rows), "E3a_pid_step", md, "E3a – Speed-PID step response (0 → 2 m/s)", "— = did not settle inside the ±2 % band within the 6 s step window (P-only: steady-state error from drag; sluggish PI: slow integral).", 2)

    # full-parking effect of PID / feed-forward on stopping precision
    path = SCENARIOS["straight"]()
    rows = []
    for name, cfgkw in {"PI + FF (default)": dict(),
                        "PI, no FF": dict(ff_gain=0.0),
                        "P only + FF": dict(ki=0.0),
                        "PI aggressive + FF": dict(kp=4.0, ki=4.0)}.items():
        cfg = FollowerConfig(**cfgkw)
        ms = pd.DataFrame([summarize(run_episode(path, PurePursuit(), cfg=cfg, speed_sigma=0.03, seed=s), path)
                           for s in range(10)])
        rows.append({"speed controller": name, "RMS speed err [m/s]": ms.rms_speed_err.mean(),
                     "final long err [cm]": ms.final_long_cm.mean(), "max final long [cm]": ms.final_long_cm.max(),
                     "time [s]": ms.time_s.mean(), "success [%]": 100 * ms.success.mean()})
    save_table(pd.DataFrame(rows), "E3b_pid_parking", md,
               "E3b – Effect of the speed controller on a full parking run (straight scenario, speed noise 0.03 m/s, 10 seeds)", "", 3)


# ---------------------------------------------------------------- E4 noise robustness
def exp_noise(md, n_seeds):
    path = SCENARIOS["L_route"]()
    sigmas = [0.0, 0.02, 0.05, 0.10, 0.20, 0.30]
    variants = [("Pure Pursuit", False), ("Stanley+FF", False), ("Stanley", False),
                ("Pure Pursuit", True), ("Stanley+FF", True)]
    rows = []
    for sg in sigmas:
        for cname, filt in variants:
            ms = pd.DataFrame([summarize(run_episode(path, CTRLS[cname](), gps_sigma=sg,
                                                     yaw_sigma=0.01 + 0.05 * sg, speed_sigma=0.02,
                                                     seed=s, use_filter=filt), path)
                               for s in range(n_seeds)])
            rows.append({"gps_sigma": sg, "controller": cname + (" + filter" if filt else ""),
                         "rms_cte_mean": ms.rms_cte.mean(), "rms_cte_std": ms.rms_cte.std(),
                         "max_cte_rev": ms.max_cte_rev.mean(),
                         "lat_mean": ms.final_lat_cm.mean(), "lat_std": ms.final_lat_cm.std(),
                         "long_mean": ms.final_long_cm.mean(), "long_std": ms.final_long_cm.std(),
                         "yaw_mean": ms.final_yaw_deg.mean(), "success": 100 * ms.success.mean()})
    df = pd.DataFrame(rows)
    t = df.rename(columns={"gps_sigma": "GPS σ [m]", "rms_cte_mean": "RMS CTE [m]", "rms_cte_std": "± std",
                           "max_cte_rev": "Max CTE rev [m]", "lat_mean": "Final lat [cm]",
                           "lat_std": "± std ", "long_mean": "Final long [cm]", "long_std": "± std  ",
                           "yaw_mean": "Final yaw [deg]", "success": "Success [%]"})
    save_table(t, "E4_noise", md, f"E4 – Robustness to GPS/compass noise, with and without the pose filter (L-route, {n_seeds} seeds per cell)",
               "Compass σ = 0.01 + 0.05·σ_gps rad, speed σ = 0.02 m/s. Filter = dead-reckoning + GPS correction (α = β = 0.1).", 3)

    names = [f"{c}{' + filter' if f else ''}" for c, f in variants]
    fig, axs = plt.subplots(1, 4, figsize=(19, 4))
    for name, (cname, filt) in zip(names, variants):
        d = df[df.controller == name]
        kw = dict(marker="o", capsize=3, color=COL[cname], ls="--" if filt else "-")
        axs[0].errorbar(d.gps_sigma, d.rms_cte_mean, d.rms_cte_std, label=name, **kw)
        axs[1].errorbar(d.gps_sigma, d.lat_mean, d.lat_std, **kw)
        axs[2].errorbar(d.gps_sigma, d.long_mean, d.long_std, **kw)
        axs[3].plot(d.gps_sigma, d.success, marker="o", color=COL[cname], ls="--" if filt else "-")
    axs[0].set_ylabel("RMS CTE [m]"); axs[1].set_ylabel("final lateral error [cm]")
    axs[2].set_ylabel("final longitudinal error [cm]"); axs[3].set_ylabel("parking success [%]")
    for a in axs:
        a.set_xlabel("GPS noise σ [m]")
    axs[0].legend(fontsize=7)
    fig.suptitle("Sensor-noise robustness (mean ± std over seeds; dashed = with pose filter)", y=1.03)
    savefig(fig, "F5_noise_robustness")


# ---------------------------------------------------------------- E5 reverse speed
def exp_speed(md):
    path = SCENARIOS["L_route"]()
    rows = []
    for vr in [0.5, 0.75, 1.0, 1.5, 2.0, 2.5]:
        for cname, cf in CTRLS.items():
            cfg = FollowerConfig(v_rev=vr)
            m = summarize(run_episode(path, cf(), cfg=cfg), path)
            rows.append({"v_rev [m/s]": vr, "controller": cname, "Max CTE rev [m]": m["max_cte_rev"],
                         "RMS CTE rev [m]": m["rms_cte_rev"], "Final lat [cm]": m["final_lat_cm"],
                         "Final yaw [deg]": m["final_yaw_deg"], "Time [s]": m["time_s"], "Parked OK": m["success"]})
    df = pd.DataFrame(rows)
    save_table(df, "E5_reverse_speed", md, "E5 – Reverse-in speed vs accuracy (L-route, noise-free)", "", 3)
    fig, axs = plt.subplots(1, 2, figsize=(11, 4))
    for cname in CTRLS:
        d = df[df.controller == cname]
        axs[0].plot(d["v_rev [m/s]"], d["Max CTE rev [m]"], "o-", color=COL[cname], label=cname)
        axs[1].plot(d["v_rev [m/s]"], d["Final yaw [deg]"], "o-", color=COL[cname])
    axs[0].set_ylabel("max CTE while reversing [m]"); axs[1].set_ylabel("final yaw error [deg]")
    for a in axs:
        a.set_xlabel("reverse speed [m/s]")
    axs[0].legend(fontsize=8)
    savefig(fig, "F6_reverse_speed")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    n_mc = 6 if a.quick else 20
    md = ["# M4 – Results\n"]
    exp_baseline(md)
    exp_sweeps(md, max(4, n_mc // 3))
    exp_pid(md)
    exp_noise(md, n_mc)
    exp_speed(md)
    os.makedirs(OUT, exist_ok=True)
    open(f"{OUT}/tables.md", "w", encoding="utf-8").write("\n".join(md))
    print(f"Done -> {OUT}/tables.md, {OUT}/figures/, {OUT}/tables/, {OUT}/logs/")
