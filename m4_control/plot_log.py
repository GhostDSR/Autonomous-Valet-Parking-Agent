"""Plot + score a run log (simulation OR Webots) against the planned path.

    python -m m4_control.plot_log results/logs/webots_run.csv path_m2.csv [out.png]
"""
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .metrics import summarize
from .path import Path, wrap


def evaluate(log_csv, path_csv):
    df = pd.read_csv(log_csv)
    path = Path.from_csv(path_csv)
    # recompute TRUE rear-axle CTE / heading error against the planned path (same for every controller)
    cte, he = [], []
    for x, y, yaw, sidx in zip(df.x, df.y, df.yaw, df.seg.astype(int)):
        seg = path.segments[sidx]
        c, k = seg.lateral_error(x, y)
        cte.append(c)
        he.append(wrap(seg.phi[k] - (yaw if seg.direction > 0 else yaw + np.pi)))
    log = {k: df[k].to_numpy() for k in ["t", "x", "y", "yaw", "v", "delta", "v_ref", "dir"]}
    log.update(cte=np.array(cte), head_err=np.array(he), done=bool(df.state.iloc[-1] == "DONE"),
               dt=float(np.median(np.diff(df.t))))
    return df, log, path, summarize(log, path)


def plot(df, log, path, out):
    fig, axs = plt.subplots(2, 2, figsize=(13, 8))
    ax = axs[0, 0]
    for sg in path.segments:
        ax.plot(sg.x[:sg.n_real], sg.y[:sg.n_real], "k--" if sg.direction > 0 else "k:", label="plan")
    ax.plot(df.x, df.y, "C0", label="driven"); ax.set_aspect("equal"); ax.legend()
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_title("trajectory")
    axs[0, 1].plot(log["t"], log["cte"]); axs[0, 1].set_ylabel("CTE [m]"); axs[0, 1].set_title("cross-track error")
    axs[1, 0].plot(log["t"], np.rad2deg(log["delta"])); axs[1, 0].set_ylabel("steering [deg]")
    axs[1, 1].plot(log["t"], log["v"], label="v"); axs[1, 1].plot(log["t"], log["v_ref"], "k--", label="v_ref")
    axs[1, 1].set_ylabel("speed [m/s]"); axs[1, 1].legend()
    for a in axs.flat[1:]:
        a.set_xlabel("time [s]")
    fig.tight_layout(); fig.savefig(out, dpi=130)


if __name__ == "__main__":
    log_csv, path_csv = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else log_csv.replace(".csv", ".png")
    df, log, path, m = evaluate(log_csv, path_csv)
    plot(df, log, path, out)
    for k, v in m.items():
        print(f"{k:16s} {v:.3f}" if isinstance(v, float) else f"{k:16s} {v}")
    print("figure ->", out)
