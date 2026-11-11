"""Performance metrics + parking success test."""
import numpy as np
from .path import wrap

# success tolerances for the final rear-axle pose (slot 2.6 m wide, car 1.9 m wide)
TOL_LAT, TOL_LONG, TOL_YAW = 0.20, 0.30, np.deg2rad(3.0)


def summarize(log, path):
    dt = log["dt"]
    tx, ty, tyaw = path.target_pose
    d = np.array([log["x"][-1] - tx, log["y"][-1] - ty])
    long_e = d @ np.array([np.cos(tyaw), np.sin(tyaw)])
    lat_e = d @ np.array([-np.sin(tyaw), np.cos(tyaw)])
    yaw_e = wrap(log["yaw"][-1] - tyaw)

    cte = np.abs(log["cte"])
    rev = log["dir"] < 0
    fwd = ~rev
    steer_rate = np.abs(np.gradient(log["delta"], dt))
    lat_acc = log["v"] ** 2 * np.tan(log["delta"]) / 2.7
    moving = np.abs(log["v"]) > 0.1

    def rms(a):
        return float(np.sqrt(np.mean(a ** 2))) if len(a) else np.nan

    success = bool(log["done"] and abs(lat_e) <= TOL_LAT and abs(long_e) <= TOL_LONG
                   and abs(yaw_e) <= TOL_YAW)
    return {
        "rms_cte": rms(cte[moving]),
        "max_cte": float(cte.max()),
        "rms_cte_fwd": rms(cte[fwd & moving]),
        "rms_cte_rev": rms(cte[rev & moving]),
        "max_cte_rev": float(cte[rev].max()) if rev.any() else np.nan,
        "rms_head_deg": float(np.rad2deg(rms(log["head_err"][moving]))),
        "final_lat_cm": float(abs(lat_e) * 100),
        "final_long_cm": float(abs(long_e) * 100),
        "final_yaw_deg": float(abs(np.rad2deg(yaw_e))),
        "time_s": float(log["t"][-1]),
        "rms_steer_rate": rms(steer_rate[moving]),
        "max_lat_acc": float(np.abs(lat_acc).max()),
        "rms_speed_err": rms((log["v_ref"] - log["v"])[moving]),
        "success": success,
    }
