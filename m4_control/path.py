"""Path representation: smoothing, fillets, gear-change segmentation."""
import numpy as np


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def resample(xy, ds=0.05):
    """Resample a polyline to (approximately) uniform spacing ds."""
    xy = np.asarray(xy, float)
    d = np.hypot(*np.diff(xy, axis=0).T)
    keep = np.r_[True, d > 1e-9]
    xy = xy[keep]
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    n = max(int(round(s[-1] / ds)), 1) + 1
    si = np.linspace(0, s[-1], n)
    return np.c_[np.interp(si, s, xy[:, 0]), np.interp(si, s, xy[:, 1])]


def fillet_polyline(pts, radius, ds=0.05):
    """Replace every corner of a polyline (e.g. a raw A* path) with a circular arc.

    An Ackermann car cannot follow 90-degree corners; this makes the path
    drivable. The radius is shrunk automatically if the legs are too short.
    """
    pts = np.asarray(pts, float)
    out = [pts[0]]
    for k in range(1, len(pts) - 1):
        A, B, C = pts[k - 1], pts[k], pts[k + 1]
        u1, u2 = A - B, C - B
        l1, l2 = np.linalg.norm(u1), np.linalg.norm(u2)
        u1, u2 = u1 / l1, u2 / l2
        theta = np.arccos(np.clip(u1 @ u2, -1, 1))          # interior angle
        if theta > np.pi - 1e-3:                             # collinear
            out.append(B)
            continue
        t = radius / np.tan(theta / 2)
        t_max = 0.5 * min(l1, l2)
        r = radius
        if t > t_max:
            t, r = t_max, t_max * np.tan(theta / 2)
        T1, T2 = B + u1 * t, B + u2 * t
        bis = (u1 + u2) / np.linalg.norm(u1 + u2)
        ctr = B + bis * r / np.sin(theta / 2)
        a1 = np.arctan2(*(T1 - ctr)[::-1])
        a2 = np.arctan2(*(T2 - ctr)[::-1])
        da = wrap(a2 - a1)
        m = max(int(abs(da) * r / ds), 2)
        ang = a1 + da * np.linspace(0, 1, m)
        out.extend(ctr + r * np.c_[np.cos(ang), np.sin(ang)])
    out.append(pts[-1])
    return np.array(out)


def _moving_avg(a, w):
    if w < 2:
        return a
    w = int(w) | 1
    pad = np.pad(a, w // 2, mode="edge")
    return np.convolve(pad, np.ones(w) / w, mode="valid")


class Segment:
    """One continuous drive in a single gear (forward or reverse)."""

    def __init__(self, xy, direction, ds=0.05, ext=3.0, k_smooth=0.5):
        xy = resample(xy, ds)
        self.direction = 1 if direction >= 0 else -1
        self.n_real = len(xy)
        if ext > 0:   # virtual straight extension so look-ahead never runs off the end
            t = xy[-1] - xy[-2]
            t /= np.linalg.norm(t)
            xy = np.vstack([xy, xy[-1] + np.outer(np.arange(1, int(ext / ds) + 1) * ds, t)])
        self.x, self.y = xy[:, 0], xy[:, 1]
        self.s = np.r_[0.0, np.cumsum(np.hypot(np.diff(self.x), np.diff(self.y)))]
        self.phi = np.arctan2(np.gradient(self.y), np.gradient(self.x))   # travel direction
        kappa = np.gradient(np.unwrap(self.phi), self.s)
        self.kappa = _moving_avg(kappa, k_smooth / ds)                    # + = CCW
        self.s_end = self.s[self.n_real - 1]

    def nearest(self, px, py, i0=0, window=150):
        """Nearest path index (incl. virtual extension), searching forward from i0."""
        i1 = min(i0 + window, len(self.x))
        d2 = (self.x[i0:i1] - px) ** 2 + (self.y[i0:i1] - py) ** 2
        return i0 + int(np.argmin(d2))

    def lateral_error(self, px, py):
        """Global, evaluation-grade (projected) signed cross-track error and index."""
        n = self.n_real
        d2 = (self.x[:n] - px) ** 2 + (self.y[:n] - py) ** 2
        k = int(np.argmin(d2))
        best, sgn = np.sqrt(d2[k]), 0.0
        for kk in (k - 1, k):
            if kk < 0 or kk + 1 >= n:
                continue
            A = np.array([self.x[kk], self.y[kk]])
            AB = np.array([self.x[kk + 1], self.y[kk + 1]]) - A
            P = np.array([px, py]) - A
            t = np.clip(P @ AB / (AB @ AB), 0, 1)
            dv = P - t * AB
            dist = np.hypot(*dv)
            if dist <= best:
                best, sgn = dist, np.sign(AB[0] * dv[1] - AB[1] * dv[0])
        if sgn == 0.0:
            sgn = np.sign(-(px - self.x[k]) * np.sin(self.phi[k]) + (py - self.y[k]) * np.cos(self.phi[k])) or 1.0
        return sgn * best, k


class Path:
    def __init__(self, segments):
        self.segments = segments

    @classmethod
    def from_points(cls, xy, direction=None, ds=0.05, ext=3.0):
        """xy: (N,2) waypoints, direction: (N,) of +1 (forward) / -1 (reverse)."""
        xy = np.asarray(xy, float)
        direction = np.ones(len(xy)) if direction is None else np.asarray(direction)
        cuts = [0] + [i for i in range(1, len(xy)) if direction[i] != direction[i - 1]] + [len(xy)]
        segs = []
        for a, b in zip(cuts[:-1], cuts[1:]):
            a0 = a - 1 if a > 0 else a      # share the cusp point between the two segments
            segs.append(Segment(xy[a0:b], direction[a], ds, ext))
        return cls(segs)

    @classmethod
    def from_csv(cls, fname, **kw):
        """CSV with header x,y[,dir] -- the format M2 should export."""
        d = np.genfromtxt(fname, delimiter=",", names=True)
        dirn = d["dir"] if "dir" in d.dtype.names else None
        return cls.from_points(np.c_[d["x"], d["y"]], dirn, **kw)

    def to_csv(self, fname):
        rows = []
        for sg in self.segments:
            rows += [(sg.x[k], sg.y[k], sg.direction) for k in range(sg.n_real)]
        np.savetxt(fname, rows, delimiter=",", header="x,y,dir", comments="", fmt="%.4f")

    @property
    def target_pose(self):
        """(x, y, vehicle yaw) of the final rear-axle pose = the parking goal."""
        s = self.segments[-1]
        k = s.n_real - 1
        yaw = s.phi[k] + (np.pi if s.direction < 0 else 0.0)
        return s.x[k], s.y[k], wrap(yaw)
