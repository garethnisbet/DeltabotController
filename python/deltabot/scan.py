"""Scans: move the stage through a set of points and measure at each one.

    scanner = Scanner(bot, Camera())
    data = scanner.ascan("x", -1, 1, 21, Circles(), Peaks("x"), SaveImage())
    data["x"], data["cx"]          # numpy columns
    data.plot("x", "cx")

Scans run in the plane (``x``, ``y`` in mm, through the kinematics) or on the
raw motors (``m0``, ``m1``, ``m2`` in steps, straight to the screws)::

    scanner.ascan("m0", -2000, 2000, 21, Circles())       # one screw on its own
    scanner.dmmesh("m1", -500, 500, 5, "m2", -500, 500, 5)

Every point records where the stage was asked to go (``x``, ``y`` or
``m0``..``m2``), where its steps say it went (``x_rb``, ``y_rb``, ``s0``..``s2``, ``strain``), when
(``time``, seconds from the start) and, with a camera, which viewer frame the
measurements came from (``frame_id``).  Detectors add their own columns.

Each scan goes into its own numbered directory under the data directory::

    scans/scan_0007_ascan/
        data.csv        one row per point, written as it is measured
        meta.json       the command, geometry, viewer settings, timing
        images/0000.png one per point with SaveImage

The CSV is flushed after every point, so an aborted or crashed scan keeps
everything it measured.  Ctrl-C stops the stage and ends the scan cleanly.
"""

import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .config import Geometry
from .kinematics import Pose, common_mode, reachable, screws_to_pose
from .png import read_png, write_png

NAN = float("nan")


# ------------------------------------------------------------------ detectors
class Detector:
    """Something measured at every scan point.

    Subclasses say what they need from the viewer and turn a
    :class:`~deltabot.camera.Frame` into named values.
    """

    hough = False            # needs Hough detection
    fits: Tuple[str, ...] = ()   # projection directions to fit: "top"/"right"
    image: Optional[str] = None  # "grey" or "raw" if it needs the image

    def read(self, frame, scan: "ScanContext") -> Dict[str, object]:
        raise NotImplementedError


class Circles(Detector):
    """Hough circle centres and radii, in pixels.

    Columns ``cx``, ``cy``, ``radius`` and ``n_circles``; with ``n`` > 1 the
    first ``n`` circles get ``cx0``, ``cy0``, ``radius0``, ``cx1``...  Missing
    circles are NaN.  Circles come in the viewer's order (strongest first).

    By default this reads the frame's raw detections.  The viewer's
    stabilised circles need a circle on two consecutive frames and are
    smoothed over several, so after a move they lag the true position; pass
    ``stabilised=True`` only when streaming with a generous settle time.
    """

    hough = True

    def __init__(self, n: int = 1, stabilised: bool = False):
        self.n = n
        self.stabilised = stabilised

    def read(self, frame, scan):
        h = frame.hough
        found = h.circles if self.stabilised else h.raw_circles
        out = {"n_circles": len(found)}
        for i in range(self.n):
            tag = "" if self.n == 1 else str(i)
            cx, cy, r = ([int(v) for v in found[i]] if i < len(found)
                         else (NAN, NAN, NAN))
            out[f"cx{tag}"], out[f"cy{tag}"], out[f"radius{tag}"] = cx, cy, r
        return out


class Peaks(Detector):
    """Gaussian peak centres from the viewer's projection fits, in pixels.

    ``axes`` is ``"x"`` (the top projection, summed down columns), ``"y"``
    (the right projection) or ``"xy"``.  Columns per axis: ``peak_x``,
    ``sigma_x``, ``amp_x``, ``offset_x``, ``r2_x``; with ``n`` > 1,
    ``peak_x0``, ``peak_x1``... ordered by position so a column follows the
    same peak through the scan.  The number of Gaussians fitted is the
    viewer's setting - ``cam.remote.set_fit_params(n_gaussians=2)``.
    """

    _DIRECTIONS = {"x": "top", "y": "right"}

    def __init__(self, axes: str = "xy", n: int = 1):
        if not axes or set(axes) - set("xy"):
            raise ValueError("axes must be 'x', 'y' or 'xy'")
        self.axes = "".join(a for a in "xy" if a in axes)
        self.n = n
        self.fits = tuple(self._DIRECTIONS[a] for a in self.axes)

    def read(self, frame, scan):
        out = {}
        for axis in self.axes:
            fit = frame.fits.get(self._DIRECTIONS[axis])
            gaussians = [] if fit is None else sorted(fit.gaussians,
                                                      key=lambda g: g["mean"])
            for i in range(self.n):
                tag = axis if self.n == 1 else f"{axis}{i}"
                g = gaussians[i] if i < len(gaussians) else {}
                out[f"peak_{tag}"] = g.get("mean", NAN)
                out[f"sigma_{tag}"] = abs(g.get("sigma", NAN))
                out[f"amp_{tag}"] = g.get("amplitude", NAN)
            out[f"offset_{axis}"] = NAN if fit is None else fit.offset
            out[f"r2_{axis}"] = NAN if fit is None else fit.r_squared
        return out


class SaveImage(Detector):
    """Save the frame each point was measured on, as ``images/NNNN.png``.

    ``raw=True`` (the default) keeps the camera frame as it came, colour
    included; ``raw=False`` saves the greyscale image the analysis ran on.
    The ``image`` column holds the file name.

    PNGs hold 8 or 16 bit values exactly.  The greyscale image is float64
    even off an 8-bit camera, so whole-number values are stored as 8 or 16
    bit; an image PNG cannot hold exactly (fractional or negative values) is
    saved as ``NNNN.npy`` instead, and the column names that file.
    """

    def __init__(self, raw: bool = True):
        self.image = "raw" if raw else "grey"

    def read(self, frame, scan):
        if scan.path is None or frame.image is None:
            return {"image": ""}
        img = _png_ready(frame.image)
        (scan.path / "images").mkdir(exist_ok=True)
        if img is None:
            name = f"images/{scan.point:04d}.npy"
            np.save(scan.path / name, frame.image)
        else:
            name = f"images/{scan.point:04d}.png"
            write_png(scan.path / name, img)
        return {"image": name}


def _png_ready(img: np.ndarray) -> Optional[np.ndarray]:
    """The image as uint8/uint16 if PNG can store it exactly, else None."""
    if img.ndim not in (2, 3) or (img.ndim == 3 and img.shape[2] > 4):
        return None
    if img.dtype in (np.uint8, np.uint16):
        return img
    if img.size == 0 or img.dtype.kind not in "uif":
        return None
    if img.dtype.kind == "f" and not np.array_equal(img, np.rint(img)):
        return None
    lo, hi = img.min(), img.max()
    if lo < 0 or hi > 65535:
        return None
    return img.astype(np.uint8 if hi <= 255 else np.uint16)


class Custom(Detector):
    """Any function of the frame: ``Custom(lambda frame: {"mean": frame.image.mean()}, image="grey")``.

    Say what the function needs from the viewer with ``hough``, ``fit``
    (``"top"``, ``"right"``, ``"both"``) and ``image`` (``"grey"``/``"raw"``).
    The function gets the :class:`~deltabot.camera.Frame` (``None`` when the
    scan has no camera) and returns a dict of values.
    """

    def __init__(self, fn: Callable, hough: bool = False,
                 fit: Optional[str] = None, image: Optional[str] = None):
        self.fn = fn
        self.hough = hough
        self.fits = {None: (), "both": ("top", "right")}.get(fit, (fit,))
        self.image = image
        self.needs_camera = hough or bool(self.fits) or image is not None

    def read(self, frame, scan):
        return dict(self.fn(frame))


def _needs(detectors: Sequence[Detector]):
    hough = any(d.hough for d in detectors)
    directions = {f for d in detectors for f in d.fits}
    fit = ("both" if directions == {"top", "right"}
           else next(iter(directions)) if directions else None)
    images = {d.image for d in detectors if d.image}
    if len(images) > 1:
        raise ValueError("detectors want both the raw and the greyscale image; "
                         "one scan can record only one")
    image = next(iter(images)) if images else None
    camera = hough or fit is not None or image is not None or any(
        not isinstance(d, Custom) or d.needs_camera for d in detectors)
    return camera, hough, fit, image


# --------------------------------------------------------------------- data
class ScanContext:
    """What a detector knows about where it is in the scan."""

    def __init__(self, path: Optional[Path]):
        self.path = path
        self.point = 0


class ScanData:
    """A scan's table and metadata, live or loaded from disk.

    ``data["cx"]`` is a column (a float array where the column is numeric);
    ``data.columns`` names them; ``data.meta`` is the metadata;
    ``data.image(i)`` loads point ``i``'s saved image.
    """

    def __init__(self, path: Optional[Path], rows: List[dict], meta: dict):
        self.path = path
        self.rows = rows
        self.meta = meta

    @property
    def columns(self) -> List[str]:
        return list(self.rows[0]) if self.rows else []

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, column: str) -> np.ndarray:
        values = [row.get(column, NAN) for row in self.rows]
        try:
            return np.array([NAN if v in ("", None) else float(v) for v in values])
        except (TypeError, ValueError):
            return np.array(values, dtype=object)

    def image(self, point: int) -> np.ndarray:
        name = self.rows[point].get("image")
        if not name or self.path is None:
            raise KeyError(f"point {point} has no saved image")
        path = self.path / name
        return read_png(path) if path.suffix == ".png" else np.load(path)

    def plot(self, x: str, *ys: str, ax=None):
        """Plot columns against ``x`` (all numeric columns if none named)."""
        import matplotlib.pyplot as plt

        ys = ys or tuple(c for c in self.columns if c not in (
            x, "point", "time", "frame_id", "image")
            and self[c].dtype != object and not np.all(np.isnan(self[c])))
        if ax is None:
            fig, ax = plt.subplots()
        for y in ys:
            ax.plot(self[x], self[y], "o-", label=y)
        ax.set_xlabel(x)
        ax.set_title(self.meta.get("name", ""))
        if len(ys) > 1:
            ax.legend()
        plt.show(block=False)
        return ax

    def __repr__(self) -> str:
        state = self.meta.get("status", "")
        return (f"ScanData({self.meta.get('name', '?')!r}, {len(self)} points, "
                f"{state}, columns={self.columns})")


def load_scan(path) -> ScanData:
    """Load a scan directory written by :class:`Scanner`."""
    path = Path(path)
    meta = json.loads((path / "meta.json").read_text())
    with open(path / "data.csv", newline="") as fh:
        rows = list(csv.DictReader(fh))
    return ScanData(path, rows, meta)


# ------------------------------------------------------------------ scanner
class Scanner:
    """Runs scans on a :class:`~deltabot.DeltaBot`, optionally with a camera.

    ``settle`` is the pause after each move before measuring, for the
    flexures to stop ringing.  ``data_dir`` is where scans are written;
    ``None`` keeps them in memory only.
    """

    def __init__(self, bot, camera=None, data_dir: Optional[str] = "scans",
                 settle: float = 0.2, verbose: bool = True):
        self.bot = bot
        self.camera = camera
        self.data_dir = None if data_dir is None else Path(data_dir)
        self.settle = settle
        self.verbose = verbose
        self.last = None          # the last ScanData
        self.last_frame = None    # the last Frame measured

    # ---------------------------------------------------------- scan types
    def ascan(self, axis: str, start: float, stop: float, num: int,
              *detectors: Detector, **kw) -> ScanData:
        """Scan one axis through ``num`` points from ``start`` to ``stop``,
        holding the others where they are.

        ``axis`` is ``"x"`` or ``"y"`` (mm, through the kinematics) or a raw
        motor, ``"m0"``, ``"m1"`` or ``"m2"`` (absolute steps, that screw
        alone - see :meth:`run`)."""
        points, space, _ = self._line(axis, np.linspace(start, stop, num), False)
        return self.run(points, *detectors, kind="ascan", space=space, **kw)

    def dscan(self, axis: str, start: float, stop: float, num: int,
              *detectors: Detector, return_to_start: bool = True, **kw) -> ScanData:
        """:meth:`ascan` relative to where the stage is now, returning there
        afterwards (also after Ctrl-C)."""
        points, space, here = self._line(axis, np.linspace(start, stop, num), True)
        return self.run(points, *detectors, kind="dscan", space=space,
                        return_to=here if return_to_start else None, **kw)

    def mmesh(self, a: str, a_start: float, a_stop: float, a_num: int,
              b: str, b_start: float, b_stop: float, b_num: int,
              *detectors: Detector, snake: bool = True, **kw) -> ScanData:
        """A grid over two raw motors in absolute steps: rows along motor
        ``a``, stepping motor ``b``; the third motor holds."""
        return self._motor_grid(a, a_start, a_stop, a_num, b, b_start, b_stop,
                                b_num, detectors, snake, False, False, "mmesh", kw)

    def dmmesh(self, a: str, a_start: float, a_stop: float, a_num: int,
               b: str, b_start: float, b_stop: float, b_num: int,
               *detectors: Detector, snake: bool = True,
               return_to_start: bool = True, **kw) -> ScanData:
        """:meth:`mmesh` relative to the motors' current steps."""
        return self._motor_grid(a, a_start, a_stop, a_num, b, b_start, b_stop,
                                b_num, detectors, snake, True, return_to_start,
                                "dmmesh", kw)

    def mesh(self, x_start: float, x_stop: float, x_num: int,
             y_start: float, y_stop: float, y_num: int,
             *detectors: Detector, snake: bool = True, **kw) -> ScanData:
        """A grid, rows along x stepping in y.  ``snake`` reverses every other
        row so the stage never makes a long return move."""
        points = self._grid(np.linspace(x_start, x_stop, x_num),
                            np.linspace(y_start, y_stop, y_num), snake)
        return self.run(points, *detectors, kind="mesh", **kw)

    def dmesh(self, x_start: float, x_stop: float, x_num: int,
              y_start: float, y_stop: float, y_num: int,
              *detectors: Detector, snake: bool = True,
              return_to_start: bool = True, **kw) -> ScanData:
        """:meth:`mesh` relative to where the stage is now."""
        x0, y0 = self._here()
        points = self._grid(x0 + np.linspace(x_start, x_stop, x_num),
                            y0 + np.linspace(y_start, y_stop, y_num), snake)
        return self.run(points, *detectors, kind="dmesh",
                        return_to=(x0, y0) if return_to_start else None, **kw)

    def count(self, *detectors: Detector) -> dict:
        """Measure once where the stage is, without moving or saving."""
        x, y = self._here()
        camera, hough, fit, image = _needs(detectors)
        cam = self._camera_for(camera)
        if cam:
            cam.prepare(hough=hough)
        return self._measure(ScanContext(None), {"x": x, "y": y}, 0.0,
                             detectors, cam, hough, fit, image)

    # ------------------------------------------------------------ the loop
    def run(self, points: Iterable[Sequence[float]], *detectors: Detector,
            space: str = "xy", settle: Optional[float] = None,
            name: Optional[str] = None, kind: str = "scan",
            return_to: Optional[Sequence[float]] = None,
            notes: str = "") -> ScanData:
        """Visit each point and measure with every detector.

        With ``space="xy"`` a point is ``(x, y)`` in mm and every move goes
        through the kinematics, so nothing enters the over-constrained
        direction.

        With ``space="steps"`` a point is ``(m0, m1, m2)``, absolute step
        positions sent straight to the three motors - raw motor control, with
        no working radius check.  A point whose steps do not sum to zero
        strains the flexures (the ``strain`` column shows how much).  Points
        are checked against the firmware's step limits before anything moves.
        """
        geom: Geometry = self.bot.geometry
        if space == "xy":
            points = [(float(x), float(y)) for x, y in points]
            outside = [p for p in points if not reachable(Pose(*p), geom)]
            if outside:
                raise ValueError(f"{len(outside)} point(s) lie outside the "
                                 f"{geom.max_radius_mm} mm working radius, e.g. {outside[0]}")
            names: Tuple[str, ...] = ("x", "y")
            move = lambda p: self.bot.move_xy(*p)
        elif space == "steps":
            points = [tuple(int(round(s)) for s in p) for p in points]
            if any(len(p) != 3 for p in points):
                raise ValueError("a step point needs all three motor positions")
            self._check_limits(points)
            names = self._MOTORS
            move = lambda p: self.bot.move_steps(*p)
        else:
            raise ValueError("space must be 'xy' or 'steps'")
        settle = self.settle if settle is None else settle
        camera, hough, fit, image = _needs(detectors)
        cam = self._camera_for(camera)

        path = self._new_dir(name or kind)
        ctx = ScanContext(path)
        meta = {
            "name": path.name if path else (name or kind),
            "kind": kind, "notes": notes, "status": "running",
            "started": datetime.now().isoformat(timespec="seconds"),
            "space": space, "axes": list(names),
            "points": points, "settle_s": settle,
            "detectors": [f"{type(d).__name__}({_describe(d)})" for d in detectors],
            "geometry": {k: getattr(geom, k) for k in geom.__dataclass_fields__},
            "port": self.bot.conn.port,
        }
        rows: List[dict] = []
        data = self.last = ScanData(path, rows, meta)
        writer = fh = None
        t0 = time.time()
        if cam:
            cam.prepare(hough=hough)
            meta["viewer"] = {"hough_params": cam.remote.hough_params(),
                              "fit_params": _settings(cam.remote.fit_params()),
                              "streaming": cam.streaming, "discard": cam.discard}
        self._write_meta(data)
        if self.verbose:
            print(f"{meta['name']}: {len(points)} points"
                  + (f" -> {path}" if path else ""))
        try:
            for i, p in enumerate(points):
                ctx.point = i
                move(p)
                if settle:
                    time.sleep(settle)
                row = self._measure(ctx, dict(zip(names, p)), t0, detectors,
                                    cam, hough, fit, image)
                rows.append(row)
                if path is not None:
                    if writer is None:
                        fh = open(path / "data.csv", "w", newline="")
                        writer = csv.DictWriter(fh, fieldnames=list(row), restval="")
                        writer.writeheader()
                    writer.writerow(row)
                    fh.flush()
                if self.verbose:
                    self._progress(i, len(points), row)
            meta["status"] = "complete"
        except KeyboardInterrupt:
            meta["status"] = "aborted"
            self._safely(self.bot.stop)
            print(f"\naborted after {len(rows)} of {len(points)} points")
        except Exception as exc:
            meta["status"] = f"failed: {type(exc).__name__}: {exc}"
            self._safely(self.bot.stop)
            raise
        finally:
            if fh is not None:
                fh.close()
            if return_to is not None:
                self._safely(lambda: move(return_to))
            meta["finished"] = datetime.now().isoformat(timespec="seconds")
            meta["duration_s"] = round(time.time() - t0, 3)
            meta["measured"] = len(rows)
            self._write_meta(data)
        return data

    def _measure(self, ctx, commanded, t0, detectors, cam, hough, fit, image) -> dict:
        steps = self.bot.positions()
        geom = self.bot.geometry
        ext = [geom.steps_to_mm(i, s) for i, s in enumerate(steps)]
        pose = screws_to_pose(ext, geom)
        row = {"point": ctx.point, "time": round(time.time() - t0, 4) if t0 else 0.0,
               **commanded, "x_rb": pose.x, "y_rb": pose.y,
               "s0": steps[0], "s1": steps[1], "s2": steps[2],
               "strain": common_mode(ext)}
        frame = None
        if cam:
            frame = self.last_frame = cam.acquire(hough=hough, fit=fit, image=image)
            row["frame_id"] = frame.frame_id
        for d in detectors:
            row.update(d.read(frame, ctx))
        return row

    # ------------------------------------------------------------- helpers
    def _camera_for(self, needed: bool):
        if needed and self.camera is None:
            raise RuntimeError("these detectors need the camera viewer - start it "
                               "with 'python3 image_visualiser.py --serve' and "
                               "connect with connect_camera()")
        return self.camera if needed else None

    _MOTORS = ("m0", "m1", "m2")

    def _here(self, space: str = "xy") -> tuple:
        if space == "steps":
            return tuple(self.bot.positions())
        p = self.bot.pose()
        return p.x, p.y

    def _motor(self, name: str) -> int:
        if name not in self._MOTORS:
            raise ValueError("motor must be 'm0', 'm1' or 'm2'")
        return self._MOTORS.index(name)

    def _line(self, axis, values, relative):
        """Points along one axis, the rest held; returns (points, space, here)."""
        if axis in ("x", "y"):
            space, k = "xy", ("x", "y").index(axis)
        elif axis in self._MOTORS:
            space, k = "steps", self._motor(axis)
        else:
            raise ValueError("axis must be 'x', 'y', 'm0', 'm1' or 'm2'")
        here = self._here(space)
        points = []
        for v in values:
            p = list(here)
            p[k] = (here[k] if relative else 0) + v
            points.append(tuple(p))
        return points, space, here

    def _motor_grid(self, a, a_start, a_stop, a_num, b, b_start, b_stop, b_num,
                    detectors, snake, relative, return_to_start, kind, kw):
        ia, ib = self._motor(a), self._motor(b)
        if ia == ib:
            raise ValueError("a motor mesh needs two different motors")
        here = self._here("steps")
        base_a, base_b = (here[ia], here[ib]) if relative else (0, 0)
        points = []
        for va, vb in self._grid(base_a + np.linspace(a_start, a_stop, a_num),
                                 base_b + np.linspace(b_start, b_stop, b_num), snake):
            p = list(here)
            p[ia], p[ib] = va, vb
            points.append(tuple(p))
        return self.run(points, *detectors, kind=kind, space="steps",
                        return_to=here if relative and return_to_start else None, **kw)

    def _check_limits(self, points) -> None:
        lo, hi = self.bot.status()["limits"]
        if lo == hi:            # firmware limits switched off
            return
        bad = [p for p in points if any(not lo <= s <= hi for s in p)]
        if bad:
            raise ValueError(f"{len(bad)} point(s) lie outside the firmware's "
                             f"{lo}..{hi} step limits, e.g. {bad[0]}")

    @staticmethod
    def _grid(xs, ys, snake):
        points = []
        for j, y in enumerate(ys):
            row = xs[::-1] if snake and j % 2 else xs
            points.extend((x, y) for x in row)
        return points

    def _new_dir(self, name: str) -> Optional[Path]:
        if self.data_dir is None:
            return None
        self.data_dir.mkdir(parents=True, exist_ok=True)
        numbers = [int(p.name.split("_")[1]) for p in self.data_dir.glob("scan_*")
                   if p.name.split("_")[1].isdigit()]
        safe = "".join(c if c.isalnum() or c in "-." else "_" for c in name)
        path = self.data_dir / f"scan_{max(numbers, default=0) + 1:04d}_{safe}"
        path.mkdir()
        return path

    @staticmethod
    def _write_meta(data: ScanData) -> None:
        if data.path is not None:
            (data.path / "meta.json").write_text(
                json.dumps(data.meta, indent=2, default=_json_default))

    @staticmethod
    def _safely(fn) -> None:
        try:
            fn()
        except Exception as exc:
            print(f"warning: {exc}")

    @staticmethod
    def _progress(i, n, row):
        skip = {"point", "time", "x", "y", "m0", "m1", "m2", "x_rb", "y_rb",
                "s0", "s1", "s2", "strain", "frame_id", "image"}
        extra = [(k, v) for k, v in row.items() if k not in skip][:5]
        text = "  ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}"
                         for k, v in extra)
        if "x" in row:
            where = f"x={row['x']:+.4f} y={row['y']:+.4f}"
        else:
            where = (" ".join(f"{m}={row[m]:+d}" for m in ("m0", "m1", "m2"))
                     + f"  strain={row['strain']:+.4f}")
        print(f"  {i + 1:>4}/{n}  {where}  {text}")


def _describe(d: Detector) -> str:
    fields = {k: v for k, v in vars(d).items() if k not in ("fn", "fits", "needs_camera")}
    return ", ".join(f"{k}={v!r}" for k, v in fields.items())


def _settings(fit_params: dict) -> dict:
    return {k: v for k, v in fit_params.items() if k not in ("top", "right")}


def _json_default(obj):
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, float) and math.isnan(obj):
        return None
    return str(obj)
