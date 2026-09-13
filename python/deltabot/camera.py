"""Camera measurements for scans, through the MicroscopeViewer's remote API.

The viewer runs as its own process with its API switched on::

    python3 image_visualiser.py --serve

and this module drives it from the DeltaBot side: after each stage move it
makes sure the numbers it reads - Hough circles, Gaussian fits, the image
itself - come from a frame that was taken *after* the move, not one still
sitting in the pipeline from before it.

How that frame is obtained depends on what the viewer is doing:

* **not streaming** - ``grab`` one frame.  Slower (the camera is opened for
  every frame) but completely deterministic.
* **streaming** - let ``discard`` frames go past, then take the next.

Either way the frame is then read with ``measure``, which copies its
results and image in one step and fits *that frame's* projections away from
the viewer's GUI thread, so the fits match the circles and image exactly.

``viewer_api`` needs only the standard library and NumPy, so this imports it
straight from a checkout of https://github.com/garethnisbet/MicroscopeViewer:
a sibling ``MicroscopeViewer`` (or ``CameraViewer``) directory next to this
repository, or wherever ``CAMERA_VIEWER_PATH`` points.
"""

import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np


def _viewer_api():
    try:
        import viewer_api
    except ImportError:
        parent = Path(__file__).resolve().parents[3]
        env = os.environ.get("CAMERA_VIEWER_PATH")
        candidates = ([Path(env)] if env else
                      [parent / "MicroscopeViewer", parent / "CameraViewer"])
        path = next((p for p in candidates if (p / "viewer_api.py").is_file()), None)
        if path is None:
            raise ImportError(
                "cannot find viewer_api.py in "
                + " or ".join(str(p) for p in candidates)
                + " - clone https://github.com/garethnisbet/MicroscopeViewer next "
                "to this repository, or set CAMERA_VIEWER_PATH") from None
        sys.path.insert(0, str(path))
        import viewer_api
    return viewer_api


@dataclass
class Frame:
    """One analysed camera frame, as a scan point sees it."""

    frame_id: int
    time: float                      # viewer's time.time() for the frame
    hough: Optional[object]          # HoughResult, or None with detection off
    fits: dict = field(default_factory=dict)   # {"top": FitResult|None, ...}
    image: Optional[np.ndarray] = None


class Camera:
    """A served MicroscopeViewer, used as a detector.

        cam = Camera()                        # 127.0.0.1:8765
        cam.remote.enable_hough(min_radius=20, max_radius=60)
        frame = cam.acquire(hough=True, fit="both", image="raw")

    ``cam.remote`` is the underlying ``viewer_api.RemoteViewer`` for anything
    else - setting parameters, starting the stream, reading results.
    """

    def __init__(self, host: str = "127.0.0.1", port: Optional[int] = None,
                 name: Optional[str] = None, discard: int = 2,
                 timeout: float = 10.0, fit_timeout: float = 120.0):
        api = _viewer_api()
        self.error = api.ViewerAPIError
        self.remote = api.connect(host=host, port=port or api.DEFAULT_PORT,
                                  name=name)
        # Stream frames already buffered by the driver when the move ends
        # would otherwise be read as the new position.
        self.discard = discard
        self.timeout = timeout
        self.fit_timeout = fit_timeout

    def __repr__(self) -> str:
        return f"Camera({self.remote!r})"

    @property
    def streaming(self) -> bool:
        return self.remote.frame_info()["streaming"]

    def prepare(self, hough: bool = False) -> None:
        """Check the viewer can supply what a scan will ask for.  Detection
        must already be on - its parameters are the user's to set."""
        if hough and not self.remote.hough_params()["enabled"]:
            raise self.error(
                "Hough detection is off in the viewer - set it up first, e.g. "
                "cam.remote.enable_hough(min_radius=20, max_radius=60)")

    def acquire(self, hough: bool = False, fit: Optional[str] = None,
                image: Optional[str] = None) -> Frame:
        """Analyse a frame taken after this call and return it.

        *fit* is ``"top"``, ``"right"``, ``"both"`` or ``None``; *image* is
        ``"grey"``, ``"raw"`` or ``None`` for no image.
        """
        info = self.remote.frame_info()
        if info["streaming"]:
            self.remote.wait_for_frame(frames=self.discard + 1,
                                       after=info["frame_id"],
                                       timeout=self.timeout)
        else:
            info = self.remote.grab()

        snap = self.remote.measure(fit=fit, include_image=image is not None,
                                   raw=image == "raw", timeout=self.fit_timeout)
        if not info["streaming"] and snap["frame_id"] != info["frame_id"]:
            raise self.error(
                f"the viewer loaded another image (frame {snap['frame_id']}) "
                f"while frame {info['frame_id']} was being measured")
        if hough and snap["hough"] is None:
            raise self.error("Hough detection was switched off mid-scan")
        return Frame(frame_id=snap["frame_id"], time=snap["frame_time"],
                     hough=snap["hough"], fits=snap["fits"],
                     image=snap.get("image"))
