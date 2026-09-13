"""Remote control for the DeltaBot in IPython, with the camera viewer as a detector.

    cd python
    ipython -i -m deltabot.session                  # auto-detect the Nano
    ipython -i -m deltabot.session -- --port sim    # simulator

Start the viewer first, in its own terminal, so the session can find it::

    python3 image_visualiser.py --serve

The prompt then has:

    bot                     the DeltaBot
    cam                     the viewer (cam.remote is its full remote API), or None
    wh()  mv(x, y)  mvr(dx, dy)                   plane, mm
    wm()  mvm(m0, m1, m2)  mvrm(m0, m1, m2)       raw motors, steps
    zero()                                        current position becomes the origin
    ascan  dscan  mesh  dmesh  mmesh  dmmesh  run_scan  count
    Circles  Peaks  SaveImage  Custom
    load_scan  connect_camera  scanner
"""

import argparse
import atexit

from .camera import Camera
from .config import Geometry
from .controller import DeltaBot
from .kinematics import common_mode, screws_to_pose
from .scan import Circles, Custom, Peaks, SaveImage, Scanner, ScanData, load_scan


def main(argv=None) -> dict:
    ap = argparse.ArgumentParser(description="DeltaBot IPython session")
    ap.add_argument("--port", default="auto", help="serial port, 'auto' or 'sim'")
    ap.add_argument("--gain", type=float, help="override lateral_gain")
    ap.add_argument("--viewer-host", default="127.0.0.1")
    ap.add_argument("--viewer-port", type=int, default=None)
    ap.add_argument("--viewer-name", default=None,
                    help="viewer to use when several are open")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--data-dir", default="scans")
    ap.add_argument("--settle", type=float, default=0.2,
                    help="seconds to wait after each move before measuring")
    opts = ap.parse_args(argv)

    geometry = Geometry()
    if opts.gain is not None:
        geometry.lateral_gain = opts.gain
    bot = DeltaBot(opts.port, geometry)
    atexit.register(bot.close)
    scanner = Scanner(bot, data_dir=opts.data_dir, settle=opts.settle)

    def connect_camera(host=opts.viewer_host, port=opts.viewer_port,
                       name=opts.viewer_name, **kw):
        """Connect (or reconnect) to a served viewer and use it for scans."""
        scanner.camera = Camera(host=host, port=port, name=name, **kw)
        return scanner.camera

    cam = None
    if not opts.no_camera:
        try:
            cam = connect_camera()
        except Exception as exc:
            print(f"no camera viewer: {exc}\n"
                  "  start it with 'python3 image_visualiser.py --serve', "
                  "then: cam = connect_camera()")

    def wh():
        """Where is the stage."""
        ext = bot.extensions()
        print(f"  {bot.pose()}   z~{bot.z():+.4f} mm")
        print("  screws  " + "  ".join(f"{i}:{h:+.4f} mm" for i, h in enumerate(ext))
              + f"   preload {common_mode(ext):+.4f} mm")

    def mv(x=None, y=None):
        """Move to an absolute position, mm."""
        bot.move_xy(x, y)
        wh()

    def mvr(dx=0.0, dy=0.0):
        """Move by a relative distance, mm."""
        bot.nudge_xy(dx, dy)
        wh()

    def wm():
        """Where are the raw motors: steps, screw extension, and the result."""
        steps = bot.positions()
        ext = [bot.geometry.steps_to_mm(i, s) for i, s in enumerate(steps)]
        print("  motors  " + "  ".join(f"m{i}:{s:+d}" for i, s in enumerate(steps)))
        print("  screws  " + "  ".join(f"m{i}:{h:+.4f} mm" for i, h in enumerate(ext))
              + f"   preload {common_mode(ext):+.4f} mm")
        print(f"  {screws_to_pose(ext, bot.geometry)}")

    def mvm(m0=None, m1=None, m2=None):
        """Move raw motors to absolute step positions; omitted motors stay put.

        No kinematics: unless the three steps sum to zero this strains the
        flexures - wm() shows the preload it leaves."""
        target = [c if t is None else t
                  for c, t in zip(bot.positions(), (m0, m1, m2))]
        bot.move_steps(*target)
        wm()

    def mvrm(m0=0, m1=0, m2=0):
        """Move raw motors by a relative number of steps."""
        bot.move_steps(m0, m1, m2, relative=True)
        wm()

    def zero():
        """Make the current position the origin: motors 0 steps, x = y = 0.

        The machine has no endstops, so this is its only reference.  Any
        preload present (from raw motor moves) becomes part of the new zero
        and stays in every later move."""
        steps = bot.positions()
        ext = [bot.geometry.steps_to_mm(i, s) for i, s in enumerate(steps)]
        strain = common_mode(ext)
        bot.zero()
        print("  zeroed; was motors " + "  ".join(f"m{i}:{s:+d}" for i, s in enumerate(steps))
              + f"   {screws_to_pose(ext, bot.geometry)}")
        if abs(strain) > 1e-6:
            print(f"  note: {strain:+.4f} mm of preload is now part of the zero")
        wm()

    print(f"DeltaBot on {bot.conn.port}: {bot.banner}")
    print(f"camera: {cam if cam else 'not connected'}   scans -> {opts.data_dir}/")
    print(__doc__.split("The prompt then has:")[1].rstrip())
    return dict(
        bot=bot, cam=cam, scanner=scanner, connect_camera=connect_camera,
        wh=wh, mv=mv, mvr=mvr, wm=wm, mvm=mvm, mvrm=mvrm, zero=zero,
        ascan=scanner.ascan, dscan=scanner.dscan, mesh=scanner.mesh,
        dmesh=scanner.dmesh, mmesh=scanner.mmesh, dmmesh=scanner.dmmesh,
        run_scan=scanner.run, count=scanner.count,
        Circles=Circles, Peaks=Peaks, SaveImage=SaveImage, Custom=Custom,
        ScanData=ScanData, load_scan=load_scan,
    )


if __name__ == "__main__":
    globals().update(main())
