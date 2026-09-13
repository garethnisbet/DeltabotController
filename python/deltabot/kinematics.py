"""Kinematics for the DeltaBot's flexure stage.

The mechanism is *over-constrained*: three screw jacks drive a platform that
has two useful degrees of freedom.  Each screw pushes on a compliant leg that
converts its vertical travel into an in-plane displacement of the top plate
along that leg's own direction, so the platform translates in X and Y.  Z is
not commanded - it follows as a second order consequence of moving off centre,
the way the end of a link dips as it swings.

Writing the three leg directions as unit vectors ``u_i`` at 120 degrees and the
screw extensions as ``h_i`` mm, each leg constrains the platform displacement
``d`` along its own direction:

    d . u_i = gain * h_i

Three equations for two unknowns, which is the over-constraint.  Inverting them
is trivial - ``h_i = (d . u_i) / gain`` - and the three extensions it produces
always sum to zero.  Going forwards needs a least squares fit, and because the
three unit vectors are 120 degrees apart that comes out as

    d = (2/3) * gain * sum_i h_i * u_i

The redundant combination is the common mode ``h_1 = h_2 = h_3``: it satisfies
none of the three constraints at once, so instead of moving the platform it
drives all three legs together and strains the flexures against each other.
The inverse above puts exactly zero into it.  Driving a single screw on its own
does the opposite: two thirds of its travel becomes motion, one third goes into
fighting the other two legs.

The Z estimate is a small-angle model, ``z = -(x^2 + y^2) / (2 L)`` for an
effective leg length ``L``, plus any common mode that has been dialled in.  It
is an estimate for information, not a controlled axis.
"""

from dataclasses import dataclass
from math import atan2, cos, degrees, hypot, radians, sin
from typing import Sequence, Tuple

from .config import Geometry


@dataclass
class Pose:
    """Platform position in the plane, in mm from the zero position."""

    x: float
    y: float

    @property
    def r(self) -> float:
        """Distance from centre, mm."""
        return hypot(self.x, self.y)

    @property
    def theta(self) -> float:
        """Direction from centre, degrees counter-clockwise from +X."""
        return degrees(atan2(self.y, self.x))

    def __str__(self) -> str:
        return f"x={self.x:+.4f} mm  y={self.y:+.4f} mm  (r={self.r:.4f} mm, {self.theta:+.1f} deg)"


def leg_directions(geom: Geometry) -> Tuple[Tuple[float, float], ...]:
    """In-plane direction each screw drives the platform along."""
    return tuple((cos(radians(t)), sin(radians(t))) for t in geom.screw_angles_deg)


def screws_to_pose(heights: Sequence[float], geom: Geometry) -> Pose:
    """Forward kinematics: three screw extensions (mm) -> platform X, Y."""
    if len(heights) != 3:
        raise ValueError("need exactly three screw extensions")
    g = (2.0 / 3.0) * geom.lateral_gain
    x = g * sum(h * u[0] for h, u in zip(heights, leg_directions(geom)))
    y = g * sum(h * u[1] for h, u in zip(heights, leg_directions(geom)))
    return Pose(x, y)


def pose_to_screws(pose: Pose, geom: Geometry) -> Tuple[float, float, float]:
    """Inverse kinematics: platform X, Y -> three screw extensions (mm).

    Each leg simply has to follow the platform along its own direction.  The
    three extensions sum to zero, so none of the command goes into straining
    the over-constrained direction.
    """
    g = geom.lateral_gain
    if g == 0:
        raise ValueError("lateral_gain must not be zero")
    return tuple((pose.x * u[0] + pose.y * u[1]) / g for u in leg_directions(geom))


def common_mode(heights: Sequence[float]) -> float:
    """The over-constrained part of a set of screw extensions, in mm.

    Zero for anything :func:`pose_to_screws` produces.  A non-zero value means
    the three legs are being pushed together: preload, not motion.
    """
    return sum(heights) / 3.0


def estimate_z(pose: Pose, geom: Geometry, heights: Sequence[float] = ()) -> float:
    """Second order estimate of how far the platform has risen or fallen, mm."""
    z = -(pose.x ** 2 + pose.y ** 2) / (2.0 * geom.leg_length_mm)
    if len(heights) == 3:
        z += geom.common_mode_gain * common_mode(heights)
    return z


def reachable(pose: Pose, geom: Geometry) -> bool:
    """Is this position inside the configured working radius?"""
    return pose.r <= geom.max_radius_mm + 1e-9
