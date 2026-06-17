"""
gaitlib/config.py — Mounting configuration.

The mounting config makes the library AGNOSTIC to the number of sensors. It declares:

  * ``sensors``  : which sensor/node ids are present and the body segment each one is
                   mounted on (e.g. ``{"RF": "foot", "RS": "shank", ...}``).
  * ``joints``   : the joint topology — joint name -> ``(distal_node, proximal_node)``;
                   the joint angle is the orientation of the distal segment relative to
                   the proximal one (sagittal flexion).
  * ``foot_node``: the node used for gait-event detection (foot strike / toe-off).
  * ``pelvis_node`` (optional): the node used for turnaround / steady-state detection.
  * ``rates``    : declared per-channel sample rates (Hz). ``imu_hz`` covers accel+gyro;
                   ``mag_hz`` may differ — the library aligns the magnetometer to the IMU
                   rate internally (a no-op when they already match).
  * ``fusion``   : Madgwick gains + complementary-filter time constant + run modes.
  * ``strict``   : if True, a joint whose two segments are not both present raises;
                   if False (default), it is skipped with a warning.

The DEFAULT config is **4 sensors on one (right) leg**: {foot, shank, thigh, pelvis}
forming joints {ankle, knee, hip}. Both legs are supported by declaring more sensors and
more joints (see ``both_legs_config``).
"""
from __future__ import annotations

from dataclasses import dataclass, field, asdict
import copy


# Canonical node -> segment for the Geneva 8-IMU layout (used by the convenience configs).
_GENEVA_SEGMENTS = {
    "RF": "right_foot", "RS": "right_shank", "RT": "right_thigh",
    "LF": "left_foot",  "LS": "left_shank",  "LT": "left_thigh",
    "SA": "pelvis",
}


@dataclass
class FusionParams:
    """Orientation-fusion parameters (reused by the kinematic core, unchanged in substance)."""
    run_modes: tuple = ("6dof",)   # which orientation modes to compute; 6dof is primary
    beta_6dof: float = 0.033       # Madgwick gradient gain (6-DOF / IMU)
    beta_9dof: float = 0.05        # Madgwick gradient gain (9-DOF / MARG)
    joint_tau_s: float = 0.3       # complementary-filter time constant for joint angles (s)

    @classmethod
    def coerce(cls, v):
        if isinstance(v, cls):
            return v
        v = dict(v or {})
        rm = v.get("run_modes", ("6dof",))
        return cls(run_modes=tuple(rm),
                   beta_6dof=float(v.get("beta_6dof", 0.033)),
                   beta_9dof=float(v.get("beta_9dof", 0.05)),
                   joint_tau_s=float(v.get("joint_tau_s", 0.3)))


@dataclass
class MountingConfig:
    """Declares which sensors are present, their segments, and the joint topology.

    See the module docstring for field meanings. Construct directly, via the helpers
    (:func:`default_config`, :func:`both_legs_config`), or from a plain dict / YAML mapping
    with :meth:`coerce`.
    """
    sensors: dict                                  # node_id -> segment name
    joints: dict                                   # joint name -> (distal_node, proximal_node)
    foot_node: str | None = None                   # node for gait events
    pelvis_node: str | None = None                 # node for turnaround detection (optional)
    rates: dict = field(default_factory=lambda: {"imu_hz": 100.0, "mag_hz": 100.0})
    fusion: FusionParams = field(default_factory=FusionParams)
    strict: bool = False                           # hard-fail on a missing-segment joint

    # -- construction -------------------------------------------------------- #
    def __post_init__(self):
        # normalise joints to tuples; coerce nested fusion/rates
        self.joints = {k: tuple(v) for k, v in self.joints.items()}
        self.fusion = FusionParams.coerce(self.fusion)
        self.rates = dict(self.rates or {})
        self.rates.setdefault("imu_hz", 100.0)
        self.rates.setdefault("mag_hz", self.rates["imu_hz"])

    @classmethod
    def coerce(cls, v) -> "MountingConfig":
        """Accept a MountingConfig, a dict, or None (-> default)."""
        if v is None:
            return default_config()
        if isinstance(v, cls):
            return v
        v = dict(v)
        return cls(
            sensors=dict(v["sensors"]),
            joints={k: tuple(t) for k, t in v["joints"].items()},
            foot_node=v.get("foot_node"),
            pelvis_node=v.get("pelvis_node"),
            rates=dict(v.get("rates", {})),
            fusion=FusionParams.coerce(v.get("fusion", {})),
            strict=bool(v.get("strict", False)),
        )

    def to_dict(self) -> dict:
        d = asdict(self)
        d["joints"] = {k: list(v) for k, v in self.joints.items()}
        d["fusion"]["run_modes"] = list(self.fusion.run_modes)
        return d

    # -- queries ------------------------------------------------------------- #
    @property
    def imu_hz(self) -> float:
        return float(self.rates.get("imu_hz", 100.0))

    @property
    def mag_hz(self) -> float:
        return float(self.rates.get("mag_hz", self.imu_hz))

    @property
    def nodes(self) -> list:
        return list(self.sensors.keys())

    def resolve_joints(self, present_nodes):
        """Split joints into (computable, skipped) given the nodes actually present.

        A joint is computable when both of its segments are present. In strict mode a
        missing segment raises ValueError; otherwise the joint is returned in ``skipped``.
        """
        present = set(present_nodes)
        ok, skipped = {}, {}
        for name, (dist, prox) in self.joints.items():
            if dist in present and prox in present:
                ok[name] = (dist, prox)
            else:
                missing = [n for n in (dist, prox) if n not in present]
                skipped[name] = missing
        if self.strict and skipped:
            raise ValueError(
                "strict mode: missing sensors for joints "
                + ", ".join(f"{j} (need {m})" for j, m in skipped.items()))
        return ok, skipped


# --------------------------------------------------------------------------- #
def default_config(side: str = "right", *, imu_hz: float = 100.0,
                   mag_hz: float | None = None, run_modes=("6dof",)) -> MountingConfig:
    """DEFAULT mounting: 4 sensors on ONE leg -> {foot, shank, thigh, pelvis}.

    Joints: ankle (foot-shank), knee (shank-thigh), hip (thigh-pelvis). ``side`` is
    "right" (nodes RF/RS/RT/SA) or "left" (LF/LS/LT/SA).
    """
    s = "R" if side.lower().startswith("r") else "L"
    foot, shank, thigh, pelvis = f"{s}F", f"{s}S", f"{s}T", "SA"
    sensors = {foot: _GENEVA_SEGMENTS[foot], shank: _GENEVA_SEGMENTS[shank],
               thigh: _GENEVA_SEGMENTS[thigh], pelvis: _GENEVA_SEGMENTS[pelvis]}
    joints = {"ankle": (foot, shank), "knee": (shank, thigh), "hip": (thigh, pelvis)}
    return MountingConfig(
        sensors=sensors, joints=joints, foot_node=foot, pelvis_node=pelvis,
        rates={"imu_hz": imu_hz, "mag_hz": (mag_hz if mag_hz is not None else imu_hz)},
        fusion=FusionParams(run_modes=tuple(run_modes)))


def both_legs_config(*, imu_hz: float = 100.0, mag_hz: float | None = None,
                     run_modes=("6dof",)) -> MountingConfig:
    """Mounting for BOTH legs: 7 sensors (LF/LS/LT/RF/RS/RT + pelvis), 6 joints.

    Gait events are detected on the right foot by default; the left is still computed for
    its joint angles. Declaring the second leg is all it takes — the library is otherwise
    agnostic to how many sensors are present.
    """
    sensors = {n: _GENEVA_SEGMENTS[n] for n in ("RF", "RS", "RT", "LF", "LS", "LT", "SA")}
    joints = {
        "ankle_r": ("RF", "RS"), "knee_r": ("RS", "RT"), "hip_r": ("RT", "SA"),
        "ankle_l": ("LF", "LS"), "knee_l": ("LS", "LT"), "hip_l": ("LT", "SA"),
    }
    return MountingConfig(
        sensors=sensors, joints=joints, foot_node="RF", pelvis_node="SA",
        rates={"imu_hz": imu_hz, "mag_hz": (mag_hz if mag_hz is not None else imu_hz)},
        fusion=FusionParams(run_modes=tuple(run_modes)))
