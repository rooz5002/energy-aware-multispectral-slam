"""Energy-aware scheduler (paper Fig. 2), with the rules fixed in docs/DECISIONS.md.

    if Q_RGB < tau_Q and thermal quality adequate: increase thermal weight / thermal features
    if   E_avail >= tau_E1:  Level 0 (Normal)
    elif E_avail >= tau_E2:  Level 1 (Energy saving)
    else:                    Level 2 (Tracking priority)

All Q values passed in are NORMALISED by the modality's clear-condition mean (D6), so one tau_Q
serves both modalities.

- Levels: progressive (one level per transition), hysteresis band, minimum dwell time.
- Thermal priority (D8): while Q_RGB < tau_Q and Q_TH >= tau_Q, omega_TH >= thermal_priority_weight.
- Thermal sensor activation u_TH (D9): thermal is switched off at Level 1 or 2 once Q_RGB has stayed
  >= tau_Q + band for the minimum dwell time; it is switched on again immediately when Q_RGB < tau_Q,
  on tracking loss, or when the scheduler returns to Level 0.
- Recovery override (D10): after a tracking loss the Level 0 feature budget is used and thermal is forced
  on until tracking has been OK for recovery_hold_s.
Every change of state is recorded in `transitions`.
"""
import math
from dataclasses import dataclass, field

from .weights import feature_split, weights


@dataclass(frozen=True)
class LevelSettings:
    n_features: int             # total ORB features per frame
    min_keyframe_interval: int  # frames; 0 = ORB-SLAM3 default policy
    local_ba: bool
    loop_closure: str           # "full" | "reduced" | "off"
    local_ba_every: int = 1     # local BA for one keyframe in this many
    frame_stride: int = 1       # process one frame in this many (2 halves the processing rate)


@dataclass(frozen=True)
class SchedulerParams:
    tau_Q: float                # on the normalised quality scale
    tau_E1: float               # fraction of E0
    tau_E2: float               # fraction of E0
    levels: tuple               # (LevelSettings, LevelSettings, LevelSettings)
    hysteresis: float = 0.02    # band: fraction of E0 for energy, absolute on normalised Q
    min_dwell_s: float = 2.0
    thermal_priority_weight: float = 0.7
    n_min_per_modality: int = 50
    thermal_off_rule: bool = False
    recovery_hold_s: float = 2.0


@dataclass
class Control:
    """Values applied to SLAM for one frame (guide section 6.4)."""
    level: int
    n_features_total: int
    n_features: dict
    omega: dict
    min_keyframe_interval: int
    local_ba_enabled: bool
    loop_closure_mode: str
    local_ba_every: int
    enabled: dict               # sensor activation u_m for this frame
    thermal_priority: bool
    recovery: bool


def _known(x):
    return x is not None and not (isinstance(x, float) and math.isnan(x))


@dataclass
class Scheduler:
    p: SchedulerParams
    adaptive_weights: bool = True      # False: fixed weights (static / scheduling-only ablation)
    energy_scheduling: bool = True     # False: Level 0 always (static / weighting-only ablation)
    fixed_omega: dict = None
    level: int = 0
    thermal_priority: bool = False
    thermal_on: bool = True
    recovery: bool = False
    _last_change_t: float = None
    _last_priority_t: float = None
    _rgb_good_since: float = None
    _ok_since: float = None
    transitions: list = field(default_factory=list)

    def _log(self, t, kind, old, new, **extra):
        self.transitions.append(dict(t=t, kind=kind, old=old, new=new, **extra))

    def _target_level(self, e_frac):
        p, h = self.p, self.p.hysteresis
        # moving to a lower-energy level happens at the threshold; moving back needs threshold + band
        if self.level == 0:
            return 0 if e_frac >= p.tau_E1 else 1
        if self.level == 1:
            if e_frac >= p.tau_E1 + h:
                return 0
            return 1 if e_frac >= p.tau_E2 else 2
        return 1 if e_frac >= p.tau_E2 + h else 2

    def frame_stride(self) -> int:
        """Processing-rate divider for the coming frame (1 during the recovery override)."""
        return 1 if self.recovery else self.p.levels[self.level].frame_stride

    def modalities_in_use(self, active: dict) -> dict:
        """Which modalities deliver an image for the coming frame (before step() is called)."""
        return dict(active, thermal=active.get("thermal", False) and self.thermal_on)

    def step(self, t: float, q: dict, e_frac: float, active: dict, tracking_ok: bool = True) -> Control:
        """t: time (s); q: modality -> normalised Q (None/NaN if not available); e_frac: E_avail / E0;
        active: modalities the configuration uses; tracking_ok: the previous frame was tracked."""
        p = self.p
        both = active.get("rgb", False) and active.get("thermal", False)

        # ---- energy level
        if self.energy_scheduling:
            target = self._target_level(e_frac)
            dwell_ok = self._last_change_t is None or t - self._last_change_t >= p.min_dwell_s
            if target != self.level and dwell_ok:
                new = self.level + (1 if target > self.level else -1)  # progressive: one level at a time
                self._log(t, "level", self.level, new, e_frac=e_frac)
                self.level, self._last_change_t = new, t
        lv = p.levels[self.level]

        # ---- recovery override (D10)
        if self.energy_scheduling:
            if not tracking_ok:
                if not self.recovery:
                    self._log(t, "recovery", False, True, level=self.level)
                self.recovery, self._ok_since = True, None
            elif self.recovery:
                if self._ok_since is None:
                    self._ok_since = t
                if t - self._ok_since >= p.recovery_hold_s:
                    self._log(t, "recovery", True, False, level=self.level)
                    self.recovery = False

        # ---- thermal sensor activation u_TH (D9)
        if both and self.energy_scheduling and p.thermal_off_rule:
            q_rgb = q.get("rgb")
            good = _known(q_rgb) and q_rgb >= p.tau_Q + p.hysteresis
            self._rgb_good_since = (self._rgb_good_since if self._rgb_good_since is not None else t) if good else None
            if not self.thermal_on:
                if not tracking_ok or self.recovery or self.level == 0 or not _known(q_rgb) or q_rgb < p.tau_Q:
                    self._log(t, "thermal_power", False, True, level=self.level, q_rgb=q_rgb, tracking_ok=tracking_ok)
                    self.thermal_on = True
            elif (self.level >= 1 and tracking_ok and not self.recovery and good
                  and t - self._rgb_good_since >= p.min_dwell_s):
                self._log(t, "thermal_power", True, False, level=self.level, q_rgb=q_rgb)
                self.thermal_on, self.thermal_priority = False, False
        elif not self.thermal_on:
            self.thermal_on = True
        enabled = self.modalities_in_use(active)

        # ---- weights
        if enabled.get("rgb") and enabled.get("thermal"):
            if not self.adaptive_weights:
                omega = dict(self.fixed_omega)
            elif not (_known(q.get("rgb")) and _known(q.get("thermal"))):
                omega = {"rgb": 0.5, "thermal": 0.5}   # thermal was just switched on: no thermal quality yet
            else:
                omega = weights(q, enabled)
                # hysteresis on tau_Q: enter below tau_Q, leave above tau_Q + band; minimum dwell on the switch
                degraded = q["rgb"] < (p.tau_Q + p.hysteresis if self.thermal_priority else p.tau_Q)
                prio = degraded and q["thermal"] >= p.tau_Q
                dwell_ok = self._last_priority_t is None or t - self._last_priority_t >= p.min_dwell_s
                if prio != self.thermal_priority and dwell_ok:
                    self._log(t, "thermal_priority", self.thermal_priority, prio, q_rgb=q["rgb"], q_thermal=q["thermal"])
                    self.thermal_priority, self._last_priority_t = prio, t
                if self.thermal_priority and omega["thermal"] < p.thermal_priority_weight:
                    omega = {"thermal": p.thermal_priority_weight, "rgb": 1.0 - p.thermal_priority_weight}
        else:
            omega = {m: (1.0 if enabled.get(m) else 0.0) for m in ("rgb", "thermal")}

        n_total = p.levels[0].n_features if self.recovery else lv.n_features
        return Control(level=self.level, n_features_total=n_total,
                       n_features=feature_split(omega, n_total, enabled, p.n_min_per_modality),
                       omega=omega, min_keyframe_interval=lv.min_keyframe_interval, local_ba_enabled=lv.local_ba,
                       loop_closure_mode=lv.loop_closure, local_ba_every=lv.local_ba_every, enabled=enabled,
                       thermal_priority=self.thermal_priority, recovery=self.recovery)
