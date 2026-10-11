import numpy as np
import pytest

from controller import quality
from controller.energy import EnergyAccount, EnergyCounterError, RaplReader
from controller.scheduler import LevelSettings, Scheduler, SchedulerParams
from controller.thermal import ThermalPreprocessor
from degradation.thermal_sensor import ThermalSensorModel
from controller.weights import feature_split, weights

LEVELS = (LevelSettings(1200, 0, True, "full"), LevelSettings(800, 15, True, "reduced"),
          LevelSettings(500, 30, True, "off"))
BOTH = {"rgb": True, "thermal": True}


def params(**kw):
    base = dict(tau_Q=0.45, tau_E1=0.6, tau_E2=0.3, levels=LEVELS, hysteresis=0.02, min_dwell_s=2.0)
    base.update(kw)
    return SchedulerParams(**base)


def textured(seed=0, size=(240, 320)):
    return np.random.default_rng(seed).integers(0, 256, size, dtype=np.uint8)


# ----------------------------------------------------------------- quality
def test_entropy_constant_image_is_zero():
    assert quality.entropy_score(np.full((240, 320), 77, np.uint8)) == 0.0


def test_entropy_bounds():
    assert quality.entropy_score(textured()) == pytest.approx(1.0, abs=0.01)
    two = np.zeros((10, 10), np.uint8)
    two[:, 5:] = 255
    assert quality.entropy_score(two) == pytest.approx(1 / 8)


def test_blur_sign_sharper_is_lower():
    import cv2
    sharp = textured()
    blurred = cv2.GaussianBlur(sharp, (0, 0), 3.0)
    v_ref = quality.variance_of_laplacian(sharp)
    b_sharp, _ = quality.blur_score(sharp, v_ref)
    b_blur, _ = quality.blur_score(blurred, v_ref)
    assert b_sharp < b_blur
    assert b_sharp == 0.0 and 0.0 < b_blur <= 1.0


@pytest.mark.parametrize("count,budget,expected", [(0, 500, 0.0), (250, 500, 0.5), (900, 500, 1.0), (10, 0, 0.0)])
def test_feature_score_normalised(count, budget, expected):
    assert quality.feature_score(count, budget) == expected


def test_quality_formula_and_bounds():
    p = quality.QualityParams(alpha=0.4, beta=0.4, gamma=0.2, v_ref={"rgb": 1e9})
    q = quality.assess(textured(), 300, 600, "rgb", p)
    assert q.Q == pytest.approx(0.4 * q.N + 0.4 * q.H - 0.2 * q.B)
    for v in (q.N, q.H, q.B):
        assert 0.0 <= v <= 1.0
    assert -p.gamma <= q.Q <= p.alpha + p.beta


def test_quality_rejects_colour_input():
    p = quality.QualityParams(0.4, 0.4, 0.2, {"rgb": 100.0})
    with pytest.raises(ValueError):
        quality.assess(np.zeros((4, 4, 3), np.uint8), 1, 1, "rgb", p)


# ----------------------------------------------------------------- weights
@pytest.mark.parametrize("q", [{"rgb": 0.7, "thermal": 0.3}, {"rgb": 0.01, "thermal": 0.9},
                               {"rgb": -0.1, "thermal": 0.4}, {"rgb": 0.0, "thermal": 0.0},
                               {"rgb": -0.2, "thermal": -0.1}])
def test_weights_sum_to_one(q):
    w = weights(q, BOTH)
    assert sum(w.values()) == pytest.approx(1.0)
    assert all(0.0 <= v <= 1.0 for v in w.values())


def test_weights_proportional_and_inactive():
    assert weights({"rgb": 0.6, "thermal": 0.2}, BOTH) == pytest.approx({"rgb": 0.75, "thermal": 0.25})
    assert weights({"rgb": 0.6, "thermal": 0.2}, {"rgb": True, "thermal": False}) == {"rgb": 1.0, "thermal": 0.0}
    assert weights({"rgb": 0.0, "thermal": 0.0}, BOTH) == {"rgb": 0.5, "thermal": 0.5}


@pytest.mark.parametrize("w_rgb", [0.0, 0.01, 0.3, 0.5, 0.97, 1.0])
@pytest.mark.parametrize("n_total", [500, 800, 1200])
def test_feature_split_sums_and_minimum(w_rgb, n_total):
    n = feature_split({"rgb": w_rgb, "thermal": 1 - w_rgb}, n_total, BOTH, n_min=50)
    assert n["rgb"] + n["thermal"] == n_total
    assert min(n.values()) >= 50


def test_feature_split_single_modality():
    assert feature_split({"rgb": 1.0, "thermal": 0.0}, 1200, {"rgb": True, "thermal": False}) == \
        {"rgb": 1200, "thermal": 0}


# --------------------------------------------------------------- scheduler
GOOD = {"rgb": 0.6, "thermal": 0.6}


@pytest.mark.parametrize("e,level", [(0.61, 0), (0.60, 0), (0.59, 1)])
def test_level_at_above_below_tau_e1(e, level):
    s = Scheduler(params(min_dwell_s=0.0))
    assert s.step(0.0, GOOD, e, BOTH).level == level


@pytest.mark.parametrize("e,level", [(0.31, 1), (0.30, 1), (0.29, 2)])
def test_level_at_above_below_tau_e2(e, level):
    s = Scheduler(params(min_dwell_s=0.0))
    s.step(0.0, GOOD, 0.5, BOTH)  # now in level 1
    assert s.step(1.0, GOOD, e, BOTH).level == level


def test_progressive_one_level_per_transition():
    s = Scheduler(params(min_dwell_s=0.0))
    assert s.step(0.0, GOOD, 0.1, BOTH).level == 1   # target is 2, but only one step at a time
    assert s.step(0.1, GOOD, 0.1, BOTH).level == 2


def test_hysteresis_band_on_recovery():
    s = Scheduler(params(min_dwell_s=0.0))
    s.step(0.0, GOOD, 0.59, BOTH)
    assert s.step(1.0, GOOD, 0.61, BOTH).level == 1    # inside the band: stay
    assert s.step(2.0, GOOD, 0.62, BOTH).level == 0    # threshold + band reached
    s.step(3.0, GOOD, 0.5, BOTH)
    s.step(4.0, GOOD, 0.2, BOTH)
    assert s.level == 2
    assert s.step(5.0, GOOD, 0.31, BOTH).level == 2
    assert s.step(6.0, GOOD, 0.32, BOTH).level == 1


def test_min_dwell_time():
    s = Scheduler(params(min_dwell_s=2.0))
    assert s.step(0.0, GOOD, 0.5, BOTH).level == 1
    assert s.step(1.0, GOOD, 0.1, BOTH).level == 1     # too soon
    assert s.step(2.0, GOOD, 0.1, BOTH).level == 2
    assert [tr["new"] for tr in s.transitions if tr["kind"] == "level"] == [1, 2]


def test_level_settings_applied():
    s = Scheduler(params(min_dwell_s=0.0))
    c = s.step(0.0, GOOD, 0.9, BOTH)
    assert (c.n_features_total, c.loop_closure_mode, c.min_keyframe_interval) == (1200, "full", 0)
    c = s.step(1.0, GOOD, 0.5, BOTH)
    assert (c.n_features_total, c.loop_closure_mode, c.min_keyframe_interval) == (800, "reduced", 15)
    c = s.step(2.0, GOOD, 0.1, BOTH)
    assert (c.n_features_total, c.loop_closure_mode, c.local_ba_enabled) == (500, "off", True)
    assert sum(c.n_features.values()) == 500


def test_scheduler_off_stays_level_zero():
    s = Scheduler(params(), energy_scheduling=False)
    assert s.step(0.0, GOOD, 0.01, BOTH).level == 0


def test_thermal_priority_when_rgb_degraded():
    s = Scheduler(params())
    c = s.step(0.0, {"rgb": 0.44, "thermal": 0.50}, 0.9, BOTH)
    assert c.thermal_priority and c.omega["thermal"] == pytest.approx(0.7)
    assert sum(c.omega.values()) == pytest.approx(1.0)
    assert c.n_features["thermal"] > c.n_features["rgb"]
    # at the threshold RGB is not "below tau_Q", but the band keeps priority on until tau_Q + band
    assert s.step(1.0, {"rgb": 0.46, "thermal": 0.50}, 0.9, BOTH).thermal_priority
    assert not s.step(2.0, {"rgb": 0.48, "thermal": 0.50}, 0.9, BOTH).thermal_priority


def test_thermal_priority_minimum_dwell():
    s = Scheduler(params(min_dwell_s=2.0))
    assert s.step(0.0, {"rgb": 0.30, "thermal": 0.60}, 0.9, BOTH).thermal_priority
    assert s.step(0.5, {"rgb": 0.60, "thermal": 0.60}, 0.9, BOTH).thermal_priority       # too soon to switch back
    assert not s.step(2.0, {"rgb": 0.60, "thermal": 0.60}, 0.9, BOTH).thermal_priority
    assert not s.step(2.5, {"rgb": 0.30, "thermal": 0.60}, 0.9, BOTH).thermal_priority   # and too soon to re-enter
    assert s.step(4.0, {"rgb": 0.30, "thermal": 0.60}, 0.9, BOTH).thermal_priority


def test_no_thermal_priority_at_threshold_or_if_thermal_inadequate():
    assert not Scheduler(params()).step(0.0, {"rgb": 0.45, "thermal": 0.6}, 0.9, BOTH).thermal_priority
    c = Scheduler(params()).step(0.0, {"rgb": 0.2, "thermal": 0.3}, 0.9, BOTH)
    assert not c.thermal_priority
    assert c.omega == pytest.approx({"rgb": 0.4, "thermal": 0.6})


def test_thermal_priority_keeps_larger_eq2_weight():
    c = Scheduler(params()).step(0.0, {"rgb": 0.05, "thermal": 0.6}, 0.9, BOTH)
    assert c.omega["thermal"] == pytest.approx(0.6 / 0.65)


def test_fixed_weights_ignore_quality():
    s = Scheduler(params(), adaptive_weights=False, fixed_omega={"rgb": 0.5, "thermal": 0.5})
    c = s.step(0.0, {"rgb": 0.01, "thermal": 0.9}, 0.9, BOTH)
    assert c.omega == {"rgb": 0.5, "thermal": 0.5} and not c.thermal_priority
    assert c.n_features == {"rgb": 600, "thermal": 600}


# ------------------------------------------------------------------ energy
def test_energy_account_integrates():
    acc = EnergyAccount(100.0, {"rgb": 1.0, "thermal": 2.0, "depth": 0.5})
    p = acc.update(2.0, 20.0, {"rgb": True, "thermal": False, "depth": True})
    assert p == pytest.approx(1.5 + 10.0)
    assert acc.consumed == pytest.approx(23.0)
    assert acc.available == pytest.approx(77.0) and acc.fraction == pytest.approx(0.77)


def test_rapl_reader_wraparound_and_missing(tmp_path):
    with pytest.raises(EnergyCounterError):
        RaplReader(tmp_path)
    z = tmp_path / "intel-rapl:0"
    z.mkdir()
    (z / "name").write_text("package-0\n")
    (z / "max_energy_range_uj").write_text("1000000\n")
    (z / "energy_uj").write_text("900000\n")
    sub = tmp_path / "intel-rapl:0:0"   # sub-zone must not be double counted
    sub.mkdir()
    (sub / "name").write_text("core\n")
    r = RaplReader(tmp_path)
    (z / "energy_uj").write_text("950000\n")
    assert r.joules() == pytest.approx(0.05)
    (z / "energy_uj").write_text("100000\n")   # wrapped
    assert r.joules() == pytest.approx(0.05 + 0.15)   # 50 mJ to the top of the range, then 100 mJ


# ----------------------------------------------------------------- thermal
def test_thermal_fixed_window():
    pre = ThermalPreprocessor(280.0, 330.0, clahe=False)
    raw = np.array([[27000, 28000, 30500, 33000, 40000]], np.uint16)
    assert pre(raw).tolist() == [[0, 0, 128, 255, 255]]   # 305 K is mid-window (127.5, rounded)
    # fixed window: the same temperature maps to the same value whatever else is in the frame
    assert pre(np.array([[30500, 30500]], np.uint16))[0, 0] == 128


# ------------------------------------------------- D7: blur indicator under noise
def test_prefiltered_blur_not_fooled_by_noise():
    import cv2
    rng = np.random.default_rng(0)
    base = cv2.GaussianBlur(textured(), (0, 0), 1.5)
    v_ref = quality.variance_of_laplacian(base)
    blurred = cv2.GaussianBlur(base, (0, 0), 3.0)
    noisy_blurred = np.clip(blurred + rng.normal(0, 4, blurred.shape), 0, 255).astype(np.uint8)
    b_blur, _ = quality.blur_score(noisy_blurred, v_ref)
    assert b_blur > 0.5


# ------------------------------------------------- D9: thermal sensor activation
def rule_params(**kw):
    return params(thermal_off_rule=True, min_dwell_s=2.0, tau_Q=0.6, **kw)


def test_thermal_stays_on_at_level_zero():
    s = Scheduler(rule_params())
    for t in range(10):
        c = s.step(float(t), {"rgb": 1.0, "thermal": 1.0}, 0.9, BOTH)
    assert c.enabled["thermal"] and c.level == 0


def test_thermal_off_after_dwell_at_level_one_and_back_on():
    s = Scheduler(rule_params())
    good = {"rgb": 1.0, "thermal": 1.0}
    assert s.step(0.0, good, 0.5, BOTH).enabled["thermal"]          # level 1 now, RGB good since t=0
    assert s.step(1.9, good, 0.5, BOTH).enabled["thermal"]          # dwell not reached
    c = s.step(2.0, good, 0.5, BOTH)
    assert not c.enabled["thermal"] and c.omega == {"rgb": 1.0, "thermal": 0.0}
    assert c.n_features == {"rgb": 800, "thermal": 0}
    assert s.modalities_in_use(BOTH) == {"rgb": True, "thermal": False}
    # RGB degrades: thermal is back immediately, with equal weights until thermal quality is known
    c = s.step(2.1, {"rgb": 0.5, "thermal": None}, 0.5, BOTH)
    assert c.enabled["thermal"] and c.omega == {"rgb": 0.5, "thermal": 0.5}
    assert [tr["new"] for tr in s.transitions if tr["kind"] == "thermal_power"] == [False, True]


def test_thermal_off_needs_continuous_good_rgb_and_band():
    s = Scheduler(rule_params())
    s.step(0.0, {"rgb": 1.0, "thermal": 1.0}, 0.5, BOTH)
    s.step(1.5, {"rgb": 0.61, "thermal": 1.0}, 0.5, BOTH)           # inside the band: resets the timer
    assert s.step(2.5, {"rgb": 1.0, "thermal": 1.0}, 0.5, BOTH).enabled["thermal"]
    assert not s.step(4.5, {"rgb": 1.0, "thermal": 1.0}, 0.5, BOTH).enabled["thermal"]


def test_thermal_on_at_tracking_loss_and_rule_can_be_disabled():
    s = Scheduler(rule_params())
    good = {"rgb": 1.0, "thermal": None}
    s.step(0.0, good, 0.5, BOTH); s.step(2.0, good, 0.5, BOTH)
    assert not s.thermal_on
    assert s.step(2.1, good, 0.5, BOTH, tracking_ok=False).enabled["thermal"]
    off = Scheduler(params(thermal_off_rule=False))
    for t in (0.0, 3.0, 6.0):
        assert off.step(t, {"rgb": 1.0, "thermal": 1.0}, 0.5, BOTH).enabled["thermal"]


# ------------------------------------------------- D10: recovery override
def test_recovery_uses_level_zero_budget_until_ok_for_hold_time():
    s = Scheduler(params(min_dwell_s=0.0))
    s.step(0.0, GOOD, 0.5, BOTH); s.step(0.1, GOOD, 0.1, BOTH)
    assert s.level == 2
    c = s.step(1.0, GOOD, 0.1, BOTH, tracking_ok=False)
    assert c.recovery and c.level == 2 and c.n_features_total == 1200 and c.loop_closure_mode == "off"
    assert s.step(1.5, GOOD, 0.1, BOTH).recovery                     # OK since 1.5
    assert s.step(3.4, GOOD, 0.1, BOTH).recovery
    c = s.step(3.5, GOOD, 0.1, BOTH)
    assert not c.recovery and c.n_features_total == 500
    assert [(tr["old"], tr["new"]) for tr in s.transitions if tr["kind"] == "recovery"] == [(False, True), (True, False)]


def test_no_recovery_override_without_scheduler():
    s = Scheduler(params(), energy_scheduling=False)
    assert not s.step(0.0, GOOD, 0.9, BOTH, tracking_ok=False).recovery


# ------------------------------------------------- D2: thermal sensor model
def test_thermal_sensor_model_deterministic_and_blurs():
    raw = np.full((240, 320), 58000, np.uint16)
    raw[:, 160:] = 59000
    m1, m2 = (ThermalSensorModel(raw.shape, 0.005, seed=5) for _ in range(2))
    a, b = m1(raw, 7), m2(raw, 7)
    assert a.tobytes() == b.tobytes() and a.tobytes() != m1(raw, 8).tobytes()
    assert 58000 < a[:, 159].mean() < a[:, 160].mean() < 59000        # the edge is no longer a step
    assert abs((m1(raw, 7) - m1(raw, 8))[:, :100].std() * 0.005 - 0.05 * 2 ** 0.5) < 0.01   # NETD 0.05 K
