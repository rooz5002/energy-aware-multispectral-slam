import numpy as np
import pytest

from controller import quality
from degradation.depth import apply_depth
from degradation.profiles import Profile, apply

FOG = Profile("fog_test", "fog", onset_s=10.0, ramp_s=5.0, hold_s=20.0, peak=0.8, seed=7)


def frame(seed=1):
    return np.random.default_rng(seed).integers(0, 256, (120, 160, 3), dtype=np.uint8)


@pytest.mark.parametrize("t,expected", [(0.0, 0.0), (10.0, 0.0), (12.5, 0.4), (15.0, 0.8), (25.0, 0.8),
                                        (35.0, 0.8), (37.5, 0.4), (40.0, 0.0), (99.0, 0.0)])
def test_severity_schedule(t, expected):
    assert FOG.severity(t) == pytest.approx(expected)
    assert FOG.offset_s == 40.0


@pytest.mark.parametrize("kind", ["fog", "lowlight", "motion_blur"])
def test_same_seed_identical_bytes(kind):
    p = Profile("p", kind, 0.0, 1.0, 10.0, 0.9, seed=3)
    a, _ = apply(p, frame(), 5.0, 42)
    b, _ = apply(p, frame(), 5.0, 42)
    assert a.tobytes() == b.tobytes()


def test_identical_regardless_of_processing_order():
    p = Profile("p", "fog", 0.0, 1.0, 10.0, 0.9, seed=3)
    first, _ = apply(p, frame(), 5.0, 10)
    for i in (3, 99, 7):          # process other frames in between
        apply(p, frame(), 5.0, i)
    again, _ = apply(p, frame(), 5.0, 10)
    assert first.tobytes() == again.tobytes()


def test_different_seed_or_frame_differs():
    p3, p4 = (Profile("p", "lowlight", 0.0, 1.0, 10.0, 0.9, seed=s) for s in (3, 4))
    assert apply(p3, frame(), 5.0, 1)[0].tobytes() != apply(p4, frame(), 5.0, 1)[0].tobytes()
    assert apply(p3, frame(), 5.0, 1)[0].tobytes() != apply(p3, frame(), 5.0, 2)[0].tobytes()


def test_clear_frames_untouched_and_input_not_modified():
    img = frame()
    before = img.copy()
    out, s = apply(FOG, img, 0.0, 0)
    assert s == 0.0 and out is img
    apply(FOG, img, 20.0, 5)
    assert np.array_equal(img, before)


@pytest.mark.parametrize("kind", ["fog", "lowlight", "motion_blur"])
def test_degradation_lowers_sharpness(kind):
    import cv2
    p = Profile("p", kind, 0.0, 1.0, 10.0, 0.9, seed=3)
    img = frame()
    out, s = apply(p, img, 5.0, 0)
    assert s == pytest.approx(0.9) and out.dtype == np.uint8 and out.shape == img.shape
    g0, g1 = (cv2.cvtColor(x, cv2.COLOR_BGR2GRAY) for x in (img, out))
    assert quality.variance_of_laplacian(g1) < quality.variance_of_laplacian(g0)


# ------------------------------------------------- D3: depth in fog
def depth_frame():
    d = np.tile(np.linspace(500, 9000, 160), (120, 1)).astype(np.uint16)   # 0.5 .. 9 m
    d[:10] = 0
    return d


def test_depth_untouched_when_clear_or_not_fog():
    d = depth_frame()
    out, valid = apply_depth(FOG, d, 0.0, 3, 1000.0)
    assert out is d and valid == pytest.approx((d > 0).mean())
    low = Profile("l", "lowlight", 0.0, 1.0, 10.0, 0.9, seed=3)
    assert apply_depth(low, d, 5.0, 3, 1000.0)[0] is d


def test_depth_fog_dropout_noise_and_determinism():
    d = depth_frame()
    a, va = apply_depth(FOG, d, 20.0, 5, 1000.0)
    b, vb = apply_depth(FOG, d, 20.0, 5, 1000.0)
    assert a.tobytes() == b.tobytes() and va == vb
    assert (a[:10] == 0).all()                                   # invalid stays invalid
    near, far = a[10:, :20], a[10:, -20:]
    assert (far == 0).mean() > (near == 0).mean() + 0.2          # dropout grows with range
    assert 0.40 < (far == 0).mean() < 0.56                       # 0.6 * 0.8 * 1 beyond z_max = 3 m
    assert 0.05 < (near == 0).mean() < 0.22                      # about 0.6 * 0.8 * (0.5..1.5 m / 3 m)
    err_near = np.abs(near[near > 0].astype(float) - d[10:, :20][near > 0])
    err_far = np.abs(far[far > 0].astype(float) - d[10:, -20:][far > 0])
    assert err_near.mean() < 40 and err_far.mean() > 500         # sigma = 0.01 + 0.03 * s * z^2 (metres)
