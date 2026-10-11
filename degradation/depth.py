"""Depth degradation in fog/smoke (DECISIONS D3). Low light and motion blur leave depth untouched.

For a fog profile with severity s at this frame and depth z (metres):
  dropout:  a valid pixel becomes invalid with probability 0.6 * s * min(z / Z_MAX, 1)
  noise:    remaining pixels get Gaussian noise with sigma_z = 0.01 m + 0.03 * s * z^2
Same severity schedule and seed as the RGB degradation; seeded by (seed, frame index), so identical
for every configuration.
"""
import numpy as np

from .profiles import Profile

Z_MAX_M = 3.0           # D435 listed maximum range (DECISIONS v3, D3); dropout probability saturates here
DROPOUT_GAIN = 0.6
SIGMA_BASE_M = 0.01
SIGMA_GAIN = 0.03


def apply_depth(profile: Profile, depth: np.ndarray, t: float, frame_index: int, depth_scale: float):
    """depth: uint16, depth_scale counts per metre, 0 = invalid. Returns (degraded depth, valid fraction)."""
    s = profile.severity(t) if profile.kind == "fog" else 0.0
    if s <= 0.0:
        return depth, float((depth > 0).mean())
    rng = np.random.default_rng([profile.seed, 2, frame_index])
    z = depth.astype(np.float32) / depth_scale
    valid = depth > 0
    drop = rng.random(depth.shape, dtype=np.float32) < DROPOUT_GAIN * s * np.minimum(z / Z_MAX_M, 1.0)
    z = z + rng.standard_normal(depth.shape, dtype=np.float32) * (SIGMA_BASE_M + SIGMA_GAIN * s * z * z)
    keep = valid & ~drop & (z > 0)
    out = np.where(keep, np.clip(np.rint(z * depth_scale), 1, 65535), 0).astype(np.uint16)
    return out, float(keep.mean())
