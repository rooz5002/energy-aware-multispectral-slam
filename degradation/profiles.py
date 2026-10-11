"""Degradation of the RGB stream at replay time (guide section 5).

Thermal is never degraded: this models thermal imaging's insensitivity to smoke and
darkness and is an explicit modelling assumption (docs/IMPLEMENTATION_NOTES.md).

A profile is a time schedule clear -> ramp in -> hold -> ramp out -> clear with severity
in [0, 1]. All randomness comes from a generator seeded by (profile seed, frame index),
so a frame's degraded pixels do not depend on which other frames were processed and are
bit-identical across configurations and runs.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Profile:
    name: str
    kind: str            # "none" | "fog" | "lowlight" | "motion_blur"
    onset_s: float = 0.0     # start of ramp in
    ramp_s: float = 0.0      # ramp duration (in and out)
    hold_s: float = 0.0      # duration at peak severity
    peak: float = 0.0        # peak severity in [0, 1]
    seed: int = 0

    @property
    def offset_s(self) -> float:
        """End of ramp out."""
        return self.onset_s + 2 * self.ramp_s + self.hold_s

    def severity(self, t: float) -> float:
        if self.kind == "none" or t <= self.onset_s or t >= self.offset_s:
            return 0.0
        up_end, down_start = self.onset_s + self.ramp_s, self.onset_s + self.ramp_s + self.hold_s
        if t < up_end:
            return self.peak * (t - self.onset_s) / self.ramp_s
        if t <= down_start:
            return self.peak
        return self.peak * (self.offset_s - t) / self.ramp_s


def _rng(profile: Profile, frame_index: int):
    return np.random.default_rng([profile.seed, frame_index])


def fog(bgr, s, rng):
    """Blend toward grey (airlight), lose contrast, slight blur."""
    img = bgr.astype(np.float32)
    transmission = 1.0 - 0.92 * s
    airlight = 185.0 + rng.normal(0.0, 1.5)  # slow flicker of the scattering medium
    img = img * transmission + airlight * (1.0 - transmission)
    sigma = 0.3 + 2.2 * s
    img = cv2.GaussianBlur(img, (0, 0), sigma)
    img += rng.normal(0.0, 1.0 + 2.0 * s, img.shape).astype(np.float32)
    return np.clip(np.rint(img), 0, 255).astype(np.uint8)


def lowlight(bgr, s, rng):
    """Reduce exposure, apply gamma, then shot (Poisson) and read (Gaussian) noise."""
    img = bgr.astype(np.float32) / 255.0
    gain = 1.0 - 0.95 * s
    img = np.power(img * gain, 1.0 + 1.2 * s)
    full_well = 400.0 * (1.0 - 0.9 * s) + 8.0   # fewer photo-electrons -> more shot noise
    img = rng.poisson(img * full_well).astype(np.float32) / full_well
    img += rng.normal(0.0, 0.004 + 0.012 * s, img.shape).astype(np.float32)
    return np.clip(np.rint(img * 255.0), 0, 255).astype(np.uint8)


def motion_blur(bgr, s, rng):
    """Directional (horizontal) box kernel, length grows with severity."""
    length = 1 + 2 * int(round(12 * s))
    if length <= 1:
        return bgr.copy()
    kernel = np.zeros((length, length), np.float32)
    kernel[length // 2, :] = 1.0 / length
    return cv2.filter2D(bgr, -1, kernel, borderType=cv2.BORDER_REPLICATE)


_KINDS = {"fog": fog, "lowlight": lowlight, "motion_blur": motion_blur}


def apply(profile: Profile, bgr: np.ndarray, t: float, frame_index: int) -> tuple:
    """Returns (degraded image, severity). The input is not modified."""
    s = profile.severity(t)
    if s <= 0.0:
        return bgr, 0.0
    return _KINDS[profile.kind](bgr, s, _rng(profile, frame_index)), s
