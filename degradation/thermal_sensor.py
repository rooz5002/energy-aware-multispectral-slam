"""Thermal camera model applied to the rendered radiometric frames at replay (DECISIONS D2).

Gazebo renders sharp, noise-free thermal frames. This adds, at the native 320x240 resolution:
  1. optics blur (Gaussian, sigma in pixels),
  2. fixed-pattern noise (one seeded pattern per run of the model, constant over time),
  3. temporal noise (NETD), seeded by (seed, frame index).
Like the RGB degradation, it is deterministic: the same seed gives bit-identical frames.
"""
import cv2
import numpy as np


class ThermalSensorModel:
    def __init__(self, shape, kelvin_per_count, blur_sigma_px=0.7, netd_k=0.05, fixed_pattern_k=0.02, seed=0,
                 contrast_gain=1.0, contrast_pivot_k=None):
        """contrast_gain < 1 compresses scene contrast about contrast_pivot_k before the sensor noise is added
        (used only by the sparse-thermal robustness variant, DECISIONS D14)."""
        self.kpc, self.sigma, self.netd, self.seed = kelvin_per_count, blur_sigma_px, netd_k, seed
        self.gain = contrast_gain
        self.pivot = (contrast_pivot_k / kelvin_per_count) if contrast_pivot_k is not None else None
        self.fpn = (np.random.default_rng([seed, 0xF1]).normal(0.0, fixed_pattern_k, shape) / kelvin_per_count).astype(np.float32)

    def __call__(self, raw16: np.ndarray, frame_index: int) -> np.ndarray:
        """raw16: uint16 radiometric counts. Returns float32 counts (same unit)."""
        img = raw16.astype(np.float32)
        if self.sigma > 0:
            img = cv2.GaussianBlur(img, (0, 0), self.sigma)
        if self.gain != 1.0:
            img = self.pivot + self.gain * (img - self.pivot)
        noise = np.random.default_rng([self.seed, 1, frame_index]).normal(0.0, self.netd / self.kpc, img.shape)
        return img + self.fpn + noise.astype(np.float32)
