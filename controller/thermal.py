"""Thermal preprocessing: 16-bit radiometric frame -> 8-bit image with a FIXED temperature window.

The window is fixed per sequence (not per-frame auto-scaling, which would create
artificial contrast changes), followed by optional CLAHE with fixed parameters.
"""
import cv2
import numpy as np


class ThermalPreprocessor:
    def __init__(self, t_min_k: float, t_max_k: float, kelvin_per_count: float = 0.01,
                 clahe: bool = True, clahe_clip: float = 2.0, clahe_grid: int = 8):
        if t_max_k <= t_min_k:
            raise ValueError("empty temperature window")
        self.lo = t_min_k / kelvin_per_count
        self.scale = 255.0 / ((t_max_k - t_min_k) / kelvin_per_count)
        self.clahe = cv2.createCLAHE(clipLimit=clahe_clip, tileGridSize=(clahe_grid, clahe_grid)) if clahe else None

    def __call__(self, raw: np.ndarray) -> np.ndarray:
        """raw: radiometric counts, uint16 or float."""
        img = np.clip(np.rint((raw.astype(np.float32) - self.lo) * self.scale), 0, 255).astype(np.uint8)
        return self.clahe.apply(img) if self.clahe is not None else img
