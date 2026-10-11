"""Prepare the sensor streams of a sequence for replay: this stands in for the physical sensors.

RGB: degradation profile, then grey. Depth: fog degradation (D3). Thermal: sensor model (D2: optics blur,
fixed-pattern and temporal noise at native resolution), fixed temperature window to 8 bit, CLAHE, then
bilinear resize to the RGB pixel grid. Everything is seeded and identical for every configuration.
"""
import cv2

from controller.thermal import ThermalPreprocessor
from degradation.depth import apply_depth
from degradation.profiles import apply
from degradation.thermal_sensor import ThermalSensorModel
from harness.sequence import load_yaml


def thermal_sensor(seq, variant=None, variant_params=None):
    """variant: name of an entry in sensors.yaml:thermal_variants (replay-time change of the thermal stream),
    or pass variant_params = dict(blur_sigma_px, contrast_gain, contrast_pivot_k) directly."""
    cfg = load_yaml("sensors.yaml")
    m = cfg["thermal_model"]
    extra = {}
    if variant_params is None and variant:
        variant_params = cfg["thermal_variants"][variant]
    if variant_params:
        extra = dict(contrast_gain=variant_params["contrast_gain"], contrast_pivot_k=variant_params["contrast_pivot_k"])
        m = dict(m, blur_sigma_px=variant_params["blur_sigma_px"])
    shape = seq.thermal_raw(0).shape
    return ThermalSensorModel(shape, seq.camera["thermal_resolution_K_per_count"], blur_sigma_px=m["blur_sigma_px"],
                              netd_k=m["netd_k"], fixed_pattern_k=m["fixed_pattern_k"], seed=m["seed"], **extra)


def thermal_preprocessor(seq, calib):
    return ThermalPreprocessor(*calib["thermal_window_K"], kelvin_per_count=seq.camera["thermal_resolution_K_per_count"],
                               **calib["thermal_preprocessing"])


class Streams:
    """Frame-by-frame access to the prepared streams (index = frame number in the sequence)."""

    def __init__(self, seq, profile, calib=None, thermal_variant=None, thermal_variant_params=None):
        self.seq, self.profile = seq, profile
        self.sensor = thermal_sensor(seq, thermal_variant, thermal_variant_params) if seq.has_thermal else None
        self.pre = thermal_preprocessor(seq, calib) if (calib and seq.has_thermal) else None

    def grey(self, i):
        bgr, severity = apply(self.profile, self.seq.rgb(i), self.seq.stamps[i], i)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), severity

    def depth(self, i):
        return apply_depth(self.profile, self.seq.depth(i), self.seq.stamps[i], i, self.seq.camera["depth_scale"])

    def thermal_counts(self, i):
        """Radiometric counts after the sensor model, native resolution (float32)."""
        return self.sensor(self.seq.thermal_raw(i), i)

    def thermal8(self, i):
        """8-bit thermal image at native resolution."""
        return self.pre(self.thermal_counts(i))

    def thermal8_on_rgb_grid(self, i, size):
        return cv2.resize(self.thermal8(i), size, interpolation=cv2.INTER_LINEAR)
