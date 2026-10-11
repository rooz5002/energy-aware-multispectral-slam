"""Perception quality (paper Eq. 1):  Q_m(t) = alpha*N_m(t) + beta*H_m(t) - gamma*B_m(t).

All three indicators are normalised to [0, 1]. No ROS or simulator dependency:
this module is used unchanged on the real robot.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class QualityParams:
    alpha: float
    beta: float
    gamma: float
    v_ref: dict  # modality -> reference variance of Laplacian on clear frames (frozen after tuning)


@dataclass(frozen=True)
class Quality:
    N: float
    H: float
    B: float
    Q: float
    var_laplacian: float


def feature_score(count: float, budget: float) -> float:
    """N_m: tracked inliers (or detected keypoints before tracking starts) over the modality's feature budget."""
    if budget <= 0:
        return 0.0
    return float(min(1.0, max(0.0, count / budget)))


def entropy_score(img8: np.ndarray) -> float:
    """H_m: Shannon entropy of the 8-bit image in bits, divided by 8."""
    hist = np.bincount(img8.ravel(), minlength=256).astype(np.float64)
    p = hist[hist > 0] / img8.size
    return float(-(p * np.log2(p)).sum() / 8.0)


BLUR_PREFILTER_SIGMA_PX = 1.0


def variance_of_laplacian(img8: np.ndarray) -> float:
    """Variance of the Laplacian after a Gaussian pre-filter, so pixel noise is not read as sharpness."""
    smooth = cv2.GaussianBlur(img8, (0, 0), BLUR_PREFILTER_SIGMA_PX)
    return float(cv2.Laplacian(smooth, cv2.CV_64F).var())


def blur_score(img8: np.ndarray, v_ref: float):
    """B_m = 1 - min(1, varLaplacian / V_ref): larger B means blurrier. Returns (B, varLaplacian)."""
    v = variance_of_laplacian(img8)
    return float(1.0 - min(1.0, v / v_ref)), v


def assess(img8: np.ndarray, feature_count: float, feature_budget: float, modality: str, p: QualityParams) -> Quality:
    if img8.dtype != np.uint8 or img8.ndim != 2:
        raise ValueError("quality is computed on a single-channel 8-bit image")
    N = feature_score(feature_count, feature_budget)
    H = entropy_score(img8)
    B, v = blur_score(img8, p.v_ref[modality])
    return Quality(N=N, H=H, B=B, Q=p.alpha * N + p.beta * H - p.gamma * B, var_laplacian=v)
