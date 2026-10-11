"""Procedural RGB (albedo) textures for the worlds.

Everything is seeded, so regenerating gives identical files. Thermal structure does
not come from textures; see the thermal decals in build_worlds.py.
"""
from pathlib import Path

import cv2
import numpy as np

SIZE = 1024
COUNTS = {  # texture family -> number of variants
    "rgb_wall": 12, "rgb_plain": 6, "rgb_floor": 4, "rgb_ceiling": 2, "rgb_machine": 8,
    "rgb_wood": 4, "rgb_door": 3, "rgb_crate": 4,
}


def _noise(rng, sigma, size=SIZE):
    img = cv2.GaussianBlur(rng.random((size, size)).astype(np.float32), (0, 0), sigma)
    return (img - img.min()) / (img.max() - img.min() + 1e-9)


def _colour(rng, lo=40, hi=230):
    return tuple(int(v) for v in rng.integers(lo, hi, 3))


def _text(img, rng, org, scale=1.0, colour=(20, 20, 20)):
    chars = "ABCDEFGHKLMNPRSTXYZ0123456789"
    s = "".join(rng.choice(list(chars), int(rng.integers(3, 8))))
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour, max(1, int(2 * scale)), cv2.LINE_AA)


def _poster(img, rng, x, y, w, h):
    cv2.rectangle(img, (x, y), (x + w, y + h), _colour(rng), -1)
    cv2.rectangle(img, (x, y), (x + w, y + h), (25, 25, 25), 4)
    for _ in range(int(rng.integers(3, 8))):
        px, py = int(x + rng.integers(8, max(9, w - 40))), int(y + rng.integers(20, max(21, h - 10)))
        kind = rng.integers(0, 3)
        if kind == 0:
            _text(img, rng, (px, py), float(rng.uniform(0.6, 1.4)), _colour(rng, 0, 255))
        elif kind == 1:
            cv2.circle(img, (px, py), int(rng.integers(8, 30)), _colour(rng, 0, 255), -1)
        else:
            cv2.rectangle(img, (px, py), (px + int(rng.integers(15, 60)), py + int(rng.integers(15, 60))),
                          _colour(rng, 0, 255), -1)


def _paint(rng, base, amount=18, grain=4):
    n = _noise(rng, 50)[..., None] * amount + rng.normal(0, grain, (SIZE, SIZE, 1))
    return np.clip(np.array(base, np.float32)[None, None, :] + n - amount / 2, 0, 255).astype(np.uint8).copy()


# ---------------------------------------------------------------- RGB textures
def rgb_wall(rng):
    img = _paint(rng, rng.integers(150, 225, 3))
    cv2.rectangle(img, (0, SIZE - 60), (SIZE, SIZE), (60, 55, 50), -1)  # skirting board
    for _ in range(int(rng.integers(3, 6))):
        w, h = int(rng.integers(140, 320)), int(rng.integers(140, 320))
        _poster(img, rng, int(rng.integers(10, SIZE - w - 10)), int(rng.integers(60, SIZE - h - 120)), w, h)
    for _ in range(10):  # sockets, switches, marks
        x, y = int(rng.integers(20, SIZE - 60)), int(rng.integers(20, SIZE - 80))
        cv2.rectangle(img, (x, y), (x + 34, y + 34), (240, 240, 240), -1)
        cv2.rectangle(img, (x, y), (x + 34, y + 34), (40, 40, 40), 2)
        cv2.circle(img, (x + 17, y + 17), 5, (30, 30, 30), -1)
    _fine_marks(img, rng, 260)
    return img


def _fine_marks(img, rng, count):
    """Small scuffs, chips, pin holes and pencil marks: detail that survives a close-up view."""
    for _ in range(count):
        x, y = int(rng.integers(4, SIZE - 4)), int(rng.integers(4, SIZE - 4))
        shade = int(rng.integers(20, 110)) if rng.random() < 0.7 else int(rng.integers(225, 255))
        kind = rng.integers(0, 3)
        if kind == 0:
            cv2.circle(img, (x, y), int(rng.integers(2, 6)), (shade, shade, shade), -1)
        elif kind == 1:
            cv2.rectangle(img, (x, y), (x + int(rng.integers(3, 10)), y + int(rng.integers(3, 10))), (shade,) * 3, -1)
        else:
            cv2.line(img, (x, y), (x + int(rng.integers(-18, 18)), y + int(rng.integers(-18, 18))), (shade,) * 3, 2)


def rgb_plain(rng):
    """Low-texture wall for the corridor: bare paint, faint stains, one small sign."""
    img = _paint(rng, rng.integers(165, 200, 3), amount=10, grain=2)
    cv2.rectangle(img, (0, SIZE - 40), (SIZE, SIZE), (90, 90, 90), -1)
    x, y = int(rng.integers(100, SIZE - 300)), int(rng.integers(250, 500))
    cv2.rectangle(img, (x, y), (x + 170, y + 90), (235, 235, 235), -1)
    cv2.rectangle(img, (x, y), (x + 170, y + 90), (30, 30, 30), 3)
    _text(img, rng, (x + 10, y + 60), 1.0)
    for _ in range(4):
        px, py = int(rng.integers(0, SIZE)), int(rng.integers(0, SIZE))
        cv2.circle(img, (px, py), int(rng.integers(3, 9)), (110, 105, 100), -1)
    return img


def rgb_floor(rng):
    img = _paint(rng, rng.integers(120, 180, 3), amount=25, grain=7)
    for i in range(0, SIZE, 128):  # tile joints
        cv2.line(img, (i, 0), (i, SIZE), (70, 70, 70), 4)
        cv2.line(img, (0, i), (SIZE, i), (70, 70, 70), 4)
    for _ in range(260):  # speckles, scuffs
        x, y = int(rng.integers(0, SIZE)), int(rng.integers(0, SIZE))
        cv2.circle(img, (x, y), int(rng.integers(2, 9)), _colour(rng, 30, 220), -1)
    for _ in range(12):
        x, y = int(rng.integers(0, SIZE - 80)), int(rng.integers(0, SIZE - 80))
        cv2.line(img, (x, y), (x + int(rng.integers(-80, 80)), y + int(rng.integers(-80, 80))), (45, 45, 45), 3)
    return img


def rgb_ceiling(rng):
    img = _paint(rng, (225, 225, 220), amount=8, grain=3)
    for i in range(0, SIZE, 256):
        cv2.line(img, (i, 0), (i, SIZE), (150, 150, 150), 6)
        cv2.line(img, (0, i), (SIZE, i), (150, 150, 150), 6)
    return img


def rgb_machine(rng):
    img = _paint(rng, rng.integers(60, 200, 3), amount=20, grain=5)
    for _ in range(8):
        x, y = int(rng.integers(0, SIZE - 260)), int(rng.integers(0, SIZE - 260))
        w, h = int(rng.integers(90, 260)), int(rng.integers(90, 260))
        cv2.rectangle(img, (x, y), (x + w, y + h), _colour(rng), -1)
        cv2.rectangle(img, (x, y), (x + w, y + h), (20, 20, 20), 3)
        _text(img, rng, (x + 8, y + h // 2), 0.9, (250, 250, 250))
    for x in range(0, SIZE, 80):  # warning stripes
        cv2.line(img, (x, SIZE - 70), (x + 40, SIZE), (0, 200, 240), 18)
    for _ in range(60):  # bolts
        x, y = int(rng.integers(0, SIZE)), int(rng.integers(0, SIZE))
        cv2.circle(img, (x, y), 6, (30, 30, 30), -1)
        cv2.circle(img, (x, y), 3, (190, 190, 190), -1)
    _fine_marks(img, rng, 150)
    return img


def rgb_wood(rng):
    img = _paint(rng, (rng.integers(70, 120), rng.integers(110, 160), rng.integers(150, 200)), amount=30, grain=5)
    for y in range(0, SIZE, int(rng.integers(18, 34))):
        cv2.line(img, (0, y), (SIZE, y + int(rng.integers(-12, 12))), (50, 80, 110), 2)
    for _ in range(8):
        cv2.circle(img, (int(rng.integers(0, SIZE)), int(rng.integers(0, SIZE))), int(rng.integers(8, 22)),
                   (40, 60, 85), 3)
    _fine_marks(img, rng, 120)
    return img


def rgb_door(rng):
    img = _paint(rng, rng.integers(70, 150, 3), amount=12, grain=3)
    cv2.rectangle(img, (40, 40), (SIZE - 40, SIZE - 20), (30, 30, 30), 10)
    cv2.rectangle(img, (140, 120), (SIZE - 140, 460), (40, 40, 40), 6)
    cv2.rectangle(img, (140, 540), (SIZE - 140, 900), (40, 40, 40), 6)
    cv2.circle(img, (SIZE - 110, 520), 26, (200, 200, 200), -1)
    cv2.rectangle(img, (380, 200), (640, 290), (240, 240, 240), -1)
    _text(img, rng, (395, 265), 1.2)
    _fine_marks(img, rng, 120)
    return img


def rgb_crate(rng):
    img = rgb_wood(rng)
    for i in range(0, SIZE, 170):
        cv2.line(img, (i, 0), (i, SIZE), (30, 45, 60), 8)
    cv2.line(img, (0, 0), (SIZE, SIZE), (30, 45, 60), 14)
    cv2.rectangle(img, (300, 380), (720, 640), (235, 235, 235), -1)
    _text(img, rng, (320, 540), 2.2)
    return img



def generate(out_dir: Path, keep_existing=False):
    """Write all textures; returns {family: [paths]}. keep_existing: delete nothing, write only missing files."""
    if not keep_existing:
        for stale in out_dir.glob("*.png"):
            stale.unlink()
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = {}
    for fam_idx, (family, count) in enumerate(sorted(COUNTS.items())):
        fn = globals()[family]
        paths[family] = []
        for i in range(count):
            rng = np.random.default_rng(1000 * (fam_idx + 1) + i)
            path = out_dir / f"{family}_{i}.png"
            if not (keep_existing and path.exists()):
                cv2.imwrite(str(path), fn(rng))
            paths[family].append(path)
    return paths
