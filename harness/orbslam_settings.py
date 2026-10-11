"""Write the ORB-SLAM3 settings file for a sequence from its camera.yaml."""
from pathlib import Path

# Depth handling shared by all configurations (documented in IMPLEMENTATION_NOTES): values of the TUM RGB-D
# examples shipped with ORB-SLAM3. Points closer than baseline * ThDepth (3.2 m) are treated as close points.
STEREO_BASELINE_M = 0.08
TH_DEPTH = 40.0

TEMPLATE = """%YAML:1.0
File.version: "1.0"
Camera.type: "PinHole"
Camera1.fx: {fx:.6f}
Camera1.fy: {fy:.6f}
Camera1.cx: {cx:.6f}
Camera1.cy: {cy:.6f}
Camera1.k1: 0.0
Camera1.k2: 0.0
Camera1.p1: 0.0
Camera1.p2: 0.0
Camera.width: {width}
Camera.height: {height}
Camera.fps: {fps}
Camera.RGB: 0
Stereo.ThDepth: {th_depth}
Stereo.b: {baseline}
RGBD.DepthMapFactor: {depth_scale}
ORBextractor.nFeatures: {n_features}
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
"""


def write_settings(camera: dict, path, n_features: int = 1200) -> Path:
    """camera: contents of camera.yaml. Images are passed to ORB-SLAM3 as BGR (Camera.RGB: 0) or grey."""
    c = camera["rgb"]
    Path(path).write_text(TEMPLATE.format(fx=c["fx"], fy=c["fy"], cx=c["cx"], cy=c["cy"], width=c["width"],
                                          height=c["height"], fps=int(round(camera["fps"])), th_depth=TH_DEPTH,
                                          baseline=STEREO_BASELINE_M, depth_scale=camera["depth_scale"],
                                          n_features=n_features))
    return Path(path)
