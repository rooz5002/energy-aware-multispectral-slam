#!/usr/bin/env python3
"""Build sim/models/tb3_waffle_ms from the upstream TurtleBot 3 Waffle model.

Changes relative to upstream (third_party/turtlebot3_simulations, pinned commit):
  * lidar sensor removed (not used; saves rendering time),
  * the 1920x1080 RGB camera is replaced by a co-located sensor head
    (RGB 640x480, depth 640x480, thermal 320x240; same pose, same FOV, 30 Hz),
  * a ground-truth odometry publisher (true world pose of base_footprint) is added.
"""
import re
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
UPSTREAM = HERE.parents[1] / "third_party/turtlebot3_simulations/turtlebot3_gazebo/models"
OUT = HERE / "tb3_waffle_ms"

HFOV = 1.047  # rad, ~60 deg (guide section 4.2)
# all visibility bits except the thermal-decal flag (0x2, see sim/worldgen/build_worlds.py)
NO_DECALS = 0x0FFFFFFF & ~0x2
RATE = 30
# kelvin per count of the 16-bit thermal image. 0.005 gives a 327.7 K ceiling and halves the simulator's
# temperature quantisation step compared with 0.01 (see sim/worldgen/build_worlds.py).
THERMAL_RESOLUTION_K = 0.005

SENSOR_HEAD = f"""
      <sensor name="rgb" type="camera">
        <always_on>true</always_on>
        <update_rate>{RATE}</update_rate>
        <topic>ms/rgb/image</topic>
        <gz_frame_id>camera_rgb_frame</gz_frame_id>
        <camera name="rgb">
          <visibility_mask>{NO_DECALS}</visibility_mask>
          <camera_info_topic>ms/rgb/camera_info</camera_info_topic>
          <horizontal_fov>{HFOV}</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>50</far></clip>
          <noise><type>gaussian</type><mean>0.0</mean><stddev>0.007</stddev></noise>
        </camera>
      </sensor>
      <sensor name="depth" type="depth_camera">
        <always_on>true</always_on>
        <update_rate>{RATE}</update_rate>
        <topic>ms/depth/image</topic>
        <gz_frame_id>camera_rgb_frame</gz_frame_id>
        <camera name="depth">
          <visibility_mask>{NO_DECALS}</visibility_mask>
          <camera_info_topic>ms/depth/camera_info</camera_info_topic>
          <horizontal_fov>{HFOV}</horizontal_fov>
          <image><width>640</width><height>480</height><format>R_FLOAT32</format></image>
          <clip><near>0.05</near><far>50</far></clip>
        </camera>
      </sensor>
      <sensor name="thermal" type="thermal">
        <always_on>true</always_on>
        <update_rate>{RATE}</update_rate>
        <topic>ms/thermal/image</topic>
        <gz_frame_id>camera_rgb_frame</gz_frame_id>
        <camera name="thermal">
          <camera_info_topic>ms/thermal/camera_info</camera_info_topic>
          <horizontal_fov>{HFOV}</horizontal_fov>
          <image><width>320</width><height>240</height><format>L16</format></image>
          <clip><near>0.05</near><far>50</far></clip>
        </camera>
        <plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor">
          <min_temp>253.15</min_temp>
          <max_temp>327.67</max_temp>
          <resolution>{THERMAL_RESOLUTION_K}</resolution>
        </plugin>
      </sensor>
"""

GROUND_TRUTH = """
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odom_topic>ms/ground_truth</odom_topic>
      <odom_publish_frequency>50</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
"""


def main():
    sdf = (UPSTREAM / "turtlebot3_waffle/model.sdf").read_text()
    sdf = sdf.replace('<model name="turtlebot3_waffle">', '<model name="tb3_waffle_ms">')
    sdf, n = re.subn(r'\s*<sensor name="hls_lfcd_lds".*?</sensor>', "", sdf, flags=re.S)
    assert n == 1, "lidar sensor not found"
    sdf, n = re.subn(r'\s*<sensor name="camera" type="camera">.*?</sensor>', SENSOR_HEAD, sdf, flags=re.S)
    assert n == 1, "camera sensor not found"
    # ground truth must not share the wheel-odometry TF topic
    sdf = sdf.replace("  </model>", GROUND_TRUTH + "\n  </model>")
    assert "ms/ground_truth" in sdf

    OUT.mkdir(exist_ok=True)
    (OUT / "model.sdf").write_text(sdf)
    (OUT / "model.config").write_text(
        '<?xml version="1.0"?>\n<model>\n  <name>tb3_waffle_ms</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.8">model.sdf</sdf>\n'
        "  <description>TurtleBot 3 Waffle with co-located RGB, depth and thermal cameras.</description>\n</model>\n")
    common = HERE / "turtlebot3_common"
    if common.exists():
        shutil.rmtree(common)
    shutil.copytree(UPSTREAM / "turtlebot3_common", common)
    (OUT / "sensors.yaml").write_text(
        f"# written by build_robot_model.py; copied into every recorded sequence\n"
        f"thermal_resolution_K_per_count: {THERMAL_RESOLUTION_K}\nfps: {RATE}\nhorizontal_fov_rad: {HFOV}\n")
    print("wrote", OUT / "model.sdf")


if __name__ == "__main__":
    main()
